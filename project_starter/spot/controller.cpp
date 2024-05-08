/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <random>

using namespace std;
using namespace Eigen;
using namespace Sai2Primitives;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/spot.urdf";

// States 
enum State {
	POSTURE = 0, 
	FEET_FIXED_MOTION,
    FLIGHT
};

int main() {
	// initial state 
	int state = POSTURE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

    // create tasks
    auto body_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, "body", Affine3d::Identity());
    body_task->disableInternalOtg();
    body_task->setPosControlGains(400, 40, 0);
    body_task->setOriControlGains(400, 40, 0);
    Vector3d body_pos;

	// create map for feet task 
    std::map<std::string, std::shared_ptr<Sai2Primitives::MotionForceTask>> primary_tasks;
    const std::vector<std::string> primary_control_links = {"front_left_foot", "front_right_foot", "rear_left_foot", "rear_right_foot"};
    const std::vector<Vector3d> primary_control_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)};
    std::vector<Vector3d> controlled_directions_translation = {Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()};

    for (int i = 0; i < primary_control_links.size(); ++i) {        
        Affine3d compliant_frame = Affine3d::Identity();
        compliant_frame.translation() = primary_control_points[i];
        primary_tasks[primary_control_links[i]] = std::make_shared<Sai2Primitives::MotionForceTask>(robot, 
                                                                                                    primary_control_links[i], 
                                                                                                    controlled_directions_translation, 
                                                                                                    std::vector<Vector3d>{}, 
                                                                                                    compliant_frame);
        primary_tasks[primary_control_links[i]]->disableInternalOtg();
        primary_tasks[primary_control_links[i]]->setDynamicDecouplingType(Sai2Primitives::DynamicDecouplingType::FULL_DYNAMIC_DECOUPLING);
        primary_tasks[primary_control_links[i]]->setPosControlGains(400, 40, 0);
        primary_tasks[primary_control_links[i]]->setOriControlGains(400, 40, 0);
        primary_tasks[primary_control_links[i]]->handleAllSingularitiesAsType1(true);
    }

    MatrixXd U = MatrixXd::Zero(robot->dof() - 6, robot->dof());
    U.block(0, 6, robot->dof() - 6, robot->dof() - 6).setIdentity();

	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->disableInternalOtg();
	VectorXd q_desired = robot->q();
	joint_task->setGains(400, 40, 0);
	joint_task->setGoalPosition(q_desired);

	// get starting poses
    std::vector<Affine3d> primary_starting_pose;
    for (int i = 0; i < primary_control_links.size(); ++i) {
        Affine3d current_pose;
        current_pose.translation() = robot->position(primary_control_links[i], primary_control_points[i]);
        current_pose.linear() = robot->rotation(primary_control_links[i]);
        primary_starting_pose.push_back(current_pose);
    }

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();
	
		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			command_torques = joint_task->computeTorques();

			if ((robot->q() - q_desired).norm() < 1e-2) {
				cout << "Posture To Motion" << endl;
				for (auto name : primary_control_links) {
					primary_tasks[name]->reInitializeTask();
				}
                body_pos = robot->position("body", Vector3d(0, 0, 0));
				joint_task->reInitializeTask();

				state = FEET_FIXED_MOTION;
			}
		} else if (state == FEET_FIXED_MOTION) {
            // get underactuation matrix
            MatrixXd Jr = MatrixXd::Zero(3 * 4, robot->dof());
            for (int i = 0; i < 4; ++i) {
                Jr.block(3 * i, 0, 3, robot->dof()) = robot->Jv(primary_control_links[i], primary_control_points[i]);
            }
            MatrixXd Nr = robot->nullspaceMatrix(Jr);
            MatrixXd UNr = U * Nr;
            MatrixXd UNr_pre_inverse = UNr * robot->MInv() * UNr.transpose();
            MatrixXd UNr_bar = robot->MInv() * UNr.transpose() * \
                                    (UNr_pre_inverse).completeOrthogonalDecomposition().pseudoInverse();

            // update body task
            N_prec.setIdentity();
            body_task->updateTaskModel(N_prec);
            N_prec = body_task->getTaskAndPreviousNullspace();
                
            // redundancy completion 
            joint_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            command_torques.setZero();

            body_task->setGoalPosition(body_pos + Vector3d(0, 0, 0.1 * sin(2 * M_PI * time)));
            command_torques += body_task->computeTorques() + joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector();  // compute joint task torques if DOF isn't filled
            command_torques = U.transpose() * UNr_bar.transpose() * command_torques;  // project underactuation 
            
        } else if (state == FLIGHT) {
            // update primary task model
            N_prec.setIdentity();
            for (auto it = primary_tasks.begin(); it != primary_tasks.end(); ++it) {
                it->second->updateTaskModel(N_prec);
                // N_prec = it->second->getTaskAndPreviousNullspace();
            }
                
            // redundancy completion 
            joint_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            command_torques.setZero();

            int i = 0;
            for (auto name : primary_control_links) {
                primary_tasks[name]->setGoalPosition(primary_starting_pose[i].translation());
                // primary_tasks[name]->setGoalOrientation();
                command_torques += primary_tasks[name]->computeTorques();
                ++i;
            }
            
            command_torques += joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector();  // compute joint task torques if DOF isn't filled
        }

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
