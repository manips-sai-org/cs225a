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

// States 
enum State {
	POSTURE = 0, 
	MOTION
};

Eigen::VectorXd generateRandomVector(double lowerBound, double upperBound, int size) {
    // Initialize a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(lowerBound, upperBound);

    // Generate random vector
    Eigen::VectorXd randomVec(size);
    for (int i = 0; i < size; ++i) {
        randomVec(i) = dis(gen);
    }

    return randomVec;
}

int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/ocean1/ocean1.urdf";

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
	VectorXd command_torques = VectorXd::Zero(dof); 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// create map for arm pose tasks
    std::map<std::string, std::shared_ptr<Sai2Primitives::MotionForceTask>> pose_tasks;

    const std::vector<std::string> control_links = {"endEffector_left", "endEffector_right"};
    const std::vector<Vector3d> control_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0)};

    for (int i = 0; i < control_links.size(); ++i) {        
        Affine3d compliant_frame = Affine3d::Identity();
        compliant_frame.translation() = control_points[i];
        pose_tasks[control_links[i]] = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_links[i], compliant_frame);
        pose_tasks[control_links[i]]->disableInternalOtg();
        pose_tasks[control_links[i]]->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
        pose_tasks[control_links[i]]->setPosControlGains(400, 40, 0);
        pose_tasks[control_links[i]]->setOriControlGains(400, 40, 0);
    }

    // base partial joint task 
    int num_base_joints = 6;
	MatrixXd base_selection_matrix = MatrixXd::Zero(num_base_joints, robot->dof());
	base_selection_matrix.block(0, 0, num_base_joints, num_base_joints).setIdentity();
    cout << base_selection_matrix << endl;
	auto base_task = std::make_shared<Sai2Primitives::JointTask>(robot, base_selection_matrix);
	base_task->setGains(400, 40, 0);

    // posture task
    auto posture_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	posture_task->disableInternalOtg();
	VectorXd q_desired = robot->q();
	posture_task->setGains(400, 40, 0);
	posture_task->setGoalPosition(q_desired);

	// get starting poses
    std::vector<Affine3d> starting_pose;
    for (int i = 0; i < control_links.size(); ++i) {
        Affine3d current_pose;
        current_pose.translation() = robot->position(control_links[i], control_points[i]);
        current_pose.linear() = robot->rotation(control_links[i]);
        starting_pose.push_back(current_pose);
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
			posture_task->updateTaskModel(N_prec);

			command_torques = posture_task->computeTorques();

			if ((robot->q() - q_desired).norm() < 1e-2) {
				cout << "Posture To Motion" << endl;
				for (auto name : control_links) {
					pose_tasks[name]->reInitializeTask();
				}
				posture_task->reInitializeTask();

				state = MOTION;
			}
		} else if (state == MOTION) {
            // update body task model
            N_prec.setIdentity();
            base_task->updateTaskModel(N_prec);
            N_prec = base_task->getTaskAndPreviousNullspace();

            // update pose task models
            for (auto it = pose_tasks.begin(); it != pose_tasks.end(); ++it) {
                it->second->updateTaskModel(N_prec);
                // N_prec = it->second->getTaskAndPreviousNullspace();
            }

            // get pose task Jacobian stack 
            MatrixXd J_pose_tasks(6 * control_links.size(), robot->dof());
            for (int i = 0; i < control_links.size(); ++i) {
                J_pose_tasks.block(6 * i, 0, 6, robot->dof()) = robot->J(control_links[i], control_points[i]);
            }        
            N_prec = robot->nullspaceMatrix(J_pose_tasks);
                
            // redundancy completion
            posture_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            command_torques.setZero();

            // base task
            command_torques += base_task->computeTorques();

            // pose tasks
            int i = 0;
            for (auto name : control_links) {
                pose_tasks[name]->setGoalPosition(starting_pose[i].translation() + generateRandomVector(-0.05, 0.05, 3));
                // pose_tasks[name]->setGoalOrientation();
                command_torques += pose_tasks[name]->computeTorques();
                ++i;
            }
            
            // posture task and coriolis compensation
            command_torques += posture_task->computeTorques() + robot->coriolisForce();
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
