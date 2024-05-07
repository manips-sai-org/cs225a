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
const string robot_file = "./resources/toro.urdf";

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

	// create maps for tasks
    std::map<std::string, std::shared_ptr<Sai2Primitives::MotionForceTask>> primary_tasks;
    std::map<std::string, std::shared_ptr<Sai2Primitives::MotionForceTask>> secondary_tasks;

    const std::vector<std::string> primary_control_links = {"trunk", "la_link6", "ra_link6", "LL_foot", "RL_foot"};
    const std::vector<Vector3d> primary_control_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)};
    // const std::vector<std::string> secondary_control_links = {"neck_link2", "la_link4", "ra_link4", "LL_KOSY_L56", "RL_KOSY_L56"};
    // const std::vector<Vector3d> secondary_control_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)};
    const std::vector<std::string> secondary_control_links = {"neck_link2"};
    const std::vector<Vector3d> secondary_control_points = {Vector3d(0, 0, 0)};

    for (int i = 0; i < primary_control_links.size(); ++i) {        
        Affine3d compliant_frame = Affine3d::Identity();
        compliant_frame.translation() = primary_control_points[i];
        primary_tasks[primary_control_links[i]] = std::make_shared<Sai2Primitives::MotionForceTask>(robot, primary_control_links[i], compliant_frame);
        primary_tasks[primary_control_links[i]]->disableInternalOtg();
        primary_tasks[primary_control_links[i]]->setDynamicDecouplingType(Sai2Primitives::MotionForceTask::FULL_DYNAMIC_DECOUPLING);
        primary_tasks[primary_control_links[i]]->setPosControlGains(400, 40, 0);
        primary_tasks[primary_control_links[i]]->setOriControlGains(400, 40, 0);
    }

	for (int i = 0; i < secondary_control_links.size(); ++i) {        
        Affine3d compliant_frame = Affine3d::Identity();
        compliant_frame.translation() = secondary_control_points[i];
        secondary_tasks[secondary_control_links[i]] = std::make_shared<Sai2Primitives::MotionForceTask>(robot, secondary_control_links[i], compliant_frame);
        secondary_tasks[secondary_control_links[i]]->disableInternalOtg();
        // secondary_tasks[secondary_control_links[i]]->enableJointHandling(false);  // don't perform singularity handling in second-level tasks 
        secondary_tasks[secondary_control_links[i]]->setDynamicDecouplingType(Sai2Primitives::MotionForceTask::FULL_DYNAMIC_DECOUPLING);
        secondary_tasks[secondary_control_links[i]]->setPosControlGains(400, 40, 0);
        secondary_tasks[secondary_control_links[i]]->setOriControlGains(400, 40, 0);
    }

	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->disableInternalOtg();
	VectorXd q_desired = robot->q();
	joint_task->setGains(400, 40, 0);
	joint_task->setGoalPosition(q_desired);

	// get starting poses
    std::vector<Affine3d> primary_starting_pose;
    std::vector<Affine3d> secondary_starting_pose;
    for (int i = 0; i < primary_control_links.size(); ++i) {
        Affine3d current_pose;
        current_pose.translation() = robot->position(primary_control_links[i], primary_control_points[i]);
        current_pose.linear() = robot->rotation(primary_control_links[i]);
        primary_starting_pose.push_back(current_pose);
    }
    for (int i = 0; i < secondary_control_links.size(); ++i) {
        Affine3d current_pose;
        current_pose.translation() = robot->position(secondary_control_links[i], secondary_control_points[i]);
        current_pose.linear() = robot->rotation(secondary_control_links[i]);
        secondary_starting_pose.push_back(current_pose);
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
				for (auto name : secondary_control_links) {
					secondary_tasks[name]->reInitializeTask();
				}
				joint_task->reInitializeTask();

				state = MOTION;
			}
		} else if (state == MOTION) {
            // update primary task model
            N_prec.setIdentity();
            for (auto it = primary_tasks.begin(); it != primary_tasks.end(); ++it) {
                it->second->updateTaskModel(N_prec);
                // N_prec = it->second->getTaskAndPreviousNullspace();
            }

            // get primary task jacobian stack 
            MatrixXd J_primary_tasks(6 * primary_control_links.size(), robot->dof());
            for (int i = 0; i < primary_control_links.size(); ++i) {
                J_primary_tasks.block(6 * i, 0, 6, robot->dof()) = robot->J(primary_control_links[i], primary_control_points[i]);
            }        
            N_prec = robot->nullspaceMatrix(J_primary_tasks);

            // update secondary task model
            for (auto it = secondary_tasks.begin(); it != secondary_tasks.end(); ++it) {
                it->second->updateTaskModel(N_prec);
                // N_prec = it->second->getTaskAndPreviousNullspace();  // 
            }

            // get secondary task jacobian stack 
            MatrixXd J_secondary_tasks(6 * secondary_control_links.size(), dof);
            for (int i = 0; i < secondary_control_links.size(); ++i) {
                J_secondary_tasks.block(6 * i, 0, 6, dof) = robot->J(secondary_control_links[i], secondary_control_points[i]);
            }
            N_prec = robot->nullspaceMatrix(J_secondary_tasks) * N_prec;
                
            // redundancy completion
            joint_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            command_torques.setZero();

            int i = 0;
            for (auto name : primary_control_links) {
                primary_tasks[name]->setGoalPosition(primary_starting_pose[i].translation() + generateRandomVector(-0.05, 0.05, 3));
                // primary_tasks[name]->setGoalOrientation();
                command_torques += primary_tasks[name]->computeTorques();
                ++i;
            }
            
            i = 0;
            for (auto name : secondary_control_links) {
                secondary_tasks[name]->setGoalPosition(secondary_starting_pose[i].translation() + generateRandomVector(-0.05, 0.05, 3));
                // secondary_tasks[name]->setGoalOrientation();
                command_torques += secondary_tasks[name]->computeTorques();
                ++i;
            }

            command_torques += joint_task->computeTorques() + robot->coriolisForce();
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
