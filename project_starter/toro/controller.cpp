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

int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/toro/toro.urdf";

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

    // trunk task (first 6 joints)
    auto trunk_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, "hip_base", Affine3d::Identity());
    trunk_task->disableInternalOtg();
    trunk_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
    trunk_task->setPosControlGains(400, 40, 0);
    trunk_task->setOriControlGains(400, 40, 0);

	// create maps for tasks
    std::map<std::string, std::shared_ptr<Sai2Primitives::MotionForceTask>> primary_tasks;
    std::map<std::string, std::shared_ptr<Sai2Primitives::MotionForceTask>> secondary_tasks;

    const std::vector<std::string> primary_control_links = {"hip_base", "la_end_effector", "ra_end_effector", "LL_end_effector", "RL_end_effector"};
    const std::vector<Vector3d> primary_control_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)};
    const std::vector<std::string> secondary_control_links = {"neck_link2", "la_link4", "ra_link4", "LL_KOSY_L56", "RL_KOSY_L56"};
    const std::vector<Vector3d> secondary_control_points = {Vector3d(0, 0, 0.11), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)};
    // const std::vector<std::string> secondary_control_links = {"neck_link2"};
    // const std::vector<Vector3d> secondary_control_points = {Vector3d(0, 0, 0.11)};

    for (int i = 0; i < primary_control_links.size(); ++i) {        
        Affine3d compliant_frame = Affine3d::Identity();
        compliant_frame.translation() = primary_control_points[i];
        primary_tasks[primary_control_links[i]] = std::make_shared<Sai2Primitives::MotionForceTask>(robot, primary_control_links[i], compliant_frame);
        primary_tasks[primary_control_links[i]]->disableInternalOtg();
        primary_tasks[primary_control_links[i]]->setDynamicDecouplingType(Sai2Primitives::DynamicDecouplingType::FULL_DYNAMIC_DECOUPLING);
        primary_tasks[primary_control_links[i]]->setPosControlGains(400, 40, 0);
        primary_tasks[primary_control_links[i]]->setOriControlGains(400, 40, 0);
        primary_tasks[primary_control_links[i]]->handleAllSingularitiesAsType1(true);
    }

	for (int i = 0; i < secondary_control_links.size(); ++i) {        
        Affine3d compliant_frame = Affine3d::Identity();
        compliant_frame.translation() = secondary_control_points[i];
        secondary_tasks[secondary_control_links[i]] = std::make_shared<Sai2Primitives::MotionForceTask>(robot, secondary_control_links[i], compliant_frame);
        secondary_tasks[secondary_control_links[i]]->disableInternalOtg();
        secondary_tasks[secondary_control_links[i]]->disableSingularityHandling();  // don't perform singularity handling in second-level tasks 
        secondary_tasks[secondary_control_links[i]]->setDynamicDecouplingType(Sai2Primitives::DynamicDecouplingType::FULL_DYNAMIC_DECOUPLING);
        secondary_tasks[secondary_control_links[i]]->setPosControlGains(400, 40, 0);
        secondary_tasks[secondary_control_links[i]]->setOriControlGains(400, 40, 0);
    }

	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->disableInternalOtg();
	VectorXd q_desired = robot->q();
	joint_task->setGains(400, 40, 0);
    joint_task->setDynamicDecouplingType(Sai2Primitives::DynamicDecouplingType::FULL_DYNAMIC_DECOUPLING);
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
            // update trunk model
            N_prec.setIdentity();

            // update primary task model
            for (auto it = primary_tasks.begin(); it != primary_tasks.end(); ++it) {
                it->second->updateTaskModel(N_prec);
                // N_prec = it->second->getTaskAndPreviousNullspace();
            }

            // get primary task jacobian stack 
            MatrixXd J_primary_tasks(6 * primary_control_links.size(), robot->dof());
            for (int i = 0; i < primary_control_links.size(); ++i) {
                J_primary_tasks.block(6 * i, 0, 6, robot->dof()) = robot->J(primary_control_links[i], primary_control_points[i]);
            }        
            N_prec = robot->nullspaceMatrix(J_primary_tasks) * N_prec;

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
                if (i != 0) {
                    // primary_tasks[name]->setGoalPosition(primary_starting_pose[i].translation() + 0.05 * Vector3d(sin(2 * M_PI * time), 0, sin(2 * M_PI * time)));
                    primary_tasks[name]->setGoalPosition(primary_starting_pose[i].translation() + \
                            1 * Vector3d(0.5 * sin(2 * M_PI * 0.1 * time), 0.5 * sin(2 * M_PI * 0.1 * time), 0.5 * sin(2 * M_PI * 0.1 * time)));
                    // primary_tasks[name]->setGoalOrientation();
                }
                command_torques += primary_tasks[name]->computeTorques();
                ++i;
            }
            
            i = 0;
            for (auto name : secondary_control_links) {
                // secondary_tasks[name]->setGoalPosition();
                // secondary_tasks[name]->setGoalOrientation();
                command_torques += secondary_tasks[name]->computeTorques();
                ++i;
            }

            command_torques += 0 * joint_task->computeTorques() + robot->coriolisForce();  // compute joint task torques if DOF isn't filled
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
