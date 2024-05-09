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
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/mmp_panda/mmp_panda.urdf";

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

	// arm task 
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.07);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setPosControlGains(400, 40, 0);
	pose_task->setOriControlGains(400, 40, 0);

	Vector3d ee_pos = robot->position(control_link, control_point);
	Matrix3d ee_ori = robot->rotation(control_link);

	// base partial joint task 
	MatrixXd base_selection_matrix = MatrixXd::Zero(3, robot->dof());
	base_selection_matrix(0, 0) = 1;
	base_selection_matrix(1, 1) = 1;
	base_selection_matrix(2, 2) = 1;
	auto base_task = std::make_shared<Sai2Primitives::JointTask>(robot, base_selection_matrix);
	base_task->setGains(400, 40, 0);

	Vector3d base_pose = robot->q().head(3);

	// joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->setGains(400, 40, 0);

	VectorXd q_desired(dof);
	q_desired.tail(7) << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_desired.tail(7) *= M_PI / 180.0;
	q_desired.head(3) << 0, 0, 0;
	joint_task->setGoalPosition(q_desired);

	bool arm_driven_control = true;
	// bool arm_driven_control = false;
	if (arm_driven_control) {
		base_task->setGains(0, 40, 0);
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
				pose_task->reInitializeTask();
				base_task->reInitializeTask();
				joint_task->reInitializeTask();

				ee_pos = robot->position(control_link, control_point);
				ee_ori = robot->rotation(control_link);
				base_pose = robot->q().head(3);

				if (arm_driven_control) {
					pose_task->setGoalPosition(ee_pos + Vector3d(0.8, 0.8, 0));
					pose_task->setGoalOrientation(AngleAxisd(M_PI / 6, Vector3d::UnitX()).toRotationMatrix() * ee_ori);
				} else {
					base_task->setGoalPosition(base_pose + Vector3d(0.2, 0.2, 0.5));
				}

				state = MOTION;
			}
		} else if (state == MOTION) {
			// update goal positions and orientations of base and arm 

			// update task model for arm-driven motion
			if (arm_driven_control) {
				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				base_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
				joint_task->updateTaskModel(base_task->getTaskAndPreviousNullspace());
			} else {
				// update task model for base -> arm motion 
				N_prec.setIdentity();
				base_task->updateTaskModel(N_prec);
				pose_task->updateTaskModel(base_task->getTaskAndPreviousNullspace());
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
			}

			command_torques = pose_task->computeTorques() + base_task->computeTorques() + joint_task->computeTorques();
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
