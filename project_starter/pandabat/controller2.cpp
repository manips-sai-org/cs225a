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
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_bat.urdf";
	
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

	// arm task
	const string control_link = "end-effector";
	const Vector3d control_point = Vector3d(0, 0, 0.5); // control point is at the end of the end-effector
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setPosControlGains(400, 40, 0);
	pose_task->setOriControlGains(400, 40, 0); 

	// Desired end-effector position
	Vector3d desired_position = Vector3d(0.5, 0, 1.0);

	// Desired end-effector orientation
	Vector3d desired_direction = Vector3d(0, -1.0, 0).normalized();
	Vector3d current_z_axis = Vector3d(0.0, 0.0, 1.0); // World Frame, Link 0 z axis
	Quaterniond rotation_quat = Quaterniond::FromTwoVectors(current_z_axis, desired_direction);
	Matrix3d desired_orientation = rotation_quat.toRotationMatrix();
	
	// joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->setGains(400, 40, 2);
	VectorXd q_desired(dof);
	q_desired.head(7) << 0, 0, 0, 0, 0, 0, 0;
	q_desired.head(7) *= M_PI / 180.0;
	joint_task->setGoalPosition(q_desired);

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
			pose_task->updateTaskModel(N_prec);

			// Set goal position and orientation
			pose_task->setGoalPosition(desired_position);
			pose_task->setGoalOrientation(desired_orientation);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			// Check if the end-effector is close enough to the desired position
			if ((robot->position(control_link, control_point) - desired_position).norm() < 1e-2 &&
			    (robot->rotation(control_link) - desired_orientation).norm() < 1e-2) {
				cout << "Posture To Motion" << endl;
				state = MOTION;
			}
		} else if (state == MOTION) {
			// update goal position and orientation
			desired_position = Vector3d(1, 0, 1.0);
			pose_task->setGoalPosition(desired_position);
			pose_task->setGoalOrientation(desired_orientation);
			desired_direction = Vector3d(1.0, 0, 0).normalized();

			// current_z_axis = Vector3d(0.0, 0.0, 1.0); // World Frame, Link 0 z axis
			rotation_quat = Quaterniond::FromTwoVectors(current_z_axis, desired_direction);
			Matrix3d desired_orientation = rotation_quat.toRotationMatrix();

			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);

			command_torques = pose_task->computeTorques();
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
