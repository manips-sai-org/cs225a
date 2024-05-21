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
	VectorXd command_torques = VectorXd::Zero(dof);  // panda 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	const string control_link = "end-effector";
	const Vector3d control_point = Vector3d(0, 0, -0.5);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setPosControlGains(400, 40, 0);
	pose_task->setOriControlGains(400, 40, 0);

	Vector3d ee_pos;
	Matrix3d ee_ori;

	// gripper partial joint task 
	MatrixXd bat_selection_matrix = MatrixXd::Zero(2, robot->dof());
	bat_selection_matrix(0, dof -2) = 1;
	bat_selection_matrix(1, dof - 1) = 1;

	//auto gripper_task = std::make_shared<Sai2Primitives::JointTask>(robot, bat_selection_matrix);
	//gripper_task->setDynamicDecouplingType(Sai2Primitives::DynamicDecouplingType::IMPEDANCE);
	//double kp_gripper = 5e3;
	//double kv_gripper = 1e2;
	//gripper_task->setGains(kp_gripper, kv_gripper, 0);
	//gripper_task->setGains(kp_gripper, kv_gripper, 0);

	// joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->setGains(400, 40, 2);

	VectorXd q_desired(dof);
	q_desired.head(7) << 0, 0, 0, 0, 0, 0, 90;
	q_desired.head(7) *= M_PI / 180.0;
	//cout << q_desired.transpose() << endl;;
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
			joint_task->updateTaskModel(N_prec);

			//cout << "joint_task " << joint_task.transpose() << endl;
			command_torques = joint_task->computeTorques();
			//cout << "command_torques " << command_torques.transpose() << endl;
			//cout << (robot->q() - q_desired).transpose() << endl;
			if ((robot->q() - q_desired).norm() < 1e-1) {
				cout << "Posture To Motion" << endl;
				pose_task->reInitializeTask();
				//gripper_task->reInitializeTask();
				joint_task->reInitializeTask();

				ee_pos = robot->position(control_link, control_point);
				cout << ee_pos << endl;
				ee_ori = robot->rotation(control_link);

				pose_task->setGoalPosition(ee_pos + Vector3d(.5, .5, 0));
				pose_task->setGoalOrientation(AngleAxisd(M_PI / 2, Vector3d::UnitZ()).toRotationMatrix() * ee_ori);
				//gripper_task->setGoalPosition(Vector2d(0.02, -0.02));

				state = MOTION;
			}
		} else if (state == MOTION) {
			// update goal position and orientation

			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			//gripper_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();// + gripper_task->computeTorques() +;
			//cout << (robot->position(control_link, control_point) -(ee_pos + Vector3d(.5, .5, 0))).norm() << endl;
			if ((robot->position(control_link, control_point) - (ee_pos + Vector3d(.5, .5, 0))).norm() < 1e-1){
				//cout << "reached new pos" << endl;
			}
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

