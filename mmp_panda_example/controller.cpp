/**
 * @file controller.cpp
 * @brief Controller file 
 */

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/mmp_panda.urdf";

#include "redis_keys.h"

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0,0,0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	posori_task->_use_interpolation_flag = true;
	posori_task->_otg->setMaxLinearVelocity(0.8);
	
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 400;
	posori_task->_kv_pos = 40;
	posori_task->_kp_ori = 400;
	posori_task->_kv_ori = 40;

	// set the current EE posiiton as the desired EE position
	Vector3d x_desired = Vector3d::Zero(3);
	robot->position(x_desired, control_link, control_point);
	Vector3d x_init = x_desired;

	// partial joint task to control the mobile base 
	vector<int> base_joint_selection{0, 1, 2};
	auto base_task = new Sai2Primitives::PartialJointTask(robot, base_joint_selection);
	base_task->_use_interpolation_flag = false;  // turn off if trajectory following; else turn on
	base_task->_use_velocity_saturation_flag = true;
	base_task->_saturation_velocity << 0.2, 0.2, 0.3;  // adjust based on speed
	
	VectorXd base_task_torques = VectorXd::Zero(dof);
	base_task->_kp = 400;
	base_task->_kv = 40;
	
	VectorXd base_pose_desired = initial_q.head(3);
	VectorXd base_pose_init = base_pose_desired;

	// joint (posture) task
	vector<int> arm_joint_selection{3, 4, 5, 6, 7, 8, 9};
	auto arm_joint_task = new Sai2Primitives::PartialJointTask(robot, arm_joint_selection);
	arm_joint_task->_use_interpolation_flag = false;

	VectorXd arm_joint_task_torques = VectorXd::Zero(dof);
	arm_joint_task->_kp = 100;
	arm_joint_task->_kv = 20;

	// set the desired posture
	VectorXd q_desired = initial_q.tail(7);
	q_desired << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_desired *= M_PI/180.0;
	arm_joint_task->_desired_position = q_desired;
	
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// sample desired set points
		x_desired = x_init + Vector3d(0.2 * sin(time), 0.2 * sin(time), 0 * sin(time));
		x_desired(0) += base_pose_desired(0);  // need to update the "base" position of the robot arm 
		x_desired(1) += base_pose_desired(1);
		base_pose_desired = base_pose_init + Vector3d(0.2 * sin(time), 0.2 * sin(time), 0.2 * sin(time));  // if following trajectory (take care of conflicting motions)
		// base_pose_desired = base_pose_init;  // if holding 

		// set controller inputs
		posori_task->_desired_position = x_desired;
		base_task->_desired_position = base_pose_desired;
		arm_joint_task->_desired_position = q_desired;

		// update task model and set hierarchy
		/*
			arm-driven motion hieararchy: arm -> base -> arm nullspace
		*/ 
		N_prec.setIdentity();
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;	
		base_task->updateTaskModel(N_prec);
		N_prec = base_task->_N;	
		arm_joint_task->updateTaskModel(N_prec);

		// /*
		// 	base-driven motion hieararchy: base -> arm -> arm nullspace 
		// */
		// N_prec.setIdentity();
		// base_task->updateTaskModel(N_prec);
		// N_prec = base_task->_N;	
		// posori_task->updateTaskModel(N_prec);
		// N_prec = posori_task->_N;	
		// arm_joint_task->updateTaskModel(N_prec);

		// compute torques
		posori_task->computeTorques(posori_task_torques);
		base_task->computeTorques(base_task_torques);
		arm_joint_task->computeTorques(arm_joint_task_torques);

		command_torques = posori_task_torques + base_task_torques + arm_joint_task_torques;

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);

	return 0;
}
