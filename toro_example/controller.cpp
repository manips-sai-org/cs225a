// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

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

const string robot_file = "./resources/toro.urdf";	

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1

int state = JOINT_CONTROLLER;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";

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

	// pose task for right foot 
	string control_link = "RL_foot";
	Vector3d control_point = Vector3d(0,0,0);
	auto posori_task_footR = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	posori_task_footR->setDynamicDecouplingFull();

	// #ifdef USING_OTG
	// 	posori_task_footR->_use_interpolation_flag = true;
	// #else
	// 	posori_task_footR->_use_velocity_saturation_flag = true;
	// #endif
	
	VectorXd posori_task_torques_footR = VectorXd::Zero(dof);
	posori_task_footR->_kp_pos = 400.0;
	posori_task_footR->_kv_pos = 20.0;
	posori_task_footR->_kp_ori = 400.0;
	posori_task_footR->_kv_ori = 20.0;

	// set desired position and orientation to the initial configuration
	Vector3d x_pos;
	robot->positionInWorld(x_pos, control_link, control_point);
	Matrix3d x_ori;
	robot->rotationInWorld(x_ori, control_link);
	posori_task_footR->_desired_position = x_pos;
	posori_task_footR->_desired_orientation = x_ori; 

	// pose task for left foot 
	control_link = "LL_foot";
	control_point = Vector3d(0,0,0);
	auto posori_task_footL = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	posori_task_footL->setDynamicDecouplingFull();

	// #ifdef USING_OTG
	// 	posori_task_footL->_use_interpolation_flag = true;
	// #else
	// 	posori_task_footL->_use_velocity_saturation_flag = true;
	// #endif
	
	VectorXd posori_task_torques_footL = VectorXd::Zero(dof);
	posori_task_footL->_kp_pos = 400.0;
	posori_task_footL->_kv_pos = 20.0;
	posori_task_footL->_kp_ori = 400.0;
	posori_task_footL->_kv_ori = 20.0;

	// set desired position and orientation to the initial configuration
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	posori_task_footL->_desired_position = x_pos;
	posori_task_footL->_desired_orientation = x_ori; 

	// pose task for right hand 
	control_link = "ra_link6";
	control_point = Vector3d(0,0,0);
	auto posori_task_handR = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	posori_task_handR->setDynamicDecouplingFull();

	#ifdef USING_OTG
		posori_task_handR->_use_interpolation_flag = true;
	#else
		posori_task_handR->_use_velocity_saturation_flag = true;
	#endif
	
	VectorXd posori_task_torques_handR = VectorXd::Zero(dof);
	posori_task_handR->_kp_pos = 200.0;
	posori_task_handR->_kv_pos = 20.0;
	posori_task_handR->_kp_ori = 200.0;
	posori_task_handR->_kv_ori = 20.0;

	// set two goal positions/orientations 
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	posori_task_handR->_desired_position = x_pos + Vector3d(0.5, -0.2, 0.8);
	posori_task_handR->_desired_orientation = AngleAxisd(M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori; 
	// posori_task_handR->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori; 

	// pose task for left hand
	control_link = "la_link6";
	control_point = Vector3d(0,0,0);
	auto posori_task_handL = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	posori_task_handL->setDynamicDecouplingFull();

	#ifdef USING_OTG
		posori_task_handL->_use_interpolation_flag = true;
	#else
		posori_task_handL->_use_velocity_saturation_flag = true;
	#endif
	
	VectorXd posori_task_torques_handL = VectorXd::Zero(dof);
	posori_task_handL->_kp_pos = 200.0;
	posori_task_handL->_kv_pos = 20.0;
	posori_task_handL->_kp_ori = 200.0;
	posori_task_handL->_kv_ori = 20.0;

	// set two goal positions/orientations 
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	posori_task_handL->_desired_position = x_pos + Vector3d(0.5, 0.2, 0.8);
	posori_task_handL->_desired_orientation = AngleAxisd(-M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori; 
	// posori_task_handR->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori; 

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->setDynamicDecouplingFull();

	#ifdef USING_OTG
		joint_task->_use_interpolation_flag = true;
	#else
		joint_task->_use_velocity_saturation_flag = true;
	#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100.0;
	joint_task->_kv = 20.0;

	// set desired joint posture to be the initial robot configuration
	VectorXd q_init_desired = robot->_q;
	joint_task->_desired_position = q_init_desired;

	// gravity vector
	VectorXd g(dof);

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

		// update model
		robot->updateModel();
		
		// calculate torques to fix the feet 
		N_prec.setIdentity();
		posori_task_footR->updateTaskModel(N_prec);
		posori_task_footR->computeTorques(posori_task_torques_footR);

		N_prec = posori_task_footR->_N;
		posori_task_footL->updateTaskModel(N_prec);
		posori_task_footL->computeTorques(posori_task_torques_footL);

		// calculate torques to move right hand
		N_prec = posori_task_footL->_N;
		posori_task_handR->updateTaskModel(N_prec);
		posori_task_handR->computeTorques(posori_task_torques_handR);

		// calculate torques to move left hand
		N_prec = posori_task_handR->_N;
		posori_task_handL->updateTaskModel(N_prec);
		posori_task_handL->computeTorques(posori_task_torques_handL);

		// calculate torques to maintain joint posture
		N_prec = posori_task_handL->_N;
		joint_task->updateTaskModel(N_prec);
		joint_task->computeTorques(joint_task_torques);

		// calculate gravity torques (if needed)
		robot->gravityVector(g);

		// calculate torques 
		command_torques = posori_task_torques_footR + posori_task_torques_footL + \
							posori_task_torques_handR + posori_task_torques_handL + joint_task_torques;  // gravity compensation handled in sim

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
