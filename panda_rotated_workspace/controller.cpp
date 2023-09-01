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

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/panda_arm.urdf";

const string FORWARD_KEY = "sai2::forward";
const string BACKWARD_KEY = "sai2::backward";
const string UP_KEY = "sai2::up";
const string DOWN_KEY = "sai2::down";
const string LEFT_KEY = "sai2::left";
const string RIGHT_KEY = "sai2::right";

enum State 
{
	POSTURE = 0, 
	MOTION
};

int main() {

	// initial state 
	int state = POSTURE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.15);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	posori_task->_use_interpolation_flag = true;
	posori_task->_use_velocity_saturation_flag = true;

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 400.0;
	posori_task->_kv_pos = 40.0;
	posori_task->_kp_ori = 400.0;
	posori_task->_kv_ori = 40.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 400.0;
	joint_task->_kv = 40.0;

	VectorXd q_init_desired(dof);
	// q_init_desired << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_init_desired << 0, -15, -15, -80, 0, 90, 0;
	q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;

	// containers
	Vector3d ee_pos;
	Matrix3d ee_rot;
	Matrix3d ee_rot_des;
	// Matrix3d R = Matrix3d::Identity();
	// Matrix3d R = AngleAxisd(15 * M_PI / 180, Vector3d(0, 1, 0)).toRotationMatrix();
	// Matrix3d R = AngleAxisd(30 * M_PI / 180, Vector3d(0, 1, 0)).toRotationMatrix();
	Matrix3d R = AngleAxisd(45 * M_PI / 180, Vector3d(0, 1, 0)).toRotationMatrix();
	ee_rot_des << 0, -1, 0,
					-1, 0, 0,
					0, 0, -1;
// 	ee_rot_des << 0.873895,      -0.25,   0.416903,
//  -0.254887,  -0.965926, -0.0449434,
//   0.413933, -0.0669871,  -0.907839;
	// ee_rot_des = ee_rot_des * R;

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

	int up, down, right, left, forward, backward;
	redis_client.addIntToReadCallback(0, UP_KEY, up);
	redis_client.addIntToReadCallback(0, DOWN_KEY, down);
	redis_client.addIntToReadCallback(0, RIGHT_KEY, right);
	redis_client.addIntToReadCallback(0, LEFT_KEY, left);
	redis_client.addIntToReadCallback(0, FORWARD_KEY, forward);
	redis_client.addIntToReadCallback(0, BACKWARD_KEY, backward);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long counter = 0;

	runloop = true;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// update model
		robot->updateModel();
	
		if (state == POSTURE) {
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques;

			if ( (robot->_q - q_init_desired).norm() < 0.15 ) {
				cout << "Posture To Motion" << endl;
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				robot->position(ee_pos, control_link, control_point);
				robot->rotation(ee_rot, control_link);
				// posori_task->_desired_position = ee_pos - Vector3d(-0.1, -0.1, 0.1);
				// posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
				// posori_task->_desired_orientation = AngleAxisd(0.0000000000000001, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

				posori_task->_desired_orientation = R.transpose() * ee_rot_des;
				std::cout << "Orientation: " << ee_rot << "\n";
				
				state = MOTION;
			}
		} else if (state == MOTION) {

			// update desired position with keyboard readings  
			double step = 0.02;
			if (up) {
				posori_task->_desired_position += R * Vector3d(0, 0, step);
				redis_client.set(UP_KEY, "0");
			} else if (down) {
				posori_task->_desired_position -= R * Vector3d(0, 0, step);
				redis_client.set(DOWN_KEY, "0");
			} else if (left) {
				posori_task->_desired_position -= R * Vector3d(step, 0, 0);
				redis_client.set(LEFT_KEY, "0");
			} else if (right) {
				posori_task->_desired_position += R * Vector3d(step, 0, 0);
				redis_client.set(RIGHT_KEY, "0");
			} else if (forward) {
				posori_task->_desired_position += R * Vector3d(0, step, 0);
				redis_client.set(FORWARD_KEY, "0");
			} else if (backward) {
				posori_task->_desired_position -= R * Vector3d(0, step, 0);
				redis_client.set(BACKWARD_KEY, "0");
			}

			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques;
		}

		// execute redis write callback
		redis_client.executeWriteCallback(0);	

		counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
