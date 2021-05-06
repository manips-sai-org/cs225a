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

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
std::string DESIRED_EE_POS_KEY;
std::string EE_POS_KP_KV_KEY;
std::string DESIRED_JOINT_ANGLES_KEY;
std::string JOINT_ANGLES_KP_KV_KEY;
std::string MOBILE_BASE_FOLLOW_KEY;
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
	DESIRED_EE_POS_KEY = "sai2::cs225a::project::inputs::x_des";
	EE_POS_KP_KV_KEY = "sai2::cs225a::project::inputs::kp_kv_x";
	DESIRED_JOINT_ANGLES_KEY = "sai2::cs225a::project::inputs::q_des";
	JOINT_ANGLES_KP_KV_KEY = "sai2::cs225a::project::inputs::kp_kv_q";
	MOBILE_BASE_FOLLOW_KEY = "sai2::cs225a::project::inputs::mobile_base_follow";

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

#ifdef USING_OTG
	posori_task->_use_interpolation_flag = true;
	posori_task->_otg->setMaxLinearVelocity(0.8);
#else
	posori_task->_use_velocity_saturation_flag = true;
#endif
	
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	Vector2d kp_kv_x = Vector2d(200.0, 20.0);
	posori_task->_kp_pos = kp_kv_x(0);
	posori_task->_kv_pos = kp_kv_x(1);
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// set the current EE posiiton as the desired EE position
	Vector3d x_desired = Vector3d::Zero(3);
	robot->position(x_desired, control_link, control_point);

	// joint (posture) task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = true;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	Vector2d kp_kv_q = Vector2d(50.0, 12.0);
	joint_task->_kp = kp_kv_q(0);
	joint_task->_kv = kp_kv_q(1);

	// set the desired posture
	VectorXd q_desired = initial_q;
	q_desired << 0.0, 0.0, 0.0, -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_desired *= M_PI/180.0;
	joint_task->_desired_position = q_desired;
	bool mobile_base_follow = false;

	// initialize redis inputs
	redis_client.setEigenMatrixJSON(DESIRED_EE_POS_KEY, x_desired);
	redis_client.setEigenMatrixJSON(EE_POS_KP_KV_KEY, kp_kv_x);
	redis_client.setEigenMatrixJSON(DESIRED_JOINT_ANGLES_KEY, q_desired);
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KP_KV_KEY, kp_kv_q);
	redis_client.set(MOBILE_BASE_FOLLOW_KEY, std::to_string(mobile_base_follow));

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

		// read controller inputs from redis
		x_desired = redis_client.getEigenMatrixJSON(DESIRED_EE_POS_KEY);
		kp_kv_x = redis_client.getEigenMatrixJSON(EE_POS_KP_KV_KEY);
		q_desired = redis_client.getEigenMatrixJSON(DESIRED_JOINT_ANGLES_KEY);
		kp_kv_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KP_KV_KEY);
		istringstream(redis_client.get(MOBILE_BASE_FOLLOW_KEY)) >> mobile_base_follow;

		// check for mobile base following
		if(mobile_base_follow)
		{
			q_desired(0) = x_desired(0) - 0.3;
			q_desired(1) = x_desired(1) - 0.3;
		}

		// set controller inputs
		posori_task->_desired_position = x_desired;
		posori_task->_kp_pos = kp_kv_x(0);
		posori_task->_kv_pos = kp_kv_x(1);
		joint_task->_desired_position = q_desired;
		joint_task->_kp = kp_kv_q(0);
		joint_task->_kv = kp_kv_q(1);

		// update model
		robot->updateModel();

		// update task model and set hierarchy
		N_prec.setIdentity();
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;
		joint_task->updateTaskModel(N_prec);

		// compute torques
		posori_task->computeTorques(posori_task_torques);
		joint_task->computeTorques(joint_task_torques);

		command_torques = posori_task_torques + joint_task_torques;

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
