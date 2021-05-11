#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

// definitions for terminal input parsing 
#define QUESTION_1A   11
#define QUESTION_1B   12
#define QUESTION_2A   21
#define QUESTION_2B   22
#define QUESTION_2C   23
#define QUESTION_2D   24
#define QUESTION_4A   41
#define QUESTION_4B   42

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

// helper function 
double sat(double x) {
	if (abs(x) <= 1.0) {
		return x;
	}
	else {
		return signbit(x);
	}
}

int main(int argc, char* argv[]) {

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
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = atoi(argv[1]);  // read terminal input 

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1A)
		{
			// part 1
			VectorXd F(3), g(dof), joint_task_torque(dof);
			Vector3d x, xd, x_vel, p;

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(x_vel, link_name, pos_in_link);
			robot->gravityVector(g);

			xd << 0.3 + 0.1*sin(M_PI * time), 0.1 + 0.1*cos(M_PI * time), 0.5;
			double kp = 100;
			double kv = 20;
			double kpj = 50;
			double kvj = 14;
			auto qd = robot->_q;
			qd.setZero();
			joint_task_torque = N.transpose()*( - (kpj * (robot->_q - qd)) - kvj * robot->_dq);

			F = Lambda * (- kp * (x - xd) - kv * x_vel);
			command_torques = Jv.transpose()*F + joint_task_torque + g;
		}
		else if(controller_number == QUESTION_1B)
		{
			// part 2
			VectorXd F(3), g(dof), b(dof), joint_task_torque(dof);
			Vector3d x, xd, x_vel, p;
			Vector3d dxd, ddxd;

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(x_vel, link_name, pos_in_link);
			robot->gravityVector(g);

			xd << 0.3 + 0.1*sin(M_PI * time), 0.1 + 0.1*cos(M_PI * time), 0.5;
			dxd << 0.1 * M_PI * cos(M_PI * time), - 0.1 * M_PI * sin(M_PI * time), 0;
			ddxd << - 0.1 * pow(M_PI,2) * sin(M_PI * time), - 0.1 * pow(M_PI,2) * cos(M_PI * time), 0;
			double kp = 100;
			double kv = 20;
			double kpj = 50;
			double kvj = 14;
			auto qd = robot->_q;
			qd.setZero();
			joint_task_torque = N.transpose() * (- kpj * (robot->_q - qd) - kvj * robot->_dq);

			F = Lambda * (- kp * (x - xd) - kv * (x_vel - dxd) + ddxd );
			command_torques = Jv.transpose() * F + joint_task_torque + g;
		}

		// ---------------------------  question 2 ---------------------------------------
		else if(controller_number == QUESTION_2A)
		{
			// part 1
			VectorXd F(3), g(dof), Gamma_damp(dof);
			Vector3d x, xd, x_vel;

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(x_vel, link_name, pos_in_link);
			robot->gravityVector(g);

			xd << -0.1, 0.15, 0.2;
			double kp = 100;
			double kv = 20;
			double kdamp = 7;

			Gamma_damp = - (N.transpose() * (kdamp * robot->_dq));
			F = Lambda * ( - kp * (x - xd) - kv * x_vel);
			command_torques = Jv.transpose() * F + g + Gamma_damp;
		}
		else if(controller_number == QUESTION_2B)
		{
			// part 2
			VectorXd F(3), g(dof), Gamma_damp(dof), Gamma_mid(dof);
			Vector3d x, xd, x_vel;
			VectorXd q_high(dof), q_low(dof);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(x_vel, link_name, pos_in_link);
			robot->gravityVector(g);
			
			q_low << RAD(-165), RAD(-100), RAD(-165), RAD(-170), RAD(-165), RAD(0), RAD(-165);
			q_high << RAD(165), RAD(100), RAD(165), RAD(-30), RAD(165), RAD(210), RAD(165);
			xd << -0.1, 0.15, 0.2;
			double kp = 100;
			double kv = 20;
			double kdamp = 7;
			double kmid = 25;

			Gamma_mid = - (kmid * (2 * robot->_q - (q_high + q_low)));
			Gamma_damp = - (kdamp * robot->_dq);
			F = Lambda * (- kp * (x - xd) - kv * x_vel);
			command_torques = Jv.transpose() * F + g + N.transpose() * Gamma_damp + N.transpose() * Gamma_mid;
		}
		else if(controller_number == QUESTION_2C)
		{
			// part 3
			VectorXd F(3), g(dof), Gamma_damp(dof), Gamma_mid(dof);
			Vector3d x, xd, x_vel;
			VectorXd q_high(dof), q_low(dof);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(x_vel, link_name, pos_in_link);
			robot->gravityVector(g);

			q_high << RAD(165), RAD(100), RAD(165), RAD(-30), RAD(165), RAD(210), RAD(165);
			q_low << RAD(-165), RAD(-100), RAD(-165), RAD(-170), RAD(-165), RAD(0), RAD(-165);
			xd << -0.65, -0.45, 0.7;
			double kp = 100;
			double kv = 20;
			double kdamp = 7;
			double kmid = 25;

			Gamma_mid = - (kmid * (2 * robot->_q - (q_high + q_low)));
			Gamma_damp = - (kdamp * robot->_dq);
			F = Lambda * (- kp * (x - xd) - kv * x_vel);
			command_torques = Jv.transpose() * F + g + N.transpose() * Gamma_damp + N.transpose() * Gamma_mid;
		}
		else if (controller_number == QUESTION_2D)
		{
			// part 4
			VectorXd F(3), g(dof), Gamma_damp(dof), Gamma_mid(dof);
			Vector3d x, xd, x_vel;
			VectorXd q_high(dof), q_low(dof);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(x_vel, link_name, pos_in_link);
			robot->gravityVector(g);

			q_high << RAD(165), RAD(100), RAD(165), RAD(-30), RAD(165), RAD(210), RAD(165);
			q_low << RAD(-165), RAD(-100), RAD(-165), RAD(-170), RAD(-165), RAD(0), RAD(-165);
			xd << -0.65, -0.45, 0.7;
			double kp = 100;
			double kv = 20;
			double kdamp = 7;
			double kmid = 25;

			Gamma_mid = - (kmid * (2 * robot->_q - (q_high + q_low)));
			Gamma_damp = - (kdamp * robot->_dq);
			F = Lambda * (- kp * (x - xd) - kv * x_vel);
			command_torques = Jv.transpose() * F + g + N.transpose() * Gamma_damp + Gamma_mid;
		}

		// ---------------------------  question 3 ---------------------------------------
		else if(controller_number == QUESTION_3)
		{
			Vector3d x, x_vel, w;
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(x_vel, link_name, pos_in_link);
			robot->angularVelocity(w, link_name);
			robot->Jv(Jv, link_name, pos_in_link);

			Matrix3d R;
			robot->rotation(R, link_name);

			Matrix3d Rd;
			Rd << cos(M_PI / 3.0), 0, sin(M_PI / 3.0),
				  0, 1, 0,
				  -sin(M_PI / 3.0), 0, cos(M_PI / 3.0);

			Vector3d xd;
			xd << 0.6, 0.3, 0.5;

			MatrixXd J(6, dof);
			robot->J_0(J, link_name, pos_in_link);

			robot->nullspaceMatrix(N, J);

			MatrixXd Lambda0(6, 6);
			robot->taskInertiaMatrix(Lambda0, J);

			double kp = 50;
			double kv = 20;
			double kpj = 50;
			double kvj = 14;

			Vector3d delta_phi;
			delta_phi = -0.5 * (R.col(0).cross(Rd.col(0)) + R.col(1).cross(Rd.col(1)) + R.col(2).cross(Rd.col(2)));

			Vector3d pd_x = - kp * (x - xd) - kv * x_vel;
			Vector3d pd_w = kp * (- delta_phi) - kv * w;
			VectorXd pd(6);
			pd << pd_x[0], pd_x[1], pd_x[2], pd_w[0], pd_w[1], pd_w[2];

			VectorXd F(6);
			F = Lambda0 * pd;

			VectorXd g(dof);
			robot->gravityVector(g);

			command_torques = J.transpose() * F - N.transpose() * kvj * robot->_dq + g;
		}

		// ---------------------------  question 4 ---------------------------------------
		else if(controller_number == QUESTION_4A)		
		{
			// part 1
			VectorXd F(3), g(dof), joint_task_torque(dof);
			Vector3d x, xd, x_vel, p;

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(x_vel, link_name, pos_in_link);
			robot->gravityVector(g);

			xd << 0.6, 0.3, 0.4;
			double kp = 200;
			double kv = 25;
			double kpj = 50;
			double kvj = 14;
			auto qd = initial_q;
			joint_task_torque = N.transpose() * (- (kpj * (robot->_q - qd)) - (kvj * robot->_dq));

			F = Lambda * (- kp * (x - xd) - kv * x_vel);
			command_torques = joint_task_torque + g + Jv.transpose() * F;
		}
		else if (controller_number == QUESTION_4B)
		{
			// part 2			
			VectorXd F(3), g(dof), joint_task_torque(dof);
			Vector3d x, xd, dx_d, x_vel, p;

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(x_vel, link_name, pos_in_link);
			robot->gravityVector(g);

			xd << 0.6, 0.3, 0.4;
			double kp = 200;
			double kv = 25;
			double Vmax = 0.1;

			dx_d = - kp / kv * (x - xd);
			double nu = sat(Vmax / dx_d.norm());

			double kpj = 50;
			double kvj = 14;
			auto qd = initial_q;
			joint_task_torque = N.transpose() * (- kpj * (robot->_q - 0*qd) - kvj * robot->_dq);

			F = Lambda * (- kv * (x_vel - nu * dx_d));
			command_torques = joint_task_torque + g + Jv.transpose() * F;
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
