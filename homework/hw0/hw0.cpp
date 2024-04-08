// some standard library includes
#include <math.h>

#include <iostream>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

// sai2 main libraries includes
#include "Sai2Model.h"

// sai2 utilities from sai2-common
#include "timer/LoopTimer.h"
#include "redis/RedisClient.h"

// redis keys
#include "redis_keys.h"

// for handling ctrl+c and interruptions properly
#include <signal.h>
bool runloop = true;
void sighandler(int) { runloop = false; }

// namespaces for compactness of code
using namespace std;
using namespace Eigen;

// config file names and object names
const string robot_file = "${CS225A_URDF_FOLDER}/rprbot.urdf";

int main() {
	Sai2Model::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// Make sure redis-server is running at localhost with default port 6379
	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, true);

	/*
	These are mathematical vectors from the library Eigen, you can read up on the documentation online.
	You can input your joint information and read sensor data C++ style "<<" or ">>". Make sure you only 
	expect to read or are writing #D.O.F. number of values.
	*/
	int dof = robot->dof();
	Eigen::VectorXd robot_q = Eigen::VectorXd::Zero(dof);
	Eigen::VectorXd robot_dq = Eigen::VectorXd::Zero(dof);
	robot_q << 0.0, 0.6, M_PI/3; // Joint 1,2,3 Coordinates (radians, meters, radians)
	robot_dq << 1.0, 0.0, 0.0; // Joint 1,2,3 Velocities (radians/sec, meters/sec, radians/sec), not used here
	robot->setQ(robot_q);
	robot->setDq(robot_dq);

	/* 
	Here we use our redis set method to serialize an 'Eigen' vector into a specific Redis Key
	Changing set to get populates the 'Eigen' vector given
	This key is then read by the physics integrator or visualizer to update the system
	*/
	redis_client.setEigen(JOINT_ANGLES_KEY, robot->q());
	redis_client.setEigen(JOINT_VELOCITIES_KEY, robot->dq());

	/*
	Update model calculates and updates robot kinematics model information 
	(calculate current jacobian, mass matrix, etc..)
	Values taken from robot->q() will be updated to currently set _q values
	*/
	robot->updateModel();

	cout << endl << endl;

	// operational space
	const string ee_link_name = "link2"; // Link of the "Task" or "End Effector"
	
	// Empty default values
	Vector3d ee_pos_in_link = Vector3d(0.0, 0.0, 0.0); // Position of Task Frame in relation to Link Frame (When using custom E.E. attachment, etc..)
	Vector3d ee_position = Vector3d::Zero(); // 3d vector of zeros to fill with the end effector position
	MatrixXd ee_jacobian(3, dof); // Empty Jacobian Matrix sized to right size
	VectorXd g(dof); // Empty Gravity Vector

	// Examples how to update and get position, Jacobians, gravity vectors
	ee_position = robot->position(ee_link_name, ee_pos_in_link); // get end-effector's position, and write into ee_position
	cout << "End effector position w.r.t. ground :" << endl;
	cout << ee_position.transpose() << endl << endl;

	ee_jacobian = robot->Jv(ee_link_name, ee_pos_in_link); // get jacobian, and write into ee_jacobian
	cout << "Printing Jacobian and Mass matrix : " << endl;
	cout << ee_jacobian << endl; // Print Jacobian
	cout << robot->M() << endl << endl; // Print Mass Matrix, you can index into this variable (and all 'Eigen' types)!

	g = robot->jointGravityVector(); // get gravity vector, and write into g
	cout << "Printing gravity : " << endl;
	cout << endl << g.transpose() << endl << endl;

	/* 
	Retrieve multiple values of the gravity or M with a for loop of calling robot->SetQ(robot_q), 
	setting redis keys for display update if needed and don't forget robot->updateModel()! 
	We'll have a logger for you later to dump redis values at whatever rate you choose
	*/

	// *************************************************************************
	// **********************    WRITE YOUR CODE AFTER    **********************
	// *************************************************************************

	// ---------------------------  question 2-b -------------------------------
	ee_pos_in_link = Vector3d(0.0, 0.0, 0.0); // modify this

	// ---------------------------  question 2-c -------------------------------
	// part i
	robot_q << 0.0, 0.0, 0.0; // modify this
	robot->setQ(robot_q);
	robot->updateKinematics();
	ee_position = robot->position(ee_link_name, ee_pos_in_link);
	cout << "========================================= Q2-c-i" << endl << endl;
	cout << "End effector position for configuration i\n" << ee_position.transpose() << endl << endl;
	// part ii
	robot_q << 0.0, 0.0, 0.0; // modify this
	robot->setQ(robot_q);
	robot->updateKinematics();
	ee_position = robot->position(ee_link_name, ee_pos_in_link);
	cout << "========================================= Q2-c-ii" << endl << endl;
	cout << "End effector position for configuration i\n" << ee_position.transpose() << endl << endl;

	// ---------------------------  question 2-d -------------------------------
	// part i
	robot_q << 0.0, 0.0, 0.0; // modify this
	robot->setQ(robot_q);
	robot->updateKinematics();
	ee_jacobian = robot->Jv(ee_link_name, ee_pos_in_link);
	cout << "========================================= Q2-d-ii" << endl << endl;
	cout << "Jv for configuration d-i\n" << ee_jacobian << endl << endl;
	// part ii
	robot_q << 0.0, 0.0, 0.0; // modify this
	robot->setQ(robot_q);
	robot->updateKinematics();
	ee_jacobian = robot->Jv(ee_link_name, ee_pos_in_link);
	cout << "========================================= Q2-d-ii" << endl << endl;
	cout << "Jv for configuration d-ii\n" << ee_jacobian << endl << endl;

	// ---------------------------  question 2-e -------------------------------
	// part i
	ofstream file_2e_i;
	file_2e_i.open("../../hw0/data_files/q2-e-i.txt");
	robot_q << 0.0, 0.0, 0.0; // modify this
	robot->setQ(robot_q);
	robot->updateModel();
	file_2e_i << 0 << "\t" << 0 << "\t" << 0 << "\n"; // modify this
	int n_steps = 250;
	for(int i=0 ; i < n_steps ; i++)
	{
		// write your code
	}
	file_2e_i.close();

	// part ii
	ofstream file_2e_ii;
	file_2e_ii.open("../../hw0/data_files/q2-e-ii.txt");
	robot_q << 0.0, 0.0, 0.0; // modify this
	robot->setQ(robot_q);
	robot->updateModel();
	file_2e_ii << 0 << "\t" << 0 << "\t" << 0 << "\n"; // modify this
	n_steps = 250;
	for(int i=0 ; i < n_steps ; i++)
	{
		// write your code
	}
	file_2e_ii.close();

	// ---------------------------  question 2-f -------------------------------
	// part i
	ofstream file_2f_i;
	file_2f_i.open("../../hw0/data_files/q2-f-i.txt");
	robot_q << 0.0, 0.0, 0.0; // modify this
	robot->setQ(robot_q);
	robot->updateModel();
	g = robot->jointGravityVector();
	file_2f_i << g.transpose() << "\n";
	n_steps = 250;
	for(int i=0 ; i < n_steps ; i++)
	{
		// write your code
	}
	file_2f_i.close();

	// part ii
	ofstream file_2f_ii;
	file_2f_ii.open("../../hw0/data_files/q2-f-ii.txt");
	robot_q << 0.0, 0.0, 0.0; // modify this
	robot->setQ(robot_q);
	robot->updateModel();
	g = robot->jointGravityVector();
	file_2f_ii << g.transpose() << "\n";
	n_steps = 250;
	for(int i=0 ; i < n_steps ; i++)
	{
		// write your code
	}
	file_2f_ii.close();

	// -----------------  question 2-g : extra credit--------------------------
	// extra credit
	VectorXd grav_bis = VectorXd::Zero(4);
	
	// write your code

    return 0;
}
