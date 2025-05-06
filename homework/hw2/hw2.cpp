// some standard library includes
#include <math.h>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// sai main libraries includes
#include "SaiModel.h"

// sai utilities from sai-common
#include "timer/LoopTimer.h"
#include "redis/RedisClient.h"

// redis keys
#include "redis_keys.h"

// for handling ctrl+c and interruptions properly
#include <signal.h>
bool runloop = true;

void sighandler(int) { runloop = false; }
    // handle break for file writing
    std::ofstream jointFile;
    void onSigInt(int) {
    std::cerr << "Caught SIGINT, closing file and exiting.\n";
    if (jointFile.is_open()) jointFile.close();
    std::exit(0);
    }

// namespaces for compactness of code
using namespace std;
using namespace Eigen;

// config file names and object names
const string robot_file = "${CS225A_URDF_FOLDER}/panda/panda_arm.urdf";

int main(int argc, char** argv) {
    SaiModel::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);

    // check for command line arguments
    if (argc != 2) {
        cout << "Incorrect number of command line arguments" << endl;
        cout << "Expected usage: ./{HW_NUMBER} {QUESTION_NUMBER}" << endl;
        return 1;
    }
    // convert char to int, check for correct controller number input
    string arg = argv[1];
    int controller_number;
    try {
        size_t pos;
        controller_number = stoi(arg, &pos);
        if (pos < arg.size()) {
            cerr << "Trailing characters after number: " << arg << '\n';
            return 1;
        }
        else if (controller_number < 1 || controller_number > 4) {
            cout << "Incorrect controller number" << endl;
            return 1;
        }
    } catch (invalid_argument const &ex) {
        cerr << "Invalid number: " << arg << '\n';
        return 1;
    } catch (out_of_range const &ex) {
        cerr << "Number out of range: " << arg << '\n';
        return 1;
    }

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots
    auto robot = new SaiModel::SaiModel(robot_file);

    // prepare controller
	int dof = robot->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.10);
	VectorXd control_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	Jv = robot->Jv(link_name, pos_in_link);
	Lambda = robot->taskInertiaMatrix(Jv);
	J_bar = robot->dynConsistentInverseJacobian(Jv);
	N = robot->nullspaceMatrix(Jv);

    // flag for enabling gravity compensation
    bool gravity_comp_enabled = false;

    // start redis client
    auto redis_client = SaiCommon::RedisClient();
    redis_client.connect();

    // setup send and receive groups
    VectorXd robot_q = redis_client.getEigen(JOINT_ANGLES_KEY);
    VectorXd robot_dq = redis_client.getEigen(JOINT_VELOCITIES_KEY);
    redis_client.addToReceiveGroup(JOINT_ANGLES_KEY, robot_q);
    redis_client.addToReceiveGroup(JOINT_VELOCITIES_KEY, robot_dq);

    redis_client.addToSendGroup(JOINT_TORQUES_COMMANDED_KEY, control_torques);
    redis_client.addToSendGroup(GRAVITY_COMP_ENABLED_KEY, gravity_comp_enabled);

    redis_client.receiveAllFromGroup();
    redis_client.sendAllFromGroup();

    // update robot model from simulation configuration
    robot->setQ(robot_q);
    robot->setDq(robot_dq);
    robot->updateModel();

    // record initial configuration
    VectorXd initial_q = robot->q();

    // create a loop timer
    const double control_freq = 1000;
    SaiCommon::LoopTimer timer(control_freq);

    jointFile.open("../../homework/hw2/q1_joint_pos.csv");
    if(!jointFile){
        std::cerr << "ERROR: could not open joint_positions.csv for writing\n";
        return 1;
    }
    jointFile << "t,x,y,z,q0,q1,q2,q3,q4,q5,q6\n"; // header
    std::size_t counter = 0;

    while (runloop) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime();

        // read robot state from redis
        redis_client.receiveAllFromGroup();
        robot->setQ(robot_q);
        robot->setDq(robot_dq);
        robot->updateModel();

        // **********************
        // WRITE YOUR CODE AFTER
        // **********************

        // ---------------------------  question 1 ---------------------------------------
        if(controller_number == 1) {
            // define Kv, Kp
            Eigen::Vector<double,7> Kp, Kv;
            Kp.setZero();  Kv.setZero();

            for(int i = 0; i < 6; ++i) {
                Kp[i] = 400.0;
                Kv[i] =  50.0;
            }
            Kp[6] = 50.0;
            double d = 0.1425;
            Kv[6] = -d; // assuming d = 0 (no damping term in URDF)

            Eigen::Vector<double,7> q_desired = robot_q;
            q_desired[6] = 0.1; // change desired configuration of joint 6

            Eigen::Matrix<double,7,1> tau_proportional =  Kp.asDiagonal() * (robot_q - q_desired);
            Eigen::Matrix<double,7,1> tau_derivative =  Kv.asDiagonal() * robot_dq;
            
            control_torques = -tau_proportional - tau_derivative + robot->coriolisPlusGravity();

                        // write joint values as comma separated
            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << robot_q(6) << '\n';
            }
        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {
            VectorXd ee_desired(3);
            ee_desired << 0.3, 0.1, 0.5; 

            VectorXd x = robot->positionInWorld(link_name, pos_in_link);
            VectorXd xdot = Jv * robot_dq;

            double Kp = 200;
            double Kv = 30;
            Eigen::Vector<double,7> Kvj;
            Kvj.setZero();

            for(int i = 0; i < 7; ++i) {
                Kvj[i] =  50.0;
            }
            Eigen::Matrix<double,7,1> tau_derivative =  Kvj.asDiagonal() * robot_dq;
            Eigen::Matrix<double,7,1> tau_derivative_null = -1 * N.transpose() * robot->M() * (Kvj.asDiagonal() * robot_dq);

            VectorXd F = Lambda * (-Kp * (x - ee_desired) - Kv * xdot);
            control_torques = (Jv.transpose() * F) + tau_derivative_null + robot->jointGravityVector();
            
            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << x(0) << ','
                    << x(1) << ','
                    << x(2) << ','
                    << robot_q(0) << ','
                    << robot_q(1) << ','
                    << robot_q(2) << ','
                    << robot_q(3) << ','
                    << robot_q(4) << ','
                    << robot_q(5) << ','
                    << robot_q(6) << '\n';
            }

        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {
            VectorXd ee_desired(3);
            ee_desired << 0.3, 0.1, 0.5; 

            Jv = robot->Jv(link_name, pos_in_link);
	        Lambda = robot->taskInertiaMatrix(Jv);
	        J_bar = robot->dynConsistentInverseJacobian(Jv);
	        N = robot->nullspaceMatrix(Jv);

            VectorXd x = robot->positionInWorld(link_name, pos_in_link);
            VectorXd xdot = robot->linearVelocity(link_name, pos_in_link);
            MatrixXd Kvj =  5*MatrixXd::Identity(7, 7);

            double Kp = 200;
            double Kv = 2*std::sqrt(Kp);

            VectorXd p = J_bar.transpose() * robot->jointGravityVector();

            VectorXd F = Lambda * (-1*(Kp * (x - ee_desired)) - (Kv * xdot)) + p;
            Eigen::Matrix<double,7,1> tau_derivative_null = -1 * N.transpose() * (robot->M()) * (Kvj * robot_dq);

            control_torques = (Jv.transpose() * F) + tau_derivative_null;
            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << x(0) << ','
                    << x(1) << ','
                    << x(2) << ','
                    << robot_q(0) << ','
                    << robot_q(1) << ','
                    << robot_q(2) << ','
                    << robot_q(3) << ','
                    << robot_q(4) << ','
                    << robot_q(5) << ','
                    << robot_q(6) << '\n';
            }

        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {


            Jv = robot->Jv(link_name, pos_in_link);
	        Lambda = robot->taskInertiaMatrix(Jv);
	        J_bar = robot->dynConsistentInverseJacobian(Jv);
	        N = robot->nullspaceMatrix(Jv);

            VectorXd x = robot->positionInWorld(link_name, pos_in_link);
            VectorXd xdot = robot->linearVelocity(link_name, pos_in_link);
            MatrixXd Kvj =  5*MatrixXd::Identity(7, 7);
            MatrixXd Kpj =  30*MatrixXd::Identity(7, 7);

            double Kp = 200;
            double Kv = 2*std::sqrt(Kp);

            VectorXd ee_desired(3);
            ee_desired << 0.3 + 0.1*sin(M_PI*time), 0.1 + 0.1*cos(M_PI*time), 0.5; 

            VectorXd p = J_bar.transpose() * robot->jointGravityVector();
            VectorXd F = Lambda * (-1*(Kp * (x - ee_desired)) - (Kv * xdot)) + p;
            Eigen::Matrix<double,7,1> tau_derivative_null = -1* N.transpose() * (robot->M()) * (Kvj * robot_dq + Kpj * robot_q);
            control_torques = (Jv.transpose() * F) + tau_derivative_null;

            
            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << x(0) << ','
                    << x(1) << ','
                    << x(2) << ','
                    << robot_q(0) << ','
                    << robot_q(1) << ','
                    << robot_q(2) << ','
                    << robot_q(3) << ','
                    << robot_q(4) << ','
                    << robot_q(5) << ','
                    << robot_q(6) << '\n';
            }

        }

        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.sendAllFromGroup();
    }

    jointFile.close();

    control_torques.setZero();
    gravity_comp_enabled = true;
    redis_client.sendAllFromGroup();

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
