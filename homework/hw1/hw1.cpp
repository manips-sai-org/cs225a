#include <math.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <csignal>

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

// namespaces for compactness of code
using namespace std;
using namespace Eigen;

// config file names and object names
const string robot_file = "${CS225A_URDF_FOLDER}/panda/panda_arm_controller.urdf";

std::ofstream jointFile;
void onSigInt(int) {
    std::cerr << "Caught SIGINT, closing file and exiting.\n";
    if (jointFile.is_open()) jointFile.close();
    std::exit(0);
}


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
        else if (controller_number < 1 || controller_number > 5) {
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
    VectorXd control_torques = VectorXd::Zero(dof);

    // flag for enabling gravity compensation
    bool gravity_comp_enabled = false;

    // start redis client
    auto redis_client = SaiCommon::RedisClient();
    redis_client.connect();

    // setup send and receive groups
    VectorXd robot_q = VectorXd::Zero(dof);
    VectorXd robot_dq = VectorXd::Zero(dof);
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

    // define gravity force and preallocate Jv and payload torque memory
    Eigen::Vector3d g_world = robot->worldGravity();
    const double m_payload = 2.5;
    Eigen::Matrix<double,3,7> Jv;
    Eigen::VectorXd T_payload(7);
    Eigen::Vector3d payload_offset(0,0,0.0);

    // record initial configuration
    VectorXd initial_q = robot->q();

    // create a loop timer
    const double control_freq = 1000;
    SaiCommon::LoopTimer timer(control_freq);


    std::signal(SIGINT, onSigInt);

    jointFile.open("../../homework/hw1/q1_joint_pos.csv");
    if(!jointFile){
        std::cerr << "ERROR: could not open joint_positions.csv for writing\n";
        return 1;
    }
    jointFile << "t,q0,q2,q3\n"; // header

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
        // algorithmic: write out full equation, get a term from mass matrix htat is multiplied by your theta double dot. Then calculate appropriate gains. 
        // easier: export your theta0 q values and plot them (like HW 1). See if it is overdamped or underdamped based on response of theta.
        // Get a nice plotting setup going from text file to plot. Just change path.
        // Or learn how to read redis keys using a coding paltform like python, and directly capture the keys as they are written
        // simulator <->redis<->controller: is OpenSai. We dump from controller to txt file and plot. Fancy: directly go from redis to plotting
        // redis key. Sai::Robot::Panda::q::1 (like a key for a python dictionary)
        // In python, set up a redis connection, youd have a while loop running that ocntinuosuly pulls in the current value form the desired key, and exports it to MATplotlib. 
        // If you had a plotter that updated live, youd be watching it osciallte when you run, otherwise static.
        // between 0 and 1000
        

        VectorXd q_zero(dof); // dof is an int pulled from robot object
        q_zero << -80.0, -45.0, 0.0, -125.0, 0.0, 80.0, 0.0; // degrees
        q_zero << q_zero * M_PI / 180.0; // radians

        VectorXd q_desired(dof); // dof is an int pulled from robot object
        q_desired << 90.0, -45.0, 0.0, -125.0, 0.0, 80.0, 0.0; // degrees
        q_desired << q_desired * M_PI / 180.0; // radians

        if(controller_number == 1) {

            double kp = 400.0;      // chose your p gain
            double kv = 50.0;      // chose your d gain

            control_torques.setZero();  // change to the control torques you compute
            control_torques = -kp*(robot_q - q_desired) 
                        - kv * robot_dq;
            
            // write joint values as comma separated
            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << robot_q(0) << ','
                    << robot_q(2) << ','
                    << robot_q(3) << '\n';
            }
        }
        
        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {

            double kp = 400.0;      // chose your p gain
            double kv = 50.0;      // chose your d gain

            // PD + gravity
            control_torques = -kp * (robot_q - q_desired)
                      -kv * robot_dq
                      + robot->jointGravityVector();
            
            // write joint values as comma separated
            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << robot_q(0) << ','
                    << robot_q(2) << ','
                    << robot_q(3) << '\n';
            }

        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {
            
            double kp = 400.0;      // chose your p gain
            double kv = 40.0;      // chose your d gain

            control_torques = robot->M() * (-kp * (robot_q - q_desired) 
                        - kv * robot_dq) 
                        + robot->jointGravityVector();
            
            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << robot_q(0) << ','
                    << robot_q(2) << ','
                    << robot_q(3) << '\n';
            }
        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {

            double kp = 400.0;      // chose your p gain
            double kv = 40.0;      // chose your d gain

            control_torques = robot->M() * (-kp * (robot_q - q_desired)
                      -kv * robot_dq)
                      + robot->coriolisPlusGravity();
            
            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << robot_q(0) << ','
                    << robot_q(2) << ','
                    << robot_q(3) << '\n';
            }
        }

        // ---------------------------  question 5 ---------------------------------------
        // Use controller 4 for question 5. Use this controller for bonus.
        else if(controller_number == 5) {

            double kp = 400.0;      // chose your p gain
            double kv = 40.0;      // chose your d gain

            Jv = robot->JvWorldFrame("link7", payload_offset);
            T_payload = Jv.transpose() * (m_payload * g_world);
            // MatrixXd new_mass = robot->M();
            // new_mass(8) += 2.5;

            control_torques = robot->M() * (-kp * (robot_q - q_desired)
                      -kv * robot_dq)
                      + robot->coriolisPlusGravity()
                      + T_payload;

            std::cout << "T_payload = " << T_payload.transpose() << "\n";

            
            if (++counter % 10 == 0) {
                jointFile
                    << time << ','
                    << robot_q(0) << ','
                    << robot_q(2) << ','
                    << robot_q(3) << '\n';
            }
        }
        if (counter == 100) counter = 0;
        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.setInt("sai::simviz::gravity_comp_enabled", 0);
        redis_client.sendAllFromGroup();
    }
    
    jointFile.close();

    control_torques.setZero(); // safety feature to make sure you only apply torques when you really want to
    redis_client.sendAllFromGroup();

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
