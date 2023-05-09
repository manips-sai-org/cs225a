#include <Sai2Model.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

#include <signal.h>
// flags for simulation and controller states
bool runloop = false;
void sighandler(int){runloop = false;}
bool fSimulationLoopDone = false;
bool fControllerLoopDone = false;

// helper function 
double sat(double x);

// function for converting string to bool
bool string_to_bool(const std::string& x);

// function for converting bool to string
inline const char * const bool_to_string(bool b);

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/panda_collision.urdf";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY  = "cs225a::robot::panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::robot::panda::sensors::dq";
const std::string OBJ_JOINT_ANGLES_KEY  = "cs225a::object::cup::sensors::q";
const std::string OBJ_JOINT_VELOCITIES_KEY = "cs225a::object::cup::sensors::dq";
// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs225a::robot::panda::actuators::fgc";
// - read + write: 
const std::string SIMULATION_LOOP_DONE_KEY = "cs225a::simulation::done";
const std::string CONTROLLER_LOOP_DONE_KEY = "cs225a::controller::done";

int main() {

    // Make sure redis-server is running at localhost with default port 6379
    // start redis client
    RedisClient redis_client = RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots, read current state nd update the model
    Eigen::Vector3d robot_offset = Eigen::Vector3d(0.0, 0.3, 0.0);
    Eigen::Matrix3d R_world_robot = Eigen::Matrix3d::Identity();
    // Eigen::Matrix3d R_world_robot = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())
                                    // * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                                    // * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

    Eigen::Affine3d T_world_robot = Eigen::Affine3d::Identity();
    T_world_robot.translation() = robot_offset;
    T_world_robot.linear() = R_world_robot;

    auto robot = new Sai2Model::Sai2Model(robot_file, false, T_world_robot);
    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
    robot->updateModel();

    // prepare controller
    // note: please use sai2-primitive functions - explicit control law shown here for clarity
    int dof = robot->dof();
    const string link_name = "link7";
    const Vector3d pos_in_link = Vector3d(0, 0, 0.15);

    VectorXd control_torques = VectorXd::Zero(dof);
    VectorXd gravity = VectorXd::Zero(dof);

    Vector3d x1_des;
    x1_des << 0, -0.8, 0.50;
    Vector3d x2_des;
    x2_des << 0, -0.6, 0.55;

    // model quantities for operational space control
    MatrixXd Jv = MatrixXd::Zero(3,dof);
    MatrixXd Lambda = MatrixXd::Zero(3,3);
    MatrixXd J_bar = MatrixXd::Zero(dof,3);
    MatrixXd N = MatrixXd::Zero(dof,dof);

    robot->Jv(Jv, link_name, pos_in_link);
    robot->taskInertiaMatrix(Lambda, Jv);
    robot->dynConsistentInverseJacobian(J_bar, Jv);
    robot->nullspaceMatrix(N, Jv);

    VectorXd F(6), g(dof), b(dof), joint_task_torque(dof), Gamma_damp(dof), Gamma_mid(dof);;
    VectorXd q_high(dof), q_low(dof);
    Vector3d x, x_vel, p, w;
    Vector3d dxd, ddxd;
    Matrix3d R, Rd;

    Rd << 0.696707, -0.717356, -7.0252e-12,
            -0.717356, -0.696707, -6.82297e-12,
            0, 9.79318e-12, -1;

    // create a loop timer
    double control_freq = 1000;
    LoopTimer timer;
    timer.setLoopFrequency(control_freq);   // 1 KHz
    timer.initializeTimer(1000000); // 1 ms pause before starting loop
    bool fTimerDidSleep = true;
    double start_time = timer.elapsedTime(); // secs

    unsigned long long counter = 0;

    runloop = true;
    while (runloop)
    {
        // fTimerDidSleep = timer.waitForNextLoop(); // commented out to let current controller loop finish before next loop

        // read simulation state
        fSimulationLoopDone = string_to_bool(redis_client.get(SIMULATION_LOOP_DONE_KEY));

        // run controller loop when simulation loop is done
        if (fSimulationLoopDone) {
            // read robot state from redis
            robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
            robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

            // update robot model and compute gravity
            robot->updateModel();
            robot->gravityVector(g);

            // compute control torques
            robot->position(x, link_name, pos_in_link);
            robot->linearVelocity(x_vel, link_name, pos_in_link);
            robot->angularVelocity(w, link_name);
            robot->rotation(R, link_name);

            MatrixXd J(6, dof);
            robot->J_0(J, link_name, pos_in_link);
            robot->nullspaceMatrix(N, J);

            MatrixXd Lambda0(6, 6);
            robot->taskInertiaMatrix(Lambda0, J);

            // note: please use sai2-primitive functions - explicit control law shown here for clarity

            double kp = 25;
            double kv = 10;
            double kpj = 10;
            double kvj = 5;
            double kdamp = 10;
            double kmid = 10;

            // q_high << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
            // q_low << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

            // Gamma_mid = - (kmid * (2 * robot->_q - (q_high + q_low)));
            Gamma_damp = - (kdamp * robot->_dq);

            Vector3d delta_phi;
            delta_phi = -0.5 * (R.col(0).cross(Rd.col(0)) + R.col(1).cross(Rd.col(1)) + R.col(2).cross(Rd.col(2)));

            double Vmax = 0.5;
            dxd = - kp / kv * (x - x1_des);
            double nu = sat(Vmax / dxd.norm());

            Vector3d pd_x = - kp * nu * (x - x1_des) - kv * x_vel;
            Vector3d pd_w = kp * (- delta_phi) - kv * w;
            VectorXd pd(6);
            pd << pd_x[0], pd_x[1], pd_x[2], pd_w[0], pd_w[1], pd_w[2];

            VectorXd F(6);
            F = Lambda0 * pd;
            control_torques = J.transpose() * F + N.transpose() * ( Gamma_damp ) + 0*g;  // gravity is compensated in simviz loop as of now

            // send torques to redis
            redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, control_torques);

            // ask for next simulation loop
            fSimulationLoopDone = false;
            redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));

            ++counter;
        }

        // controller loop is done
        fControllerLoopDone = true;
        redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));
    }

    control_torques.setZero();
    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, control_torques);

    // controller loop is turned off
    fControllerLoopDone = false;
    redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Control Loop run time  : " << end_time << " seconds\n";
    // std::cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    // std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
    std::cout << "Control Loop updates   : " << counter << "\n";

    return 0;
}


//------------------------------------------------------------------------------

double sat(double x) {
    if (abs(x) <= 1.0) {
        return x;
    }
    else {
        return signbit(x);
    }
}

//------------------------------------------------------------------------------

bool string_to_bool(const std::string& x) {
    assert(x == "false" || x == "true");
    return x == "true";
}

//------------------------------------------------------------------------------

inline const char * const bool_to_string(bool b)
{
    return b ? "true" : "false";
}
