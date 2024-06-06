/**
 * @file simviz.cpp
 * @brief Simulation and visualization of panda robot with 1 DOF gripper 
 * 
 */

#include <math.h>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <random>
#include <sstream> 

#include "Sai2Graphics.h"
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"

std::atomic<bool> fSimulationRunning = true;
void sighandler(int signum) {
    fSimulationRunning = false;
}

#include "redis_keys.h"

using namespace Eigen;
using namespace std;

// mutex and globals
VectorXd ui_torques;
mutex mutex_torques, mutex_update;

// Global variables
mutex ball_mutex;
bool new_ball_ready = false;
Eigen::Vector3d new_ball_position;
Eigen::Vector3d new_ball_velocity;
bool ball_launched = false;
bool strike_thrown = true;
Eigen::Vector3d last_ball_pose;

// specify urdf and robots 
static const string robot_name = "panda_arm_bat";

// dynamic objects information
const vector<std::string> object_names = {"baseball"};
vector<Affine3d> object_poses;
vector<VectorXd> object_velocities;
const int n_objects = object_names.size();

// simulation thread + function prototypes
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);
void computeZIntersectionWithPlane(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, Sai2Common::RedisClient& redis_client, std::shared_ptr<Sai2Graphics::Sai2Graphics> graphics);
void handleUserInput(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim, std::shared_ptr<Sai2Graphics::Sai2Graphics> graphics);
Eigen::Vector3d mapVelocity(double input_vy, double input_vz);

// void playSound(const string& soundPath) {
//     string command = "cmd.exe /C powershell -c (New-Object Media.SoundPlayer \"" + soundPath + "\").PlaySync()";
//     system(command.c_str());
// }

int main() {
	Sai2Model::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_bat.urdf";
	static const string world_file = string(PANDABAT_FOLDER) + "/world.urdf";
	std::cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, "Baseball Panda", false);
	graphics->setBackgroundColor(25.0/255, 189.0/255, 255.0/255);  // set blue background 	
	//graphics->showLinkFrame(true, robot_name, "link7", 0.2);  
	//graphics->showLinkFrame(true, robot_name, "link0", 0.25);
	//graphics->showLinkFrame(true, robot_name, "end-effector", 0.2);
	// graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes 
	// graphics->addUIForceInteraction(robot_name);

	// load robots
	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
	// robot->setQ();
	// robot->setDq();
	robot->updateModel();
	ui_torques = VectorXd::Zero(robot->dof());

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);
	sim->setJointPositions(robot_name, robot->q());
	sim->setJointVelocities(robot_name, robot->dq());

	// fill in object information 
	for (int i = 0; i < n_objects; ++i) {
		object_poses.push_back(sim->getObjectPose(object_names[i]));
		object_velocities.push_back(sim->getObjectVelocity(object_names[i]));
	}

	/*------- Set up baseball ball -------*/
	// std::string input_line;  // To store the whole input line
	// double x, y, z;          // Variables to store user input coordinates for position
	// std::cout << "Enter initial position of " << object_names[0] << " (x y z): ";
	// std::getline(std::cin, input_line);  // Read the whole line of input
	// std::istringstream iss_pos(input_line);  // Use istringstream to process the line
	// iss_pos >> x >> y >> z;  // Extract position coordinates from the line

	Eigen::Affine3d new_pose;
	new_pose.translation() = Eigen::Vector3d(1.2, 1.0, 1.0);
	sim->setObjectPose(object_names[0], new_pose);
	object_poses[0] = new_pose; // Update the pose in the global vector

	// double vx, vy, vz;       // Variables to store user input coordinates for velocity
	// std::cout << "Enter initial velocity of " << object_names[0] << " (vx vy vz): ";
	// std::getline(std::cin, input_line);  // Read the whole line of input for velocity
	// std::istringstream iss_vel(input_line);  // Process the line for velocity
	// iss_vel >> vx >> vy >> vz;  // Extract velocity coordinates

	// // Set the velocity with zero angular components
	// Eigen::VectorXd initial_velocity(6);
	// initial_velocity << vx, vy, vz, 0, 0, 0;  // No angular velocity
	// sim->setObjectVelocity(object_names[0], initial_velocity);
	// object_velocities[0] = initial_velocity; // Update the velocity in the global vector

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// init redis client values 
	redis_client.setEigen(JOINT_ANGLES_KEY, robot->q()); 
	redis_client.setEigen(JOINT_VELOCITIES_KEY, robot->dq()); 
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * robot->q());
	redis_client.set(BALL_TRAJECTORY, "");
	redis_client.setBool(NEW_BALL_SIGNAL, false);
	redis_client.setEigen(BALL_POS, Eigen::Vector3d(1.2, 10.0, 1.0));
	redis_client.setEigen(BALL_VEL, Eigen::Vector3d(1.2, 10.0, 1.0));
	redis_client.set(BUTTON_LAST_STATE, "false");
	redis_client.set(CONTROLLER_STATE, "");

	// start simulation thread
	thread sim_thread(simulation, sim);

	// Start the user input thread
    thread input_thread(handleUserInput, sim, graphics);

	// while window is open:
	while (graphics->isWindowOpen() && fSimulationRunning) {		
        graphics->updateRobotGraphics(robot_name, redis_client.getEigen(JOINT_ANGLES_KEY));
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				graphics->updateObjectGraphics(object_names[i], object_poses[i]);
			}
		}
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			ui_torques = graphics->getUITorques(robot_name);
		}
	}

    // stop simulation
	fSimulationRunning = false;
	if (input_thread.joinable()) {
		input_thread.join();
	}
	if (sim_thread.joinable()) {
		sim_thread.join();
	}

	return 0;
}

//------------------------------------------------------------------------------
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	// fSimulationRunning = true;

    // create redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

	// create a timer
	double sim_freq = 2000;
	Sai2Common::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);
    sim->enableGravityCompensation(true);
	sim->enableJointLimits(robot_name);

	//playSound("/mnt/c/Users/enriq/Desktop/Spring2024/CS225A/OpenSai/cs225a-baseball/project_starter/pandabat/WiiSports.wav");

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		VectorXd control_torques = redis_client.getEigen(JOINT_TORQUES_COMMANDED_KEY);
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(robot_name, control_torques + ui_torques);
		}

		if (new_ball_ready) {
			lock_guard<mutex> lock(ball_mutex);
			sim->setObjectPose("baseball", Affine3d(Translation3d(new_ball_position)));
			Eigen::VectorXd velocity(6);
			velocity << 0, new_ball_velocity.y(), 0, 0, 0, 0;  // Set linear velocity, no angular
			sim->setObjectVelocity("baseball", velocity);
			new_ball_ready = false;
		}

		sim->integrate();
        redis_client.setEigen(JOINT_ANGLES_KEY, sim->getJointPositions(robot_name));
        redis_client.setEigen(JOINT_VELOCITIES_KEY, sim->getJointVelocities(robot_name));

		// update object information 
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				object_poses[i] = sim->getObjectPose(object_names[i]);
				object_velocities[i] = sim->getObjectVelocity(object_names[i]);
			}
		}
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}

// NOTE: Sperate thread is running this, and cin does not understand when the input has been closed so killing the simulation
// will leave the window hanging. Need to kill the terminal too so that the simulation actually stops since it waits for the
// input_thread to join. To do this, just do CTR+D in the terminal or just hit enter.
void handleUserInput(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim, std::shared_ptr<Sai2Graphics::Sai2Graphics> graphics) {

    // create redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    string input_line;
    double x, z, vy = -9.8;  // Preset negative y velocity

	ball_launched = false;
	new_ball_ready = false;
	redis_client.set(BUTTON_LAST_STATE, "true");
	// cout << endl << "Enter 'x z' coordinates for the new ball: ";
    // while (fSimulationRunning) {
	// 	// Prompt for the next set of coordinates
	// 	if (ball_launched == false) {
	// 		Vector3d OS_X = redis_client.getEigen(BALL_POS);
	// 		Eigen::Affine3d new_pose;
	// 		new_pose.translation() = OS_X;
	// 		last_ball_pose = OS_X;
	// 		sim->setObjectPose(object_names[0], new_pose);
	// 		object_poses[0] = new_pose;
	// 	}
		

    //     // if (getline(cin, input_line)) {
    //     //     istringstream iss(input_line);
    //     //     if (iss >> x >> z) {
    //     //         lock_guard<mutex> lock(ball_mutex);
    //     //         new_ball_position = Eigen::Vector3d(x, 5.0, z);
    //     //         new_ball_velocity = Eigen::Vector3d(0, vy, 0);
    //     //         new_ball_ready = true;
    //     //         cout << "New ball placed." << endl;

    //     //         // Calculate and announce the crossing position
    //     //         computeZIntersectionWithPlane(new_ball_position, new_ball_velocity, redis_client);

	// 	// 		cout << endl << "Enter 'x z' coordinates for the new ball: ";
    //     //     }
    //     // }

    //     if (redis_client.get(BUTTON_LAST_STATE) == "true") {  // Check if the button state is true
    //         cout << "Button press detected, launching new ball." << endl;

    //         lock_guard<mutex> lock(ball_mutex);
    //         new_ball_position = last_ball_pose;  	// Set new position
	// 		Eigen::Vector3d velocityLast = redis_client.getEigen(BALL_VEL);
    //         new_ball_velocity = Eigen::Vector3d(0, -9.8, 0);  		// Set new velocity
    //         new_ball_ready = true;
    //         ball_launched = true;
    //         computeZIntersectionWithPlane(new_ball_position, new_ball_velocity, redis_client);
    //         redis_client.set(BUTTON_LAST_STATE, "false");  // Acknowledge handling by setting to false
    //     }
    // }

	while (fSimulationRunning) {
        string controllerState = redis_client.get(CONTROLLER_STATE);
		if (strike_thrown == false){
			if (object_poses[0].translation().y() < -4){
				cout << "resetting" << endl;
				strike_thrown = true;
				ball_launched = false;
				graphics->setCameraPose("camera_E_Pitcher", 
										Eigen::Vector3d(1.2, 7, 1.5),
										Eigen::Vector3d(0.0, 0.0, 1.0),
										Eigen::Vector3d(1.2, 0, 1.5));
			}
		} else if (controllerState == "WAITING1" && ball_launched == false) {
            // Update ball's position continuously in WAITING1 state
			Vector3d currentPosition = redis_client.getEigen(BALL_POS);
			// Vector3d currentPosition = Eigen::Vector3d(0.9, 5, 1.6);
			Eigen::Affine3d new_pose;
			new_pose.translation() = currentPosition;
			sim->setObjectPose(object_names[0], new_pose);
			object_poses[0] = new_pose;
			//cout << "Pos updated." << endl;
		

            // Check if the button is pressed to launch the ball
            if (redis_client.get(BUTTON_LAST_STATE) == "false") {

                cout << "Button press release in WAITING1 state, launching new ball." << endl;

                lock_guard<mutex> lock(ball_mutex);
                new_ball_position = currentPosition;  // Use the current position

				// Get y-component of velocity from Redis, map it, and set x, z to zero
                Vector3d input_v = redis_client.getEigen(BALL_VEL);
				// Vector3d input_v = Eigen::Vector3d(0, -4.8, 0);
				double input_vy = input_v[1];
				double input_vz = input_v[2];
                new_ball_velocity = mapVelocity(input_vy, input_vz);  // Use mapped velocity
                //new_ball_velocity = Eigen::Vector3d(0, -9.8, 0);  // Use predefined velocity
				
				ball_launched = true;
                new_ball_ready = true;
                computeZIntersectionWithPlane(new_ball_position, new_ball_velocity, redis_client, graphics);
                redis_client.set(BUTTON_LAST_STATE, "true");  // Acknowledge handling by setting to true
            }
        } else if (controllerState == "POSTURE") {
            // Reset the ball_launched flag when in POSTURE state
            ball_launched = false;
			graphics->setCameraPose("camera_E_Pitcher", 
							Eigen::Vector3d(1.2, 7, 1.5),
							Eigen::Vector3d(0.0, 0.0, 1.0),
							Eigen::Vector3d(1.2, 0, 1.5));
        } 
		
		else if (controllerState == "FOLLOWTHROUGH") {
			graphics->setCameraPose("camera_E_Pitcher", 
							Eigen::Vector3d(2.5, -8.8, 1.9),
							Eigen::Vector3d(0.0, 0.0, 1.0),
							Eigen::Vector3d(2.35, -7.8, 1.8));
		} 
    }
}

void computeZIntersectionWithPlane(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, Sai2Common::RedisClient& redis_client, std::shared_ptr<Sai2Graphics::Sai2Graphics> graphics) {
    double g = -9.81;
    double y0 = position.y();  // Initial y-coordinate
    double vy = velocity.y();  // Initial velocity in the y-direction
    double z0 = position.z();  // Initial z-coordinate
    double x = position.x();   // Initial x-coordinate (unchanged during flight atm, change later with 3d input)

    if (vy == 0) {
        cout << "Velocity in y-direction is zero, ball will not reach y = 0." << endl;
        return;
    }

    double t = -y0 / vy;

    if (t < 0) {
        cout << "The ball will not reach y = 0 in positive time." << endl;
        return;
    }

    double z = z0 + 0.5 * g * t * t; // Calculating z at time t

    double strikeZoneBottom = 0.15;
    double strikeZoneTop = 0.65;
    double strikeZoneLeft = 1.0;
    double strikeZoneRight = 1.4;

	if (x >= strikeZoneLeft && x <= strikeZoneRight && z >= strikeZoneBottom && z <= strikeZoneTop) {
		cout << "The ball will cross the strike zone at x = " << x << ", z = " << z << " at time t = " << t << " seconds." << endl;
		std::stringstream message;
		message << x << " " << z << " " << t;
		redis_client.set(BALL_TRAJECTORY, message.str());
		redis_client.setBool(NEW_BALL_SIGNAL, true);
		ball_launched = true;
		strike_thrown = true;
	} else {
		graphics->setCameraPose("camera_E_Pitcher", 
							Eigen::Vector3d(2.5, -2, 1),
							Eigen::Vector3d(0.0, 0.0, 1.0),
							Eigen::Vector3d(1.0, 0, 0.5));
		ball_launched = true;
		strike_thrown = false;
		cout << "The ball will miss the strike zone, crossing at x = " << x << ", z = " << z << " at time t = " << t << " seconds." << endl;
	}
}

Eigen::Vector3d mapVelocity(double input_vy, double input_vz) {
    // Map from [0, -5] to [-5, -10]
    double mapped_vy = (2*input_vy) - 5.0;

    // Clipping the mapped y-velocity to ensure it remains within the new bounds
    if (mapped_vy < -10) {
        mapped_vy = -10;
    } else if (mapped_vy > -5) {
        mapped_vy = -5;
    }

	double mapped_vz = 1.0 + (2*input_vz);
	if (input_vz < 1.0) {
		input_vz = 1.0;
	}

    // Return the new velocity vector with x and z set to zero
    return Eigen::Vector3d(0, mapped_vy, input_vz);
}