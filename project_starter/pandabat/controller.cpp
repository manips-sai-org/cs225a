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
using namespace Sai2Primitives;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#include "redis_keys.h"

// States 
enum State {
	POSTURE = 0, 
	MOTION
};

int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_bat.urdf";

	// initial state 
	int state = POSTURE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  // panda 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	const string control_link = "end-effector";
	const Vector3d control_point = Vector3d(0, 0, 0.4);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setPosControlGains(1000, 0, 0);
	pose_task->setOriControlGains(400, 0, 0);
	MatrixXd startOrientation;
	MatrixXd endOrientation;
	
	Vector3d startPosition;
	Vector3d startVelocity =Vector3d(0,0,0);
	Vector3d desired_endPosition;
	Vector3d desired_endVelocity;
	Vector3d trajectory;


	Vector3d axis;
	double thetaFinal;
	double tSwing = 1.0;
	double startTime;
	double curTime;
	MatrixXd traj = MatrixXd::Identity(12,12);
	traj << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			1, 0, 0, tSwing, 0, 0, tSwing*tSwing, 0, 0, tSwing*tSwing*tSwing, 0, 0,
			0, 1, 0, 0, tSwing, 0, 0, tSwing*tSwing, 0, 0, tSwing*tSwing*tSwing, 0,
			0, 0, 1, 0, 0, tSwing, 0, 0, tSwing*tSwing, 0, 0, tSwing*tSwing*tSwing, 
			0, 0, 0, 1, 0, 0, 2*tSwing, 0, 0, 3*tSwing*tSwing, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 2*tSwing, 0, 0, 3*tSwing*tSwing, 0, 
			0, 0, 0, 0, 0, 1, 0, 0, 2*tSwing, 0, 0, 3*tSwing*tSwing;
	
	//cout << traj << endl;


	// joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->setGains(400, 40, 2);

	VectorXd q_desired(dof);
	VectorXd q_initial(dof);
	q_initial = robot->q(); 
	q_desired = q_initial;
	q_desired(0) = -M_PI/2;
	joint_task->setGoalPosition(q_desired);
	//cout << robot->rotation(control_link);
	
	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

	while (runloop) {
		
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();
	
		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			command_torques = joint_task->computeTorques();

			if ((robot->q() - q_desired).norm() < 1e-2) {
				
				cout << "Reached Start Position" << endl;
				startPosition = robot->position(control_link,control_point);
				startOrientation = robot->rotation(control_link);
				endOrientation = MatrixXd::Zero(3,3);
				endOrientation(0,2) = 1;
				endOrientation(1,1) = -1;
				endOrientation(2,0) = 1;
				MatrixXd intermediateMatrix = startOrientation.transpose()*endOrientation;
				thetaFinal = acos((intermediateMatrix(0,0)+intermediateMatrix(1,1)+intermediateMatrix(1,1)-1)/2);
				axis = 1/(2*sin(thetaFinal))*Vector3d((intermediateMatrix(2,1) - intermediateMatrix(1,2)),(intermediateMatrix(0,2) - intermediateMatrix(2,0)),(intermediateMatrix(1,0) - intermediateMatrix(0,1)));
				//cout << thetaFinal << endl;
				//cout << axis.transpose() << endl;
				//cout << robot->rotation(control_link)*desired_orientation << endl;
				MatrixXd test = startOrientation * AngleAxisd(thetaFinal, axis).toRotationMatrix();
				cout << test << endl;
				state = MOTION;
				startTime = time;
			}
		} else if (state == MOTION) {
			// update goal position and orientation
			curTime = time;
			desired_endPosition = Vector3d(1.2, 0, .4);
			desired_endVelocity = Vector3d(0, 5, 1);
			VectorXd conditions(startPosition.size() + startVelocity.size() + desired_endPosition.size() + desired_endVelocity.size());
			conditions << startPosition, startVelocity, desired_endPosition, desired_endVelocity;
			VectorXd a = traj.lu().solve(conditions);
			double dt = curTime - startTime;
			trajectory << a(0) + a(3)*dt + a(6)*dt*dt + a(9)*dt*dt*dt,
						  a(1) + a(4)*dt + a(7)*dt*dt + a(10)*dt*dt*dt,
						  a(2) + a(5)*dt + a(8)*dt*dt + a(11)*dt*dt*dt;
			cout << trajectory.transpose() << " " << dt << endl;
			pose_task->reInitializeTask();
			joint_task->reInitializeTask();
			pose_task->setGoalPosition(trajectory);
			pose_task->setGoalOrientation(endOrientation);
				
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();
			if ((robot->position(control_link, control_point) - desired_endPosition).norm() < 1e-1){
				cout << "reached new pos" << endl;
			}
		}

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		
	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}

