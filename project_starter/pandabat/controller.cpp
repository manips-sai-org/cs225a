/**
 * @file controller.cpp
 * @brief Controller file
 *
 */

#include <Sai2Model.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream> 

#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
using namespace std;
using namespace Eigen;
using namespace Sai2Primitives;

#include <signal.h>
bool runloop = false;
void sighandler(int) { runloop = false; }

#include "redis_keys.h"

// States
enum State {
    POSTURE = 0,
    MOTION,
    FOLLOWTHROUGH,
    RESET,
    WAITING1,
    WAITING2
};

void parseTrajectoryMessage(const string& message, Vector3d& position, double& timeToZone) {
    stringstream ss(message);
    double x, z, t;
    if (ss >> x >> z >> t) {
        position.x() = x;
        position.z() = z;
        position.y() = 0.0;  // Since strike zone is in the xz plane
    }
    timeToZone = t;
}

string stateToString(int state) {
    switch (state) {
        case POSTURE:
            return "POSTURE";
        case MOTION:
            return "MOTION";
        case FOLLOWTHROUGH:
            return "FOLLOWTHROUGH";
        case RESET:
            return "RESET";
        case WAITING1:
            return "WAITING1";
	    case WAITING2:
            return "WAITING2";
        default:
            return "UNKNOWN";
    }
}

Eigen::Matrix3d orthogonalize(Eigen::Matrix3d& mat) {
    // Perform SVD decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Get the U matrix, which has orthogonal columns
    return svd.matrixU() * svd.matrixV().transpose();
}

void orthonormalize(Eigen::Matrix3d& rot) {
    Vector3d x = rot.col(0);
    Vector3d y = rot.col(1);
    Vector3d z = rot.col(2);

    // normalize
    x.normalize();
    y.normalize();
    z.normalize();

    // orthogonalize
    double errorXY = 0.5 * x.dot(y);
    double errorYZ = 0.5 * y.dot(z);
    double errorZX = 0.5 * z.dot(x);
    rot.col(0) = x - errorXY * y - errorZX * z;
    rot.col(1) = y - errorXY * x - errorYZ * z;
    rot.col(2) = z - errorYZ * y - errorZX * x;
}

Vector3d computeAxis(double theta, Matrix3d m) {
    return 1 / (2 * sin(theta)) * Vector3d((m(2, 1) - m(1, 2)), (m(0, 2) - m(2, 0)), (m(1, 0) - m(0, 1)));
}

MatrixXd computeTrajMatrix(double tSwing) {
    MatrixXd traj = MatrixXd::Identity(12, 12);
    traj << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        1, 0, 0, tSwing, 0, 0, tSwing * tSwing, 0, 0, tSwing * tSwing * tSwing, 0, 0,
        0, 1, 0, 0, tSwing, 0, 0, tSwing * tSwing, 0, 0, tSwing * tSwing * tSwing, 0,
        0, 0, 1, 0, 0, tSwing, 0, 0, tSwing * tSwing, 0, 0, tSwing * tSwing * tSwing,
        0, 0, 0, 1, 0, 0, 2 * tSwing, 0, 0, 3 * tSwing * tSwing, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 2 * tSwing, 0, 0, 3 * tSwing * tSwing, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 2 * tSwing, 0, 0, 3 * tSwing * tSwing;
    return traj;
}

Vector3d calculateTrajectory(VectorXd a, double dt) {
    return Vector3d(a(0) + a(3) * dt + a(6) * dt * dt + a(9) * dt * dt * dt,
                    a(1) + a(4) * dt + a(7) * dt * dt + a(10) * dt * dt * dt,
                    a(2) + a(5) * dt + a(8) * dt * dt + a(11) * dt * dt * dt);
}

VectorXd calculateVelocity(VectorXd a, double dt) {
    return Vector3d(a(3) + a(6) * 2 * dt + a(9) * 3 * dt * dt,
                    a(4) + a(7) * 2 * dt + a(10) * 3 * dt * dt,
                    a(5) + a(8) * 2 * dt + a(11) * 3 * dt * dt);
}
VectorXd calculateAcceleration(VectorXd a, double dt) {
    return Vector3d(a(6) * 2 + a(9) * 6 * dt,
                    a(7) * 2 + a(10) * 6 * dt,
                    a(8) * 2 + a(11) * 6 * dt);
}

int main() {
    // Location of URDF files specifying world and robot information
    static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_bat.urdf";

    // initial state
    int state = POSTURE;
    string controller_status = "1";
    int last_state = -1;  // Initialize with an invalid state

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
    redis_client.setBool(NEW_BALL_SIGNAL, false);

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
    pose_task->setPosControlGains(500, 30, 0);
    pose_task->setOriControlGains(150, 30, 0);
    MatrixXd startOrientation;
    MatrixXd endOrientation;
    Matrix3d desired_orientation;

    // Rotation Matrices of Bat pointing in primary vector directions
    Matrix3d posX = Matrix3d::Zero(3, 3);
    posX(0, 2) = 1;
    posX(1, 1) = -1;
    posX(2, 0) = 1;
    Matrix3d posY = MatrixXd::Zero(3, 3);
    posY(0, 1) = 1;
    posY(1, 2) = 1;
    posY(2, 0) = 1;
    Matrix3d negY = MatrixXd::Zero(3, 3);
    negY(0, 1) = -1;
    negY(1, 2) = -1;
    negY(2, 0) = 1;

    Vector3d startPosition;
    Vector3d start;
    Vector3d startVelocity;
    Vector3d desired_endPosition;
    Vector3d desired_intermediatePoint;
    Vector3d desired_endVelocity;
    Vector3d trajectory;
    Vector3d velocity;
    Vector3d acceleration;

    Vector3d axis;
    double thetaFinal;
    double tSwing = 0.5;
    double tReset = 5;
    double tFollow = 2;
    double startTime;
    double curTime;
    double bufferTime = 0.0;

    MatrixXd traj;
    VectorXd a;

    Vector3d ball_position;
    double timeToZone;

    // joint task
    auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
    joint_task->setGains(500, 30, 0);

    VectorXd q_desired(dof);
    VectorXd q_initial(dof);
    q_initial = robot->q();
    q_desired = q_initial;
    q_desired(0) = -M_PI / 2;
    joint_task->setGoalPosition(q_desired);

    // create a loop timer`
    runloop = true;
    double control_freq = 1000;
    double data_freq = 120;       // Position data frequency in Hz
    Sai2Common::LoopTimer timer(control_freq, 1e6);
    deque<Vector3d> position_history;
	deque<double> time_history;

    long long int data_counter = 0;
    int data_interval = static_cast<int>(control_freq / data_freq);

    // Step 1: Calibration
	Vector3d OS_X_0 = Eigen::Vector3d(1.2, 5.0, 1.4);
	Matrix3d OS_R_0 = robot->rotation(control_link);

	Matrix3d OS_R_OT;
	OS_R_OT << -1, 0, 0,
				0, 0, 1,
				0, 1, 0;
	OS_R_OT = AngleAxisd(M_PI, Vector3d (0,0,1)).toRotationMatrix() * OS_R_OT;
	
	Vector3d OT_X_0 = redis_client.getEigen(RIGID_BODY_POS);
	Vector4d OT_Q_0 = redis_client.getEigen(RIGID_BODY_ORI);
	Quaterniond quaternion(OT_Q_0[3], OT_Q_0[0], OT_Q_0[1], OT_Q_0[2]);
	Matrix3d OT_R_0 = quaternion.toRotationMatrix(); 	// Convert the quaternion to a rotation matrix


    ofstream trajectoryData;
    trajectoryData.open("../../project_starter/pandabat/trajectoryData.txt");

    while (runloop) {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();

        // update robot
        robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
        robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
        robot->updateModel();

        Vector3d OS_Xd;
		Matrix3d OS_Rd;


		if (data_counter % data_interval == 0 ) {

			// Step 2: relative position and orientation read 
			Vector3d pos = redis_client.getEigen(RIGID_BODY_POS);
			Vector3d OT_X = pos;

			Vector4d quat = redis_client.getEigen(RIGID_BODY_ORI);
			Quaterniond quatObj(quat[3], quat[0], quat[1], quat[2]);
			Matrix3d OT_R = quatObj.toRotationMatrix();

			Vector3d OT_X_rel = OT_X - OT_X_0;
			Matrix3d OT_R_rel = OT_R.transpose() * OT_R_0;

			// Step 3: tracking position and orientation in OpenSai Frame
			Vector3d OS_X = OS_X_0 + OS_R_OT * OT_X_rel;
			Matrix3d OS_R = OS_R_OT * OT_R_rel * OS_R_OT.transpose() * OS_R_0;

			OS_Xd = OS_X;
			OS_Rd = OS_R;

			// Add position and time to history
			position_history.push_back(OS_X);
			time_history.push_back(time);

			// Ensure we have enough data points to compute the velocity
			if (position_history.size() > 1) {
				// Compute the finite difference for velocity
				Vector3d pos_prev = position_history[position_history.size() - 2];
				double time_prev = time_history[time_history.size() - 2];

				double dt = time - time_prev;
				
				Vector3d velocity = (OS_X - pos_prev) / dt;

                redis_client.setEigen(BALL_POS,OS_X);
                redis_client.setEigen(BALL_VEL, velocity);

			// Keep only the latest 2 positions and times
			if (position_history.size() > 2) {
				position_history.pop_front();
				time_history.pop_front();
			}
        }
		}

        data_counter++;


        // Print state name only on state change
        if (state != last_state) {
            cout << "Current State: " << stateToString(state) << endl;
            redis_client.set(CONTROLLER_STATE, stateToString(state));
            last_state = state;  // Update last_state to the current state
        }

        // See if a ball has been launched to the strike zone
        bool new_ball_ready = redis_client.getBool(NEW_BALL_SIGNAL);

        if (state == POSTURE) {
			// update task model
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			command_torques = joint_task->computeTorques();
			if ((robot->q() - q_desired).norm() < 1e-2) {
				cout << "Reached Start Position" << endl;
				state = WAITING1;
				// Initialize or update the starting position and orientation of the end-effector,
        		// which will be used in subsequent states for motion planning.
				startPosition = robot->position(control_link, control_point);
				startOrientation = robot->rotation(control_link);
                startTime = time;
			}
		} else if (state == WAITING1) {
			// Continuously update the robot's position to maintain the ready posture
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			command_torques = joint_task->computeTorques();

			// Check if a new ball trajectory is ready
			if (new_ball_ready) {
				std::string trajectory_str = redis_client.get(BALL_TRAJECTORY);
				parseTrajectoryMessage(trajectory_str, ball_position, timeToZone);
				cout << "Trigger Swing. Ball trajectory: " << trajectory_str << endl;
                if (timeToZone <= tSwing + bufferTime){
                    // Reset the new ball signal
                    redis_client.setBool(NEW_BALL_SIGNAL, false);

                    // set tSwing to timeToZone
                    tSwing = timeToZone - bufferTime;

                    // Re-update starting position and velocity based on current state
                    start = robot->position(control_link, control_point);
                    startVelocity = robot->linearVelocity(control_link, control_point);

                    // Calculate axis of rotation for the motion phase
                    MatrixXd intermediateMatrix = startOrientation.transpose() * posX;
                    thetaFinal = acos((intermediateMatrix(0, 0) + intermediateMatrix(1, 1) + intermediateMatrix(1, 1) - 1) / 2);
                    axis = computeAxis(thetaFinal, intermediateMatrix);

                    // Prepare conditions for motion trajectory
                    desired_endPosition = Vector3d(ball_position.x(), 0.0, ball_position.z()); 
                    desired_endVelocity = Vector3d(-3, 15, 1.5);  // Trying out increasing the velocity since right now the hit is very weak
                    VectorXd conditions(startPosition.size() + startVelocity.size() + desired_endPosition.size() + desired_endVelocity.size());
                    conditions << start, startVelocity, desired_endPosition, desired_endVelocity;
                    traj = computeTrajMatrix(tSwing);
                    a = traj.lu().solve(conditions);

                    // Transition to motion
                    state = MOTION;
                    pose_task->reInitializeTask();
                    joint_task->reInitializeTask();
                    startTime = time;
                } else {
                    startTime = time;
                    state = WAITING2;
                }
			}
        } else if (state == WAITING2){
            // Continuously update the robot's position to maintain the ready posture
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			command_torques = joint_task->computeTorques();
            curTime = time;
            double dt = curTime - startTime;
            if (timeToZone - dt <= tSwing + bufferTime){
                // Reset the new ball signal
                redis_client.setBool(NEW_BALL_SIGNAL, false);

                // Re-update starting position and velocity based on current state
                start = robot->position(control_link, control_point);
                startVelocity = robot->linearVelocity(control_link, control_point);

                // Calculate axis of rotation for the motion phase
                MatrixXd intermediateMatrix = startOrientation.transpose() * posX;
                thetaFinal = acos((intermediateMatrix(0, 0) + intermediateMatrix(1, 1) + intermediateMatrix(1, 1) - 1) / 2);
                axis = computeAxis(thetaFinal, intermediateMatrix);

                // Prepare conditions for motion trajectory
                desired_endPosition = Vector3d(ball_position.x(), 0.0, ball_position.z()); 
                desired_endVelocity = Vector3d(-3, 15, 1.5);  // Trying out increasing the velocity since right now the hit is very weak
                VectorXd conditions(startPosition.size() + startVelocity.size() + desired_endPosition.size() + desired_endVelocity.size());
                conditions << start, startVelocity, desired_endPosition, desired_endVelocity;
                traj = computeTrajMatrix(tSwing);
                a = traj.lu().solve(conditions);

                // Transition to motion
                state = MOTION;
                pose_task->reInitializeTask();
                joint_task->reInitializeTask();
                cout << "Trigger Swing. Ball trajectory: " << ball_position.transpose() << endl;
                startTime = time;
                
            }
		} else if (state == MOTION) {
            // update goal position, velocity, accleration
            curTime = time;
            double dt = curTime - startTime;
            trajectory = calculateTrajectory(a, dt);
            velocity = calculateVelocity(a, dt);
            acceleration = calculateAcceleration(a, dt);
            pose_task->setGoalPosition(trajectory);
            pose_task->setGoalLinearVelocity(velocity);
            pose_task->setGoalLinearAcceleration(acceleration);
            // update goal orientation:
            desired_orientation = startOrientation * AngleAxisd((thetaFinal) * dt / tSwing, axis).toRotationMatrix();
            // orthonormalize(desired_orientation);
            desired_orientation = orthogonalize(desired_orientation);
            pose_task->setGoalOrientation(desired_orientation);
            // update joint position to avoid joint limits
            q_desired(0) = M_PI / 2 * dt / tSwing - M_PI / 2;
            joint_task->setGoalPosition(q_desired);

            // turn off velocity saturation and internal trajectory generation
            pose_task->disableInternalOtg();
            pose_task->disableVelocitySaturation();
            // update task model
            N_prec.setIdentity();
            pose_task->updateTaskModel(N_prec);
            joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
            command_torques = pose_task->computeTorques() + joint_task->computeTorques();
            trajectoryData << robot->position(control_link, control_point).transpose() << " " << trajectory.transpose() << " " << dt << '\n';
            // cout << command_torques.transpose() << endl;
            if (dt > tSwing) {
                cout << tSwing << " Seconds Passed" << endl;
                cout << robot->position(control_link, control_point).transpose() << " " << ball_position.transpose() << endl;
                // Update Starting Position and Velocity
                startPosition = robot->position(control_link, control_point);
                startVelocity = robot->linearVelocity(control_link, control_point);
                // Update Starting Orienation and calculate axis of rotation for next state
                startOrientation = robot->rotation(control_link);
                MatrixXd intermediateMatrix = startOrientation.transpose() * posY;
                thetaFinal = acos((intermediateMatrix(0, 0) + intermediateMatrix(1, 1) + intermediateMatrix(1, 1) - 1) / 2);
                axis = computeAxis(thetaFinal, intermediateMatrix);
                // Update desired end position and velocity of next state
                desired_endPosition = Vector3d(0, 1.2, ball_position.z());
                desired_endVelocity = Vector3d(0, 0, 0);
                VectorXd conditions(startPosition.size() + startVelocity.size() + desired_endPosition.size() + desired_endVelocity.size());
                conditions << startPosition, startVelocity, desired_endPosition, desired_endVelocity;
                traj = computeTrajMatrix(tFollow);
                a = traj.lu().solve(conditions);
                state = FOLLOWTHROUGH;
                startTime = time;
            }
        } else if (state == FOLLOWTHROUGH) {
            // update goal position, velocity, accleration
            curTime = time;
            double dt = curTime - startTime;
            trajectory = calculateTrajectory(a, dt);
            velocity = calculateVelocity(a, dt);
            acceleration = calculateAcceleration(a, dt);
            pose_task->setGoalPosition(trajectory);
            pose_task->setGoalLinearVelocity(velocity);
            pose_task->setGoalLinearAcceleration(acceleration);
            // update goal orientation:
            desired_orientation = startOrientation * AngleAxisd(thetaFinal * dt / tFollow, axis).toRotationMatrix();
            // orthonormalize(desired_orientation);
            desired_orientation = orthogonalize(desired_orientation);
            pose_task->setGoalOrientation(desired_orientation);
            // update joint position to avoid joint limits
            q_desired(0) = M_PI / 2 * dt / tFollow;
            joint_task->setGoalPosition(q_desired);

            // turn off velocity saturation and internal trajectory generation
            pose_task->disableInternalOtg();
            pose_task->disableVelocitySaturation();
            // update task model
            N_prec.setIdentity();
            pose_task->updateTaskModel(N_prec);
            joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
            command_torques = pose_task->computeTorques() + joint_task->computeTorques();
            /**/
            if (dt > tFollow) {
                cout << tFollow << " seconds passed" << endl;
                // Update Starting Orienation and calculate axis of rotation for next state
                startOrientation = robot->rotation(control_link);
                MatrixXd intermediateMatrix = startOrientation.transpose() * negY;
                thetaFinal = acos((intermediateMatrix(0, 0) + intermediateMatrix(1, 1) + intermediateMatrix(1, 1) - 1) / 2);
                axis = computeAxis(thetaFinal, intermediateMatrix);
                state = RESET;
                startTime = time;
            }
        } else if (state == RESET) {
            curTime = time;
            double dt = curTime - startTime;
            trajectory = Vector3d(1.2 * sin(M_PI * dt / tReset), 1.2 * cos(M_PI * dt / tReset), .4);
            velocity = Vector3d(1.2 * M_PI / tReset * cos(M_PI * dt / tReset), -1.2 * M_PI * dt / tReset * sin(M_PI * dt / tReset), .4);
            acceleration = Vector3d(-1.2 * M_PI * M_PI / tReset / tReset * sin(M_PI * dt / tReset), -1.2 * M_PI * M_PI / tReset / tReset * cos(M_PI * dt / tReset), .4);
            pose_task->setGoalPosition(trajectory);
            pose_task->setGoalLinearVelocity(velocity);
            pose_task->setGoalLinearAcceleration(acceleration);
            // update goal orientation:
            desired_orientation = startOrientation * AngleAxisd(thetaFinal * dt / tReset, axis).toRotationMatrix();
            // orthonormalize(desired_orientation);
            desired_orientation = orthogonalize(desired_orientation);
            pose_task->setGoalOrientation(desired_orientation);
            // update joint position to avoid joint limits
            q_desired(0) = M_PI / 2 - M_PI * dt / tReset;
            joint_task->setGoalPosition(q_desired);
            N_prec.setIdentity();
            pose_task->updateTaskModel(N_prec);
            joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
            // turn off velocity saturation and internal trajectory generation
            pose_task->disableInternalOtg();
            pose_task->disableVelocitySaturation();
            // update task model
            N_prec.setIdentity();
            pose_task->updateTaskModel(N_prec);
            joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
            command_torques = pose_task->computeTorques() + joint_task->computeTorques();
            if (dt > tReset) {
                cout << "full cycle complete!" << endl;
                redis_client.setBool(NEW_BALL_SIGNAL, false);
                tSwing = 0.5;
                state = POSTURE;
            }
        }


        // execute redis write callback
        redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    }
    trajectoryData.close();

    timer.stop();
    cout << "\nSimulation loop timer stats:\n";
    timer.printInfoPostRun();
    redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

    return 0;
}