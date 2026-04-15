/**
 * @file Human.cpp
 * @author William Chong (wmchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2024-04-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "Human.h"

namespace Optitrack {

AngleAxisd orientationDiffAngleAxis(const Matrix3d& goal_orientation,
									const Matrix3d& current_orientation,
									const double scaling_factor = 1.0) {
	if (scaling_factor < 0 || scaling_factor > 1) {
		throw std::runtime_error(
			"Scaling factor must be between 0 and 1 in "
			"scaledOrientationErrorFromAngleAxis");
	}

	// expressed in base frame common to goal and current orientation
	AngleAxisd current_orientation_from_goal_orientation_aa(
		current_orientation * goal_orientation.transpose());

	return AngleAxisd(
		scaling_factor * current_orientation_from_goal_orientation_aa.angle(),
		current_orientation_from_goal_orientation_aa.axis());
}

Vector3d angleAxisToVector(const AngleAxisd& aa) {
	return aa.angle() * aa.axis();
}

Matrix3d reOrthogonalizeRotationMatrix(const Matrix3d& rotationMatrix) {
    // Perform Singular Value Decomposition (SVD)
    JacobiSVD<Matrix3d> svd(rotationMatrix, ComputeFullU | ComputeFullV);

    // Reconstruct the rotation matrix using the left singular vectors and right singular vectors
    Matrix3d orthoRotationMatrix = svd.matrixU() * svd.matrixV().transpose();

    return orthoRotationMatrix;
}

Human::Human(const std::vector<std::string>& link_names) {
    for (auto name : link_names) {
        m_initial_poses[name] = Affine3d::Identity();
    }

    m_filter = std::make_unique<SaiCommon::ButterworthFilter>(0.1);
}

// zero-pose calibration
void Human::calibratePose(const std::vector<std::string>& link_names,
                          const std::vector<Affine3d>& current_poses,
                          const int n_samples,
                          const bool reset) {
    if (reset) {
        for (auto name : link_names) {
            m_initial_poses[name] = Affine3d::Identity();
        }
    } 

    for (int i = 0; i < link_names.size(); ++i) {
        std::string name = link_names[i];
        m_initial_poses[name].translation() += (1. / n_samples) * current_poses[i].translation();
        m_initial_poses[name].linear() += (1. / n_samples) * current_poses[i].linear();
        m_initial_poses[name].linear() = reOrthogonalizeRotationMatrix(m_initial_poses[name].linear());  // arithmetic average with normalization
    }
}

// zero-pose calibration with one sample 
void Human::calibratePose(const std::vector<std::string>& link_names,
                          const std::vector<Affine3d>& current_poses,
                          const bool reset) {
    if (reset) {
        for (auto name : link_names) {
            m_initial_poses[name] = Affine3d::Identity();
        }
    } 

    for (int i = 0; i < link_names.size(); ++i) {
        std::string name = link_names[i];
        m_initial_poses[name].translation() = current_poses[i].translation();
        m_initial_poses[name].linear() = current_poses[i].linear();
    }
}

// get relative pose from the calibrated starting positions 
// assumes that the robot will never be recentered 
std::vector<Affine3d> Human::relativePose(const std::vector<std::string> link_names,
                                          const std::vector<Affine3d>& current_poses) {
    std::vector<Affine3d> relative_pose;
    for (int i = 0; i < link_names.size(); ++i) {
        std::string name = link_names[i];
        // std::cout << "Optitrack zero pose: \n" << m_initial_poses[name].linear() << "\n";
        // std::cout << "Rotation: \n" << m_rotations[name] << "\n";
        Affine3d transform_diff = Affine3d::Identity();

        // compute orientation difference
        transform_diff.linear() = m_rotations[name] * orientationDiffAngleAxis(m_initial_poses[name].linear(), current_poses[i].linear()) * m_rotations[name].transpose();

        // transform_diff.linear() = current_poses[i].linear() * m_initial_poses[name].linear().transpose();  // rotate from current to goal orientation
        // apply similarity transform to relative rotation motion 
        // from the right: rotate from robot to optitrack, apply rotation deviation in optitrack, rotate from optitrack to robot
        // transform_diff.linear() = reOrthogonalizeRotationMatrix(m_rotations[name] * transform_diff.linear() * m_rotations[name].transpose());

        transform_diff.translation() = m_rotations[name] * (current_poses[i].translation() - m_initial_poses[name].translation());
        relative_pose.push_back(transform_diff);
    }
    return relative_pose;
}

/*
    Getters and setters 
*/

/*
    Set rotation matrix to rotate from optitrack marker frame to robot link frame 
*/
void Human::setRotationReference(const std::string& name, const Matrix3d& R) {
    m_rotations[name] = R;
}

void Human::setMultiRotationReference(const std::vector<std::string>& link_names,
                                      const std::vector<Matrix3d>& R) {
    for (int i = 0; i < link_names.size(); ++i) {
        setRotationReference(link_names[i], R[i]);
    }
}

/*
    Get and set poses
*/
Affine3d Human::getInitialPose(const std::string name) {
    return m_initial_poses[name];
}

std::vector<Affine3d> Human::getMultiInitialPose(const std::vector<std::string>& link_names) {
    std::vector<Affine3d> poses;
    for (auto name : link_names) {
        poses.push_back(m_initial_poses[name]);
    }
    return poses;
}

void Human::setInitialPose(std::string name, const Affine3d& pose) {
    m_initial_poses[name] = pose;
}

void Human::setMultiInitialPose(const std::vector<std::string>& link_names,
                                const std::vector<Affine3d>& poses) {
    for (int i = 0; i < link_names.size(); ++i) {
        setInitialPose(link_names[i], poses[i]);
    }
}

}  // namespace 