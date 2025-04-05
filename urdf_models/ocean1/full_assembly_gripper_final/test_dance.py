#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np

def publish_joint_states():
    rospy.init_node('joint_state_publisher')

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(100)  # 10 Hz

    while not rospy.is_shutdown():
        joint_state = JointState()

        # Populate the joint state message
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()

        # List of joint names
        joint_state.name = [
            'head_base_1', 'head_1_2',

            'right_arm_base_1', 'right_arm_1_2', 'right_arm_2_3', 'right_arm_3_4', 'right_arm_4_5', 
            'left_arm_base_1', 'left_arm_1_2', 'left_arm_2_3', 'left_arm_3_4', 'left_arm_4_5',

            'right_arm_RightDistalJoint', 'right_arm_RightMedialJoint', 'right_arm_RightProximalJoint', 
            'right_arm_LeftDistalJoint', 'right_arm_LeftMedialJoint', 'right_arm_LeftProximalJoint',

            'left_arm_RightDistalJoint', 'left_arm_RightMedialJoint', 'left_arm_RightProximalJoint', 
            'left_arm_LeftDistalJoint', 'left_arm_LeftMedialJoint', 'left_arm_LeftProximalJoint'
        ]

        head_idces = [0, 1]
        right_arm_idces = list(range(2, 7))
        left_arm_idces = list(range(7, 12))
        right_hand_idces = list(range(12, 18))
        left_hand_idces = list(range(18, 24))

        # Example joint positions (replace with actual values)
        joint_state.position = np.array([0.0]*len(joint_state.name))
        eps = np.random.uniform(low=-1.0, high=1.0, size=joint_state.position.shape)

        joint_state.position += eps / 100.

        for head_idx in head_idces:
            joint_state.position[head_idx] *= 10.

        for right_hand_idx1, right_hand_idx2 in zip(*np.array_split(right_hand_idces, 2)):
            temp = (abs(joint_state.position[right_hand_idx1]) + abs(joint_state.position[right_hand_idx2])) / 2.
            if (right_hand_idx1-12) % 2 != 0:
                joint_state.position[right_hand_idx1] = joint_state.position[right_hand_idx2] = -100. * temp
            else:
                joint_state.position[right_hand_idx1] = joint_state.position[right_hand_idx2] = +50. * temp
        
        for left_hand_idx1, left_hand_idx2 in zip(*np.array_split(left_hand_idces, 2)):
            temp = (abs(joint_state.position[left_hand_idx1]) + abs(joint_state.position[left_hand_idx2])) / 2.
            if (left_hand_idx1-12) % 2 != 0:
                joint_state.position[left_hand_idx1] = joint_state.position[left_hand_idx2] = -100. * temp
            else:
                joint_state.position[left_hand_idx1] = joint_state.position[left_hand_idx2] = +50. * temp

        # Optionally, you can add velocity and effort if needed
        joint_state.velocity = []
        joint_state.effort = []

        # Publish the joint state
        pub.publish(joint_state)

        # Sleep to maintain the desired rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass