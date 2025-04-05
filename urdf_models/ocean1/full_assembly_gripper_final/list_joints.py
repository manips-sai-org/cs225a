import rospy
from sensor_msgs.msg import JointState

def joint_states_callback(msg):
    joint_names = msg.name
    print("Joint names: ", joint_names)

def main():
    rospy.init_node('joint_names_printer')
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
