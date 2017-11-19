#!/usr/bin/env python
# license removed for brevity

#from baxter_interface import gripper as robot_gripper
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from intera_core_msgs.msg import (
    EndpointState
)
from moveit_msgs.msg import OrientationConstraint, Constraints

from intera_interface import gripper as robot_gripper
from std_msgs.msg import String
import moveit_commander
import rospy
import sys



rospy.init_node('planning', anonymous=True)
right_gripper = robot_gripper.Gripper('right')
pose = None

def planning():
    #Set up the right gripper

    rate = rospy.Rate(10) # 10hz
    def gripper_state_callback(message):
        # print(message.pose)
        global pose
        pose = message.pose
        
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, gripper_state_callback)
    # rospy.Subscriber("mouth_pose", Pose, callback)
    # rospy.Subscriber("marshmallow_pose", Pose, callback)
    # while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        # rate.sleep()
    while True:
        i = raw_input("Press enter to continue, type anything to exit")
        c_pose = pose
        print("Current pose {}".format(pose))
        if i:
            break
        do_grip()


        # fixed pose things

    # rospy.spin()

def do_grip():
    right_gripper.calibrate()
    rospy.sleep(2.0)
    c = 1
    MAX_TRIES=5
    right_gripper.open()

    while True:
        #Close the right gripper
        print('Closing...')
        right_gripper.close()
        rospy.sleep(1.0)
        print('Closed gripper')

        print("Gripper has {}N force applied.".format(right_gripper.get_force()))
        if right_gripper.is_gripping():
            print("Gripper has gotten marshmallow on try {}".format(c))
            # import pdb; pdb.set_trace()
            return True
        else:
            print("Failed attempt {}, trying again".format(c))
            

        c = c+1
        if c > MAX_TRIES:
            print("Failed after {} tries, quitting...".format(MAX_TRIES))
            return False

        # Open the right gripper
        print('Opening...')
        right_gripper.open()
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        planning()
    except rospy.ROSInterruptException:
        pass
