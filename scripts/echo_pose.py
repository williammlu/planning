#!/usr/bin/env python
# license removed for brevity
from baxter_interface import gripper as robot_gripper
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
from copy import deepcopy
import tf
import pdb

rospy.init_node('echo_pose')
pose = None
moveit_commander.roscpp_initialize(sys.argv)

right_gripper = robot_gripper.Gripper('right')
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
right_arm = moveit_commander.MoveGroupCommander('right_arm')
# right_arm.set_planner_id('RRTConnectkConfigDefault')
right_arm.set_planning_time(20)
tfl = tf.TransformListener()


def planning():
    #Set up the right gripper
    rate = rospy.Rate(10) # 10hz
    def gripper_state_callback(message):
        global pose
        pose = message.pose
    def update_pose():
        global pose
        # .... some code here should take some time to let tfl updating its tf cache...
        tf = tfl.lookupTransform("base", "right_gripper",  rospy.Time(0))
        pose = Pose()
        pose.position.x = tf[0][0]
        pose.position.y = tf[0][1]
        pose.position.z = tf[0][2]

        pose.orientation.x = tf[1][0]
        pose.orientation.y = tf[1][1]
        pose.orientation.z = tf[1][2]
        pose.orientation.w = tf[1][3]
        print "base -> right_gripper pose"
        print str(pose)

    rospy.sleep(1)
    global pose

    while True:
        update_pose()
        rospy.sleep(1)




if __name__ == '__main__':
    try:
        planning()
    except rospy.ROSInterruptException:
        pass
