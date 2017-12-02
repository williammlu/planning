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
from intera_interface import Limb

from std_msgs.msg import String
import moveit_commander
import rospy
import sys
from copy import deepcopy
import tf
import pdb

rospy.init_node('default_position')
pose = None
moveit_commander.roscpp_initialize(sys.argv)

right_gripper = robot_gripper.Gripper('right')
robot = moveit_commander.RobotCommander()
right_arm = moveit_commander.MoveGroupCommander('right_arm')


def planning():

    rate = rospy.Rate(10) # 10hz
    limb = Limb('right_arm')
    default_joints = {'head_pan':-4.2551240234375, 'right_j0': -2.3731005859375, 'right_j1':-2.4028828125, 'right_j2':1.658787109375, 'right_j3': 0.7297041015625, 'right_j4':1.2216513671875, 'right_j5':0.31765625, 'right_j6':-4.6892177734375, 'torso_t0':0.0}
    
    #right_arm.set_joint_positions(default_joints)
    limb.set_joint_positions(default_joints)



if __name__ == '__main__':
    try:
        planning()
    except rospy.ROSInterruptException:
        pass
