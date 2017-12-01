#!/usr/bin/env python
# license removed for brevity
from baxter_interface import gripper as robot_grippe
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
import tf


GRIPPER_LENGTH = 0.04 # TODO: fix temporary value of 4 cm after measuring
IDEAL_FEEDING_DIST = 0.05 # how many cm away from mouth should gripper go?

def main():
    rospy.init_node('broadcast_tf')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, GRIPPER_LENGTH),
                         (0.0, 0.0, 0.0, 0.0),
                         rospy.Time.now(),
                         "right_gripper_tip",
                         "right_gripper")
        br.sendTransform((0.0, 0.0, GRIPPER_LENGTH + IDEAL_FEEDING_DIST),
                         (0.0, 0.0, 0.0, 0.0),
                         rospy.Time.now(),
                        "ideal_mouth_tf",
                         "right_gripper")
        print("test")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
