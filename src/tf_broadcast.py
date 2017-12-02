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


GRIPPER_LENGTH = 0.07 # 7cm long
IDEAL_FEEDING_DIST = 0.05 # how many cm away from mouth should gripper go?
tf_listener = tf.TransformListener()

def main():
    rospy.init_node('broadcast_tf')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        m_pos, _ = tf_listener.lookupTransform('base', 'marshmellow_0', rospy.Time(0))
        face_pos, face_orr = tf_listener.lookupTransform('base', 'marshmellow_0', rospy.Time(0))

        if m_pos:
            br.sendTransform((m_pos.x, m_pos.y, m_pos.z + 0.10 + GRIPPER_LENGTH),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "marshmallow_waypoint_goal",
                             "base")
            br.sendTransform((m_pos.x, m_pos.y, m_pos.z + GRIPPER_LENGTH),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "marshmallow_final_goal",
                             "base")
        else:
            print("No marshmallow_pose detected")
        
e
        if face_pos:
            br.sendTransform((0.0, 0.0, IDEAL_FEEDING_DIST + GRIPPER_LENGTH),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "face_gripper_goal",
                             "face")
        else:
            print("No face detected")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
