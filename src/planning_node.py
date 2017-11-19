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
from copy import deepcopy



rospy.init_node('planning', anonymous=True)
pose = None
moveit_commander.roscpp_initialize(sys.argv)


right_gripper = robot_gripper.Gripper('right')
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
right_arm = moveit_commander.MoveGroupCommander('right_arm')
right_arm.set_planner_id('RRTConnectkConfigDefault')
right_arm.set_planning_time(10)

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

    pose_i, pose_g = initialize()
    print("Pose I: {}".format(pose_i.position.z))
    print("Pose G: {}".format(pose_g.position.z))

    

    while True:
        i = raw_input("Press enter to continue, type anything to exit")
        print("Current pose {}".format(pose))
        if i:
            right_gripper.open()
            rospy.sleep(3.0)
            break
        move(pose_i, has_orientation_constraint=True)
        print("Current z: {}".format(pose.position.z))
        do_grip()
        move(pose_g, has_orientation_constraint=True)


        # fixed pose things

    # rospy.spin()

def initialize():
    right_gripper.open()
    start_pose = deepcopy(pose)
    end_pose = deepcopy(pose)
    end_pose.position.z += 0.1
    wait_time = 3.0
    move(end_pose,has_orientation_constraint=True)
    print('Finished initializing, wait {} seconds'.format(wait_time))
    rospy.sleep(wait_time)
    return start_pose, end_pose

def move(pose_g, has_orientation_constraint=True):
    right_arm.set_pose_target(pose_g)
    right_arm.set_start_state_to_current_state()

    if (has_orientation_constraint):
        orien_const = OrientationConstraint()
        orien_const.link_name = "right_gripper";
        orien_const.header.frame_id = "base";
        orien_const.orientation= pose.orientation
        orien_const.absolute_x_axis_tolerance = 0.1;
        orien_const.absolute_y_axis_tolerance = 0.1;
        orien_const.absolute_z_axis_tolerance = 0.05;
        orien_const.weight = 1.0;
        consts = Constraints()
        consts.orientation_constraints = [orien_const]
        right_arm.set_path_constraints(consts)

    right_plan = right_arm.plan()
    right_arm.execute(right_plan)



def do_grip():
    right_gripper.calibrate()
    rospy.sleep(1.0)
    c = 1
    MAX_TRIES=2
    right_gripper.open()

    while True:
        c_pose = pose
        print("Current pose {}".format(pose))
        #Close the right gripper
        print('Closing...')
        right_gripper.close()
        rospy.sleep(3.0)
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
            right_gripper.open()
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
