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
import tf
import pdb

rospy.init_node('planning')
pose = None
moveit_commander.roscpp_initialize(sys.argv)

right_gripper = robot_gripper.Gripper('right')
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
right_arm = moveit_commander.MoveGroupCommander('right_arm')
right_arm.set_planner_id('RRTConnectkConfigDefault')
right_arm.set_planning_time(20)
tfl = tf.TransformListener()

def initialize_gripper():
    right_gripper.reboot()
    rospy.sleep(3.0)
    right_gripper.calibrate()
    #right_gripper.set_holding_force(10)
    rospy.sleep(3.0)
    assert right_gripper.is_ready()

def planning():
    #Set up the right gripper

    rate = rospy.Rate(10) # 10hz
    def gripper_state_callback(message):
        # print(message.pose)
        global pose
        pose = message.pose
    def update_pose():
        global pose
        # .... some code here should take some time to let tfl updating its tf cache...
        tf = tfl.lookupTransform("base", "right_gripper", rospy.Time(0))
        pose = Pose()
        pose.position.x = tf[0][0]
        pose.position.y = tf[0][1]
        pose.position.z = tf[0][2]

        pose.orientation.x = tf[1][0]
        pose.orientation.y = tf[1][1]
        pose.orientation.z = tf[1][2]
        pose.orientation.w = tf[1][3]

    # rospy.Subscriber("mouth_pose", Pose, callback)
    # rospy.Subscriber("marshmallow_pose", Pose, callback)
    rospy.sleep(1)
    global pose

    update_pose()
    print pose
    print("--------")

    #pose = right_arm.get_current_pose().pose
    #print pose
    #print("--------")

    #rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, gripper_state_callback)
    #rospy.sleep(1)
    #print pose
    #print("--------")
    #pose += 5
    # print pose
    # import time
    # time.sleep(1)
    # print pose
    # return
    # pose_i, pose_g = initialize()
    actions = initialize()
    # print("Pose I: {}".format(pose_i.position.z))
    # print("Pose G: {}".format(pose_g.position.z))

    
    for action in actions:
        action.execute()
    print "Action sequence finished"
    return
            
     


    # while True:
        # update_pose()
        # i = raw_input("Press enter to continue, type anything to exit")
        # if len(i) > 0:
            # right_gripper.open()
            # rospy.signal_shutdown(".")
            # exit()
# 
        # move(pose_i, has_orientation_constraint=True)
        # update_pose()
        # print("Current z {}".format(pose.position.z))
        # print("Pose I: {}".format(pose_i.position.z))
        # do_grip()
        # move(pose_g, has_orientation_constraint=True)
        # update_pose()
        # print("Current z: {}".format(pose.position.z))
        # print("Pose G: {}".format(pose_g.position.z))


        # fixed pose things

    # rospy.spin()

def initialize():
    assert right_gripper.is_ready()
    right_gripper.open()
    actions = []

    pose1 = deepcopy(pose)
    pose2 = deepcopy(pose)
    pose2.position.z += 0.4
    pose3 = deepcopy(pose)
    pose3.position.x += 0.2


    actions.append(Action(Action.MOVE, pose2))
    actions.append(Action(Action.FUNCTION, initialize_gripper))
    actions.append(Action(Action.MOVE, pose1))
    actions.append(Action(Action.GRIPPER, Action.CLOSE))
    actions.append(Action(Action.MOVE, pose2))
    actions.append(Action(Action.MOVE, pose3))
    actions.append(Action(Action.GRIPPER, Action.OPEN))
    actions.append(Action(Action.MOVE, pose2))
    actions.append(Action(Action.MOVE, pose1))

    
    wait_time = 2.0
    print('Finished initializing, wait {} seconds'.format(wait_time))
    rospy.sleep(wait_time)

    return actions

def move(pose, has_orientation_constraint=True):
    right_arm.set_pose_target(pose)
    right_arm.set_start_state_to_current_state()

    if (has_orientation_constraint):
        orien_const = OrientationConstraint()
        orien_const.link_name = "right_gripper";
        orien_const.header.frame_id = "base";
        orien_const.orientation= pose.orientation
        orien_const.absolute_x_axis_tolerance = 0.05;
        orien_const.absolute_y_axis_tolerance = 0.05;
        orien_const.absolute_z_axis_tolerance = 0.05;
        orien_const.weight = 1.0;
        consts = Constraints()
        consts.orientation_constraints = [orien_const]
        right_arm.set_path_constraints(consts)

    right_arm.set_goal_position_tolerance(0.005) 
    right_arm.set_num_planning_attempts(3) # take best of 3 for accuracy of 5 mm
    right_plan = right_arm.plan()
    right_arm.execute(right_plan)



def do_grip():
    # c = 1
    # MAX_TRIES=5
    assert right_gripper.is_ready()
    right_gripper.open()
    rospy.sleep(2.0)
    assert right_gripper.is_ready()
    right_gripper.close()
    rospy.sleep(2.0)

    # while True:
    #     #c_pose = pose
    #     #print("Current pose {}".format(pose))
    #     #Close the right gripper
    #     print('Closing...')
    #     right_gripper.close()
    #     rospy.sleep(1.0)
    #     return True

    #     print('Closed gripper')

    #     print("Gripper has {}N force applied.".format(right_gripper.get_force()))

    #     if True or right_gripper.is_gripping():
    #         print("Gripper has gotten marshmallow on try {}".format(c))
    #         # import pdb; pdb.set_trace()
    #         return True
    #     else:
    #         print("Failed attempt {}, trying again".format(c))
    #         

    #     c = c+1
    #     if c > MAX_TRIES:
    #         print("Failed after {} tries, quitting...".format(MAX_TRIES))
    #         right_gripper.open()
    #         return False

    #     # Open the right gripper
    #     print('Opening...')
    #     right_gripper.open()
    #     rospy.sleep(1.0)

class Action():
    # def __init__(self, position, grip_action):
        # self.position = position
        # self.grip_action = grip_action # 1: grip, 0: nothing, -1: release
    GRIPPER=1
    MOVE=2
    FUNCTION=3

    CLOSE = 1
    OPEN = -1
    def __init__(self, action_type, value):
        self.action_type = action_type
        self.value = value
    
    def execute(self):
        if self.action_type == Action.GRIPPER:
            if self.value == Action.CLOSE:
                print "CLOSING GRIPPER"
                do_grip()
            elif self.value == Action.OPEN:
                print "OPENING GRIPPER"
                right_gripper.open()
        elif self.action_type == Action.MOVE:
            print "MOVING TO " + str(self.value)
            move(self.value)
        elif self.action_type == Action.FUNCTION:
            print "RUNNING CUSTOM FUNCTION"
            self.value()





if __name__ == '__main__':
    try:
        planning()
    except rospy.ROSInterruptException:
        pass
