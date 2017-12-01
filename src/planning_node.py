#!/usr/bin/env python
# license removed for brevity

from baxter_interface import gripper as robot_gripper
from copy import deepcopy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from intera_core_msgs.msg import (EndpointState)
from intera_interface import gripper as robot_gripper
from moveit_msgs.msg import OrientationConstraint, Constraints
from std_msgs.msg import String
import moveit_commander
import pdb
import rospy
import sys
import tf

rospy.init_node('planning')
pose = None
moveit_commander.roscpp_initialize(sys.argv)

right_gripper = robot_gripper.Gripper('right')
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
right_arm = moveit_commander.MoveGroupCommander('right_arm')
# right_arm.set_planner_id('RRTConnectkConfigDefault')
right_arm.set_planning_time(20)
tfl = tf.TransformListener()

should_print_pose = False

def initialize_gripper():
    right_gripper.reboot()
    rospy.sleep(3.0)
    right_gripper.calibrate()
    #right_gripper.set_holding_force(10)
    rospy.sleep(3.0)
    assert right_gripper.is_ready()

def main():
    #Set up the right gripper

    rate = rospy.Rate(10) # 10hz
    def gripper_state_callback(message):
        global pose
        pose = message.pose
    def update_pose():
        global pose
        # .... some code here should take some time to let tfl updating its tf cache...
        tf = tfl.lookupTransform("base", "right_gripper_tip",  rospy.Time(0))
        pose = Pose()
        pose.position.x = tf[0][0]
        pose.position.y = tf[0][1]
        pose.position.z = tf[0][2]

        pose.orientation.x = tf[1][0]
        pose.orientation.y = tf[1][1]
        pose.orientation.z = tf[1][2]
        pose.orientation.w = tf[1][3]
        if should_print_pose:
            print "base -> right_gripper tip pose\n" + str(pose)

    # rospy.Subscriber("mouth_pose", Pose, callback)
    # rospy.Subscriber("marshmallow_pose", Pose, callback)
    rospy.sleep(1)
    global pose

    update_pose()
    rospy.logdebug(str(pose))
    rospy.logdebug("--------")

    actions = initialize()
    execute_action_sequence(actions)
    return
            
def execute_action_sequence(actions):
    for action in actions:
        action.execute()
    rospy.logdebug( "Action sequence finished")
    return 

     

def quat_to_euler(orientation):
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return (roll,pitch,yaw)

def euler_to_quat(roll, pitch, yaw):
    pose = Pose()
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    #type(pose) = geometry_msgs.msg.Pose
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose.orientation
    

"""
Reverse direction z basis vector, z axis corresponds by yaw
"""
def flip_quat(orientation):
    r,p,y = quat_to_euler(orientation)
    r += np.pi
    return euler_to_quat(r,p,y)
    

def initialize():
    assert right_gripper.is_ready()
    right_gripper.open()
    actions = []
    global pose


    tf_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (trans,rot) = tf_listener.lookupTransform('base', '/color_tracker_8', rospy.Time(0))
            ar_pose1 = Pose()
            ar_pose1.position.x = trans[0]
            ar_pose1.position.y = trans[1]
            ar_pose1.position.z = trans[2] + 0.2
            ar_pose1.orientation = flip_quat(ar_pose1.orientation)
            print "AR Pose 1 \n" + str(ar_pose1)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    while not rospy.is_shutdown():
        try:
            ar_pose2 = Pose()
            (trans,rot) = tf_listener.lookupTransform('base', '/color_tracker_0', rospy.Time(0))
            ar_pose2.position.x = trans[0]
            ar_pose2.position.y = trans[1]
            ar_pose2.position.z = trans[2] + 0.2
            ar_pose1.orientation = flip_quat(ar_pose2.orientation)
            print "AR Pose 2 \n" + str(ar_pose2)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    

    mm_pose = ar_pose1
    start_pose = deepcopy(pose)
    mouth_pose = ar_pose2
    # actions.append(Action(Action.FUNCTION, initialize_gripper))
    actions.append(Action(Action.MOVE, mm_pose))
    # actions.append(Action(Action.GRIPPER, Action.CLOSE))
    # actions.append(Action(Action.MOVE, start_pose))
    # actions.append(Action(Action.MOVE, mouth_pose))
    # actions.append(Action(Action.GRIPPER, Action.OPEN))
    # actions.append(Action(Action.MOVE, start_pose))
    # actions.append(Action(Action.MOVE, mm_pose))

    
    rospy.logdebug('Finished initializing, wait {} seconds'.format(2.0))
    rospy.sleep(wait_time)
    return actions

def move_to_marshmallow():
    tf_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try: # TODO: change to marshmallow tf, get from Brent
            (trans,rot) = tf_listener.lookupTransform('base', '/color_tracker_8', rospy.Time(0))
            ar_pose1 = Pose()
            ar_pose1.position.x = trans[0]
            ar_pose1.position.y = trans[1]
            ar_pose1.position.z = trans[2] + 0.2
            ar_pose1.orientation = flip_quat(ar_pose1.orientation)
            print "AR Pose 1 \n" + str(ar_pose1)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue



def move(goal_pose, has_orientation_constraint=False):

    right_arm.set_pose_target(goal_pose)
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

    right_arm.set_goal_position_tolerance(0.01) 
    right_arm.set_num_planning_attempts(3) # take best of 3 for accuracy of 5 mm
    print pose


    right_plan = right_arm.plan()
    right_arm.execute(right_plan)
    # plan,fraction = right_arm.compute_cartesian_path([],  0.01, 0.01)
    # print str(plan)
    # print str(fraction)
    # right_arm.execute(plan)



def do_grip():
    # c = 1
    # MAX_TRIES=5
    assert right_gripper.is_ready()
    right_gripper.open()
    rospy.sleep(2.0)
    assert right_gripper.is_ready()
    right_gripper.close()
    rospy.sleep(2.0)


class Action():
    GRIPPER=1
    MOVE=2
    FUNCTION=3

    CLOSE = 4
    OPEN = 5
    def __init__(self, action_type, value):
        self.action_type = action_type
        self.value = value
    
    def execute(self):
        if self.action_type == Action.GRIPPER:
            if self.value == Action.CLOSE:
                rospy.logdebug( "CLOSING GRIPPER")
                do_grip()
            elif self.value == Action.OPEN:
                rospy.logdebug( "OPENING GRIPPER")
                right_gripper.open()
        elif self.action_type == Action.MOVE:
            rospy.logdebug( "MOVING TO " + str(self.value))
            move(self.value)
        elif self.action_type == Action.FUNCTION:
            rospy.logdebug( "RUNNING CUSTOM FUNCTION")
            self.value()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
