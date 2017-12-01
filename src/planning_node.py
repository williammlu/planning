# !/usr/bin/env python

import pdb

from copy import deepcopy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from intera_interface import gripper as robot_gripper
from moveit_msgs.msg import OrientationConstraint, Constraints

# from planning.srv import TriggerPhase

import numpy as np
import moveit_commander
import rospy
import sys
import tf

pose = None
rospy.init_node('planning')
moveit_commander.roscpp_initialize(sys.argv)
right_gripper = robot_gripper.Gripper('right')
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
right_arm = moveit_commander.MoveGroupCommander('right_arm')
# right_arm.set_planner_id('RRTConnectkConfigDefault')
right_arm.set_planning_time(20)
tfl = tf.TransformListener()
should_print_pose = False


def main():
    #Set up the right gripper

    rate = rospy.Rate(10) # 10hz
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
        if should_print_pose:
            print "base -> right_gripper tip pose\n" + str(pose)

    # rospy.Subscriber("mouth_pose", Pose, callback)
    # rospy.Subscriber("marshmallow_pose", Pose, callback)
    rospy.sleep(1)
    global pose

    update_pose()
    rospy.logdebug(str(pose))
    rospy.logdebug("--------")

    # rospy.Subscriber("some_location_idk", PointCloud2, callback_kinect)
    
    # for action in actions:
        # action.execute()
    # rospy.logdebug( "Action sequence finished")
    # return
            
     
    initialize()
    

    # s = rospy.Service('move_robot', TriggerPhase, move_robot)
    print "Launching service"
    while True:
        if raw_input("Type anything"):
            move_to_marshmallow()
        else:
            break

    rospy.spin()

def initialize_gripper():
    right_gripper.reboot()
    rospy.sleep(3.0)
    right_gripper.calibrate()
    #right_gripper.set_holding_force(10)
    rospy.sleep(3.0)
    assert right_gripper.is_ready()

def combine_transforms(trans1, rot1, trans2, rot2):
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat   = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)
    
    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat    = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    pdb.set_trace()
    trans3 = tf.transformations.translation_from_matrix(mat3)   
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return trans3, rot3

            
def execute_action_sequence(actions):
    for action in actions:
        action.execute()
    rospy.logdebug( "Action sequence finished")
    return 
     
# 
# def callback_kinect(data):
# 
    # count = 0
    # for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        # print("x : %f y : %f z: %f" % (p[0],p[1],p[2]))
        # scene.addCube("cube"+str(count), 0.01 ,p[0],p[1],p[2]) #create small cube constraint at each point 
    

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
    

def give_orientation(pose, orr_array):
    pose.orientation.x = orr_array[0]
    pose.orientation.y = orr_array[1]
    pose.orientation.z = orr_array[2]
    pose.orientation.w = orr_array[3]
def get_marshmallow_pose():
    tf_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (trans,rot) = tf_listener.lookupTransform('base', 'marshmellow_0', rospy.Time(0))
            rev_finger_trans, rev_finger_rot = tfl.lookupTransform("right_gripper", "right_gripper_tip", rospy.Time(0))

            combo_trans, combo_rot = combine_transforms(trans, rot, rev_finger_trans, rev_finger_rot)
            
            way_point_pose = Pose()
            way_point_pose.position.x = combo_trans[0]
            way_point_pose.position.y = combo_trans[1]
            way_point_pose.position.z = combo_trans[2] + 0.2
            way_point_pose.orientation = flip_quat(way_point_pose.orientation) # make sure to use appropriate coordinates

            marshmallow_pose = Pose()
            marshmallow_pose.position.x = combo_trans[0]
            marshmallow_pose.position.y = combo_trans[1]
            marshmallow_pose.position.z = combo_trans[2]

            vertical_dir= tf.transformations.quaternion_from_euler(0,0,-np.pi/2) # points in positive z direction

            give_orientation(way_point_pose, vertical_dir)
            give_orientation(marshmallow_pose, vertical_dir)
            

            print "Marshmallow Pose \n" + str(marshmallow_pose)
            return way_point_pose, marshmallow_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def get_mouth_pose():
    tf_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            mouth_pose = Pose()
            (trans,rot) = tf_listener.lookupTransform('base', 'face', rospy.Time(0))
            rev_mouth_trans, rev_mouth_rot = tfl.lookupTransform("right_gripper","ideal_mouth_tf", rospy.Time(0))

            combo_trans, combo_rot= combine_transforms(trans,rot, rev_mouth_trans, rev_mouth_rot)

            mouth_pose.position.x = combo_trans[0]
            mouth_pose.position.y = combo_trans[1]
            mouth_pose.position.z = combo_trans[2]
            mouth_pose.orientation = mouth_pose.orientation # TODO make sure you use real mouth orientations aka flipped kinect quaternion
            print "Mouth pose \n" + str(mouth_pose)
            return mouth_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def initialize():
    assert right_gripper.is_ready()
    right_gripper.open()
    actions = []

    # rospy.logdebug('Finished initializing, wait {} seconds'.format(2.0))
    rospy.sleep(2.0)

def move_robot(request):
    phase_id = request.phase
    print "phase_id is {}".format(phase_id)
    if phase_id == 0:
        success =  move_to_marshmallow()
    elif phase_id == 1:
        success =  move_to_mouth()
    elif phase_id == 2:
        success =  grip_marshmallow()
    elif phase_id == 3:
        success =  release_marshmallow()
    elif phase_id == 4:
        success =  move_to_initial_state()
    message = "placeholder"

    return TriggerPhaseResponse(success, message)


def move_to_marshmallow():
    way_point_pose, marshmallow_pose = get_marshmallow_pose()
    actions = []

    gripper_pose1 = Pose()
    gripper_pose1.position = way_point_pose.position
    gripper_pose1.orientation = flip_quat(way_point_pose.orientation)

    gripper_pose2 = Pose()
    gripper_pose2.position = marshmallow_pose.position
    gripper_pose2.orientation = flip_quat(marshmallow_pose.orientation)


    actions.append(Action(Action.GRIPPER, Action.OPEN))
    actions.append(Action(Action.MOVE_PRECISE, gripper_pose1))
    actions.append(Action(Action.MOVE, gripper_pose2))
    execute_action_sequence(actions)
    return True
    
def move_to_mouth():
    mouth_pose = get_mouth_pose()
    gripper_pose = Pose()
    gripper_pose.position = mouth_pose.postion
    gripper_pose.orientation = mouth_pose.orientation # don't flip since z goes into mouth

    actions = []
    actions.append(Action(Action.MOVE, gripper_pose))
    execute_action_sequence(actions)
    return True


def grip_marshmallow():
    actions = []
    actions.append(Action(Action.GRIPPER, Action.CLOSE))
    execute_action_sequence(actions)
    return True
    

def move_to_initial_state():
    # TODO
    return  True


def move(goal_pose, has_orientation_constraint=False, do_precise_movement=False):

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

    if do_precise_movement:
        right_arm.set_goal_position_tolerance(0.005) 
        right_arm.set_max_velocity_scaling_factor(0.25) # make it slow
        right_arm.set_num_planning_attempts(5) # take best of 5 for accuracy of 5mm
    else:
        right_arm.set_goal_position_tolerance(0.01) 
        right_arm.set_num_planning_attempts(3) # take best of 3 for accuracy of 1 cm
    print pose


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


class Action():
    GRIPPER=1
    MOVE=2
    FUNCTION=3
    MOVE_PRECISE = 4

    CLOSE = -1
    OPEN = -2
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
        elif self.action_type == Action.MOVE_PRECISE:
            rospy.logdebug( "PRECISE MOVEMENT TO " + str(self.value))

        elif self.action_type == Action.FUNCTION:
            rospy.logdebug( "RUNNING CUSTOM FUNCTION")
            self.value()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
