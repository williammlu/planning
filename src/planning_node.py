# !/usr/bin/env python

import pdb

from copy import deepcopy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from intera_interface import gripper as robot_gripper
from moveit_msgs.msg import OrientationConstraint, Constraints

from planning.srv import TriggerPhase

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

saved_marshmallow_pose = None
saved_mouth_pose = None


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

    rospy.sleep(1)
    global pose

    update_pose()
    rospy.logdebug(str(pose))
    rospy.logdebug("--------")

    initialize()
    s = rospy.Service('move_robot', TriggerPhase, move_robot)
    print "Launching service"


    # FOR TESTING
    while True:
        if raw_input("Prepare to move"):
            save_poses() # save current poses
            move_to_marshmallow()
            grip_marshmallow()
            raw_input("Prepare to move to mouth")
            move_to_mouth()
            delete_poses() # clear poses
        else:
            break

    rospy.spin()

def initialize_gripper():
    right_gripper.reboot()
    rospy.sleep(3.0)
    print("calibrating gripper...")
    right_gripper.calibrate()
    print("done_calibrating gripper...")
    rospy.sleep(2.0)
    #right_gripper.set_holding_force(10)
    assert right_gripper.is_ready()
    right_gripper.open()



     
# 
# def callback_kinect(data):
# 
    # count = 0
    # for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        # print("x : %f y : %f z: %f" % (p[0],p[1],p[2]))
        # scene.addCube("cube"+str(count), 0.01 ,p[0],p[1],p[2]) #create small cube constraint at each point 
    


def get_marshmallow_pose(should_remember=False):
    """
    Returns saved marshmallow poses if available
    Otherwise wait for both waypoint and final marshmallow tfs.
    """
    global saved_marshmallow_pose
    if saved_marshmallow_pose:
        return saved_marshmallow_pose
    tf_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            waypoint_pos, _ = tf_listener.lookupTransform('base', 'marshmallow_waypoint_goal', rospy.Time(0))

            goal_pos, _ = tf_listener.lookupTransform('base', 'marshmallow_final_goal', rospy.Time(0))

            way_point_pose = Pose()
            way_point_pose.position.x = waypoint_pos[0]
            way_point_pose.position.y = waypoint_pos[1]
            way_point_pose.position.z = waypoint_pos[2]

            marshmallow_pose = Pose()
            marshmallow_pose.position.x = goal_pos[0]
            marshmallow_pose.position.y = goal_pos[1]
            marshmallow_pose.position.z = goal_pos[2]

            vertical_dir= tf.transformations.quaternion_from_euler(0,0,-np.pi/2) # points in positive z direction

            give_orientation(way_point_pose, vertical_dir)
            give_orientation(marshmallow_pose, vertical_dir)


            print "Marshmallow Pose \n" + str(marshmallow_pose)
            if should_remember:
                saved_marshmallow_pose = (waypoint_pos, marshmallow_pose)
                print "Saved marshmallow pose"
            return way_point_pose, marshmallow_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def get_mouth_pose(should_remember=False):
    """
    Returns saved mouth tf as pose if available
    Otherwise wait to return mouth tf as Pose when available
    """
    global saved_mouth_pose
    if saved_mouth_pose:
        return saved_mouth_pose
    tf_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            mouth_pose = Pose()
            (trans,rot) = tf_listener.lookupTransform('base', 'face_gripper_goal', rospy.Time(0))

            mouth_pose.position.x = trans[0]
            mouth_pose.position.y = trans[1]
            mouth_pose.position.z = trans[2]

            give_orientation(mouth_pose, rot)

            print "Mouth pose \n" + str(mouth_pose)
            if should_remember:
                saved_mouth_pose = mouth_pose
                print "Saved mouth pose"
            return mouth_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def initialize():
    """
    Initializes gripper and checks if it is ready
    """
    initialize_gripper()
    assert right_gripper.is_ready()
    print("Initialized  gripper")
    rospy.sleep(2.0)


def move_robot(request):
    """
    Callback for ROS service. Does one of several predefined actions
    """

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
        success =  move_to_start_state()
    elif phase_id == 5:
        success =  perform_full_sequence()
    message = "placeholder"

    return TriggerPhaseResponse(success, message)

def save_poses():
    """
    Save marshmallow and mouth poses globally
    """
    get_marshmallow_pose(should_remember=True)
    get_mouth_pose(should_remember=True)
    rospy.sleep(1)
    print "Finished saving poses"

def delete_poses():
    """
    Nullify marshmallow and mouth poses globally
    """
    global saved_marshmallow_pose
    global saved_mouth_pose
    saved_marshmallow_pose = None
    saved_mouth_pose = None
    print "Deleted poses"


def perform_full_sequence():
    move_to_start_state()
    move_to_marshmallow()
    move_to_mouth()
    grip_marshmallow()
    release_marshmallow()

def move_to_marshmallow():
    """
    Two phase motion to move gripper to marshmallow.
    1 - Move with lesser precision to a waypoint above the marshmallow
    2 - Move with high precision from waypoint to optimal gripping position
    """
    way_point_pose, marshmallow_pose = get_marshmallow_pose()
    actions = []

    gripper_pose1 = Pose()
    gripper_pose1.position = way_point_pose.position
    gripper_pose1.orientation = flip_quat(way_point_pose.orientation)
    print "Way point pose"
    print str(way_point_pose)

    gripper_pose2 = Pose()
    gripper_pose2.position = marshmallow_pose.position
    gripper_pose2.orientation = flip_quat(marshmallow_pose.orientation)
    print "marshmallow pose"
    print str(marshmallow_pose)


    actions.append(Action(Action.GRIPPER, Action.OPEN))
    actions.append(Action(Action.MOVE, gripper_pose1))
    actions.append(Action(Action.FUNCTION, lambda: rospy.sleep(2)))
    actions.append(Action(Action.MOVE_PRECISE, gripper_pose2))
    # pdb.set_trace()
    execute_action_sequence(actions)
    return True

def move_to_mouth():
    """
    Moves gripper to mouth position. Obtains mouth goal pose (which could be saved) and moves to it.
    """
    mouth_pose = get_mouth_pose()
    gripper_pose = Pose()
    gripper_pose.position = mouth_pose.position
    gripper_pose.orientation = mouth_pose.orientation # don't flip since z goes into mouth

    actions = []
    actions.append(Action(Action.MOVE, gripper_pose))
    execute_action_sequence(actions)
    return True


def grip_marshmallow():
    """
    Closes gripper
    """
    actions = []
    actions.append(Action(Action.GRIPPER, Action.CLOSE))
    execute_action_sequence(actions)
    return True


def move(goal_pose, has_orientation_constraint=False, do_precise_movement=False):
    """
    Move end effector to a goal pose. Can enforce goal position and orientation constraints.
    """
    import pdb; pdb.set_trace()
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
        right_arm.set_max_velocity_scaling_factor(0.10) # make it slow
        right_arm.set_num_planning_attempts(10) # take best of 5 for accuracy of 5mm
    else:
        right_arm.set_max_velocity_scaling_factor(0.5) # make it slower
        right_arm.set_goal_position_tolerance(0.01) 
        right_arm.set_num_planning_attempts(5) # take best of 3 for accuracy of 1 cm
    print pose


    right_plan = right_arm.plan()
    right_arm.execute(right_plan)


def do_grip():
    """
    Close gripper with intention of grasping object. Detect if object has been gripped.
    """
    assert right_gripper.is_ready()
    right_gripper.open()
    rospy.sleep(2.0)
    assert right_gripper.is_ready()
    right_gripper.close()
    rospy.sleep(2.0)


class Action():
    """
    Wrapper class for defining and executing certain actions.
    This is helpful when chaining multiple actions together.
    """

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
            move(self.value, do_precise_movement=True)

        elif self.action_type == Action.FUNCTION:
            rospy.logdebug( "RUNNING CUSTOM FUNCTION")
            self.value()


def execute_action_sequence(actions):
    """
    Helper function to execute a list of Action objects
    """
    for action in actions:
        action.execute()
    rospy.logdebug( "Action sequence finished")
    return 

def quat_to_euler(orientation):
    """
    Convert pose's orientation (quaternion) to Euler angles
    """
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
    """
    Convert from Euler angles to pose's orientation (quaternion)
    """
    pose = Pose()
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose.orientation


def flip_quat(orientation):
    """
    Given an Pose orientation, find orientation with inverted z basis vector, by turning roll by 180 degrees.
    """
    r,p,y = quat_to_euler(orientation)
    r += np.pi
    return euler_to_quat(r,p,y)


def give_orientation(pose, orr_array):
    """
    Helper function to assign pose's orientation according to quaterion in list form.
    """
    pose.orientation.x = orr_array[0]
    pose.orientation.y = orr_array[1]
    pose.orientation.z = orr_array[2]
    pose.orientation.w = orr_array[3]


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

