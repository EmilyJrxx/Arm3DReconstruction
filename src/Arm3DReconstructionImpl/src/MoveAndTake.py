#!/usr/bin/python

import sys, math
import rospy, tf, tf2_ros, moveit_commander
import numpy as np

import moveit_msgs.msg, tf2_geometry_msg
from geometry_msgs.msg import PoseStamped
from Arm3DReconstructionImpl.msg import TakeShot
from Arm3DReconstructionImpl.msg import MoveReady

from time import sleep

taketrigger_topic = "/signal/TakeShot"
readytomove_topic = "/signal/MoveToNext"
cameraPose_topic  = "/pose/camera_pose"

def nothing(x):
    pass

def all_close(goal, actual, tolerance):
    """
    ============================================================
    Convenient method for testing if a list of values are within 
    a tolerance of their counterparts in another list.
    ============================================================
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal=True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True

class SmartHolder(object):
    def __init__(self):
        super(SmartHolder, self).__init__()         # calling parent class
        # Initializing the move_group API
        moveit_commander.roscpp_initialize(sys.argv)# parsing args in MoveIt
        rospy.init_node('aubo_mp', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator_i5" 
        # Initialize the move group for the aubo
        group = moveit_commander.MoveGroupCommander(group_name)
        # Set global reference frame
        reference_frame = "world"
        # Set aubo_arm reference frame accordingly
        group.set_pose_reference_frame(reference_frame)
        # Allow replanning to increase odds of a solution
        group.allow_replanning(True)

        display_trajectory_publisher = rospy.Publisher('/move_group/planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_frame = group.get_planning_frame()
        print ("============ Reference frame: %s ============ " % planning_frame)
        # Get the name of the end-effector link
        eef_link = group.get_end_effector_link()
        print ("============ End effector: %s =============== " % eef_link)
        group_names = robot.get_group_names()
        print "============= Robot Groups:", robot.get_group_names()
        print "============= Printing robot state: =========="
        print robot.get_current_state()

        # Allow some leeway in position (meters) and orientation (radians)
        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.01)
        group.set_planning_time(0.1)
        group.set_max_acceleration_scaling_factor(0.5)
        group.set_max_velocity_scaling_factor(0.5)

        ###########################
        # MoveIt related property
        ###########################
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.track_flag = False
        self.default_pose_flag = True
        self.grasp_ongoing = False
        self.target_point = geometry_msgs.msgs.Pose()

        ############################
        # Waypoints related property
        ############################
        self.waypoints = []
        self.currIdx = 0

        ############################
        # Subscribers and Publishers
        ############################
        # queue_size = 1, no remaining task for the sake of safety
        self.tf_listener = tf.TransformListener()
        self.camera_pose_pub = rospy.Publisher(cameraPose_topic, PoseStamped, queue_size = 1)
        self.trigger_signal_pub = rospy.Publisher(taketrigger_topic, Takeshot, queue_size = 1)
        self.ready_signal_sub = rospy.Subscriber(readytomove_topic, MoveReady, self.Callback, queue_size = 1)
        rate = rospy.Rate(10)
        rospy.spin()  

    def goToReadyPose(self):
        group = self.group
        default_joint_states = group.get_current_joint_values()
        print(type(default_joint_states), default_joint_states)

        # define ready pose with specific joint value
        default_joint_states[0] = 2.39   / 180 * math.pi
        default_joint_states[1] = 28.69 / 180 * math.pi
        default_joint_states[2] = -124.72 / 180 * math.pi
        default_joint_states[3] = -63.47  / 180 * math.pi
        default_joint_states[4] = -88.21 / 180 * math.pi
        default_joint_states[5] = 1.7620   / 180 * math.pi

        group.set_pose_target(ready_pose)
        group.go(wait=True)
        group.stop()
        rospy.sleep(1)
        current_joints = group.get_current_joint_values()
        current_pose = self.group.get_current_pose().pose
        print("current pose:", current_pose)
        return all_close(default_joint_states, current_joints, 0.01)

    # - Loading waypoints list from .txt/.xml file
    def readWaypoints(self, waypoints_path):
        with open(waypoints_path, "r") as f:
            for line in f.readlines():
                # TODO: blank line removal
                line = line.strip('\n')
                print ("[Waypoint Reading]: ", line) # debug
                coords = float(line.split())     # split with ' ' & transform to floats
                pose = geometry_msgs.msgs.Pose() # initialization
                pose.position.x = coords[0]
                pose.position.y = coords[1]
                pose.position.z = coords[2]
                pose.orientation.x = coords[3]
                pose.orientation.y = coords[4]
                pose.orientation.z = coords[5]
                pose.orientation.w = coords[6]
                self.waypoints.append(pose)
            self.target_point = self.waypoints[self.currIdx] # set init value for target points
            print("Reading Complete: ", self.waypoints.count(), "\n")
            print("Moving to ReadyPose, Waiting for Ready Move Cmd.\n")
            self.goToReadyPose()

    # - Receving 'READY' signal from Camera
    # - Moving to next waypoint;
    # - Send Signal to trigger camera taking picture.
    def moveAndSendSignal(self):
        group = self.group
        current_pose = group.get_current_pose().pose
        print("Current pose: ", current_pose)

        print("Going to waypoints: ", ++self.currIdx)
        print("\n")
        pose_goal = self.target_point
        group.set_pose_target(pose_goal)
        plan = group.plan()
        group.go(wait=True)

        group.clear_pose_targets()
        self.target_point = self.waypoints[self.currIdx]
        trigger_msg = TakeShot()
        trigger_msg.header.seq = 0
        trigger_msg.header.stamp = rospy.Time.now()
        trigger_msg.header.frame_id = "/signal"
        trigger_msg.if_take = true
        self.trigger_signal_pub.publish(trigger_msg)
        print("Publishing 'Take a Shot' Triggering Signal\n")
        print("Waiting for Ready Signal\n")
        return all_close(pose_goal, current_pose, 0.01) # test if pose_goal equals current_pose within a tolerable range

    
    # - Callback Function for ReadyMove Signal
    def Callback(self, msg):
        if (msg.if_ready):
            self.moveAndSendSignal()
        else:
            print("Not receiving ready to move signal, Aborting.\n")
            return
        

if __name__=="__main__":
    # procedure
    robot_move = SmartHolder()
    # Reading waypoints
    # robot_move.readWaypoints("WayPoints.txt")


    

          