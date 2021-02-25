#!/usr/bin/python

import sys, math, copy
import rospy, tf, tf2_ros, moveit_commander
import numpy as np

import moveit_msgs.msg, tf2_geometry_msg

from time import sleep

def nothing(x):
    pass

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
        
        self.target_point = PointStamped()

        ############################
        # Subscribers and Publishers
        ############################
        self.pose_sub = rospy.Subscriber(obj_pose_topic, PoseStampedNamed, self.obj_pose_callback, queue_size = 1)
        # queue_size = 1, no remaining task for the sake of safety
        self.tf_listener = tf.TransformListener()
        rate = rospy.Rate(10)
        rospy.spin()  

    # - Loading waypoints list from .txt/.xml file
    def readWaypoints(self):

    # - Receving 'READY' signal from Camera
    # - Moving to next waypoint;
    # - Send Signal to trigger camera taking picture.
    def moveAndSendSignal(self):

    

          