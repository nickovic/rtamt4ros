#! /usr/bin/env python
# call roscore
# $ roscore
#
# IF start in manual
# $ rosrun hsr_monitors hsr_monitor.py --freq 10
# TODO:
# 1) write all properties expected.
# 2) fault localization.
# 3) change TF to TF2
# 4) check overhead with some time command, and timing of update
# 5) Other agent senario.
# Others: robQue is annoying
#         cleraning rosDistLib
#         handling publish is annoying too.
# Limitation: rqt_plot does not plot based on header.stamp. It is back-end limitation. However, the robutness itself hanldes time-stamps exactly. That is just viewer issue.


import sys
import argparse
import logging
import copy
import Queue
import matplotlib.pyplot as plt
import timeit

import rospy
import tf
import pcl_ros
import laser_geometry.laser_geometry
import sensor_msgs.point_cloud2

import rtamt

from ros_distance_libs.rosDistLib import *

#other msg
from std_msgs.msg import String, Header, Bool
from sensor_msgs.msg import PointCloud2, PointCloud, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
from control_msgs.msg import JointTrajectoryControllerState

from rtamt_msgs.msg import FloatStamped


DEBUG = False

def print_rob(rob, name):
    if rob != []:
    	rospy.loginfo('rob {0}: {1}'.format(name, rob))
    return


def publishRobstness(publisher, rob):
	if rob != []:
		for trob in rob:
			floatStamped = FloatStamped()
			header = Header()
			header.seq = 0
			header.stamp = rospy.Time.from_sec(trob[0])
			header.frame_id = ''
			floatStamped.header = header
			floatStamped.value = trob[1]
			publisher.publish(floatStamped)
	return


class RobQue(object):
	def __init__(self, name):
		self.name = name
		self.queue = Queue.Queue()

	def putRob(self, rob):
		if rob != []:
			for trob in rob:
				self.queue.put(trob)

	def printRob(self):
		robs = []
		while not self.queue.empty():
			robs.append(self.queue.get())

		print_rob(robs, self.name)


class BoolStamped(object):
    def __init__(self, data, header):
        self.data = data
        self.header = header


class HSR_STL_monitor(object):
	def __init__(self):

		# listener of tf
		self.tfListener = tf.TransformListener()

		# laser projection
		self.lp = laser_geometry.laser_geometry.LaserProjection()

		# STL settings
		# Load the spec from STL file
		# 1) system -----
		robTopicPrefix = '/rtamt/system/'
		# collision with obstacle (Ground Truth): /hsrb/odom_ground_truth /static_distance_map_ref
		self.spec_collEgoObs_gt = rtamt.STLDenseTimeSpecification()
		self.spec_collEgoObs_gt.name = 'collEgoObs_gt'
		self.spec_collEgoObs_gt.declare_var('distEgoObs_gt', 'float')
		self.spec_collEgoObs_gt.set_var_io_type('distEgoObs_gt', 'input')
		self.spec_collEgoObs_gt.spec = 'always [0,3] (distEgoObs_gt > 0.3)'
		self.robPub_collEgoObs_gt = rospy.Publisher(robTopicPrefix+self.spec_collEgoObs_gt.name, FloatStamped, queue_size=10)

		# colliosion with agents (Ground Truth): /hsrb/odom_ground_truth /dynamic_obstacle_map_ref
		self.spec_collEgoDynamicObs_gt = rtamt.STLDenseTimeSpecification()
		self.spec_collEgoDynamicObs_gt.name = 'collEgoDynamicObs_gt'
		self.spec_collEgoDynamicObs_gt.declare_var('distEgoDynamicObs_gt', 'float')
		self.spec_collEgoDynamicObs_gt.set_var_io_type('distEgoDynamicObs_gt', 'input')
		self.spec_collEgoDynamicObs_gt.spec = 'always [0,3] (distEgoDynamicObs_gt > 0.05)'
		self.robPub_collEgoDynamicObs_gt = rospy.Publisher(robTopicPrefix+self.spec_collEgoDynamicObs_gt.name, FloatStamped, queue_size=10)

		# avoid prohibit area (Ground Truth): /hsrb/odom_ground_truth /static_obstacle_map_ref
		# TODO: use always eventually
		self.spec_avoidProhibitArea_gt = rtamt.STLDenseTimeSpecification()
		self.spec_avoidProhibitArea_gt.name = 'avoidProhibitArea_gt'
		self.spec_avoidProhibitArea_gt.declare_var('distEgoProhibitArea_gt', 'float')
		self.spec_avoidProhibitArea_gt.set_var_io_type('distEgoProhibitArea_gt', 'input')
		self.spec_avoidProhibitArea_gt.spec = 'always [0,3] (distEgoProhibitArea_gt > 0.2)'
		self.robPub_avoidProhibitArea_gt = rospy.Publisher(robTopicPrefix+self.spec_avoidProhibitArea_gt.name, FloatStamped, queue_size=10)

		# reach goal (Ground Truth): /hsrb/odom_ground_truth /goal
		self.spec_reachEgoGoal_gt = rtamt.STLDenseTimeSpecification()
		self.spec_reachEgoGoal_gt.name = 'reachEgoGoal_gt'
		self.spec_reachEgoGoal_gt.declare_var('distEgoGoal_gt', 'float')
		self.spec_reachEgoGoal_gt.set_var_io_type('distEgoGoal_gt', 'input')
		self.spec_reachEgoGoal_gt.declare_var('moveTask', 'float')
		self.spec_reachEgoGoal_gt.set_var_io_type('moveTask', 'input')
		self.spec_reachEgoGoal_gt.spec = 'always[0,50]( moveTask > 0.5 -> eventually [0,30] (distEgoGoal_gt < 0.5))'
		self.robPub_reachEgoGoal_gt = rospy.Publisher(robTopicPrefix+self.spec_reachEgoGoal_gt.name, FloatStamped, queue_size=10)
		self.robQue_reachEgoGoal_gt = RobQue(self.spec_reachEgoGoal_gt.name)

		try:
			self.spec_collEgoObs_gt.parse()
			self.spec_collEgoObs_gt.pastify()
			self.spec_collEgoDynamicObs_gt.parse()
			self.spec_collEgoDynamicObs_gt.pastify()
			self.spec_avoidProhibitArea_gt.parse()
			self.spec_avoidProhibitArea_gt.pastify()
			self.spec_reachEgoGoal_gt.parse()
			self.spec_reachEgoGoal_gt.pastify()
		except rtamt.STLParseException as err:
			print('STL Parse Exception: {}'.format(err))
			sys.exit()


		# 2) perception -----
		robTopicPrefix = '/rtamt/perception/'
		# localization error (Ground Truth): /hsrb/odom_ground_truth /global_pose
		self.spec_errLoc = rtamt.STLDenseTimeSpecification()
		self.spec_errLoc.name = 'errLoc'
		self.spec_errLoc.declare_var('errLoc', 'float')
		self.spec_errLoc.set_var_io_type('errLoc', 'input')
		self.spec_errLoc.spec = 'always [0,3] (errLoc < 0.1)'
		self.robPub_errLoc = rospy.Publisher(robTopicPrefix+self.spec_errLoc.name, FloatStamped, queue_size=10)

		# odometer error (Ground Truth): /hsrb/odom_ground_truth /hsrb/wheel_odom
		self.spec_errWheelOdom = rtamt.STLDenseTimeSpecification()
		self.spec_errWheelOdom.name = 'errWheelOdom'
		self.spec_errWheelOdom.declare_var('errWheelOdom', 'float')
		self.spec_errWheelOdom.set_var_io_type('errWheelOdom', 'input')
		self.spec_errWheelOdom.spec = 'always [0,3] (errWheelOdom < 0.1)'
		self.robPub_errWheelOdom = rospy.Publisher(robTopicPrefix+self.spec_errWheelOdom.name, FloatStamped, queue_size=10)

		# localization error LiDAR (Ground Truth): /hsrb/odom_ground_truth /hsrb/laser_odom
		self.spec_errLaserOdom = rtamt.STLDenseTimeSpecification()
		self.spec_errLaserOdom.name = 'errLaserOdom'
		self.spec_errLaserOdom.declare_var('errLaserOdom', 'float')
		self.spec_errLaserOdom.set_var_io_type('errLaserOdom', 'input')
		self.spec_errLaserOdom.spec = 'always [0,3] (errLaserOdom < 0.1)'
		self.robPub_errLaserOdom = rospy.Publisher(robTopicPrefix+self.spec_errLaserOdom.name, FloatStamped, queue_size=10)

		# LiDAR error (Grand Truth): hsrb/base_scan /static_distance_map_ref
		self.spec_errLidar = rtamt.STLDenseTimeSpecification()
		self.spec_errLidar.name = 'errLidar'
		self.spec_errLidar.declare_var('errLidar', 'float')
		self.spec_errLidar.set_var_io_type('errLidar', 'input')
		self.spec_errLidar.spec = 'always [0,3] (errLidar < 0.1)'
		self.robPub_errLidar = rospy.Publisher(robTopicPrefix+self.spec_errLidar.name, FloatStamped, queue_size=10)

		# StereoCamera error (Ground Truth): /hsrb/head_rgbd_sensor/depth_registered/rectified_points <Gazebo3dshape>
		# this time we can skipt it. too much implimentation effort.

		# Bumper error (Ground Truth): /hsrb/base_b_bumper_sensor, /hsrb/base_f_bumper_sensor, /static_distance_map_ref
		# TODO: always(distEgoObs < e -> eventuraly bumper_sensor == True)

		try:
			self.spec_errLoc.parse()
			self.spec_errLoc.pastify()
			self.spec_errWheelOdom.parse()
			self.spec_errWheelOdom.pastify()
			self.spec_errLaserOdom.parse()
			self.spec_errLaserOdom.pastify()
			self.spec_errLidar.parse()
			self.spec_errLidar.pastify()
		except rtamt.STLParseException as err:
			print('STL Parse Exception: {}'.format(err))
			sys.exit()


		# 3) planner -----
		robTopicPrefix = '/rtamt/planner/'
		# collision with obstacle map: /global_pose /static_distance_map_ref
		self.spec_collEgoObs = rtamt.STLDenseTimeSpecification()
		self.spec_collEgoObs.name = 'collEgoObs'
		self.spec_collEgoObs.declare_var('distEgoObs', 'float')
		self.spec_collEgoObs.set_var_io_type('distEgoObs', 'input')
		self.spec_collEgoObs.spec = 'always [0,3] (distEgoObs > 0.2)'
		self.robPub_collEgoObs = rospy.Publisher(robTopicPrefix+self.spec_collEgoObs.name, FloatStamped, queue_size=10)

		# colliosion with agents: /global_pose /dynamic_obstacle_map_ref
		self.spec_collEgoDynamicObs = rtamt.STLDenseTimeSpecification()
		self.spec_collEgoDynamicObs.name = 'collEgoDynamicObs'
		self.spec_collEgoDynamicObs.declare_var('distEgoDynamicObs', 'float')
		self.spec_collEgoDynamicObs.set_var_io_type('distEgoDynamicObs', 'input')
		self.spec_collEgoDynamicObs.spec = 'always [0,3] (distEgoDynamicObs > 0.2)'
		self.robPub_collEgoDynamicObs = rospy.Publisher(robTopicPrefix+self.spec_collEgoDynamicObs.name, FloatStamped, queue_size=10)

		# avoid prohibit area: /global_pose /static_obstacle_map_ref
		# TODO: always(distEgoObs < e -> eventuraly bumper_sensor == True)
		self.spec_avoidEgoProhibitArea = rtamt.STLDenseTimeSpecification()
		self.spec_avoidEgoProhibitArea.name = 'avoidEgoProhibitArea'
		self.spec_avoidEgoProhibitArea.declare_var('distEgoProhibitArea', 'float')
		self.spec_avoidEgoProhibitArea.set_var_io_type('distEgoProhibitArea', 'input')
		self.spec_avoidEgoProhibitArea.spec = 'always [0,3] (distEgoProhibitArea >= 0.2)'
		self.robPub_avoidEgoProhibitArea = rospy.Publisher(robTopicPrefix+self.spec_avoidEgoProhibitArea.name, FloatStamped, queue_size=10)

		# collision with obstacle LiDAR: hsrb/base_scan
		self.spec_collLidar = rtamt.STLDenseTimeSpecification()
		self.spec_collLidar.name = 'collLidar'
		self.spec_collLidar.declare_var('distLidar', 'float')
		self.spec_collLidar.set_var_io_type('distLidar', 'input')
		self.spec_collLidar.spec = 'always [0,3] (distLidar > 0.2)'
		self.robPub_collLidar = rospy.Publisher(robTopicPrefix+self.spec_collLidar.name, FloatStamped, queue_size=10)
		self.robQue_collLidar = RobQue(self.spec_collLidar.name)

		# collision with obstacle StereoCamera: /hsrb/head_rgbd_sensor/depth_registered/rectified_points
		self.spec_collStereoCamera = rtamt.STLDenseTimeSpecification()
		self.spec_collStereoCamera.name = 'collStereoCamera'
		self.spec_collStereoCamera.declare_var('distStereoCamera', 'float')
		self.spec_collStereoCamera.set_var_io_type('distStereoCamera', 'input')
		self.spec_collStereoCamera.spec = 'always [0,3] (distStereoCamera > 0.2)'
		self.robPub_collStereoCamera = rospy.Publisher(robTopicPrefix+self.spec_collStereoCamera.name, FloatStamped, queue_size=10)
		self.robQue_collStereoCamera = RobQue(self.spec_collStereoCamera.name)

		# collision with obstacle Bumper: /hsrb/base_b_bumper_sensor, /hsrb/base_f_bumper_sensor
		self.spec_collBumperFront = rtamt.STLDenseTimeSpecification()
		self.spec_collBumperFront.name = 'collBumperFront'
		self.spec_collBumperFront.declare_var('bumperFront', 'float')
		self.spec_collBumperFront.set_var_io_type('bumperFront', 'input')
		self.spec_collBumperFront.spec = 'always [0,3] (bumperFront < 0.5)'
		self.robPub_collBumperFront = rospy.Publisher(robTopicPrefix+self.spec_collBumperFront.name, FloatStamped, queue_size=10)
		self.robQue_collBumperFront = RobQue(self.spec_collBumperFront.name)

		self.spec_collBumperBack = rtamt.STLDenseTimeSpecification()
		self.spec_collBumperBack.name = 'collBbumperBack'
		self.spec_collBumperBack.declare_var('bumperBack', 'float')
		self.spec_collBumperBack.set_var_io_type('bumperBack', 'input')
		self.spec_collBumperBack.spec = 'always [0,3] (bumperBack < 0.5 )'
		self.robPub_collBumperBack = rospy.Publisher(robTopicPrefix+self.spec_collBumperBack.name, FloatStamped, queue_size=10)
		self.robQue_collBumperBack = RobQue(self.spec_collBumperBack.name)

		# collision with obstacle GlobalPath: /base_local_path /static_distance_map_ref
		self.spec_collGlobalPathObs = rtamt.STLDenseTimeSpecification()
		self.spec_collGlobalPathObs.name = 'collGlobalPathObs'
		self.spec_collGlobalPathObs.declare_var('distGlobalPathObs', 'float')
		self.spec_collGlobalPathObs.set_var_io_type('distGlobalPathObs', 'input')
		self.spec_collGlobalPathObs.spec = '(distGlobalPathObs > 0.1)'
		self.robPub_collGlobalPathObs = rospy.Publisher(robTopicPrefix+self.spec_collGlobalPathObs.name, FloatStamped, queue_size=10)
		self.robQue_collGlobalPathObs = RobQue(self.spec_collGlobalPathObs.name)

		# reach goal GlobalPath: /base_local_path /goal
		self.spec_reachGlobalPathGoal = rtamt.STLDenseTimeSpecification()
		self.spec_reachGlobalPathGoal.name = 'reachGlobalPathGoal'
		self.spec_reachGlobalPathGoal.declare_var('distGlobalPathGoal', 'float')
		self.spec_reachGlobalPathGoal.set_var_io_type('distGlobalPathGoal', 'input')
		self.spec_reachGlobalPathGoal.spec = '(distGlobalPathGoal < 0.1)'
		self.robPub_reachGlobalPathGoal = rospy.Publisher(robTopicPrefix+self.spec_reachGlobalPathGoal.name, FloatStamped, queue_size=10)
		self.robQue_reachGlobalPathGoal = RobQue(self.spec_reachGlobalPathGoal.name)

		# reach goal: /global_pose /goal
		self.spec_reachEgoGoal = rtamt.STLDenseTimeSpecification()
		self.spec_reachEgoGoal.name = 'reachEgoGoal'
		self.spec_reachEgoGoal.declare_var('distEgoGoal', 'float')
		self.spec_reachEgoGoal.set_var_io_type('distEgoGoal', 'input')
		self.spec_reachEgoGoal.spec = 'eventually [0,30] (distEgoGoal < 0.5)'
		self.robPub_reachEgoGoal = rospy.Publisher(robTopicPrefix+self.spec_reachEgoGoal.name, FloatStamped, queue_size=10)
		self.robQue_reachEgoGoal = RobQue(self.spec_reachEgoGoal.name)

		try:
			self.spec_collEgoObs.parse()
			self.spec_collEgoObs.pastify()
			self.spec_collEgoDynamicObs.parse()
			self.spec_collEgoDynamicObs.pastify()
			self.spec_avoidEgoProhibitArea.parse()
			self.spec_avoidEgoProhibitArea.pastify()
			self.spec_collLidar.parse()
			self.spec_collLidar.pastify()
			self.spec_collStereoCamera.parse()
			self.spec_collStereoCamera.pastify()
			self.spec_collBumperFront.parse()
			self.spec_collBumperFront.pastify()
			self.spec_collBumperBack.parse()
			self.spec_collBumperBack.pastify()
			self.spec_collGlobalPathObs.parse()
			self.spec_collGlobalPathObs.pastify()
			self.spec_reachGlobalPathGoal.parse()
			self.spec_reachGlobalPathGoal.pastify()
			self.spec_reachEgoGoal.parse()
			self.spec_reachEgoGoal.pastify()
		except rtamt.STLParseException as err:
			print('STL Parse Exception: {}'.format(err))
			sys.exit()


		# 4) controller -----
		robTopicPrefix = '/rtamt/controller/'
		# TODO: perhaps we had better to check Squared distance.
		# ref global path: /base_local_path /global_pose
		self.spec_referrLocGlobalPath = rtamt.STLDenseTimeSpecification()
		self.spec_referrLocGlobalPath.name = 'referrLocGlobalPath'
		self.spec_referrLocGlobalPath.declare_var('referrLocGlobalPath', 'float')
		self.spec_referrLocGlobalPath.set_var_io_type('referrLocGlobalPath', 'input')
		self.spec_referrLocGlobalPath.spec = 'always [0,3] (referrLocGlobalPath < 0.1)'
		self.robPub_referrLocGlobalPath = rospy.Publisher(robTopicPrefix+self.spec_referrLocGlobalPath.name, FloatStamped, queue_size=10)
		self.robQue_referrLocGlobalPath = RobQue(self.robPub_referrLocGlobalPath.name)

		# ref body control: /hsrb/command_velocity /base_velocity
		self.spec_referrBodyVel = rtamt.STLDenseTimeSpecification()
		self.spec_referrBodyVel.name = 'referrBodyVel'
		self.spec_referrBodyVel.declare_var('referrBodyVel', 'float')
		self.spec_referrBodyVel.set_var_io_type('referrBodyVel', 'input')
		self.spec_referrBodyVel.spec = 'always [0,3] (referrBodyVel < 0.1)'
		self.robPub_referrBodyVel = rospy.Publisher(robTopicPrefix+self.spec_referrBodyVel.name, FloatStamped, queue_size=10)
		self.robQue_referrBodyVel = RobQue(self.spec_referrBodyVel.name)

		# ref wheel motor control: /hsrb/omni_base_controller/internal_state
		# internally the topic has paramname, actual, desired data.
		self.spec_referrWheelVelL = rtamt.STLDenseTimeSpecification()
		self.spec_referrWheelVelL.name = 'referrWheelVelL'
		self.spec_referrWheelVelL.declare_var('referrWheelVelL', 'float')
		self.spec_referrWheelVelL.set_var_io_type('referrWheelVelL', 'input')
		self.spec_referrWheelVelL.spec = 'always [0,3] (referrWheelVelL < 0.1)'
		self.robPub_referrWheelVelL = rospy.Publisher(robTopicPrefix+self.spec_referrWheelVelL.name, FloatStamped, queue_size=10)
		self.robQue_referrWheelVelL = RobQue(self.spec_referrWheelVelL.name)

		self.spec_referrWheelVelR = rtamt.STLDenseTimeSpecification()
		self.spec_referrWheelVelR.name = 'referrWheelVelR'
		self.spec_referrWheelVelR.declare_var('referrWheelVelR', 'float')
		self.spec_referrWheelVelR.set_var_io_type('referrWheelVelR', 'input')
		self.spec_referrWheelVelR.spec = 'always [0,3] (referrWheelVelR < 0.1)'
		self.robPub_referrWheelVelR = rospy.Publisher(robTopicPrefix+self.spec_referrWheelVelR.name, FloatStamped, queue_size=10)
		self.robQue_referrWheelVelR = RobQue(self.spec_referrWheelVelR.name)

		try:
			self.spec_referrLocGlobalPath.parse()
			self.spec_referrLocGlobalPath.pastify()
			self.spec_referrBodyVel.parse()
			self.spec_referrBodyVel.pastify()
			self.spec_referrWheelVelL.parse()
			self.spec_referrWheelVelL.pastify()
			self.spec_referrWheelVelR.parse()
			self.spec_referrWheelVelR.pastify()
		except rtamt.STLParseException as err:
			print('STL Parse Exception: {}'.format(err))
			sys.exit()


		# 5) others (intermidiate variables) -----
		# We expect we can do it with hospital robot.
		# virtualBumper /hk0/zero_velocity :this one HSRB does not have.
		# local path generation status hk0/local_path_status_sim :perhaps /path_follow_action/status in HSRB
		# saftyMoveFlag /hk0/is_safety_move :this one HSRB does not have.


		# For each var from the spec, subscribe to its topic
		# system ground truth
		rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, self.odom_gt_callback, queue_size=10)
		self.loc_gt = []
		rospy.Subscriber('/static_distance_map_ref', OccupancyGrid, self.map_callback, queue_size=10)
		self.map = []
		rospy.Subscriber('/static_obstacle_map_ref', OccupancyGrid, self.prohibitMap_callback, queue_size=10)
		self.prohibitMap = []
		rospy.Subscriber('/dynamic_obstacle_map', OccupancyGrid, self.dynamicObsMap_callback, queue_size=10)
		self.dynamicObsMap = []

		# system order
		rospy.Subscriber('/goal', PoseStamped, self.goal_callback, queue_size=10)
		self.goal =[]
		rospy.Subscriber('/hsrb/command_velocity', Twist, self.baseVel_ref_callback, queue_size=10)
		self.baseVel_ref = []

		# system sensor
		rospy.Subscriber('/global_pose', PoseStamped, self.loc_callback, queue_size=10)
		self.loc = []
		rospy.Subscriber('/hsrb/wheel_odom', Odometry, self.wheelOdom_callback, queue_size=10)
		self.wheelOdom = []
		rospy.Subscriber('/hsrb/laser_odom', Odometry, self.laserOdom_callback, queue_size=10)
		self.laserOdom = []
		rospy.Subscriber('/hsrb/base_scan', LaserScan, self.lidar_callback, queue_size=10)
		self.lidar =[]
		rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.stereoCamera_callback, queue_size=10)
		self.stereoCam = []
		rospy.Subscriber('/base_velocity', Twist, self.baseVel_callback, queue_size=10)
		self.baseVel = []
		rospy.Subscriber('/hsrb/base_f_bumper_sensor', Bool, self.bumperFront_callback, queue_size=10)
		self.bumperFront = []
		rospy.Subscriber('/hsrb/base_b_bumper_sensor', Bool, self.bumperBack_callback, queue_size=10)
		self.bumperBack = []
		rospy.Subscriber('/hsrb/omni_base_controller/internal_state', JointTrajectoryControllerState, self.controllerInfo_callback, queue_size=10)
		self.controllerInfo = []

		# system intermidiate data
		# TODO: original controller path is /path_follow_action/goal /base_local_path is only for viewer
		rospy.Subscriber('/base_local_path', Path, self.globalPath_callback, queue_size=10)
		self.globalPath = []


	# it is not colled ctrl+Z
	def __del__(self):
		if DEBUG:
			plt.close()


	def loc_callback(self, poseStamped):
		self.loc = poseStamped


	def odom_gt_callback(self, odometry):
		self.loc_gt = odometry


	def wheelOdom_callback(self, odometry):
		self.wheelOdom = odometry


	def laserOdom_callback(self, odometry):
		self.laserOdom = odometry


	def goal_callback(self, poseStamped):
		self.goal = poseStamped


	# this will be called just one time.
	def map_callback(self, occupancyGrid):
		self.map = occupancyGrid

		# debug for the occupancyGrid data
		if DEBUG:
			mapFig = plt.figure(figsize = (12,12))
			ax = mapFig.add_subplot(1, 1, 1)
			occupancyGridPlot(ax, occupancyGrid)
			plt.show()


	def prohibitMap_callback(self, occupancyGrid):
		self.prohibitMap = occupancyGrid


	def dynamicObsMap_callback(self, occupancyGrid):
		self.dynamicObsMap = occupancyGrid


	def lidar_callback(self, laser_message):
		self.lidar = laser_message


	def stereoCamera_callback(self, pointCloud2):
		self.stereoCam = pointCloud2


	def globalPath_callback(self, path):
		self.globalPath = path

		if self.goal !=[]:
			goalPoseStamped = self.globalPath.poses[-1]
			distGlobalPathGoal, stamp = distPoseStamped2PoseStamped(self.goal, goalPoseStamped, True)
			data = [[stamp.to_sec(), distGlobalPathGoal]]
			rob = self.spec_reachGlobalPathGoal.update(['distGlobalPathGoal', data])
			publishRobstness(self.robPub_reachGlobalPathGoal, rob)
			self.robQue_reachGlobalPathGoal.putRob(rob)


	def baseVel_ref_callback(self, twist):
		self.baseVel_ref = twist


	def baseVel_callback(self, twist):
		self.baseVel = twist


	def bumperFront_callback(self, data):
		header = Header()
		header.seq = 0
		header.stamp = rospy.Time.now()
		header.frame_id = ''
		self.bumperFront = BoolStamped(data.data, header)


	def bumperBack_callback(self, data):
		header = Header()
		header.seq = 0
		header.stamp = rospy.Time.now()
		header.frame_id = ''
		self.bumperBack = BoolStamped(data.data, header)


	def controllerInfo_callback(self, data):
		self.controllerInfo = data


	def monitor_system_callback(self, event):
		# 1) system -----
		if self.loc_gt != [] and self.map != []:
			distEgoObs_gt, stamp = distOdometry2OccupancyGrid(self.loc_gt, self.map, True)
			data = [[stamp.to_sec(), distEgoObs_gt]]
			rob = self.spec_collEgoObs_gt.update(['distEgoObs_gt',data])
			publishRobstness(self.robPub_collEgoObs_gt, rob)
			print_rob(rob, self.spec_collEgoObs_gt.name)
		if self.loc_gt != [] and self.dynamicObsMap != []:
			distEgoDynamicObs_gt, stamp = distOdometry2OccupancyGrid(self.loc_gt, self.dynamicObsMap, True, 3)
			data = [[stamp.to_sec(), distEgoDynamicObs_gt]]
			rob = self.spec_collEgoDynamicObs_gt.update(['distEgoDynamicObs_gt',data])
			publishRobstness(self.robPub_collEgoDynamicObs_gt, rob)
			print_rob(rob, self.spec_collEgoDynamicObs_gt.name)
		if self.loc_gt != [] and self.prohibitMap != []:
			distEgoStaticObs_gt, stamp = distOdometry2OccupancyGrid(self.loc_gt, self.prohibitMap, True)
			data = [[stamp.to_sec(), distEgoStaticObs_gt]]
			rob = self.spec_avoidProhibitArea_gt.update(['distEgoProhibitArea_gt',data])
			publishRobstness(self.robPub_avoidProhibitArea_gt, rob)
			print_rob(rob, self.spec_avoidProhibitArea_gt.name)

		if self.loc_gt != [] and self.goal != []:
			distEgoGoal_gt, stamp = distPoseStamped2Odometry(self.goal, self.loc_gt, True)
			distData = [[rospy.Time.now().to_sec(), distEgoGoal_gt]] # here current time is input.
			eventData = [[rospy.Time.now().to_sec(), 1.0]]
		else:
			distData = [[rospy.Time.now().to_sec(), 0.0]] # here current time is input.
			eventData = [[rospy.Time.now().to_sec(), 0.0]]
		rob = self.spec_reachEgoGoal_gt.update(['distEgoGoal_gt', distData, 'moveTask', eventData])
		publishRobstness(self.robPub_reachEgoGoal_gt, rob)
		print_rob(rob, self.spec_reachEgoGoal_gt.name)
		rospy.logwarn('data: {}'.format(['distEgoGoal_gt', distData, 'moveTask', eventData]))
		rospy.logwarn('Rob: {}'.format(rob))

	def monitor_perception_callback(self, event):
		# 2) perception -----
		if self.loc != [] and self.loc_gt != []:
			errLoc, stamp = distPoseStamped2Odometry(self.loc, self.loc_gt)
			data = [[stamp.to_sec(), errLoc]]
			rob = self.spec_errLoc.update(['errLoc', data])
			publishRobstness(self.robPub_errLoc, rob)
			print_rob(rob, self.spec_errLoc.name)
		if self.wheelOdom != [] and self.loc_gt != []:
			wheelOdom_poseStamped = odometry2PoseStamped(self.wheelOdom)
			loc_gt_poseStamped = odometry2PoseStamped(self.loc_gt)
			while not rospy.is_shutdown():
				try:
					loc_gt_pose_frameOdom = self.tfListener.transformPose(wheelOdom_poseStamped.header.frame_id, loc_gt_poseStamped)
					break
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					continue
			errOdom, stamp = distPoseStamped2PoseStamped(wheelOdom_poseStamped, loc_gt_pose_frameOdom)
			data = [[stamp.to_sec(), errOdom]]
			rob = self.spec_errWheelOdom.update(['errWheelOdom', data])
			publishRobstness(self.robPub_errWheelOdom, rob)
			print_rob(rob, self.spec_errWheelOdom.name)
		if self.laserOdom != [] and self.loc_gt != []:
			laserOdom_poseStamped = odometry2PoseStamped(self.laserOdom)
			loc_gt_poseStamped = odometry2PoseStamped(self.loc_gt)
			while not rospy.is_shutdown():
				try:
					loc_gt_pose_frameOdom = self.tfListener.transformPose(laserOdom_poseStamped.header.frame_id, loc_gt_poseStamped)
					break
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					continue
			errOdom, stamp = distPoseStamped2PoseStamped(laserOdom_poseStamped, loc_gt_pose_frameOdom)
			data = [[stamp.to_sec(), errOdom]]
			rob = self.spec_errLaserOdom.update(['errLaserOdom', data])
			publishRobstness(self.robPub_errLaserOdom, rob)
			print_rob(rob, self.spec_errLaserOdom.name)


	#TODO: Becuase of distsPointCloud2OccupancyGrid tooks time, separately called. Maybe map filter is needed.
	def monitor_perception_callback_temp(self, event):
		if self.map != [] and self.lidar:
			t_start = timeit.default_timer()
			liderPointCloud2 = self.lp.projectLaser(self.lidar)
			lidarPointCloud = pointCloud22PointCloud(liderPointCloud2)
			while not rospy.is_shutdown():
				try:
					lidarPointCloud_frame_map = self.tfListener.transformPointCloud(self.map.header.frame_id, lidarPointCloud)
					break
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					continue
			t_convert = timeit.default_timer()
			errLidar, stamp = distPointCloud2OccupancyGrid(lidarPointCloud_frame_map, self.map, True)
			t_dist = timeit.default_timer()
			data = [[stamp.to_sec(), errLidar]]
			rob = self.spec_errLidar.update(['errLidar', data])
			t_rtamt = timeit.default_timer()
			publishRobstness(self.robPub_errLidar, rob)
			print_rob(rob, self.spec_errLidar.name)

			#rospy.logwarn('errLidar Computation time[s]: convert={:0.8f}, dist={:0.8f}, rtamt={:0.8f}'.format(t_convert-t_start, t_dist-t_convert, t_rtamt-t_dist))


	def monitor_planner_callback(self, event):
		# 3) planner -----
		if self.loc != [] and self.map:
			distEgoObs, stamp = distPoseStamped2OccupancyGrid(self.loc, self.map, True)
			data = [[stamp.to_sec(), distEgoObs]]
			rob = self.spec_collEgoObs.update(['distEgoObs',data])
			publishRobstness(self.robPub_collEgoObs, rob)
			print_rob(rob, self.spec_collEgoObs.name)
		if self.loc != [] and self.dynamicObsMap:
			distEgoDynamicObs, stamp = distPoseStamped2OccupancyGrid(self.loc, self.dynamicObsMap, True, 3)
			data = [[stamp.to_sec(), distEgoDynamicObs]]
			rob = self.spec_collEgoDynamicObs.update(['distEgoDynamicObs',data])
			publishRobstness(self.robPub_collEgoDynamicObs, rob)
			print_rob(rob, self.spec_collEgoDynamicObs.name)
		if self.loc != [] and self.prohibitMap:
			distEgoProhibitArea, stamp = distPoseStamped2OccupancyGrid(self.loc, self.prohibitMap, True)
			data = [[stamp.to_sec(), distEgoProhibitArea]]
			rob = self.spec_avoidEgoProhibitArea.update(['distEgoProhibitArea',data])
			publishRobstness(self.robPub_avoidEgoProhibitArea, rob)
			print_rob(rob, self.spec_avoidEgoProhibitArea.name)
		if self.lidar !=[]:
			distLidar = min(self.lidar.ranges)
			data = [[self.lidar.header.stamp.to_sec(), distLidar]]
			rob = self.spec_collLidar.update(['distLidar', data])
			publishRobstness(self.robPub_collLidar, rob)
			print_rob(rob, self.spec_collLidar.name)
		self.robQue_collGlobalPathObs.printRob()
		self.robQue_reachGlobalPathGoal.printRob()
		if self.loc != [] and self.goal != []:
			distEgoGoal, stamp = distPoseStamped2PoseStamped(self.goal, self.loc, True)
			data = [[stamp.to_sec(), distEgoGoal]]
			rob = self.spec_reachEgoGoal.update(['distEgoGoal', data])
			publishRobstness(self.robPub_reachEgoGoal, rob)
			print_rob(rob, self.spec_reachEgoGoal.name)
		if self.bumperFront != []:
			data = [[self.bumperFront.header.stamp.to_sec(), float(self.bumperFront.data)]]
			rob = self.spec_collBumperFront.update(['bumperFront', data])
			publishRobstness(self.robPub_collBumperFront, rob)
			print_rob(rob, self.spec_collBumperFront.name)
		if self.bumperBack != []:
			data = [[self.bumperBack.header.stamp.to_sec(), float(self.bumperBack.data)]]
			rob = self.spec_collBumperBack.update(['bumperBack', data])
			publishRobstness(self.robPub_collBumperBack, rob)
			print_rob(rob, self.spec_collBumperBack.name)

		if self.globalPath != [] and self.map != []:
			t_start = timeit.default_timer()
			distGlobalPathObs, stamp = distPath2OccupancyGrid(self.globalPath, self.map, True)
			t_dist = timeit.default_timer()
			data = [[rospy.Time.now().to_sec(), distGlobalPathObs]] # here current time is input.
			rob = self.spec_collGlobalPathObs.update(['distGlobalPathObs', data])
			t_rtamt = timeit.default_timer()
			publishRobstness(self.robPub_collGlobalPathObs, rob)
			t_pub = timeit.default_timer()
			#rospy.logwarn('collGlobalPathObs Computation time[s]: dist={:0.8f}, rtamt={:0.8f}, publish={:0.8f}'.format(t_dist-t_start, t_rtamt-t_dist, t_pub-t_rtamt))
		else:
			data = [[rospy.Time.now().to_sec(), 1.0]] # default is 1m.
			rob = self.spec_collGlobalPathObs.update(['distGlobalPathObs', data])
			publishRobstness(self.robPub_collGlobalPathObs, rob)
		#rospy.logwarn('data: {}'.format(data))
		#rospy.logwarn('Rob: {}'.format(rob))


	#TODO: Becuase of stereoCam dist tooks time, separately called.
	def monitor_planner_callback_temp(self, event):
		rospy.logwarn('here')

		if self.stereoCam != []:
			t_start = timeit.default_timer()
			points_gen = sensor_msgs.point_cloud2.read_points(self.stereoCam, field_names = ("x", "y", "z"), skip_nans=True)
			stereo_points = numpy.array([i for i in points_gen])
			t_convert = timeit.default_timer()
			distStereoCamera = distMultiPoint2Point(numpy.array(stereo_points), numpy.array([0.0,0.0,0.0]))
			t_dist = timeit.default_timer()
			data = [[self.stereoCam.header.stamp.to_sec(), distStereoCamera]]
			rob = self.spec_collStereoCamera.update(['distStereoCamera', data])
			t_rtamt = timeit.default_timer()
			publishRobstness(self.robPub_collStereoCamera, rob)
			print_rob(rob, self.spec_collStereoCamera.name)

			rospy.logwarn('collStereoCamera Computation time[s]: convert={:0.8f}, dist={:0.8f}, rtamt={:0.8f}'.format(t_convert-t_start, t_dist-t_convert, t_rtamt-t_dist))


	def monitor_controller_callback(self, event):
		# 4) controller -----
		if self.loc != [] and self.globalPath != []:
			referrLocGlobalPath, stamp = distPoseStamped2Path(self.loc, self.globalPath, True)
			data = [[stamp.to_sec(), referrLocGlobalPath]]
			rob = self.spec_referrLocGlobalPath.update(['referrLocGlobalPath', data])
			publishRobstness(self.robPub_referrLocGlobalPath, rob)
			print_rob(rob, self.spec_referrLocGlobalPath.name)
		if self.baseVel != [] and self.baseVel_ref != []:
			referrBodyVel = distTwist2Twist(self.baseVel, self.baseVel_ref)
			now = rospy.get_rostime()
			data = [[now.to_sec(), referrBodyVel]]
			rob = self.spec_referrBodyVel.update(['referrBodyVel', data])
			publishRobstness(self.robPub_referrBodyVel, rob)
			print_rob(rob, self.spec_referrBodyVel.name)
		if self.controllerInfo != []:
			data = [[self.controllerInfo.header.stamp.to_sec(), self.controllerInfo.error.velocities[0]]]
			rob = self.spec_referrWheelVelL.update(['referrWheelVelL', data])
			publishRobstness(self.robPub_referrWheelVelL, rob)
			print_rob(rob, self.spec_referrWheelVelL.name)

			data = [[self.controllerInfo.header.stamp.to_sec(), self.controllerInfo.error.velocities[1]]]
			rob = self.spec_referrWheelVelR.update(['referrWheelVelR', data])
			publishRobstness(self.robPub_referrWheelVelR, rob)
			print_rob(rob, self.spec_referrWheelVelR.name)

		# 5) others (intermidiate variables) -----


if __name__ == '__main__':
	# Process arguments
	p = argparse.ArgumentParser(description='rtamt STL Python Monitor')
	p.add_argument('--freq', nargs=1, required=True, help='Sampling frequency in Hz')
	args = p.parse_args(rospy.myargv()[1:])

	try:
		rospy.init_node('hsr_stl_monitor')
		hsr_stl_monitor = HSR_STL_monitor()
		rospy.Timer(rospy.Duration(1.0/float(args.freq[0])), hsr_stl_monitor.monitor_system_callback)
		rospy.Timer(rospy.Duration(1.0/float(args.freq[0])), hsr_stl_monitor.monitor_perception_callback)
		#rospy.Timer(rospy.Duration(1.0), hsr_stl_monitor.monitor_perception_callback_temp) #TODO: remove this
		rospy.Timer(rospy.Duration(1.0/float(args.freq[0])), hsr_stl_monitor.monitor_planner_callback)
		#rospy.Timer(rospy.Duration(1.0), hsr_stl_monitor.monitor_planner_callback_temp) #TODO: remove this. The loop does not work.
		rospy.Timer(rospy.Duration(1.0/float(args.freq[0])), hsr_stl_monitor.monitor_controller_callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
