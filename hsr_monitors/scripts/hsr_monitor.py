#! /usr/bin/env python
# call roscore
# $ roscore
#
# IF start in manual
# $ rosrun hsr_monitors hsr_monitor.py --freq 10
# TODO:
# 1) Other agent senario

import sys
import argparse
import logging
import copy
import Queue
import matplotlib.pyplot as plt

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
from tmc_navigation_msgs.msg import PathWithGoal


DEBUG = False

def print_rob(rob, spec):
        if rob != []:
                rospy.loginfo('rob {0}: {1}'.format(spec.name, rob))
        return


def print_robQue(robQue, spec):
        if not robQue.empty():
                print_rob(robQue.get(), spec)
        return


def publishRobstness(publisher, robustness):
        if robustness != []:
                for trob in robustness:
                        floatStamped = FloatStamped()
                        header = Header()
                        header.seq = 0
                        header.stamp = rospy.Time.from_sec(trob[0])
                        header.frame_id = ''
                        floatStamped.header = header
                        floatStamped.value = trob[1]
                        publisher.publish(floatStamped)


class BoolStamped(object):
        def __init__(self, data, header):
                self.data = data
                self.header = header


class HSR_STL_monitor(object):
	def __init__(self):

                robTopicName = '/rtamt/'

                # listener of tf
                self.tfListener = tf.TransformListener()

                # laser projection
                self.lp = laser_geometry.laser_geometry.LaserProjection()

                # STL settings
                # Load the spec from STL file
                # 1) system -----
                # collision with obstacle (Ground Truth): /hsrb/odom_ground_truth /static_distance_map_ref
                self.spec_collEgoObs_gt = rtamt.STLDenseTimeSpecification()
                self.spec_collEgoObs_gt.name = 'collEgoObs_gt'
                self.spec_collEgoObs_gt.declare_var('distEgoObs_gt', 'float')
                self.spec_collEgoObs_gt.set_var_io_type('distEgoObs_gt', 'input')
                self.spec_collEgoObs_gt.spec = 'always [0,10] (distEgoObs_gt >= 0.1)'
                self.robPub_collEgoObs_gt = rospy.Publisher(robTopicName+self.spec_collEgoObs_gt.name, FloatStamped, queue_size=10)
                self.robQue_collEgoObs_gt = Queue.Queue()

                # colliosion with agents (Ground Truth): /hsrb/odom_ground_truth /dynamic_obstacle_map_ref

                # avoid prohibit area (Ground Truth): /hsrb/odom_ground_truth /static_obstacle_map_ref

                # reach goal (Ground Truth): /hsrb/odom_ground_truth /goal
                self.spec_reachEgoGoal_gt = rtamt.STLDenseTimeSpecification()
                self.spec_reachEgoGoal_gt.name = 'reachEgoGoal_gt'
                self.spec_reachEgoGoal_gt.declare_var('distEgoGoal_gt', 'float')
                self.spec_reachEgoGoal_gt.set_var_io_type('distEgoGoal_gt', 'input')
                self.spec_reachEgoGoal_gt.spec = 'eventually [0,10] (distEgoGoal_gt <= 0.1)'
                self.robPub_reachEgoGoal_gt = rospy.Publisher(robTopicName+self.spec_reachEgoGoal_gt.name, FloatStamped, queue_size=10)
                self.robQue_reachEgoGoal_gt = Queue.Queue()


                try:
                        self.spec_collEgoObs_gt.parse()
                        self.spec_collEgoObs_gt.pastify()
                        self.spec_reachEgoGoal_gt.parse()
                        self.spec_reachEgoGoal_gt.pastify()
                except rtamt.STLParseException as err:
                        print('STL Parse Exception: {}'.format(err))
                        sys.exit()


                # 2) perception -----
                # localization error (Ground Truth): /hsrb/odom_ground_truth /global_pose
                self.spec_errLoc = rtamt.STLDenseTimeSpecification()
                self.spec_errLoc.name = 'errLoc'
                self.spec_errLoc.declare_var('errLoc', 'float')
                self.spec_errLoc.set_var_io_type('errLoc', 'input')
                self.spec_errLoc.spec = 'always [0,10] (errLoc >= 0.1)'
                self.robPub_errLoc = rospy.Publisher(robTopicName+self.spec_errLoc.name, FloatStamped, queue_size=10)

                # odometer error (Ground Truth): /hsrb/odom_ground_truth /hsrb/wheel_odom
                self.spec_errWheelOdom = rtamt.STLDenseTimeSpecification()
                self.spec_errWheelOdom.name = 'errWheelOdom'
                self.spec_errWheelOdom.declare_var('errWheelOdom', 'float')
                self.spec_errWheelOdom.set_var_io_type('errWheelOdom', 'input')
                self.spec_errWheelOdom.spec = 'always [0,10] (errWheelOdom >= 0.1)'
                self.robPub_errWheelOdom = rospy.Publisher(robTopicName+self.spec_errWheelOdom.name, FloatStamped, queue_size=10)

                # localization error LiDAR (Ground Truth): /hsrb/odom_ground_truth /hsrb/laser_odom
                self.spec_errLaserOdom = rtamt.STLDenseTimeSpecification()
                self.spec_errLaserOdom.name = 'errLaserOdom'
                self.spec_errLaserOdom.declare_var('errLaserOdom', 'float')
                self.spec_errLaserOdom.set_var_io_type('errLaserOdom', 'input')
                self.spec_errLaserOdom.spec = 'always [0,10] (errLaserOdom >= 0.1)'
                self.robPub_errLaserOdom = rospy.Publisher(robTopicName+self.spec_errLaserOdom.name, FloatStamped, queue_size=10)

                # LiDAR error (Grand Truth): hsrb/base_scan /static_distance_map_ref
                self.spec_errLidar = rtamt.STLDenseTimeSpecification()
                self.spec_errLidar.name = 'errLidar'
                self.spec_errLidar.declare_var('errLidar', 'float')
                self.spec_errLidar.set_var_io_type('errLidar', 'input')
                self.spec_errLidar.spec = 'always [0,10] (errLidar >= 0.1)'
                self.robPub_errLidar = rospy.Publisher(robTopicName+self.spec_errLidar.name, FloatStamped, queue_size=10)
                self.robQue_errLidar = Queue.Queue()

                # StereoCamera error (Ground Truth): /hsrb/head_rgbd_sensor/depth_registered/rectified_points <Gazebo3dshape>

                # Bumper error (Ground Truth): /hsrb/base_b_bumper_sensor, /hsrb/base_f_bumper_sensor, /static_distance_map_ref

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
                # collision with obstacle map: /global_pose /static_distance_map_ref
                self.spec_collEgoObs = rtamt.STLDenseTimeSpecification()
                self.spec_collEgoObs.name = 'collEgoObs'
                self.spec_collEgoObs.declare_var('distEgoObs', 'float')
                self.spec_collEgoObs.set_var_io_type('distEgoObs', 'input')
                self.spec_collEgoObs.spec = 'always [0,10] (distEgoObs >= 0.1)'
                self.robPub_collEgoObs = rospy.Publisher(robTopicName+self.spec_collEgoObs.name, FloatStamped, queue_size=10)
                self.robQue_collEgoObs = Queue.Queue()

                # collision with obstacle LiDAR: hsrb/base_scan
                self.spec_collLidar = rtamt.STLDenseTimeSpecification()
                self.spec_collLidar.name = 'collLidar'
                self.spec_collLidar.declare_var('distLidar', 'float')
                self.spec_collLidar.set_var_io_type('distLidar', 'input')
                self.spec_collLidar.spec = 'always [0,10] (distLidar >= 0.2)'
                self.robPub_collLidar = rospy.Publisher(robTopicName+self.spec_collLidar.name, FloatStamped, queue_size=10)
                self.robQue_collLidar = Queue.Queue()

                # collision with obstacle StereoCamera: /hsrb/head_rgbd_sensor/depth_registered/rectified_points

                # collision with obstacle Bumper: /hsrb/base_b_bumper_sensor, /hsrb/base_f_bumper_sensor

                # collision with obstacle GlobalPath: /base_local_path /static_distance_map_ref
                self.spec_collGlobalPathObs = rtamt.STLDenseTimeSpecification()
                self.spec_collGlobalPathObs.name = 'collGlobalPathObs'
                self.spec_collGlobalPathObs.declare_var('distGlobalPathObs', 'float')
                self.spec_collGlobalPathObs.set_var_io_type('distGlobalPathObs', 'input')
                self.spec_collGlobalPathObs.spec = '(distGlobalPathObs >= 0.2)'
                self.robPub_collGlobalPathObs = rospy.Publisher(robTopicName+self.spec_collGlobalPathObs.name, FloatStamped, queue_size=10)
                self.robQue_collGlobalPathObs = Queue.Queue()

                # avoid prohibit area: /global_pose /static_obstacle_map_ref

                # colliosion with agents: /global_pose /dynamic_obstacle_map_ref

                # reach goal GlobalPath: /base_local_path /goal
                self.spec_reachGlobalPathGoal = rtamt.STLDenseTimeSpecification()
                self.spec_reachGlobalPathGoal.name = 'reachGlobalPathGoal'
                self.spec_reachGlobalPathGoal.declare_var('distGlobalPathGoal', 'float')
                self.spec_reachGlobalPathGoal.set_var_io_type('distGlobalPathGoal', 'input')
                self.spec_reachGlobalPathGoal.spec = '(distGlobalPathGoal <= 0.1)'
                self.robPub_reachGlobalPathGoal = rospy.Publisher(robTopicName+self.spec_reachGlobalPathGoal.name, FloatStamped, queue_size=10)
                self.robQue_reachGlobalPathGoal = Queue.Queue()

                # reach goal: /global_pose /goal
                self.spec_reachEgoGoal = rtamt.STLDenseTimeSpecification()
                self.spec_reachEgoGoal.name = 'reachEgoGoal'
                self.spec_reachEgoGoal.declare_var('distEgoGoal', 'float')
                self.spec_reachEgoGoal.set_var_io_type('distEgoGoal', 'input')
                self.spec_reachEgoGoal.spec = 'eventually [0,10] (distEgoGoal <= 0.1)'
                self.robPub_reachEgoGoal = rospy.Publisher(robTopicName+self.spec_reachEgoGoal.name, FloatStamped, queue_size=10)
                self.robQue_reachEgoGoal = Queue.Queue()

                try:
                        self.spec_collEgoObs.parse()
                        self.spec_collEgoObs.pastify()
                        self.spec_collLidar.parse()
                        self.spec_collLidar.pastify()
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
                # ref global path: /base_local_path /global_pose

                # ref body control: /hsrb/command_velocity /base_velocity
                self.spec_referrBodyVel = rtamt.STLDenseTimeSpecification()
                self.spec_referrBodyVel.name = 'referrBodyVel'
                self.spec_referrBodyVel.declare_var('referrBodyVel', 'float')
                self.spec_referrBodyVel.set_var_io_type('referrBodyVel', 'input')
                self.spec_referrBodyVel.spec = 'always [0,10] (referrBodyVel <= 0.1)'
                self.robPub_referrBodyVel = rospy.Publisher(robTopicName+self.spec_referrBodyVel.name, FloatStamped, queue_size=10)
                self.robQue_referrBodyVel = Queue.Queue()

                # ref wheel motor control: /hsrb/omni_base_controller/internal_state
                # internally the topic has paramname, actual, desired data.
                self.spec_referrWheelVelL = rtamt.STLDenseTimeSpecification()
                self.spec_referrWheelVelL.name = 'referrWheelVelL'
                self.spec_referrWheelVelL.declare_var('referrWheelVelL', 'float')
                self.spec_referrWheelVelL.set_var_io_type('referrWheelVelL', 'input')
                self.spec_referrWheelVelL.spec = 'always [0,1] (referrWheelVelL <= 0.1)'
                self.robPub_referrWheelVelL = rospy.Publisher(robTopicName+self.spec_referrWheelVelL.name, FloatStamped, queue_size=10)

                self.spec_referrWheelVelR = rtamt.STLDenseTimeSpecification()
                self.spec_referrWheelVelR.name = 'referrWheelVelR'
                self.spec_referrWheelVelR.declare_var('referrWheelVelR', 'float')
                self.spec_referrWheelVelR.set_var_io_type('referrWheelVelR', 'input')
                self.spec_referrWheelVelR.spec = 'always [0,1] (referrWheelVelR <= 0.1)'
                self.robPub_referrWheelVelR = rospy.Publisher(robTopicName+self.spec_referrWheelVelR.name, FloatStamped, queue_size=10)

                try:
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
                rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.stereoCam_callback, queue_size=10)
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
                rospy.Subscriber('/base_local_path', Path, self.globalPath_callback, queue_size=10) #rospy.Subscriber('/base_path_with_goal', PathWithGoal, self.globalPath_callback, queue_size=10)
                self.globalPath = []


        # it is not colled ctrl+Z
        def __del__(self):
                if DEBUG:
                        plt.close()


        def loc_callback(self, poseStamped):
                self.loc = poseStamped

                if self.goal != []:
                        distEgoGoal, stamp = distPoseStamped2PoseStamped(self.goal, self.loc, True)
                        data = [[stamp.to_sec(), distEgoGoal]]
                        rob = self.spec_reachEgoGoal.update(['distEgoGoal', data])
                        publishRobstness(self.robPub_reachEgoGoal, rob)
                        if rob != []:
                                self.robQue_reachEgoGoal.put(rob)


        def odom_gt_callback(self, odometry):
                self.loc_gt = odometry

                if self.goal != []:
                        distEgoGoal_gt, stamp = distPoseStamped2Odometry(self.goal, self.loc_gt, True)
                        data = [[stamp.to_sec(), distEgoGoal_gt]]
                        rob = self.spec_reachEgoGoal_gt.update(['distEgoGoal_gt', data])
                        publishRobstness(self.robPub_reachEgoGoal_gt, rob)
                        if rob != []:
                                self.robQue_reachEgoGoal_gt.put(rob)


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


        def lidar_callback(self, laser_message):
                lidarPointCloud2 = self.lp.projectLaser(laser_message)

                if self.loc_gt != []:
                        loc_gt_poseStamped = odometry2PoseStamped(self.loc_gt)
                        while not rospy.is_shutdown():
                                try:
                                        loc_gt_pose_frame_base_rage_sensor_link = self.tfListener.transformPose(lidarPointCloud2.header.frame_id, loc_gt_poseStamped)
                                        break
                                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                        continue
                        dists, stamp = distPoseStamped2PointCloud2(loc_gt_pose_frame_base_rage_sensor_link, lidarPointCloud2)
                        distEgoObs_gt = min(dists)
                        data = [[stamp.to_sec(), distEgoObs_gt]]
                        rob = self.spec_collEgoObs_gt.update(['distEgoObs_gt',data])
                        publishRobstness(self.robPub_collEgoObs_gt, rob)
                        if rob != []:
                                self.robQue_collEgoObs_gt.put(rob)

                if self.loc != []:
                        while not rospy.is_shutdown():
                                try:
                                        loc_pose_frame_base_rage_sensor_link = self.tfListener.transformPose(lidarPointCloud2.header.frame_id, self.loc)
                                        break
                                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                        continue
                        dists, stamp = distPoseStamped2PointCloud2(loc_pose_frame_base_rage_sensor_link, lidarPointCloud2)
                        distEgoObs = min(dists)
                        data = [[stamp.to_sec(), distEgoObs]]
                        rob = self.spec_collEgoObs.update(['distEgoObs',data])
                        if rob != []:
                                self.robQue_collEgoObs.put(rob)

                distLidar = numpy.amin(laser_message.ranges)
                data = [[laser_message.header.stamp.to_sec(), distLidar]]
                rob = self.spec_collLidar.update(['distLidar', data])
                publishRobstness(self.robPub_collLidar, rob)
                if rob != []:
                        self.robQue_collLidar.put(rob)

                if self.map != []:
                        lidarPointCloud = pointCloud22PointCloud(lidarPointCloud2)
                        while not rospy.is_shutdown():
                                try:
                                        lidarPointCloud_frame_map = self.tfListener.transformPointCloud(self.map.header.frame_id, lidarPointCloud)
                                        break
                                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                        continue
                        dists, stamp = distPointCloud2OccupancyGrid(lidarPointCloud_frame_map, self.map, True)
                        errLidar = numpy.average(dists)
                        data = [[stamp.to_sec(), errLidar]]
                        rob = self.spec_errLidar.update(['errLidar', data])
                        publishRobstness(self.robPub_errLidar, rob)
                        if rob != []:
                                self.robQue_errLidar.put(rob)


        def stereoCam_callback(self, pointCloud2):
                self.stereoCam = pointCloud2


        def globalPath_callback(self, path):
                self.globalPath = path

                if self.goal !=[]:
                        goalPoseStamped = self.globalPath.poses[-1]
                        distGlobalPathGoal, stamp = distPoseStamped2PoseStamped(self.goal, goalPoseStamped, True)
                        data = [[stamp.to_sec(), distGlobalPathGoal]]
                        rob = self.spec_reachGlobalPathGoal.update(['distGlobalPathGoal', data])
                        publishRobstness(self.robPub_reachGlobalPathGoal, rob)
                        if rob != []:
                                self.robQue_reachGlobalPathGoal.put(rob)

                if self.map != []:
                        dists, stamp = distPath2OccupancyGrid(self.globalPath, self.map, True)
                        distGlobalPathObs = numpy.min(dists)
                        data = [[stamp.to_sec(), distGlobalPathObs]]
                        rob = self.spec_collGlobalPathObs.update(['distGlobalPathObs', data])
                        publishRobstness(self.robPub_collGlobalPathObs, rob)
                        if rob != []:
                                self.robQue_collGlobalPathObs.put(rob)


        def baseVel_ref_callback(self, twist):
                self.baseVel_ref = twist


        def baseVel_callback(self, twist):
                self.baseVel = twist

                if self.baseVel_ref != []:
                        referrBodyVel = distTwist2Twist(self.baseVel, self.baseVel_ref)
                        now = rospy.get_rostime()
                        data = [[now.to_sec(), referrBodyVel]]
                        rob = self.spec_referrBodyVel.update(['referrBodyVel', data])
                        publishRobstness(self.robPub_referrBodyVel, rob)
                        if rob != []:
                                self.robQue_referrBodyVel.put(rob)


        def bumperFront_callback(self, data):
                header = Header()
                header.seq = 0
                header.stamp = rospy.Time.now()
                header.frame_id = ''

                boolStamped = BoolStamped(data, header)
                self.bumperFront = boolStamped


        def bumperBack_callback(self, data):
                header = Header()
                header.seq = 0
                header.stamp = rospy.Time.now()
                header.frame_id = ''

                boolStamped = BoolStamped(data, header)
                self.bumperBack = boolStamped


        def controllerInfo_callback(self, data):
                self.controllerInfo = data

                data = [[self.controllerInfo.header.stamp.to_sec(), self.controllerInfo.error.velocities[0]]]
                rob = self.spec_referrWheelVelL.update(['referrWheelVelL', data])
                publishRobstness(self.robPub_referrWheelVelL, rob)
                print_rob(rob, self.spec_referrWheelVelL)

                data = [[self.controllerInfo.header.stamp.to_sec(), self.controllerInfo.error.velocities[1]]]
                rob = self.spec_referrWheelVelR.update(['referrWheelVelR', data])
                publishRobstness(self.robPub_referrWheelVelR, rob)
                print_rob(rob, self.spec_referrWheelVelR)


        def monitor_callback(self, event):
                # 1) system -----
                print_robQue(self.robQue_collEgoObs_gt, self.spec_collEgoObs_gt)
                print_robQue(self.robQue_reachEgoGoal_gt, self.spec_reachEgoGoal_gt)

                # 2) perception -----
                if self.loc != [] and self.loc_gt != []:
                        errLoc, stamp = distPoseStamped2Odometry(self.loc, self.loc_gt)
                        data = [[stamp.to_sec(), errLoc]]
                        rob = self.spec_errLoc.update(['errLoc', data])
                        publishRobstness(self.robPub_errLoc, rob)
                        print_rob(rob, self.spec_errLoc)
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
                        print_rob(rob, self.spec_errWheelOdom)
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
                        print_rob(rob, self.spec_errLaserOdom)
                print_robQue(self.robQue_errLidar, self.spec_errLidar)

                # 3) planner -----
                print_robQue(self.robQue_collEgoObs, self.spec_collEgoObs)
                print_robQue(self.robQue_collLidar, self.spec_collLidar)
                print_robQue(self.robQue_collGlobalPathObs, self.spec_collGlobalPathObs)
                print_robQue(self.robQue_reachGlobalPathGoal, self.spec_reachGlobalPathGoal)
                print_robQue(self.robQue_reachEgoGoal, self.spec_reachEgoGoal)

                # 4) controller -----
                print_robQue(self.robQue_referrBodyVel, self.spec_referrBodyVel)

                # 5) others (intermidiate variables) -----


if __name__ == '__main__':
        # Process arguments
        p = argparse.ArgumentParser(description='rtamt STL Python Monitor')
        p.add_argument('--freq', nargs=1, required=True, help='Sampling frequency in Hz')
        args = p.parse_args(rospy.myargv()[1:])

        try:
	        rospy.init_node('hsr_stl_monitor')
                hsr_stl_monitor = HSR_STL_monitor()
                rospy.Timer(rospy.Duration(1.0/float(args.freq[0])), hsr_stl_monitor.monitor_callback)
                rospy.spin()

        except rospy.ROSInterruptException:
                pass
