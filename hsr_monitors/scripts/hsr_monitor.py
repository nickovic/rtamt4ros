#! /usr/bin/env python
# call roscore
# $ roscore
#
# IF start in manual
# $ rosrun hsr_monitors hsr_monitor.py --freq 10
# TODO:
# 1) Other agent senario
# 2) monitor with publisher and rtp polot.

import rospy
import sys
import argparse
import logging
import copy
import Queue

import rtamt

import matplotlib.pyplot as plt

from ros_distance_libs.rosDistLib import *

#other msg
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
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


class HSR_STL_monitor(object):
	def __init__(self):
                # STL settings
                # Load the spec from STL file
                # 1) system -----
                # collision with obstacle (Grand Truth): /hsrb/odom_ground_truth /static_obstacle_map_ref
                self.spec_collEgoObs_gt = rtamt.STLDenseTimeSpecification()
                self.spec_collEgoObs_gt.name = 'collEgoObs_gt'
                self.spec_collEgoObs_gt.declare_var('distEgoObs_gt', 'float')
                self.spec_collEgoObs_gt.set_var_io_type('distEgoObs_gt', 'input')
                self.spec_collEgoObs_gt.spec = 'always [0,10] (distEgoObs_gt >= 0.1)'

                # reach goal (Grabd Truth): /hsrb/odom_ground_truth /goal
                self.spec_reachEgoGoal_gt = rtamt.STLDenseTimeSpecification()
                self.spec_reachEgoGoal_gt.name = 'reachEgoGoal_gt'
                self.spec_reachEgoGoal_gt.declare_var('distEgoGoal_gt', 'float')
                self.spec_reachEgoGoal_gt.set_var_io_type('distEgoGoal_gt', 'input')
                self.spec_reachEgoGoal_gt.spec = 'eventually [0,10] (distEgoGoal_gt <= 0.1)'
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
                # localization error (Grand Truth): /hsrb/odom_ground_truth /global_pose
                # (localization error (Grand Truth): /hk0/gazebo_pose /hk0/global_pose)
                self.spec_errLoc = rtamt.STLDenseTimeSpecification()
                self.spec_errLoc.name = 'errLoc'
                self.spec_errLoc.declare_var('errLoc', 'float')
                self.spec_errLoc.set_var_io_type('errLoc', 'input')
                self.spec_errLoc.spec = 'always [0,10] (errLoc >= 0.1)'

                # odometer error (Grand Truth): /hsrb/odom_ground_truth /hsrb/odom
                self.spec_errOdom = rtamt.STLDenseTimeSpecification()
                self.spec_errOdom.name = 'errOdom'
                self.spec_errOdom.declare_var('errOdom', 'float')
                self.spec_errOdom.set_var_io_type('errOdom', 'input')
                self.spec_errOdom.spec = 'always [0,10] (errOdom >= 0.1)'

                # localization error LiDAR (Grand Truth): /hsrb/odom_ground_truth <LiDAR localizer>

                # LiDAR error (Grand Truth): hsrb/base_scan /static_obstacle_map_ref
                self.spec_errLidar = rtamt.STLDenseTimeSpecification()
                self.spec_errLidar.name = 'errLidar'
                self.spec_errLidar.declare_var('errLidar', 'float')
                self.spec_errLidar.set_var_io_type('errLidar', 'input')
                self.spec_errLidar.spec = 'always [0,10] (errLidar >= 0.1)'

                # StereoCamera error (Grand Truth): <Setereo_RGBD> <Gazebo3dshape>

                # Bumper error (Grand Truth): <Bumper> /static_obstacle_map_ref

                try:
                        self.spec_errLoc.parse()
                        self.spec_errLoc.pastify()
                        self.spec_errOdom.parse()
                        self.spec_errOdom.pastify()
                        self.spec_errLidar.parse()
                        self.spec_errLidar.pastify()
                except rtamt.STLParseException as err:
                        print('STL Parse Exception: {}'.format(err))
                        sys.exit()


                # 3) planner -----
                # collision with obstacle map: /global_pose /static_obstacle_map_ref
                self.spec_collEgoObs = rtamt.STLDenseTimeSpecification()
                self.spec_collEgoObs.name = 'collEgoObs'
                self.spec_collEgoObs.declare_var('distEgoObs', 'float')
                self.spec_collEgoObs.set_var_io_type('distEgoObs', 'input')
                self.spec_collEgoObs.spec = 'always [0,10] (distEgoObs >= 0.1)'

                # collision with obstacle LiDAR: hsrb/base_scan
                self.spec_collLidar = rtamt.STLDenseTimeSpecification()
                self.spec_collLidar.name = 'collLidar'
                self.spec_collLidar.declare_var('distLidar', 'float')
                self.spec_collLidar.set_var_io_type('distLidar', 'input')
                self.spec_collLidar.spec = 'always [0,10] (distLidar >= 0.2)'
                self.robQue_collLidar = Queue.Queue()

                # collision with obstacle StereoCamera: <Setereo_RGBD>

                # collision with obstacle Bumper: <Bumper>

                # collision with obstacle GlobalPath: /base_path_with_goal /static_obstacle_map_ref
                self.spec_collMotionPathObs = rtamt.STLDenseTimeSpecification()
                self.spec_collMotionPathObs.name = 'distMotionPathObs'
                self.spec_collMotionPathObs.declare_var('distMotionPathObs', 'float')
                self.spec_collMotionPathObs.set_var_io_type('distMotionPathObs', 'input')
                self.spec_collMotionPathObs.spec = 'always [0,10] (distMotionPathObs >= 0.2)'
                self.robQue_collMotionPathObs = Queue.Queue()

                # reach goal GlobalPath: /base_path_with_goal /goal
                self.spec_reachGlobalPathGoal = rtamt.STLDenseTimeSpecification()
                self.spec_reachGlobalPathGoal.name = 'reachGlobalPathGoal'
                self.spec_reachGlobalPathGoal.declare_var('distGlobalPathGoal', 'float')
                self.spec_reachGlobalPathGoal.set_var_io_type('distGlobalPathGoal', 'input')
                self.spec_reachGlobalPathGoal.spec = 'eventually [0,10] (distGlobalPathGoal <= 0.1)'
                self.robQue_reachEgoGoal = Queue.Queue()

                # reach goal: /global_pose /goal
                self.spec_reachEgoGoal = rtamt.STLDenseTimeSpecification()
                self.spec_reachEgoGoal.name = 'reachEgoGoal'
                self.spec_reachEgoGoal.declare_var('distEgoGoal', 'float')
                self.spec_reachEgoGoal.set_var_io_type('distEgoGoal', 'input')
                self.spec_reachEgoGoal.spec = 'eventually [0,10] (distEgoGoal <= 0.1)'

                try:
                        self.spec_collEgoObs.parse()
                        self.spec_collEgoObs.pastify()
                        self.spec_collLidar.parse()
                        self.spec_collLidar.pastify()
                        self.spec_collMotionPathObs.parse()
                        self.spec_collMotionPathObs.pastify()
                        self.spec_reachGlobalPathGoal.parse()
                        self.spec_reachGlobalPathGoal.pastify()
                        self.spec_reachEgoGoal.parse()
                        self.spec_reachEgoGoal.pastify()
                except rtamt.STLParseException as err:
                        print('STL Parse Exception: {}'.format(err))
                        sys.exit()


                # 4) controller -----
                # ref body control: /hsrb/command_velocity /base_velocity
                self.spec_referrBodyVel = rtamt.STLDenseTimeSpecification()
                self.spec_referrBodyVel.name = 'referrBodyVel'
                self.spec_referrBodyVel.declare_var('referrBodyVel', 'float')
                self.spec_referrBodyVel.set_var_io_type('referrBodyVel', 'input')
                self.spec_referrBodyVel.spec = 'always [0,10] (referrBodyVel <= 0.1)'

                # ref wheel motor control: <ref rpm> <rpm>

                try:
                        self.spec_referrBodyVel.parse()
                        self.spec_referrBodyVel.pastify()
                except rtamt.STLParseException as err:
                        print('STL Parse Exception: {}'.format(err))
                        sys.exit()


                # 5) others (intermidiate variables) -----
                # virtualBumper /hk0/zero_velocity :this one HSRB does not have.
                # local path generation status hk0/local_path_status_sim :perhaps /path_follow_action/status in HSRB
                # saftyMoveFlag /hk0/is_safety_move :this one HSRB does not have.


                # For each var from the spec, subscribe to its topic
                # system ground truth
                rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, self.odom_gt_callback, queue_size=10)
                self.loc_gt = []
                rospy.Subscriber('/static_obstacle_map_ref', OccupancyGrid, self.map_callback, queue_size=10)
                self.map = []

                # system order
                rospy.Subscriber('/goal', PoseStamped, self.goal_callback, queue_size=10)
                self.goal =[]

                # system sensor
                rospy.Subscriber('/global_pose', PoseStamped, self.loc_callback, queue_size=10)
                self.loc = []
                rospy.Subscriber('/hsrb/odom', Odometry, self.wheelOdom_callback, queue_size=10)
                self.wheelOdom = []
                rospy.Subscriber('/hsrb/base_scan', LaserScan, self.lidar_callback, queue_size=10)

                # system intermidiate data
                rospy.Subscriber('/base_path_with_goal', PathWithGoal, self.globalMotionPath_callback, queue_size=10)
                self.globalMotionPath = PathWithGoal()

                # data init
                self.obss = []

                # Advertise the node as a publisher to the topic defined by the out var of the spec
                #var_object = self.spec.get_var_object(self.spec.out_var)
                #self.stl_publisher = rospy.Publisher('rtamt/c', var_object.__class__, queue_size=10)


        # it is not colled ctrl+Z
        def __del__(self):
                if DEBUG:
                        plt.close()


        def loc_callback(self, poseStamped):
                self.loc = poseStamped

                if self.goal != []:
                        distEgoGoal, time = distPoseStamped2PoseStamped(self.goal, self.loc, True)
                        data = [[time, distEgoGoal]]
                        rob = self.spec_reachEgoGoal.update(['distEgoGoal', data])
                        if rob != []:
                                self.robQue_reachEgoGoal.put(rob)


        def odom_gt_callback(self, odometry):
                self.loc_gt = odometry

                if self.goal != []:
                        distEgoGoal_gt, time = distPoseStamped2Odometry(self.goal, self.loc_gt, True)
                        data = [[time, distEgoGoal_gt]]
                        rob = self.spec_reachEgoGoal_gt.update(['distEgoGoal_gt', data])
                        if rob != []:
                                self.robQue_reachEgoGoal_gt.put(rob)


        def wheelOdom_callback(self, odometry):
                self.wheelOdom = odometry


        def goal_callback(self, poseStamped):
                self.goal = poseStamped


        # this will be called just one time.
        def map_callback(self, occupancyGrid):
                staticMap = occupancyGridData2staticMap(occupancyGrid)
                obsIds = numpy.transpose(numpy.nonzero(staticMap))
                self.map = mapids2mapCoordination(obsIds, occupancyGrid)

                if DEBUG:
                        mapFig = plt.figure(figsize = (12,12))
                        ax = mapFig.add_subplot(1, 1, 1)
                        occupancyGridPlot(ax, occupancyGrid)
                        plt.show()


        def lidar_callback(self, laser_message):
                scanDist = numpy.amin(laser_message.ranges)

                # Evaluate the spec
                data = [[laser_message.header.stamp.to_sec(), scanDist]]
                rob = self.spec_collLidar.update(['distLidar', data])
                self.robQue_collLidar.put(rob)


        def globalMotionPath_callback(self, pathWithGoal):
                pathDist = distPoints2poses(self.obss, pathWithGoal.poses)

                # Evaluate the spec
                data = [[pathWithGoal.header.stamp.to_sec(), pathDist]]
                rob = self.spec_collMotionPathObs.update(['distMotionPathObs', data])
                self.robQue_collMotionPathObs.put(rob)


        def monitor_callback(self, event):
                # data check
                if self.obss != [] and self.loc != [] and self.odom_gt != []:
                        # odom
                        cPose = self.loc.pose
                        if DEBUG:
                                rospy.loginfo('odometry: x: {0}, y: {1}'.format(cPose.position.x, cPose.position.y))
                        # true odom
                        tPose = self.odom_gt.pose.pose
                        if DEBUG:
                                rospy.loginfo('tOdometry: x: {0}, y: {1}'.format(tPose.position.x, tPose.position.y))
                        # error odom
                        eOdom = distP2P(tPose.position.x, tPose.position.y, cPose.position.x, cPose.position.y)
                        if DEBUG:
                                rospy.loginfo('eOdometry: {0}'.format(eOdom))

                        # collision
                        dists = distPoints2pose(self.obss, cPose)
                        dist = numpy.min(dists)
                        if DEBUG:
                                rospy.loginfo('dist ego obs: {0}'.format(dist))

                # 1) system -----
                #spec_collEgoObs_gt.update(['distEgoObs_gt', data])
                print_robQue(self.robQue_reachEgoGoal_gt, self.spec_reachEgoGoal_gt)

                # 2) perception -----
                if self.loc != [] and self.loc_gt != []:
                        errLoc, time = distPoseStamped2Odometry(self.loc, self.loc_gt)
                        data = [[time, errLoc]]
                        rob = self.spec_errLoc.update(['errLoc', data])
                        print_rob(rob, self.spec_errLoc)
                if self.wheelOdom != [] and self.loc_gt != []:
                        errOdom, time = distOdometry2Odometry(self.wheelOdom, self.loc_gt)
                        data = [[time, errOdom]]
                        rob = self.spec_errOdom.update(['errOdom', data])
                        print_rob(rob, self.spec_errOdom)
                #self.spec_errLidar

                # 3) planner -----
                #self.spec_collEgoObs.update()
                print_robQue(self.robQue_collLidar, self.spec_collLidar)
                print_robQue(self.robQue_collMotionPathObs, self.spec_collMotionPathObs)
                #self.spec_reachGlobalPathGoal
                print_robQue(self.robQue_reachEgoGoal, self.spec_reachEgoGoal)

                # 4) controller -----
                #self.spec_referrBodyVel.update()

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
