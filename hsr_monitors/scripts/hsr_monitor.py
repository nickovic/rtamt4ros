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


class HSR_STL_monitor(object):
	def __init__(self):
                # STL settings
                # Load the spec from STL file
                # 1) system -----
                # collision with obstacle (Grand Truth): /hsrb/odom_ground_truth /static_obstacle_map_ref


                # reach goal (Grabd Truth): /hsrb/odom_ground_truth <goal>



                # 2) perception -----
                # localization error (Grand Truth): /hsrb/odom_ground_truth /global_pose
                self.spec_odomErr = rtamt.STLDenseTimeSpecification()
                self.spec_odomErr.name = 'odomErr'
                self.spec_odomErr.declare_var('odomErr', 'float')
                self.spec_odomErr.set_var_io_type('odomErr', 'input')
                self.spec_odomErr.spec = 'always [0,10] (odomErr >= 0.1)'

                # odometer error (Grand Truth): /hsrb/odom_ground_truth <odometer>

                # localization error LiDAR (Grand Truth): /hsrb/odom_ground_truth <LiDAR localizer>

                # LiDAR error (Grand Truth): hsrb/base_scan /static_obstacle_map_ref

                # StereoCamera error (Grand Truth): <Setereo_RGBD> <Gazebo3dshape>

                # Bumper error (Grand Truth): <Bumper> /static_obstacle_map_ref



                # 3) planner -----
                # collision with obstacle map: /global_pose /static_obstacle_map_ref
                
                # collision with obstacle LiDAR: hsrb/base_scan
                self.spec_scanDist = rtamt.STLDenseTimeSpecification()
                self.spec_scanDist.name = 'scanDist'
                self.spec_scanDist.declare_var('scanDist', 'float')
                self.spec_scanDist.set_var_io_type('scanDist', 'input')
                self.spec_scanDist.spec = 'always [0,10] (scanDist >= 0.2)'
                self.rob_scanDist_q = Queue.Queue()

                # collision with obstacle StereoCamera: <Setereo_RGBD>

                # collision with obstacle Bumper: <Bumper>

                # collision with obstacle GlobalPath: /base_path_with_goal /static_obstacle_map_ref
                self.spec_motionPathDist = rtamt.STLDenseTimeSpecification()
                self.spec_motionPathDist.name = 'motionPathDist'
                self.spec_motionPathDist.declare_var('motionPathDist', 'float')
                self.spec_motionPathDist.set_var_io_type('motionPathDist', 'input')
                self.spec_motionPathDist.spec = 'always [0,10] (motionPathDist >= 0.2)'
                self.rob_motionPathDist_q = Queue.Queue()

                # reach goal GlobalPath: /base_path_with_goal <goal>

                # reach goal: /global_pose  <goal>



                # 4) controller -----
                # ref body control: <ref VW> <VW>

                # ref wheel motor control: <ref rpm> <rpm>



                # 5) others (intermidiate variables) -----
                # virtualBumper
                # saftyMoveFlag




                try:
                        self.spec_odomErr.parse()
                        self.spec_odomErr.pastify()
                        self.spec_scanDist.parse()
                        self.spec_scanDist.pastify()
                        self.spec_motionPathDist.parse()
                        self.spec_motionPathDist.pastify()
                except STLParseException as err:
                        print('STL Parse Exception: {}'.format(err))
                        sys.exit()

                # For each var from the spec, subscribe to its topic
                self.laser_subscriber = rospy.Subscriber('hsrb/base_scan', LaserScan, self.scan_callback, queue_size=10)
                self.odometry_subscriber = rospy.Subscriber('/global_pose', PoseStamped, self.odometry_callback, queue_size=10)
                self.poseStamped = PoseStamped()
                self.tOdometry_subscriber = rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, self.tOdometry_callback, queue_size=10)
                self.odometry = Odometry()
                self.map_subscriber = rospy.Subscriber('/static_obstacle_map_ref', OccupancyGrid, self.map_callback, queue_size=10)
                self.occupancyGrid = OccupancyGrid()
                self.motion_path_subscriber = rospy.Subscriber('/base_path_with_goal', PathWithGoal, self.motion_path_callback, queue_size=10)
                self.pathWithGoal = PathWithGoal()

                # data init
                self.obss = []
                self.poseStamped = []
                self.odometry = []

                # Advertise the node as a publisher to the topic defined by the out var of the spec
                #var_object = self.spec.get_var_object(self.spec.out_var)
                #self.stl_publisher = rospy.Publisher('rtamt/c', var_object.__class__, queue_size=10)


        # it is not colled ctrl+Z
        def __del__(self):
                if DEBUG:
                        plt.close()


        def odometry_callback(self, poseStamped):
                self.poseStamped = poseStamped
        

        def tOdometry_callback(self, odometry):
                self.odometry = odometry


        # this will be called just one time.
        def map_callback(self, occupancyGrid):
                self.occupancyGrid = occupancyGrid
                staticMap = occupancyGridData2staticMap(occupancyGrid)
                obsIds = numpy.transpose(numpy.nonzero(staticMap))
                self.obss = mapids2mapCoordination(obsIds, occupancyGrid)

                if DEBUG:
                        mapFig = plt.figure(figsize = (12,12))
                        ax = mapFig.add_subplot(1, 1, 1)
                        occupancyGridPlot(ax, occupancyGrid)
                        plt.show()


        def scan_callback(self, laser_message):
                scanDist = numpy.amin(laser_message.ranges)

                # Evaluate the spec
                data = [[laser_message.header.stamp.to_sec(), scanDist]]
                rob = self.spec_scanDist.update(['scanDist', data])
                self.rob_scanDist_q.put(rob)


        def motion_path_callback(self, pathWithGoal):
                pathDist = distPoints2poses(self.obss, pathWithGoal.poses)

                # Evaluate the spec
                data = [[pathWithGoal.header.stamp.to_sec(), pathDist]]
                rob = self.spec_motionPathDist.update(['motionPathDist', data])
                self.rob_motionPathDist_q.put(rob)


        def monitor_callback(self, event):
                # data check
                if self.obss != [] and self.poseStamped != [] and self.odometry != []:
                        # odom
                        cPose = self.poseStamped.pose
                        if DEBUG:
                                rospy.loginfo('odometry: x: {0}, y: {1}'.format(cPose.position.x, cPose.position.y))
                        # true odom
                        tPose = self.odometry.pose.pose
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

                        # evaluate
                        time = min(self.poseStamped.header.stamp.to_sec(), self.odometry.header.stamp.to_sec())
                        data = [[time, dist]]
                        rob = self.spec_odomErr.update(['odomErr', data])
        
                        rospy.loginfo('rob {0}: {1}'.format(self.spec_odomErr.name, rob))

                # print robs
                if not self.rob_scanDist_q.empty():
                        rospy.loginfo('rob {0}: {1}'.format(self.spec_scanDist.name, self.rob_scanDist_q.get()))
                if not self.rob_motionPathDist_q.empty():
                        rospy.loginfo('rob {0}: {1}'.format(self.spec_motionPathDist.name, self.rob_motionPathDist_q.get()))


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
