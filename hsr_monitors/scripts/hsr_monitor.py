#! /usr/bin/env python
# call roscore
# $ roscore
#
# IF start in manual
# $ rosrun hsr_monitor hsr_monitor.py --freq 0.1

import rospy
import sys
import argparse
import logging
import copy
import numpy
import rtamt

import matplotlib.pyplot as plt

from webotPyLib import *

#other msg
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

def occupancyGridData2staticMap(occupancyGrid):
        staticMap = numpy.asarray(occupancyGrid.data, dtype=numpy.int8).reshape(occupancyGrid.info.height, occupancyGrid.info.width)
        return staticMap


def mapids2mapCoordination(mapIds, occupancyGrid):
        if mapIds.shape == (2,):        #for 1 id case
                mapIds = numpy.array([mapIds])
        pointsGridCoordinations = mapIds*occupancyGrid.info.resolution
        #TODO condider TF!
        pointsGridCoordinations = pointsGridCoordinations + [occupancyGrid.info.origin.position.x, occupancyGrid.info.origin.position.y]
        if pointsGridCoordinations.shape == (1,2):     #for 1 id case
                pointsGridCoordinations = pointsGridCoordinations[0]
        return pointsGridCoordinations


def distPoints2Position(points, position):
        # just thinking 2D (x,y)
        if points.shape == (2,):        #for 1 id case
                points = numpy.array([points])
        dists = points - numpy.array([position.x, position.y])
        dists = numpy.square(dists)
        dists = numpy.sum(dists,axis=1)
        dists = numpy.sqrt(dists)
        if dists.shape == (1,1):     #for 1 id case
                dists = dists[0]
        return dists
        

def OccupancyGridPlot(ax, occupancyGrid):
        staticMap = occupancyGridData2staticMap(occupancyGrid)
        extent = [occupancyGrid.info.origin.position.x, occupancyGrid.info.width*occupancyGrid.info.resolution  + occupancyGrid.info.origin.position.x,
                  occupancyGrid.info.origin.position.y, occupancyGrid.info.height*occupancyGrid.info.resolution + occupancyGrid.info.origin.position.y]
        ax.imshow(staticMap, cmap=plt.cm.gray, origin='lower', extent=extent)
        obsIds = numpy.transpose(numpy.nonzero(staticMap))
        minId = obsIds.min(axis=0)
        maxId = obsIds.max(axis=0)
        minCd = mapids2mapCoordination(minId, occupancyGrid)
        maxCd = mapids2mapCoordination(maxId, occupancyGrid)
        space = 1.0
        left = minCd[0]-space
        right = maxCd[0]+space
        bottom = minCd[1]-space
        top = maxCd[1]+space
        ax.set_xlim([left, right])
        ax.set_ylim([bottom, top])
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        #ax.set_aspect('equal', adjustable='box')
        #plt.tight_layout()


class HSR_STL_monitor(object):
	def __init__(self):
                # STL settings
                # Load the spec from STL file
                self.spec = rtamt.STLIOCTSpecification()
                self.spec.name = 'HandMadeMonitor'
                self.spec.import_module('rtamt_msgs.msg', 'FloatMessage')
                self.spec.declare_var('closest_dist', 'float')
                self.spec.declare_var('c', 'FloatMessage')
                self.spec.set_var_io_type('closest_dist', 'input')
                self.spec.set_var_topic('c', 'rtamt/c')
                self.spec.spec = 'c.value = always [0:10.0] (closest_dist >= 0.2)'

                try:
                        self.spec.parse()
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

                # Advertise the node as a publisher to the topic defined by the out var of the spec
                var_object = self.spec.get_var_object(self.spec.out_var)
                self.stl_publisher = rospy.Publisher('rtamt/c', var_object.__class__, queue_size=10)


        # it is not colled ctrl+Z
        def __del__(self):
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

                mapFig = plt.figure(figsize = (12,12))
                ax = mapFig.add_subplot(1, 1, 1)
                OccupancyGridPlot(ax, occupancyGrid)
                plt.show()


        def scan_callback(self, laser_message):
                closestDist = numpy.amin(laser_message.ranges)

                # Evaluate the spec
                time_stamp = rospy.Time.now()
                robustness_msgs = self.spec.update(['closest_dist', [[time_stamp.secs, closestDist]]])
                for msg in robustness_msgs:
                        msg[1].header.stamp = rospy.Time.from_sec(msg[0])
                        rospy.loginfo('Robustness: time: {0}, value: {1}'.format(msg[0], msg[1].value))
                        self.stl_publisher.publish(msg[1])


        def monitor_callback(self, event):
                # odom
                cPose = self.poseStamped.pose
                rospy.loginfo('odometry: x: {0}, y: {1}'.format(cPose.position.x, cPose.position.y))
                # true odom
                tPose = self.odometry.pose.pose
                rospy.loginfo('tOdometry: x: {0}, y: {1}'.format(tPose.position.x, tPose.position.y))
                # error odom
                eOdom = distP2P(tPose.position.x, tPose.position.y, cPose.position.x, cPose.position.y)
                rospy.loginfo('eOdometry: {0}'.format(eOdom))
                dists = distPoints2Position(self.obss, cPose.position)
                dist = numpy.min(dists)
                rospy.loginfo('dist ego obs: {0}'.format(dist))

                #poseOGcd =  mapids2mapCoordination(cPose.position, self.occupancyGrid)
                #rospy.loginfo(poseOGcd)

                #distEgoObs = distP2P(tPose.position.x, tPose.position.y, cPose.position.x, cPose.position.y)


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
