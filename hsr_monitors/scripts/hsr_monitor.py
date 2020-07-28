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

def OccupancyGridPlot():
        pass


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
                self.tOdometry_subscriber = rospy.Subscriber('/global_pose', PoseStamped, self.tOdometry_callback, queue_size=10)
                self.poseStamped = PoseStamped()
                self.odometry_subscriber = rospy.Subscriber('/hsrb/odom_ground_truth', Odometry, self.odometry_callback, queue_size=10)
                self.odometry = Odometry()
                self.map_subscriber = rospy.Subscriber('/static_obstacle_map_ref', OccupancyGrid, self.map_callback, queue_size=10)
                self.occupancyGrid = OccupancyGrid()


                # Advertise the node as a publisher to the topic defined by the out var of the spec
                var_object = self.spec.get_var_object(self.spec.out_var)
                self.stl_publisher = rospy.Publisher('rtamt/c', var_object.__class__, queue_size=10)

        # it is not colled ctrl+Z
        def __del__(self):
                plt.close()
        

        def odometry_callback(self, Odometry_message):
                self.odometry = Odometry_message


        def tOdometry_callback(self, PoseStamped_message):
                self.poseStamped = PoseStamped_message

        # this will be called just one time.
        def map_callback(self, OccupancyGrid_message):
                rospy.loginfo('get map data')

                occupancyGrid = OccupancyGrid_message
                mapFig = plt.figure(figsize = (12,12))
                ax = mapFig.add_subplot(1, 1, 1)

                staticMap = numpy.asarray(occupancyGrid.data, dtype=numpy.int8).reshape(occupancyGrid.info.height, occupancyGrid.info.width)
                extent = [0 , occupancyGrid.info.width*occupancyGrid.info.resolution, 0, occupancyGrid.info.height*occupancyGrid.info.resolution]
                ax.imshow(staticMap, cmap=plt.cm.gray, extent=extent)
                obsIds = numpy.transpose(numpy.nonzero(staticMap))
                minId = obsIds.min(axis=0)
                maxId = obsIds.max(axis=0)
                space = 1.0
                left = (minId[0]*occupancyGrid.info.resolution)-space
                bottom = (occupancyGrid.info.height-maxId[1])*occupancyGrid.info.resolution-space
                right = (maxId[0]*occupancyGrid.info.resolution)+space
                top = (occupancyGrid.info.height-minId[1])*occupancyGrid.info.resolution+space
                ax.set_xlim([left, right])
                ax.set_ylim([bottom, top])
                ax.set_xlabel('y [m]')
                ax.set_ylabel('x [m]')
                #ax.set_aspect('equal', adjustable='box')
                #plt.tight_layout()
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
                tPose = self.poseStamped.pose
                rospy.loginfo('tOdometry: x: {0}, y: {1}'.format(tPose.position.x, tPose.position.y))
                cPose = self.odometry.pose.pose
                rospy.loginfo('odometry: x: {0}, y: {1}'.format(cPose.position.x, cPose.position.y))
                eOdom = distP2P(tPose.position.x, tPose.position.y, cPose.position.x, cPose.position.y)
                rospy.loginfo('eOdometry: {0}'.format(eOdom))


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
