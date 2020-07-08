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

#other msg
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

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
                self.ground_truth_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.grand_truth_callback, queue_size=10)

                # Advertise the node as a publisher to the topic defined by the out var of the spec
                var_object = self.spec.get_var_object(self.spec.out_var)
                self.stl_publisher = rospy.Publisher('rtamt/c', var_object.__class__, queue_size=10)
                
                rospy.spin()
        
        
        def grand_truth_callback(self, ModelStates_message):
                idx = ModelStates_message.name.index('hsrb')
                pose = ModelStates_message.pose[idx]
                rospy.loginfo('grand_truth: x: {0}, y: {1}'.format(pose.position.x, pose.position.y))


        def scan_callback(self, laser_message):
                closestDist = numpy.amin(laser_message.ranges)

                # Evaluate the spec
                time_stamp = rospy.Time.now()
                robustness_msgs = self.spec.update(['closest_dist', [[time_stamp.secs, closestDist]]])
                for msg in robustness_msgs:
                        msg[1].header.stamp = rospy.Time.from_sec(msg[0])
                        rospy.loginfo('Robustness: time: {0}, value: {1}'.format(msg[0], msg[1].value))
                        self.stl_publisher.publish(msg[1])


if __name__ == '__main__':
        # Process arguments
        p = argparse.ArgumentParser(description='rtamt STL Python Monitor')
        args = p.parse_args(rospy.myargv()[1:])

        try:
	        rospy.init_node('hsr_stl_monitor')
                hsr_stl_monitor = HSR_STL_monitor()
        except rospy.ROSInterruptException:
                pass
