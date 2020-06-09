#! /usr/bin/env python
# call roscore
# $ roscore
#
# IF start in manual
# $ rosrun hsr_monitor hsr_monitor.py

import rospy
import sys
import argparse
import logging
import copy
import numpy

#import mondiro
import rtamt_stl_pymon

#other msg
from std_msgs.msg import String
from rtamt_msgs.msg import FloatMessage
from sensor_msgs.msg import LaserScan

DEBUG = True

class HSR_STL_monitor(object):
	def __init__(self, iosem_arg):
                iosem = iosem_arg[0]
                
                # STL settings
                # Load the spec from STL file
                self.spec = rtamt_stl_pymon.STLSpecification()
                self.spec.name = 'HandMadeMonitor'
                self.spec.import_module('rtamt_msgs.msg', 'FloatMessage')
                self.spec.declare_var('closest_dist', 'float')
                self.spec.declare_var('c', 'FloatMessage')
                self.spec.spec = 'c.value = always [0:10] (closest_dist >= 0.2)'
                self.spec.iosem = iosem
                try:
                        self.spec.parse()
                except STLParseException as err:
                        print('STL Parse Exception: {}'.format(err))
                        sys.exit()

                # For each var from the spec, subscribe to its topic
                self.laser_subscriber = rospy.Subscriber('hsrb/base_scan', LaserScan, self.scan_callback, queue_size=10)
                self.laser_message = LaserScan()

                # Advertise the node as a publisher to the topic defined by the out var of the spec
                self.stl_publisher = rospy.Publisher('rtamt/c', FloatMessage, queue_size=10)
                
                self.time_index = 0;
                
                
        def scan_callback(self, data):
                self.laser_message = data
                
                
        def monitor_callback(self, event):
                closestDist = numpy.amin(self.laser_message.ranges)

                if DEBUG:
	                print 'Timer called at ' + str(event.current_real)
	                print 'closetDist=' + str(closestDist)
                        
                # Evaluate the spec
                robustness_msg  = self.spec.update(self.time_index, [('closest_dist', closestDist)])
                robustness_msg.header.seq = self.time_index
                robustness_msg.header.stamp = rospy.Time.now()
                self.time_index = self.time_index + 1
        
                # Publish the result
                rospy.loginfo('Robustness: %s', robustness_msg.value)
                self.stl_publisher.publish(robustness_msg)


if __name__ == '__main__':
        # Process arguments
        p = argparse.ArgumentParser(description='rtamt STL Python Monitor')
        p.add_argument('--freq', nargs=1, required=True, help='Sampling frequency in Hz')
        p.add_argument('--iosem', nargs=1, type=rtamt_stl_pymon.IOInterpretation, required=False, default=[rtamt_stl_pymon.IOInterpretation.STANDARD], choices=list(rtamt_stl_pymon.IOInterpretation), help='IO STL semantics')
        args = p.parse_args(rospy.myargv()[1:])

        try:
	        rospy.init_node('hsr_stl_monitor')
	        hsr_stl_monitor = HSR_STL_monitor(args.iosem)
	        rospy.Timer(rospy.Duration(1.0/float(args.freq[0])), hsr_stl_monitor.monitor_callback)
	        rospy.spin()
        except rospy.ROSInterruptException:
                pass
