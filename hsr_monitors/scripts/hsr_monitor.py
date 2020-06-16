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

#import rtamt
import rtamt
from rtamt.spec.stl.specification import STLSpecification
from rtamt.exception.stl.exception import STLParseException
from rtamt.spec.io_stl.io_interpretation import IOInterpretation

#other msg
from std_msgs.msg import String
from rtamt_msgs.msg import FloatMessage
from sensor_msgs.msg import LaserScan

DEBUG = False

class HSR_STL_monitor(object):
	def __init__(self, iosem_arg):
                iosem = iosem_arg[0]
                
                # STL settings
                # Load the spec from STL file
                self.spec = rtamt.STLIOCTSpecification()
                self.spec.name = 'HandMadeMonitor'
                self.spec.import_module('rtamt_msgs.msg', 'FloatMessage')
                self.spec.declare_var('closest_dist', 'float')
                #self.spec.declare_var('c', 'FloatMessage')
                self.spec.declare_var('c', 'float')
                self.spec.set_var_io_type('closest_dist', 'input')
                #self.spec.set_var_topic('c', 'rtamt/c')
                self.spec.set_var_io_type('c', 'output')
                #self.spec.spec = 'c.value = always [0:10] (closest_dist >= 0.2)'
                self.spec.spec = 'c = always [0:10] (closest_dist >= 0.2)'
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
                
                
                
        def scan_callback(self, data):
                self.laser_message = data
                
                
        def monitor_callback(self, event):
                closestDist = numpy.amin(self.laser_message.ranges)

                if DEBUG:
	                print 'Timer called at ' + str(event.current_real)
	                print 'closetDist=' + str(closestDist)
                
                # Evaluate the spec
                time_stamp = rospy.Time.now()
                rob = self.spec.update(['closest_dist', [(time_stamp.secs, closestDist)]])
                #robustness_msg = rob
                #robustness_msg.header.seq = robustness_msg.header.seq+1
                #robustness_msg.header.stamp = time_stamp
        
                # Publish the result
                rospy.loginfo('Robustness online: {}'.format(rob))
                #self.stl_publisher.publish(robustness_msg)


if __name__ == '__main__':
        # Process arguments
        p = argparse.ArgumentParser(description='rtamt STL Python Monitor')
        p.add_argument('--freq', nargs=1, required=True, help='Sampling frequency in Hz')
        p.add_argument('--iosem', nargs=1, type=str, required=False, default=[IOInterpretation.STANDARD], choices=list(IOInterpretation), help='IO STL semantics')

        args = p.parse_args(rospy.myargv()[1:])
        try:
	        rospy.init_node('hsr_stl_monitor')
                hsr_stl_monitor = HSR_STL_monitor(args.iosem)
	        rospy.Timer(rospy.Duration(1.0/float(args.freq[0])), hsr_stl_monitor.monitor_callback)
	        rospy.spin()
        except rospy.ROSInterruptException:
                pass
