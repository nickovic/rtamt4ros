#! /usr/bin/env python
# call roscore
# $ roscore
#
# IF start in manual
# $ rosrun rtamt4ros ros_stl_monitor_with_API2.py --freq 0.1

import rospy
import sys
import argparse
import rtamt

#other msg
from rtamt_msgs.msg import FloatMessage

DEBUG = False


class Monitor(object):
	def __init__(self):
                # STL settings
                # Load the spec from STL file
                self.spec = rtamt.STLDenseTimeSpecification()
                self.spec.name = 'HandMadeMonitor'
                self.spec.declare_var('a', 'float')
                self.spec.declare_var('c', 'float')
                self.spec.set_var_io_type('a', 'input')
                self.spec.set_var_io_type('c', 'output')
                self.spec.spec = 'c = always[0,5](a<=2)'

                try:
                        self.spec.parse()
                        self.spec.pastify()
                except Exception as err:
                        print('STL Parse Exception: {}'.format(err))
                        sys.exit()

                # For each var from the spec, subscribe to its topic
                self.a_subscriber = rospy.Subscriber('rtamt/a', FloatMessage, self.a_callback, queue_size=10)
                self.a = FloatMessage

                # Advertise the node as a publisher to the topic defined by the out var of the spec
                var_object = self.spec.get_var_object(self.spec.out_var)
                self.c_publisher = rospy.Publisher('rtamt/c', FloatMessage, queue_size=10)


        def a_callback(self, floatMessage):
                self.a = floatMessage

        def monitor_callback(self, event):
                # a
                a_data = [[self.a.header.stamp.to_nsec(), self.a.value]]
                rob = self.spec.update(['a', a_data])
                rospy.loginfo('Robustness: ' + str(rob))


if __name__ == '__main__':
        # Process arguments
        p = argparse.ArgumentParser(description='rtamt STL Python Monitor')
        p.add_argument('--freq', nargs=1, required=True, help='Sampling frequency in Hz')
        args = p.parse_args(rospy.myargv()[1:])

        try:
	        rospy.init_node('stl_monitor')
                monitor = Monitor()
                rospy.Timer(rospy.Duration(1.0/float(args.freq[0])), monitor.monitor_callback)
                rospy.spin()

        except rospy.ROSInterruptException:
                pass
