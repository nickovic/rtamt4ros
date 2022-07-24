#! /usr/bin/env python
# call roscore
# $ roscore
#
# IF start in manual
# $ rosrun rtamt4ros ros_stl_monitor_unsync_update.py --freq 1

import rospy
import sys
import argparse
import Queue

import rtamt

#other msg
from rtamt_msgs.msg import FloatStamped

DEBUG = False


class Monitor(object):
	def __init__(self):
                # STL settings
                # Load the spec from STL file
                self.spec = rtamt.STLDenseTimeSpecification()
                self.spec.name = 'HandMadeMonitor'
                self.spec.declare_var('a', 'float')
                self.spec.declare_var('b', 'float')
                self.spec.declare_var('c', 'float')
                self.spec.set_var_io_type('a', 'input')
                self.spec.set_var_io_type('b', 'input')
                self.spec.set_var_io_type('c', 'output')
                self.spec.spec = 'c = always[0,5]( (a<=2) and (b >= 3) )'

                try:
                        self.spec.parse()
                        self.spec.pastify()
                except Exception as err:
                        print('STL Parse Exception: {}'.format(err))
                        sys.exit()

                # For each var from the spec, subscribe to its topic
                self.a_subscriber = rospy.Subscriber('rtamt/a', FloatStamped, self.a_callback, queue_size=10)
                self.b_subscriber = rospy.Subscriber('rtamt/b', FloatStamped, self.b_callback, queue_size=10)

                # Advertise the node as a publisher to the topic defined by the out var of the spec
                var_object = self.spec.get_value(self.spec.out_var)
                self.c_publisher = rospy.Publisher('rtamt/c', FloatStamped, queue_size=10)

                # queue for rob
                self.rob_q = Queue.Queue()


        def a_callback(self, floatStamped):
                a = floatStamped
                a_data = [[a.header.stamp.to_nsec(), a.value]]
                rob = self.spec.update(['a', a_data])
                self.rob_q.put(rob)


        def b_callback(self, floatStamped):
                b = floatStamped
                b_data = [[b.header.stamp.to_nsec(), b.value]]
                rob = self.spec.update(['b', b_data])
                self.rob_q.put(rob)


        def monitor_callback(self, event):
                rospy.loginfo('Robustness: ' + str(self.rob_q.get()))


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
