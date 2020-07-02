#!/usr/bin/env python
import rospy
import sys
import argparse
import copy
import rtamt
from rtamt_msgs.msg import FloatMessage

def callback(data, args):
    spec = args[0]
    var_name = args[1]
    pub = args[2]

    var = copy.deepcopy(data)
    var_time = var.header.stamp.to_sec()

    robustness_msgs = spec.update([var_name, [[var_time, var]]])
    for msg in robustness_msgs:
        msg[1].header.stamp = rospy.Time.from_sec(msg[0])
        rospy.loginfo('Robustness: time: {0}, value: {1}, update:{2}'.format(msg[0], msg[1].value, var_name))
        pub.publish(msg[1])

def monitor(unit_arg):

    unit = unit_arg

    input_data_dict = dict()

    spec = rtamt.STLIOCTSpecification(1)

    spec.name = 'HandMadeMonitor'
    spec.import_module('rtamt_msgs.msg', 'FloatMessage')
    spec.declare_var('a', 'FloatMessage')
    spec.declare_var('b', 'FloatMessage')
    spec.declare_var('c', 'FloatMessage')
    spec.set_var_topic('a', 'rtamt/a')
    spec.set_var_topic('b', 'rtamt/b')
    spec.set_var_topic('c', 'rtamt/c')
    spec.spec = 'c.value = a.value<=0.5 and b.value > 0.2'

    try:
        spec.parse()
    except STLParseException as err:
        print('STL Parse Exception: {}'.format(err))
        sys.exit()

    # Initialize the ROS node with the name coming from the specification file
    rospy.init_node(spec.name, anonymous=True)
    rospy.loginfo('Initialized the node STLMonitor')

    # Advertise the node as a publisher to the topic defined by the out var of the spec
    var_object = spec.get_var_object(spec.out_var)
    topic = spec.var_topic_dict[spec.out_var]
    rospy.loginfo('Registering as publisher to topic {}'.format(topic))
    pub = rospy.Publisher(topic, var_object.__class__, queue_size=10)

    # For each var from the spec, subscribe to its topic
    for var_name in spec.free_vars:
        var_object = spec.get_var_object(var_name)
        topic = spec.var_topic_dict[var_name]
        input_data_dict[var_name] = []
        rospy.loginfo('Subscribing to topic ' + topic)
        rospy.Subscriber(topic, var_object.__class__, callback, [spec, var_name, pub])

    rospy.spin()

if __name__ == '__main__':
    # Process arguments
    p = argparse.ArgumentParser(description='rtamt STL Python Monitor')
    p.add_argument('--unit', required=True, help='Sampling unit')

    args = p.parse_args(rospy.myargv()[1:])
    try:
        monitor(args.unit)
    except rospy.ROSInterruptException:
        pass