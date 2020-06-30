#!/usr/bin/env python
import rospy
import sys
import argparse
import copy
import rtamt


def callback(data, args):
    input_data_dict = args[0]
    var_name = args[1]

    var = copy.deepcopy(data)

    input_data = input_data_dict[var_name]
    input_data.append(var)
    input_data_dict[var_name] = input_data


def monitor(period_arg, unit_arg):

    period = 3*int(period_arg[0])
    freq = 1.0 / period

    unit = unit_arg[0]

    input_data_dict = dict()

    spec = rtamt.STLIOCTSpecification(1)

    spec.name = 'HandMadeMonitor'
    spec.import_module('rtamt_msgs.msg', 'FloatMessage')
    spec.declare_var('a', 'FloatMessage')
    spec.declare_var('c', 'FloatMessage')
    spec.set_var_topic('a', 'rtamt/a')
    spec.set_var_topic('c', 'rtamt/c')
    spec.spec = 'c.value = always(a.value<=2)'

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
        rospy.Subscriber(topic, var_object.__class__, callback, [input_data_dict, var_name])

    # Set the frequency at which the monitor is evaluated
    rate = rospy.Rate(freq)

    while not rospy.is_shutdown():
        # the input has the following form
        # [
        #   ['var1', [[t11, v11], [t12, v12], ...] ],
        #   ['var2', [[t21, v21], [t22, v22], ...],
        #   ...
        # ]
        input_list = []
        for var in spec.free_vars:
            input_var_values = input_data_dict[var_name]
            var_values = []
            for var_value in input_var_values:
                now = var_value.header.stamp.to_sec();
                var_values.append([now, var_value])
            input_var_list = [var_name, var_values]
            input_list.append(input_var_list)
            input_data_dict[var_name] = []

        # Evaluate the spec
        # spec.update is of the form
        # spec.update(time_index, [('a',aObject), ('b',bObject), ('c',cObject)])


        if input_list:
            robustness_msgs = spec.update(*input_list)
            for msg in robustness_msgs:
                msg[1].header.stamp = rospy.Time.from_sec(msg[0])
                rospy.loginfo('Robustness: time: {0}, value: {1}'.format(msg[0], msg[1].value))
                pub.publish(msg[1])

        # Wait until next evaluation
        rate.sleep()

if __name__ == '__main__':
    # Process arguments
    p = argparse.ArgumentParser(description='rtamt STL Python Monitor')
    p.add_argument('--period', nargs=1, required=True, help='Sampling period')
    p.add_argument('--unit', nargs=1, required=True, help='Sampling unit')

    args = p.parse_args(rospy.myargv()[1:])
    try:
        monitor(args.period, args.unit)
    except rospy.ROSInterruptException:
        pass