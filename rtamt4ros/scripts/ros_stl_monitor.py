#!/usr/bin/env python2
import rospy
import sys
import argparse
import copy
import rtamt

def callback(data, args):
    spec = args[0]
    var_name = args[1]

    var = copy.deepcopy(data)
    spec.var_object_dict[var_name] = var



def monitor(stl_arg, period_arg, unit_arg):

    period = int(period_arg[0])
    unit = unit_arg[0]


    filename = stl_arg[0]

    spec = rtamt.STLDiscreteTimeSpecification()
    spec.set_sampling_period(period, unit)
    freq = spec.get_sampling_frequency()

    spec.spec = spec.get_spec_from_file(filename)

    try:
        spec.parse()
        spec.pastify()
    except rtamt.STLParseException as err:
        print('STL Parse Exception: {}'.format(err))
        sys.exit()

    # Initialize the ROS node with the name coming from the specification file
    rospy.init_node(spec.name, anonymous=True)
    rospy.loginfo('Initialized the node STLMonitor')

    # Advertise the node as a publisher to the topic defined by the out var of the spec
    var_object = spec.get_value(spec.out_var)
    topic = spec.var_topic_dict[spec.out_var]
    rospy.loginfo('Registering as publisher to topic {}'.format(topic))
    pub = rospy.Publisher(topic, var_object.__class__, queue_size=10)

    # For each var from the spec, subscribe to its topic
    for var_name in spec.free_vars:
        var_object = spec.get_value(var_name)
        topic = spec.var_topic_dict[var_name]
        rospy.loginfo('Subscribing to topic ' + topic)
        rospy.Subscriber(topic, var_object.__class__, callback, [spec, var_name])

    # Set the frequency at which the monitor is evaluated
    rate = rospy.Rate(freq)

    time_index = 0

    while not rospy.is_shutdown():
        var_name_object_list = []
        for var_name in spec.free_vars:
            var_name_object = (var_name, spec.get_value(var_name))
            var_name_object_list.append(var_name_object)

        # Evaluate the spec
        # spec.update is of the form
        # spec.update(time_index, [('a',aObject), ('b',bObject), ('c',cObject)])
        robustness_msg = spec.update(time_index, var_name_object_list)

        robustness_msg.header.seq = time_index
        robustness_msg.header.stamp = rospy.Time.now()
        time_index = time_index + 1

        # Publish the result
        rospy.loginfo('Robustness: logical time: {0}, value: {1}'.format(robustness_msg.header.seq, robustness_msg.value))
        pub.publish(robustness_msg)

        # Wait until next evaluation
        rate.sleep()

if __name__ == '__main__':
    # Process arguments
    p = argparse.ArgumentParser(description='rtamt STL Python Monitor')
    p.add_argument('--stl', nargs=1, required=True, help='STL specification input')
    p.add_argument('--period', nargs=1, required=True, help='Sampling period')
    p.add_argument('--unit', nargs=1, required=True, help='Sampling unit')

    args = p.parse_args(rospy.myargv()[1:])
    try:
        monitor(args.stl, args.period, args.unit)
    except rospy.ROSInterruptException:
        pass
