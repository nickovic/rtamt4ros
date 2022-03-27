#!/usr/bin/env python2
import rospy
import sys
import argparse
import copy
import rtamt


def init_spec(period, unit):
    spec = rtamt.STLDiscreteTimeSpecification()
    #spec = rtamt.StlDiscreteTimeOnlineSpecificationCpp()
    spec.set_sampling_period(period, unit)

    spec.name = 'HandMadeMonitor'
    spec.import_module('rtamt_msgs.msg', 'FloatStamped')
    spec.declare_var('req', 'FloatStamped')
    spec.declare_var('gnt', 'FloatStamped')
    spec.declare_var('rob', 'FloatStamped')
    spec.set_var_topic('req', 'rtamt/req')
    spec.set_var_topic('gnt', 'rtamt/gnt')
    spec.set_var_topic('rob', 'rtamt/gnt')
    spec.spec = 'rob.value = G[0,10](req.value >= 3) -> (F[0, 5] (gnt.value >= 3))'

    try:
        spec.parse()
        spec.pastify()
    except rtamt.STLParseException as err:
        sys.exit()
    return spec


def callback(data, args):
    spec = args[0]
    var_name = args[1]

    var = copy.deepcopy(data)
    spec.var_object_dict[var_name] = var


def sub_and_pub(spec):
    # Publish STL output var (robustness)
    topic = spec.var_topic_dict[spec.out_var]
    out = spec.get_value(spec.out_var)
    pub = rospy.Publisher(topic, out.__class__, queue_size=10)

    # Subscribe STL input vars
    for var_name in spec.free_vars:
        var_object = spec.get_value(var_name)
        topic = spec.var_topic_dict[var_name]
        rospy.loginfo('Subscribing to topic ' + topic)
        rospy.Subscriber(topic, var_object.__class__, callback, [spec, var_name])

    return pub


def mon_update(spec, time_index):
    var_name_object_list = []
    for var_name in spec.free_vars:
        var_name_object = (var_name, spec.get_value(var_name))
        var_name_object_list.append(var_name_object)

    # Update the monitor
    # spec.update is of the form
    # spec.update(time_index, [('a',aObject), ('b',bObject), ('c',cObject)])
    robustness_msg = spec.update(time_index, var_name_object_list)

    robustness_msg.header.seq = time_index
    robustness_msg.header.stamp = rospy.Time.now()

    return robustness_msg


def monitor(period, unit):
    # Init STL SPEC
    spec = init_spec(period, unit)
    # Init ROS node
    rospy.init_node(spec.name, anonymous=True)
    # Init subscriber and publisher
    pub = sub_and_pub(spec)
    # Set monitoring frequency
    #rate = rospy.Rate(spec.get_sampling_frequency())
    rate = rospy.Rate(1.0)
    # Control loop
    time_index = 0
    while not rospy.is_shutdown():
        rob_msg = mon_update(spec, time_index)
        rospy.loginfo('Robustness: logical time: {0}, value: {1}'.format(rob_msg.header.seq, rob_msg.value))
        pub.publish(rob_msg)
        time_index += 1

        # Wait until next evaluation
        rate.sleep()


if __name__ == '__main__':
    # Process arguments
    p = argparse.ArgumentParser(description='rtamt STL Python Monitor')
    p.add_argument('--period', nargs=1, required=True, help='Sampling period')
    p.add_argument('--unit', nargs=1, required=True, help='Sampling unit')

    args = p.parse_args(rospy.myargv()[1:])
    try:
        period_arg = args.period
        period = int(period_arg[0])
        args_unit = args.unit
        unit = args_unit[0]
        monitor(period, unit)
    except rospy.ROSInterruptException:
        pass