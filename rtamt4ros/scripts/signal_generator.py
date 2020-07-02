#!/usr/bin/env python

import rospy
import argparse
from rtamt_msgs.msg import FloatMessage

def signal_generator(topic, freq, inc, reset):
    pub = rospy.Publisher(topic, FloatMessage, queue_size=10)
    rospy.init_node('signal_generator', anonymous=True)

    # Create a freq[Hz] timer
    rate = rospy.Rate(freq)
    
    # Initialize data
    message = FloatMessage()
    message.header.frame_id = 'signal'
    message.header.stamp = rospy.Time.now()
    message.value = 0.0

    while not rospy.is_shutdown():
        if message.value > reset:
            message.value = 0.0
        else:
            message.value = message.value + inc

        message.header.stamp = rospy.Time.now()
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    # Process arguments
    p = argparse.ArgumentParser(description='signal generator')
    p.add_argument('--topic', type=str, required=True, help='topic name')
    p.add_argument('--freq', type=float, default=1, help='update period [Hz]')
    p.add_argument('--inc',  type=float, default=0.001, help='increment step')
    p.add_argument('--reset', type=float, default=1.0, help='reset threshold')
    args = p.parse_args(rospy.myargv()[1:])

    try:
        signal_generator(args.topic, args.freq, args.inc, args.reset)
    except rospy.ROSInterruptException:
        pass
