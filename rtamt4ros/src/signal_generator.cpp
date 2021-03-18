/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   signal_generator.cpp
 * Author: nickovic
 *
 * Created on July 6, 2019, 1:58 PM
 */

#include <ros/ros.h>
#include <rtamt_msgs/FloatMessage.h>


/*
 *
 */
int main(int argc, char** argv) {
ros::init(argc, argv, "minimal_publisher_with_timer");
    ros::NodeHandle n;
    ros::Publisher publisher_a =
            n.advertise<rtamt_msgs::FloatMessage>("rtamt/a", 1);
    ros::Publisher publisher_b =
            n.advertise<rtamt_msgs::FloatMessage>("rtamt/b", 1);

    rtamt_msgs::FloatMessage message1;
    rtamt_msgs::FloatMessage message2;

    // Create a 1Hz timer
    ros::Rate naptime(1.0);

    // Initialize data
    message1.header.frame_id = "signal_generator";
    message1.header.seq = 0;
    message1.header.stamp = ros::Time::now();
    message1.value = 0.0;

    message2.header.frame_id = "signal_generator";
    message2.header.seq = 0;
    message2.header.stamp = ros::Time::now();
    message2.value = 0.0;

    while(ros::ok()) {
        message1.header.seq++;
        message1.header.stamp = ros::Time::now();
        message1.value = message1.value + 0.1;
        if(message1.value >= 2.0){
            message1.value = 0.0;
        }

        message2.header.seq++;
        message2.header.stamp = ros::Time::now();
        message2.value = message2.value + 0.3;
        if(message2.value >= 2.0){
            message2.value = 0.0;
        }

        publisher_a.publish(message1);
        publisher_b.publish(message2);

        // sleep 1 second
        naptime.sleep();
    }
    return 0;
}


