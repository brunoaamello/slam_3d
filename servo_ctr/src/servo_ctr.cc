// Ros libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// External libraries
#include <sstream>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
    ros::init(argc, argv, "servo_ctr");

    ros::NodeHandle n;

    printf("Servo controller running, but not working!\n");

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(60);

    while(ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Fake angle: 60.00";

        msg.data = ss.str();

        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}