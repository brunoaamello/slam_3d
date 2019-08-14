#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>
#include <stdio.h>


void scanCallback(const sensor_msgs::LaserScan scan_data){
    int idx = 0;
    for(float angle = scan_data.angle_min; angle < scan_data.angle_max; angle+=scan_data.angle_increment){
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Range[" << idx << "]:" << scan_data.ranges[idx];
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        idx++;
    }

}


int main(int argc, char **argv){

    ros::init(argc, argv, "slam_3d");

    ros::NodeHandle n;

    printf("This program does run.\n");

    

    ros::Subscriber sub = n.subscribe("/scan", 1000, scanCallback);

    ros::spin();

/*
    ros::Publisher laser_pub = n.advertise<std_msgs::String>("laser_pub", 1000);
//    ros::Rate loop_rate(10);


    int count = 0;
        while (ros::ok()){
            std_msgs::String msg;
            std::stringstream ss;
            ss << "hello world " << count;
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());
            laser_pub.publish(msg);
            ros::spinOnce();
            //loop_rate.sleep();
            count++;
        }
        */

}
