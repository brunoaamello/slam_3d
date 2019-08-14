#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <stdio.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "slam_3d");

    ros::NodeHandle n;

    printf("This program does run.\n");

}
