// Own messages
#include "servo_ctr/servo_angle.h"

// Ros libraries
#include "ros/ros.h"
#include "ros/time.h"

// External libraries
#include <iostream>
#include <thread>

// Own libraries
#include "accel.hh"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "servo_ctr");

    ros::NodeHandle nodeHandle;
    servo_ctr::servo_angle msg;

    ros::Publisher anglePublisher = nodeHandle.advertise<servo_ctr::servo_angle>("/servo_control", 1000);
    
    
    Accelerometer acc(&anglePublisher);
    
    acc.run();

    return 0;
}
