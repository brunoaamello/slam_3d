// Ros libraries
#include "ros/ros.h"

//Own codes
#include "classes/controller.hh"

// External libraries
#include <iostream>

int main(int argc, char **argv) {
        
    ros::init(argc, argv, "robot_ctr");
    ros::NodeHandle n;
    std::cout << "Robot controller running.\n";
    Controller ctr;

    ros::Subscriber sub = n.subscribe("/robot_velocity", 1000, &Controller::robotVelocityCallback, &ctr);

    ros::spin();

    return 0;
}
