// Ros libraries
#include "ros/ros.h"

//Own codes
#include "scanner/lidar_data.h"
#include "servo_ctr/servo_angle.h"
#include "classes/mapper.hh"

// External libraries
#include <iostream>
 
int main (int argc, char **argv) {
    ros::init(argc, argv, "mapper");
    ros::NodeHandle n;
    std::cout << "Mapper running!\n";
    Mapper map;
    ros::Subscriber sub_lidar = n.subscribe("lidar_data", 1000, &Mapper::lidarCallback, &map);
    ros::Subscriber sub_servo = n.subscribe("servo_control", 1000, &Mapper::servoCallback, &map);

    ros::spin();
    return 0;
}
