// Ros libraries
#include "ros/ros.h"

//Own codes
#include "scanner/lidar_data.h"
#include "servo_ctr/servo_angle.h"

// External libraries
#include <iostream>
 
void receiveMessageFromLidar(const scanner::lidar_data &msg);

void receiveMessageFromServo(const servo_ctr::servo_angle &msg);

int main (int argc, char **argv) {
    ros::init(argc, argv, "mapper");
    ros::NodeHandle n;
    std::cout << "Mapper running!\n";
    ros::Subscriber sub_lidar = n.subscribe("lidar_data", 1000, receiveMessageFromLidar);
    ros::Subscriber sub_servo = n.subscribe("servo_control", 1000, receiveMessageFromServo);

    ros::spin();
    return 0;
}

void receiveMessageFromLidar(const scanner::lidar_data &msg) {
    std::cout << "Message recieved from lidar\n";
}

void receiveMessageFromServo(const servo_ctr::servo_angle &msg) {
    std::cout << "Message recieved from servo: \n";
    std::cout << "Angle: " << msg.angle_degrees << "\n";
    std::cout << "Time: " << msg.time << "\n";
}