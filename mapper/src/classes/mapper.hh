#ifndef MAPPER_HH
#define MAPPER_HH

// Own codes
#include "servo_ctr/servo_angle.h"
#include "scanner/lidar_data.h"

// Ros libs
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

// External libs
#include <vector>

class Mapper {
private:
    std::vector<servo_ctr::servo_angle> _servo_buffer;
    std::vector<scanner::lidar_data> _lidar_data;

public:
    Mapper() {

    };

    void servoCallback(const servo_ctr::servo_angle &msg) {
        std::cout << "Receiving servo data: \n";
        std::cout << "Angle: " << msg.angle_degrees << "\n";
        std::cout << "Time: " << msg.time << "\n";
        _servo_buffer.push_back(msg);
    };

    void lidarCallback(const scanner::lidar_data &msg) {
        _lidar_data.push_back(msg);
    };
    
    sensor_msgs::PointCloud2 getPointCloudMessage();
};


#endif