#ifndef MAPPER_HH
#define MAPPER_HH

// Own codes
#include "servo_ctr/servo_angle.h"
#include "scanner/lidar_data.h"

// Ros libs
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// External libs
#include <vector>
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Mapper {
private:
    std::vector<servo_ctr::servo_angle> _servo_buffer;
    std::vector<scanner::lidar_data> _lidar_data;
    double _pointcloud_id;
    double _lidar_count;

public:
    Mapper() {
        _pointcloud_id = 0;
        _lidar_count = 0;
    };

    double getLidarCount() {return _lidar_count;}

    void servoCallback(const servo_ctr::servo_angle &msg) {
        //std::cout << "Receiving servo data: \n";
        //std::cout << "Angle: " << msg.angle_degrees << "\n";
        //std::cout << "Time: " << msg.time << "\n";
        _servo_buffer.push_back(msg);
    };

    void lidarCallback(const scanner::lidar_data &msg) {
        _lidar_data.push_back(msg);
        _lidar_count++;
    };
    
    // Aparentemente mensagens publicadas do tipo PCL aparecem como
    // PointCloud2 no ROS, de acordo com: http://wiki.ros.org/pcl_ros
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudMessage(bool *isEmpty) {
        PointCloud::Ptr msg (new PointCloud);
        if(_lidar_data.size() < 1) {
            *isEmpty = true;
            return msg;
        }

        *isEmpty = false;

        msg->header.frame_id = "transform_reference";
        msg->height = 1;
        double num_points = 0;

        // Cálculo da posição:
        // [TODO] eixo Z e sincronizar mensagens
        double x = 0.0;
        double y = 0.0;
        scanner::lidar_data aux;
        int num_int = _lidar_count;
        for(int i = 0; i < num_int; i++) {
            aux = _lidar_data.back();
            _lidar_data.pop_back();
            _lidar_count--;
            num_points += aux.size;
            for(int j = 0; j < aux.size; j++) {
                double d = aux.range.back();
                double theta = aux.angle.back();
                x = d*sin(theta);
                y = d*cos(theta);
                msg->points.push_back(pcl::PointXYZ(x, y, 1.0));
                aux.range.pop_back();
                aux.angle.pop_back();
            }
        }

        msg->width = num_points;
        std::cout << "Width: " << msg->width << "\n";
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        return msg;
    }
};


#endif