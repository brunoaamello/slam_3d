#ifndef MAPPER_HH
#define MAPPER_HH

// Own codes
#include "servo_ctr/servo_angle.h"
#include "scanner/lidar_data.h"
#include "mouse_sensor/robot_position.h"

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
    std::vector<mouse_sensor::robot_position> _mouse_buffer;
    double _pointcloud_id;
    double _lidar_count;
    double _mouse_count;

    ros::Time _mouse_timer_start;

public:
    Mapper() {
        _pointcloud_id = 0;
        _lidar_count = 0;
        _mouse_count = 0;
        _mouse_timer_start = ros::Time::now();
    };

    double getLidarCount() {return _lidar_count;}

    double getMouseCount() {return _mouse_count;}

    void servoCallback(const servo_ctr::servo_angle &msg) {;
        _servo_buffer.push_back(msg);
    };

    void lidarCallback(const scanner::lidar_data &msg) {
        _lidar_data.push_back(msg);
        _lidar_count++;
    };
    
    void mouseCallback(const mouse_sensor::robot_position &msg) {
       /* if(_mouse_buffer.size() > 0 && (msg.x == _mouse_buffer.back().x && msg.y == _mouse_buffer.back().y && msg.z == _mouse_buffer.back().z)) {
            return;
        }*/
        if(ros::Time::now().toSec() - _mouse_timer_start.toSec() > 3) {
            _mouse_timer_start = ros::Time::now();

            _mouse_buffer.clear();
            _mouse_count = 0;

            std::cout << "[MAPPER] Clearing mouse buffer\n";

            return;
        }

        _mouse_buffer.push_back(msg);
        _mouse_count++;
    };

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
                double time = aux.time.back();

                // Procurar pelo elemento de mouse recebido no mesmo instante:
                std::vector<mouse_sensor::robot_position> curr_mouse_buffer = _mouse_buffer;
                double dx_robot = 0.0;
                double dy_robot = 0.0;
                double dz_robot = 0.0;
                int num_mouse_data = curr_mouse_buffer.size();
                // TODO: tratar caso de um só pacote: o pacote inicial nunca é apagado
                // Guardar na classe a posição atual do robô para o caso de não ter pacotes
                // ou tirar filtro de pacotes
                for(int k = num_mouse_data-1; k > 0; k--) {
                    if(curr_mouse_buffer[k].time == time) {
                        dx_robot = curr_mouse_buffer[k].x;
                        dy_robot = curr_mouse_buffer[k].y;
                        dz_robot = curr_mouse_buffer[k].z;

                        break;
                    } else if(curr_mouse_buffer[k].time > time && curr_mouse_buffer[k-1].time < time) {
                        double delta_time = curr_mouse_buffer[k].time-curr_mouse_buffer[k-1].time;
                        double diff_time = time-curr_mouse_buffer[k-1].time;

                        double ang_coef = (curr_mouse_buffer[k].x-curr_mouse_buffer[k-1].x)/delta_time;
                        dx_robot = ang_coef*diff_time+curr_mouse_buffer[k-1].x;

                        ang_coef = (curr_mouse_buffer[k].y-curr_mouse_buffer[k-1].y)/delta_time;
                        dy_robot = ang_coef*diff_time+curr_mouse_buffer[k-1].y;

                        ang_coef = (curr_mouse_buffer[k].z-curr_mouse_buffer[k-1].z)/delta_time;
                        dz_robot = ang_coef*diff_time+curr_mouse_buffer[k-1].z;
                        break;
                    }

                    curr_mouse_buffer.pop_back();
                    //_mouse_buffer.pop_back();
                    //_mouse_count--;
                }

                x = d*sin(theta);
                y = d*cos(theta);
                msg->points.push_back(pcl::PointXYZ(x+dx_robot, y+dy_robot, 1.0+dz_robot));
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