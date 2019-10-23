// Ros libraries
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

//Own codes
#include "scanner/lidar_data.h"
#include "servo_ctr/servo_angle.h"
#include "classes/mapper.hh"

// External libraries
#include <iostream>
 
int main (int argc, char **argv) {
    bool msgIsEmpty = false;
    ros::init(argc, argv, "mapper");
    ros::NodeHandle n;
    std::cout << "Mapper running!\n";
    Mapper map;
    ros::Subscriber sub_lidar = n.subscribe("/lidar_data", 1000000, &Mapper::lidarCallback, &map);
    ros::Subscriber sub_servo = n.subscribe("/servo_control", 1000, &Mapper::servoCallback, &map);
    ros::Publisher mapper_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud_in", 10);
    ros::Subscriber sub_mouse = n.subscribe("/robot_position", 1000, &Mapper::mouseCallback, &map);

    // Para publicar a transformação de referencial:
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);

    ros::Rate loop_rate(0.5);

    while(ros::ok()) {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "transform_reference", "my_frame")); 
        std::cout << "Numero de pacotes lidar: " << map.getLidarCount() << "\n";
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg = map.getPointCloudMessage(&msgIsEmpty);
        if(!msgIsEmpty) {
            mapper_pub.publish(msg);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
