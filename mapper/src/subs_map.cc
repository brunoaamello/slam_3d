#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include "geometry_msgs/PointStamped.h"

// External libs
#include <vector>

std::vector<geometry_msgs::PointStamped> _points;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
        //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
        geometry_msgs::PointStamped new_point;
        new_point.point.x = pt.x;
        new_point.point.y = pt.y;
        new_point.point.z = pt.z;
        new_point.header.frame_id = "transform_reference";
        new_point.header.stamp = ros::Time::now();
        _points.push_back(new_point);

    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("cloud_in", 1, callback);
  ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("my_points", 100000);
  
  ros::Rate loop_rate(60);

  while(ros::ok()) {
        if(_points.size() > 0) {
            geometry_msgs::PointStamped msg = _points.back();
            _points.pop_back();
            point_pub.publish(msg);
        }   

        
        ros::spinOnce();
        loop_rate.sleep();
    }

}