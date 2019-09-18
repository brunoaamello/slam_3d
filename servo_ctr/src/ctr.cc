// Ros libraries
#include "ros/ros.h"
#include "servo_ctr/servo_angle.h"

// External libraries
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
    ros::init(argc, argv, "servo_ctr");

    ros::NodeHandle n;

    printf("Servo controller running, but not working!\n");

    ros::Publisher chatter_pub = n.advertise<servo_ctr::servo_angle>("servo_control", 1000);
    ros::Rate loop_rate(1000);

    while(ros::ok()) {
        servo_ctr::servo_angle msg;
        //TODO: controle do servo motor e geração da mensagem com dados reais
        msg.angle_degrees = 60;
        msg.angle_radians = 3.14159265358/3;
        msg.time = 1.00;

        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}