// Own messages
#include "servo_ctr/servo_angle.h"

// Ros libraries
#include "ros/ros.h"
#include "ros/time.h"

// External libraries
#include <iostream>
#include <thread>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "servo_ctr");

    ros::NodeHandle nodeHandle;
    servo_ctr::servo_angle msg;

    printf("Servo controller running, but not working!\n");

    ros::Publisher anglePublisher = nodeHandle.advertise<servo_ctr::servo_angle>("servo_control", 1000);
    ros::Rate loop_rate(1000);

    while(ros::ok()) {
        //TODO: controle do servo motor e geração da mensagem com dados reais
        msg.angle_radians = 0;
        msg.time = ((double)ros::Time::now().toNSec())/1e9;

        anglePublisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}