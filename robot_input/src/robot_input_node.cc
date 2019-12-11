// Own code
#include "robot_input.hh"

// Own messages
#include "robot_input/velocity_message.h"

// Ros libraries
#include "ros/ros.h"

// External libraries
#include <stdio.h>
#include <thread>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "servo_ctr");

    ros::NodeHandle nodeHandle;
    robot_input::velocity_message msg;

    RobotInput* inputGrabber = new RobotInput();
    std::thread inputThread(&RobotInput::run, inputGrabber);

    ros::Publisher velocityPublisher = nodeHandle.advertise<robot_input::velocity_message>("/robot_velocity", 1000);
    ros::Rate loop_rate(10);

    while(ros::ok()) {

        msg = inputGrabber->getVelocityMsg();

        velocityPublisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    delete inputGrabber;

    inputThread.join();

    return 0;
}
