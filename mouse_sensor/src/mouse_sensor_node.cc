// Own codes
#include "mouse_sensor.hh"

// Own messages
#include "mouse_sensor/robot_position.h"

// Ros libraries
#include "ros/ros.h"
#include "ros/time.h"

// External libraries
#include <thread>

int main(int argc, char *argv[]){

    ros::init(argc, argv, "mouse_sensor_node");
    ros::NodeHandle nodeHandle;
    ros::Publisher messagePublisher = nodeHandle.advertise<mouse_sensor::robot_position>("robot_position", 1000);

    MouseSensor *myMouse;
    double loop_frequency;
    double time;
    mouse_sensor::robot_position pos;

    if(argc > 1){
        myMouse = new MouseSensor(atoi(argv[1]));
    }else{
        myMouse = new MouseSensor();
    }

    if(argc > 2){
        loop_frequency = (double) atoi(argv[2]);
    }else{
        loop_frequency = 120.0;
    }

    ros::Rate loop_rate(loop_frequency);

    std::thread myThread(&MouseSensor::run, myMouse);

    while(ros::ok()){
        time = ((double)ros::Time::now().toNSec())/1e9;
        pos.time = time;
        pos.x = myMouse->getX();
        pos.y = myMouse->getY();
        pos.z = 0.0;
        messagePublisher.publish(pos);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    delete myMouse;

    myThread.join();

}

