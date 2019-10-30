#ifndef CONTROLLER_HH
#define CONTROLLER_HH

// Ros libraries
#include "ros/ros.h"

// External libs
#include <vector>

class Controller {
private:
    std::vector<robot_input::velocity_message> _cmd_buffer;

public:

    Controller() {

    };

    void robotVelocityCallback(const robot_input::velocity_message &msg) {
        _cmd_buffer.push_back(msg);
    };


};

#endif