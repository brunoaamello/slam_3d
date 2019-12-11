#ifndef CONTROLLER_HH
#define CONTROLLER_HH

// Ros libraries
#include "ros/ros.h"

// External libraries

// Own codes
#include "robot_input/velocity_message.h"
#include "motor.hh"

class Controller {
private:
    robot_input::velocity_message _current_cmd;
    
    Motor *_left, *_right;
    
    bool isEq(robot_input::velocity_message a, robot_input::velocity_message b){
        bool eq = true;
        eq &= (a.x == b.x);
        eq &= (a.y == b.y);
        eq &= (a.z == b.z);
        eq &= (a.ang == b.ang);
        return eq;
    };
    
    void updateMotors(){
        int y, ang;
        if(_current_cmd.y < 0.0){
            y = -1;
        }else if(_current_cmd.y == 0){
            y = 0;
        }else{
            y = 1;
        }
        if(_current_cmd.ang < 0.0){
            ang = -1;
        }else if(_current_cmd.ang == 0){
            ang = 0;
        }else{
            ang = 1;
        }
        
        if(y == -1){
            if(ang == -1){
                _left->backward();
                _right->stop();
            }else if(ang == 0){
                _left->backward();
                _right->backward();
            }else{
                _left->stop();
                _right->backward();
            }
        }else if(y == 0){
            if(ang == -1){
                _left->backward();
                _right->forward();
            }else if(ang == 0){
                _left->stop();
                _right->stop();
            }else{
                _left->forward();
                _right->backward();
            }
        }else{
            if(ang == -1){
                _left->stop();
                _right->forward();
            }else if(ang == 0){
                _left->forward();
                _right->forward();
            }else{
                _left->forward();
                _right->stop();
            }
        }
    };

public:

    Controller() {
        if(wiringPiSetup()){
            fprintf(stderr, "Error initializing wiringPi, exiting.\n");
            exit(-1);
        }
        
        _left = new Motor(27, 28);
        _right = new Motor(23, 25);
        
        _left->stop();
        _right->stop();
        
        _current_cmd.x = 0.0;
        _current_cmd.y = 0.0;
        _current_cmd.z = 0.0;
        _current_cmd.ang = 0.0;
        
        
    };
    
    ~Controller(){
        delete _left, _right;        
    };

    void robotVelocityCallback(const robot_input::velocity_message &msg) {
        printf("Received new message: {%lf, %lf, %lf, %lf}\n", msg.x, msg.y, msg.z, msg.ang);
        bool update_motors = false;
        if(!isEq(_current_cmd, msg)){
            update_motors = true;
        }
        _current_cmd = msg;
        updateMotors();
    };


};

#endif
