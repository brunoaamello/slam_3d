// Own messages
#include "servo_ctr/servo_angle.h"

// Ros libraries
#include "ros/ros.h"
#include "ros/time.h"

// External libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "MPU6050.h"

class Accelerometer{
    private:
        bool _running;
	MPU6050 *_acc;
	ros::Publisher *_anglePublisher;
	uint8_t _addr;
	ros::Rate *_rate;

    public:
        static constexpr double PI = 3.1415926536;


    public:
        Accelerometer(ros::Publisher *pub, double frequency = 500.0, uint8_t address = 0x68){
		_anglePublisher = pub;
		_addr = address;
		_acc = new MPU6050(_addr);
		_rate = new ros::Rate(frequency);
	}
        ~Accelerometer(){
		_running = false;
		delete _rate;
		delete _acc;
	}

        void run(){
		servo_ctr::servo_angle msg;
		double angle;
		double time;
    
		float access;
    
		float x, y, z;
    
    
		while(true){
			time = ((double)ros::Time::now().toNSec())/1e9;
		
			_acc->getAccel(&x, &y, &z);
		
			z = -z;
			access = x;
			x = y;
			y = access;
		
			angle = atan2(y, z);
		
			msg.angle_radians = angle;
			msg.time = time;
			
			_anglePublisher->publish(msg);
			
			_rate->sleep();
			ros::spinOnce();
		}
	}

};

