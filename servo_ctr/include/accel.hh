// Own messages
#include "servo_ctr/servo_angle.h"

// Ros libraries
#include "ros/ros.h"
#include "ros/time.h"

// External libraries
#include <stdio.h>
#include <stdlib.h>
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

    private:
		double getAngle(float, x, float y, float z);

    public:
        Accelerometer(ros::Publisher *pub, double frequency = 500.0, uint8_t address = 0x68);
        ~Accelerometer();

        void run();

};

