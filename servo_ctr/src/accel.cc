#include "accel.hh"

Accelerometer::Accelerometer(ros::Publisher *pub, double frequency, uint8_t address){
	_anglePublisher = pub;
	_addr = address;
	_acc = new MPU6050(_addr);
	_rate = new ros::Rate(frequency);
}

Accelerometer::~Accelerometer(){
	_running = false;
	delete _rate;
	delete _acc;
}

double Accelerometer::getAngle(float, x, float y, float z){
	double ax = (double) x;
	double ay = (double) y;
	double az = (double) z;
	
	return ax;
	
}

void Accelerometer::run(){
    servo_ctr::servo_angle msg;
    double angle;
    double time;
    
    float x, y, z;
    
    
	while(true){
		time = ((double)ros::Time::now().toNSec())/1e9;
		
		_acc.getAccel(&x, &y, &z);
		
		msg.angle_radians = angle;
		msg.time = time;
		
		_rate->sleep();
	}
}
