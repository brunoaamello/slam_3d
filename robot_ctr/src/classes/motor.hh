#ifndef MOTOR_HH
#define MOTOR_HH

// External Libraries
#include <wiringPi.h>
#include <unistd.h>

class Motor {	
private:
	int _pin_a, _pin_b;
	bool _inverted;

	void write(int pin, bool val){
		if(_inverted){
			val = !val;
		}
		if(val){
			digitalWrite(pin, HIGH);
		}else{
			digitalWrite(pin, LOW);
		}
	};
	
public:
	Motor(int pin_a, int pin_b, bool inverted = true){
		_pin_a = pin_a;
		_pin_b = pin_b;
		_inverted = inverted;
		
		pinMode(_pin_a, OUTPUT);
		pinMode(_pin_b, OUTPUT);
		
	};

	void stop(){
		write(_pin_a, false);
		write(_pin_b, false);
	};
	
	void forward(){
		write(_pin_a, true);
		write(_pin_b, false);
	};
	
	void backward(){
		write(_pin_a, false);
		write(_pin_b, true);
	};
	
	
	
};




#endif
