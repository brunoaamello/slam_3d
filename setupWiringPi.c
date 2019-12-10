#include <wiringPi.h>
#include <stdio.h>

int main(){
	
	if(wiringPiSetup()){
		printf("Failed to initialize wiringPi, try to run with sudo.\n");
		return -1;
	}else{
		printf("WiringPi initialized successfully.\n");
	}
	return 0;
}

