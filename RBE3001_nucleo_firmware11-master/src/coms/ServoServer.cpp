#include "ServoServer.h"


void ServoServer::event(float * buffer) {
	float gripperStatus = buffer[0];
	
	myGripper->write(gripperStatus);
	wait(0.01);

}


