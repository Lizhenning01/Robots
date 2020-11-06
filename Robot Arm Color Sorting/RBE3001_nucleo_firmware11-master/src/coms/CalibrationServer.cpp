#include "CalibrationServer.h"


void CalibrationServer::event(float * buffer){
	float home[3];

	home[0] = buffer[0];
	home[1] = buffer[3];
	home[2] = buffer[6]; 
	

	  // we will be using the same memory area in which the incoming packet was stored,
	  // however, a we need to perform a type cast first (for convenience).
	uint8_t * buff = (uint8_t *) buffer;

	  // re-initialize the packet to all zeros
	for (int i = 0; i < 60; i++){
	    buff[i] = 0;
	}


	for(int i = 0; i < myPumberOfPidChannels; i++)
	{
		myPidObjects[i]->pidReset(myPidObjects[i]->GetPIDPosition() - home[i]);

		if (myPidObjects[i]->GetPIDPosition() > 3000)
			myPidObjects[i]->pidReset(myPidObjects[i]->GetPIDPosition() - 4095);

		// return pid packet data
		float position = myPidObjects[i]->GetPIDPosition();
		float velocity = myPidObjects[i]->getVelocity();
	        float torque   = myPidObjects[i]->loadCell->read();

	        buffer[(i*3)+0] = position;
	       	buffer[(i*3)+1] = velocity;
	        buffer[(i*3)+2] = torque;
	}

	

}


