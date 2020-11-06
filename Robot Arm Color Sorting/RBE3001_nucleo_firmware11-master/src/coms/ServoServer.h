#ifndef ServoServer_H
#define ServoServer_H

#include "mbed.h"
#include "Servo.h"
#include <PacketEvent.h>
#include <cmath>        // std::abs

#define SERVOSERVERID 7


class ServoServer: public PacketEventAbstract{
	//obtain these from pidServer templete
	private:
	Servo* myGripper;

	public:
	  ServoServer (Servo* gripper)
	    : PacketEventAbstract(SERVOSERVERID) {

		myGripper = gripper;
	  }
 
  	void event(float * buffer);
};


#endif /* end of include guard: ServoServer */


