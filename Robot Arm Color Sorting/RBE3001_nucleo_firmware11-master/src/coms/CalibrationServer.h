#ifndef CalibrationServer_H
#define CalibrationServer_H

#include <PID_Bowler.h>
#include <PacketEvent.h>
#include "../drivers/MyPid.h"
#include <cmath>        // std::abs

#define CALIBRATIONSERVERID 9


class CalibrationServer: public PacketEventAbstract{
	//obtain these from pidServer templete
	private:
	  PIDimp ** myPidObjects;    // array of PidServers - one for each joint
	  int myPumberOfPidChannels; 
	  
	 public:
	  CalibrationServer (PIDimp ** pidObjects, int numberOfPidChannels)
	    : PacketEventAbstract(CALIBRATIONSERVERID)
	  {
	    myPidObjects = pidObjects;
	    myPumberOfPidChannels = numberOfPidChannels;
	  }
 
  	void event(float * buffer);
};


#endif /* end of include guard: Pid_server */


