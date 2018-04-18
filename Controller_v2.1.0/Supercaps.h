#ifndef Supercaps_h
#define Supercaps_h

#include "Arduino.h"

class Supercaps{
	public:
		Supercaps();
		void doSafetyChecks(double* desPowerIn);

		bool enabled;

		double voltage;
		double current;
		double power;

		bool fault = false;
		uint32_t faultDuration = 0;
    bool allGood = true;

		char errorMsg[200];
		bool errorDisp = false;
};


#endif
