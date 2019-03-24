#include "Supercaps.h"

#include "PinAssignments.h"
#include "Constants.h"

Supercaps::Supercaps(){
	// Serial.println("Initializing Supercaps...");
}
void Supercaps::doSafetyChecks(double* setpointPower){
  allGood = true;
	if (!enabled){
		digitalWriteFast(LED3,LOW);
		return;
	}
	if (voltage<25){
		sprintf(errorMsg,"Supercaps under voltage... "
			"%.3fV\n",voltage);
		errorDisp = true;
		allGood = false;
    fault = false; // definitely don't make this true - this is needed to keep the converter on while it recharges the supercaps
		*setpointPower=min(*setpointPower+(26.1-voltage)*.1,MAXDESPOWER); // pseudocode
//    if (*setpointPower > 100){
//      *setpointPower = 100;
//  	}
	}
	else if (voltage>40.5){
		sprintf(errorMsg,"Supercaps over voltage... "
			"%.3fV\n",voltage);
		errorDisp = true;
		allGood = false;
		fault = true;
		faultDuration = 1000;
	}
  else{
    fault = false;
  }
	digitalWriteFast(LED3,allGood);
}
