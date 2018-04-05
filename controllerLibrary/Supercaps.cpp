#include "Supercaps.h"

#include "PinAssignments.h"

Supercaps::Supercaps(){
	// Serial.println("Initializing Supercaps...");
}
void Supercaps::doSafetyChecks(double* desPowerIn){
	bool allGood = true;
	if (!enabled){
		digitalWriteFast(LED3,LOW);
		return;
	}
	if (voltage<33){
		sprintf(errorMsg,"Supercaps under voltage... "
			"%.3fV\n",voltage);
		errorDisp = true;
		allGood = false;
		// *desPowerIn=min(*desPowerIn*1.1,50); // pseudocode
	}
	if (voltage>43.2){
		sprintf(errorMsg,"Supercaps over voltage... "
			"%.3fV\n",voltage);
		errorDisp = true;
		allGood = false;
		fault = true;
		faultDuration = 1000;
	}
	digitalWriteFast(LED3,allGood);
}