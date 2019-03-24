/*
	Test.h - Test library for Wiring - description
	Copyright (c) 2006 John Doe.  All right reserved.
*/

// ensure this library description is only included once
#ifndef Controller_h
#define Controller_h

// // include types & constants of Wiring core API
// #include "WConstants.h"
#include "Arduino.h"
#include <Metro.h>
#include "PinAssignments.h"
#include "Constants.h"

class FCController{
	public:

	    static const double shortCircuitStartupIntervals[6];
	    static const double shortCircuitStartupDurations[6];

		enum STARTUP_STATE {
			STARTUP_DONE=0,
			STARTUP_PURGE,
			STARTUP_POSTPURGE,
			STARTUP_SHORT1,
			STARTUP_SHORT2,
			STARTUP_SHORT3,
			STARTUP_SHORT4,
			STARTUP_SHORT5,
			STARTUP_FANACCEL
		};
    
		FCController();
		void doSafetyChecks(bool shortCircuit,double* setpointPower);
		void bootup();
    void postStartup();
		void shutdown();
		void emergencyPause();
		void emergencyShutdown();
		void startPurge();
		void update();

		bool enabled;
		bool purgeEnabled;
    bool requestShort;
    uint32_t shortDuration;

		FCController::STARTUP_STATE startingUp;
		bool shuttingDown = false;

		double voltage = 0.0;
		double current = 0.0;
		double power = 0.0;
    double temp = 0.0;

		bool fault = false;
		uint32_t faultDuration = 0;
    bool allGood = true;

		char errorMsg[200];
		bool errorDisp = false;
		void setFan(double prct);
		double getFan();

	private:

		// FC control timers
		Metro purgeTimer = Metro(PURGE_INTERVAL);
		Metro purgeEndTimer = Metro(PURGE_DURATION);
		Metro fanUpdateTimer = Metro(500);
		Metro postStartupTimer = Metro(1000);
		Metro postShutdownTimer = Metro(1000);
		Metro pausedTimer = Metro(3000);

		// FC control vars
		double fanPrct = .4;
		double purgePrct = 0.0;
		bool supplyPrct = 1;

		bool paused = false;
};

#endif

