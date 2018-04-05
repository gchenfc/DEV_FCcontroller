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
		FCController();
		void doSafetyChecks(bool shortCircuit);
		void bootup();
		void shutdown();
		void emergencyPause();
		void emergencyShutdown();
		void update();

		bool enabled;
		bool purgeEnabled;

		bool startingUp = false;
		bool shuttingDown = false;

		double voltage = 0.0;
		double current = 0.0;
		double power = 0.0;
    double temp = 0.0;

		bool fault = false;
		uint32_t faultDuration = 0;

		char errorMsg[200];
		bool errorDisp = false;
		void setFan(double prct);
		double getFan();

	private:

		// FC control timers
		Metro purgeTimer = Metro(20000);
		Metro purgeEndTimer = Metro(20);
		Metro fanUpdateTimer = Metro(500);
		Metro postStartupTimer = Metro(1000);
		Metro postShutdownTimer = Metro(1000);
		Metro pausedTimer = Metro(3000);

		// FC control vars
		double fanPrct = .40;
		double purgePrct = 0.0;
		bool supplyPrct = 1;

		bool paused = false;
};

#endif

