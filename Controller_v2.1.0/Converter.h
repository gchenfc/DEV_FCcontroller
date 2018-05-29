/*
  Converter.h - Handles voltage converting of the H2 controller board
*/

#ifndef Converter_h
#define Converter_h

#include <PID.h>
#include <Metro.h>
#include "Constants.h"

class Converter{
	public:
		Converter(uint32_t* startTime, double* desPowerIn,
							double* FCvoltage, double* FCcurrent, double* FCpower,
							double* SCvoltage, double* SCcurrent, double* SCpower);
		void doSafetyChecks();
		void pause(uint32_t duration);
		void pause();
		void shutdown();
		void resume();
		void initializeDC();
		void setDC(double DC);
		void setDC();
		void update();
		void updateSetpoint();
		void testingCycleDC();
    void startShortCircuit();
    void startShortCircuit(uint32_t duration);
    void updateSC();
		double check12V();

		double predictedM();
		double predictedM(double dutyCycle);
		double predictedD();
		double predictedD(double M);

		double* FCvoltage;
		double* FCcurrent;
		double* FCpower;
		double* SCvoltage;
		double* SCcurrent;
		double* SCpower;

		uint32_t* startTime;
		double setpointPower = 0; // constantly updated
		double* desPowerIn; // stays until user changes
    double desPowerInLowerLimit = 15;

		// converter control vars
		bool enabled = false;
    bool allGood = true;
		bool statsLag = false;
		bool shortCircuit = false;

		bool shortCircuitEnabled = false;
		Metro shortCircuitTimer = Metro(10000);
		Metro shortCircuitEndTimer = Metro(10);
		Metro shortCircuitRecovTimer = Metro(100);
    uint8_t shortCircuitStatus = SC_OFF;
    uint32_t shortCircuitDuration = 0;
    uint32_t shortCircuitRefTime = 0;

		float dutyCycleAdj = 0.0; // not implemented
		float dutyCycleBase = 0.0; // not implemented
		float dutyCycle = 0.0;
//    float delayedDC = 0.0;
		double K = 0;
		double Kcrit = 0;
		bool CCM = false;

		// converter control timers
		Metro updateDCTimer = Metro(1);
		Metro disableTimer = Metro(100000000);
		Metro statsLagTimer = Metro(100);
		Metro stable12VTimer = Metro(100);

		float error = 0.0;
		PID pid;
		// statistical error calculation
		unsigned long errorCount = 0;
		float errorMean = 0;
		float errorM2 = 0;
		long prevTime = 0;

		double stable12V = 0;

		char errorMsg[200];
		bool errorDisp = false;
		uint8_t countShorts = 0;
};

#endif

