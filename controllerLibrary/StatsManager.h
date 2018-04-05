#ifndef StatsManager_h
#define StatsManager_h

#include <i2c_t3.h>
#include "PinAssignments.h"
#include "Constants.h"
#include "Arduino.h"

class StatsManager{
	public:
		StatsManager(bool* shortCircuit, double* FCtemp,
								double* FCvoltage, double* FCcurrent, double* FCpower,
								double* SCvoltage, double* SCcurrent, double* SCpower);
		void initializeStats();
		void updateStats();

		double* FCvoltage;
		double* FCcurrent;
		double* FCpower;
		double* SCvoltage;
		double* SCcurrent;
		double* SCpower;
		double converterEff;

		int32_t status = 0;
		int32_t statusAuxNum = 0;

		bool* shortCircuit;
		double* FCtemp;
};

double LPF(double oldVal, double newVal, double alpha);
double readSCvoltage();
double readSCcurrent();
double readFCvoltage();
double readFCcurrent();
double readTemp();
double tempCtoF(double tempC);
void INAinit();
uint16_t INAreadReg(uint8_t reg);

#endif