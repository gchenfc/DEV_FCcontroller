#include "Converter.h"
#include "Arduino.h"
#include "PinAssignments.h"
#include "StatsManager.h"

Converter::Converter(uint32_t* startTime, double* desPowerIn,
							double* FCvoltage, double* FCcurrent, double* FCpower,
							double* SCvoltage, double* SCcurrent, double* SCpower){
	// Serial.println("initializing Converter...");
	this->startTime = startTime;
	this->desPowerIn = desPowerIn;
	this->FCvoltage = FCvoltage;
	this->FCcurrent = FCcurrent;
	this->FCpower = FCpower;
	this->SCvoltage = SCvoltage;
	this->SCcurrent = SCcurrent;
	this->SCpower = SCpower;

	// set up PID controller
	pid = PID(&error, &dutyCycle, Kp,Ki,Kd,FORWARD);
	pid.setLimits(0,.5);
	pid.setPLimits(-.15,.15);
	pid.setILimits(-.15,.5);
	pid.setDLimits(-.01,.01);

	prevTime = micros();

	pinMode(STABLE12V,INPUT);
}

void Converter::doSafetyChecks(){
	
	bool allGood = true;
	if ((*FCcurrent>13) && (!shortCircuit)){
		Serial.print("Too much input current - converter pausing... ");
		Serial.print(*FCcurrent,3);
		Serial.println("A");
		allGood = false;
		pause();
	}
	if (*FCvoltage>30){
		Serial.print("Too much input voltage - converter pausing... ");
		Serial.print(*FCvoltage,3);
		Serial.println("V");
		allGood = false;
		pause();
	}
	if ((*FCvoltage<3) && (*FCcurrent>1)){ // probably shorted
		countShorts += 1;
		if (countShorts<10){
			Serial.print("Short suspected - converter pausing... #");
			Serial.print(countShorts);
			Serial.print("\t");
			Serial.print(*FCvoltage,3);
			Serial.println("V");
			allGood = false;
			pause();
		}
		else{
			Serial.print("Short suspected - converter stopping... #");
			Serial.print(countShorts);
			Serial.print("\t");
			Serial.print(*FCvoltage,3);
			Serial.println("V");
			allGood = false;
			// shutdown();
			pause();
		}
	}
	if (*SCcurrent>13){
		Serial.print("Too much output current - converter pausing... ");
		Serial.print(*SCcurrent,3);
		Serial.println("A");
		allGood = false;
		pause();
	}
	if (*SCvoltage>40){
		Serial.print("Too much output voltage - converter pausing... ");
		Serial.print(*SCvoltage,3);
		Serial.println("V");
		allGood = false;
		pause();
	}
	// if (enabled && (*SCvoltage)<15){
	// 	Serial.print("INSUFFICIENT INTERNAL DRIVE VOLTAGE ON SC... ");
	// 	Serial.print(*SCvoltage,3);
	// 	Serial.println("V");
	// 	(setpointPower)+=.1;
	// 	allGood = false;
	// }
	if (enabled && (stable12V)<9){
		Serial.print("INSUFFICIENT INTERNAL DRIVE VOLTAGE ON SC - pausing for 1s... ");
		Serial.print(stable12V,3);
		Serial.println("V");
		allGood = false;
		pause();
	}
	double expectedLossPrct = max((*SCcurrent)*2,1); // misnomer
	if (enabled &&
		!shortCircuit &&
		!statsLag &&
		(abs((*SCvoltage)/(*FCvoltage) / predictedM() - 1) > (.2*expectedLossPrct)) &&
		// (abs((*SCvoltage)/(*FCvoltage) -  predictedM(delayedDC)) > (.1*expectedLossPrct)) &&
		true){ // check CCM
			
		sprintf(errorMsg,"Measurements falling outside expected range... "
			"M = %.3f\tM(D)=%.3f\n",(*SCvoltage)/(*FCvoltage),predictedM());
		errorDisp = false;

		if ((*SCvoltage/ *FCvoltage)>(1.1/(1-dutyCycle))){ // DCM
			if (CCM && (K<Kcrit)){
				sprintf(errorMsg,"%sDCM error - switching to DCM\n",errorMsg);
				errorDisp = true;

		    CCM = false;
		    initializeDC();
			}
			sprintf(errorMsg,"%sSuspected DCM control error: resuming...\n",errorMsg);
			errorDisp = false;
			// pause();
		}
		else{ // something wrong
			if ((stable12V<11) || (stable12V>15)){ // probably 12V regulator malfunctioning
				sprintf(errorMsg,"%s12V regulator malfunction - please disconnect supercaps+fuel cell and reconnect\n",errorMsg);
				errorDisp = true;
			}
			else if ((*SCvoltage/ *FCvoltage)<.9){
				sprintf(errorMsg,"%sconverter is probably broken or in strange transient state...\n",errorMsg);
				sprintf(errorMsg,"%sContinuing operation...\n",errorMsg);
				errorDisp = true;
				statsLag = true;
				statsLagTimer.reset();
			}
			else{ 
				if (abs(*SCvoltage/ *FCvoltage - predictedM()) > (.15*expectedLossPrct)){
					sprintf(errorMsg,"%sUnknown error - pausing operation\n",errorMsg);
					errorDisp = true;
					pause();
				}
				else{
					sprintf(errorMsg,"%sUnknown error - continuing operation\n",errorMsg);
					errorDisp = true;
					statsLag = true;
					statsLagTimer.reset();
				}
			}
		}
		
		allGood = false;
	}
	if (statsLag && statsLagTimer.check()){
		statsLag = false;
	}
	if (!enabled){
		allGood = false;
	}
	digitalWriteFast(LED2,allGood);
	if (allGood){
		countShorts = max(countShorts-1,0);
	}
}

// converter management
void Converter::pause(uint32_t duration){
	if (duration == 0){
		setpointPower = 0;
		duration = 0xffffffff;
	}
	dutyCycle = 0;
  digitalWrite(HFET,0);
  digitalWrite(LFET,HIGH); // inverted
	// setDC(dutyCycle);
	enabled = false;
	disableTimer.interval(duration);
	disableTimer.reset();
	pid.reset();
}
void Converter::pause(){
	pause(1000);
}
void Converter::shutdown(){
	setpointPower = 0;
	pause(0);
}
void Converter::resume(){
	Serial.println("Resuming...");
	enabled = true;
	statsLag = true;
	statsLagTimer.reset();
	setpointPower = (*FCpower);
	disableTimer.interval(10000000);
	disableTimer.reset();

	initializeDC();
}
void Converter::initializeDC(){
	// see page 775
	// digitalWriteFast(LED4,HIGH);
	pinMode(LFET,OUTPUT);
	digitalWriteFast(LFET,HIGH);
	analogWriteResolution(PWM_RES);
	analogWriteFrequency(LFET,PWM_FREQ);
	// 	FTM1_POL = 0;                  // Positive Polarity 
	// 	FTM1_OUTMASK = 0xFF;           // Use mask to disable outputs
	// 	FTM1_SC = 0x08;                // set system clock as source for FTM0
	// 	FTM1_MOD = MODULO;             // Period register
	// 	FTM1_CNTIN = 0;                // Counter initial value
	// 	FTM1_COMBINE = 0x00000031;     // COMBINE=1, COMP=1, DTEN=1, SYNCEN=1            // page 796  (COMP1 sets complement)
	// 	FTM1_MODE = 0x01;              // Enable FTM0
	// 	FTM1_SYNC = 0x02;              // PWM sync @ max loading point enable
	// 	FTM1_DEADTIME = 0b00<<6;       // DeadTimer prescale systemClk/1                 // page 801
	// 	FTM1_DEADTIME |= 0b000000;     // 1uS DeadTime, max of 63 counts of 48Mhz clock  // page 801
	// 	FTM1_C0V = 0;                  // Combine mode, pulse-width controlled by...
	// 	FTM1_C1V = MODULO/2;           //   odd channel.
	// 	FTM1_SYNC |= 0x80;             // set PWM value update
	// 	FTM1_C0SC = 0x28;              // PWM output, edge aligned, positive signal
	// 	FTM1_C1SC = 0x28;              // PWM output, edge aligned, positive signal
		
	// 	CORE_PIN3_CONFIG = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;    //config teensy output port pins
	// 	CORE_PIN4_CONFIG = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;   //config teensy output port pins

	// 	if (CCM){
	// 		FTM1_OUTMASK = 0x0;            // Turns on PWM output
	// 	}
	// 	else{
	// 		FTM1_OUTMASK = 0x1; // masks high side
	// 	}

	// FTM1_C1V = MODULO*(1); // sets both low and high side
	// FTM1_SYNC |= 0x80;

	// digitalWriteFast(LED4,LOW);
	statsLag = true;
	statsLagTimer.reset();
	pid.reset();
	// pid.setScalars(0.0,predictedD(),0.0);
	// pid.update();
}
void Converter::setDC(double DC){
	if (DC>.75){
		DC = .75;
	}
	if (DC<0){
		DC = 0;
	}
	// FTM1_C1V = MODULO*(1-DC); // sets both low and high side
	// FTM1_SYNC |= 0x80;
	analogWrite(LFET,(1-DC)*MAXPWM);
	if (CCM){
		analogWrite(HFET,(1-DC)*MAXPWM);
	}
	else{
		digitalWriteFast(HFET,LOW);
	}
}
void Converter::setDC(){
	setDC(dutyCycle);
}
void Converter::update(){
	//  testingCycleDC()

	if (stable12VTimer.check()){
		check12V();
	}

	if (!enabled){
		if (disableTimer.check()){
			resume();
		}
		else{
			dutyCycle = 0;
			setDC(dutyCycle);
			return;
		}
	}

	if(shortCircuitEnabled){
		if(shortCircuitTimer.check()){
			doShortCircuit();
		}
	}
	if(shortCircuit && shortCircuitRecovTimer.check()){
		shortCircuit = false; // to give the voltage a chance to come back up
	}
	
	if (!(updateDCTimer.check() && (!shortCircuit))){
		return;
	}
	

  // DCM criteria: K<Kcrit (then DCM occurs)
  K = 2*33e-6/((*SCvoltage)/(*SCcurrent)/PWM_FREQ);
  Kcrit = dutyCycle*(1-dutyCycle)*(1-dutyCycle);

	if (CCM && (K<Kcrit)){
    sprintf(errorMsg,"DCM criteria detected - switching to DCM...\n");
    errorDisp = true;
    CCM = false;
    initializeDC();
  }
  
   // dutyCycle = 0.1;
	error = setpointPower - (*FCpower);
	pid.update();
	updateSetpoint();

	//  dutyCycle = dutyCycleBase+dutyCycleAdj;
	
	// error stats
	if((millis()-(*startTime))>5000){
		errorCount += 1;
		float delta = error-errorMean;
		errorMean += delta/errorCount;
		float delta2 = error-errorMean;
		errorM2 += delta*delta2;
	}
//  delayedDC = LPF(delayedDC,dutyCycle,.99);
//  delayedDC = dutyCycle-.05;
 
	setDC(dutyCycle);
}
void Converter::updateSetpoint(){
	if (statsLag){
		return;
	}
	double deltaT = (double)(micros()-prevTime)/1e6;
	prevTime = micros();
	if (abs((*desPowerIn)-setpointPower)>1){ // off by more than 1
		setpointPower = setpointPower + (20)*min(deltaT,.1)*(((*desPowerIn)>setpointPower)?1:-1);
	}
	else{ // pretty close
		setpointPower = setpointPower + ((*desPowerIn)-setpointPower)*1*min(deltaT,.1);
	}
	// if (abs(error)<1){1
	// dutyCycleBase = LPF(
	//  dutyCycleBase,
	//  1-(*FCvoltage)/((*SCpower+setpointPower-(*FCpower))/*SCcurrent),
	//  .99);
	// }
	// else{
	// dutyCycleBase = LPF(
	//  dutyCycleBase,
	//  1-(*FCvoltage)/((*SCpower+setpointPower-(*FCpower))/*SCcurrent),
	//  .9999);
	// }
}
void Converter::testingCycleDC(){
	int tConst = 180;
	float minVal = 0;
	float maxVal = 50;
	float stayTime = 15;
	float deltaPower = (maxVal-minVal)/(tConst/2/stayTime);
	if((millis()-(*startTime))/1000.0>tConst){
		(*startTime)=millis();
	}
	if (((millis()-(*startTime))/1000.0)<(tConst/2)){
		(*desPowerIn) = (millis()-(*startTime))/1000.0*((maxVal-minVal)/tConst*2) + minVal;
		(*desPowerIn) = round((*desPowerIn)/deltaPower)*deltaPower;
	}
	else{
		(*desPowerIn) = 2*(maxVal-minVal) - (millis()-(*startTime))/1000.0*((maxVal-minVal)/tConst*2) + minVal;
		(*desPowerIn) = round((*desPowerIn)/deltaPower)*deltaPower;
	}
	if((*desPowerIn)>11){
		(*desPowerIn)=11;
	}
}
double Converter::predictedM(){
  return predictedM(dutyCycle);
}
double Converter::predictedM(double D){
  if (K<Kcrit){ // DCM
    return (1+sqrt(1+4*D*D/K))/2;
  }
  else{
    return 1/(1-D);
  }
}
double Converter::predictedD(){
  return predictedD((*SCvoltage)/(*FCvoltage));
}
double Converter::predictedD(double M){
  if (K<Kcrit){ // DCM
    return sqrt(((2*M-1)*(2*M-1)-1)/4*K);
  }
  else{
    return 1-1/M;
  }
}

void Converter::doShortCircuit(){ // WARNING - THIS CODE IS BLOCKING, intentional
	// shortCircuit = true;
	// shortCircuitEndTimer.reset();
	// float oldDC = dutyCycle;
	// dutyCycle = 1;
	// setDC(dutyCycle);
	// while(!shortCircuitEndTimer.check()){
	//   // updateStats();
	//   // printStatsSerial(); // so that data collection is more accurate
	// }
	// dutyCycle = oldDC;
	// setDC(dutyCycle);
	// shortCircuitRecovTimer.reset();
}
double Converter::check12V(){
//	stable12V = LPF(stable12V,analogRead(STABLE12V)*3.0/1024*STABLE12_MULT * 12.23/13.15,.8);
//	return stable12V;
  stable12V = 12;
  return 12;
}
