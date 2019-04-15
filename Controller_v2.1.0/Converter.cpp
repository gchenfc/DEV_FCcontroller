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
	pid.setLimits(0,.80);
	pid.setPLimits(-.15,.15);
	pid.setILimits(-.15,.80);
	pid.setDLimits(-.01,.01);

	prevTime = micros();

	pinMode(STABLE12V,INPUT);
}

void Converter::doSafetyChecks(){
	allGood =true;
	if ((true) && (*SCvoltage < 33)){ // hack - restructure this code later
//    setpointPower += .05;
//    setpointPower = min(setpointPower,100);
	}
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
	if ((!shortCircuit) && (*FCvoltage<3) && (*FCcurrent>1)){ // probably shorted
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
	if (*SCvoltage>38){
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
					sprintf(errorMsg,"%sUnknown error - not pausing operation\n",errorMsg);
					errorDisp = true;
					statsLag = true;
					statsLagTimer.reset();
//          pid.reset();
//          setpointPower = 10;
//					pause();
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
	if (!enabled){
		return;
	}
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
	pid.reset();

	initializeDC();
}
void Converter::initializeDC(){
	// see page 775
	// digitalWriteFast(LED4,HIGH);
	pinMode(LFET,OUTPUT);
	digitalWriteFast(LFET,HIGH);
//  pinMode(HFET,OUTPUT);
//  digitalWriteFast(HFET,LOW);
	analogWriteResolution(PWM_RES);
	analogWriteFrequency(LFET,PWM_FREQ);
//  analogWriteFrequency(HFET,PWM_FREQ);
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
	if ((!shortCircuit) && (DC>.75)){
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
//    analogWrite(HFET,.1*MAXPWM);
	}
	else{
		analogWrite(HFET,0);
//		digitalWriteFast(HFET,LOW);
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

	if(shortCircuitEnabled){
		if(shortCircuitTimer.check()){
			startShortCircuit();
		}
	}
	if(shortCircuit){
		updateSC();
	}
	
	if (!enabled){
		if (disableTimer.check()){
			resume();
		}
		else if (!shortCircuit){
			dutyCycle = 0;
			setDC(dutyCycle);
		}
	}
	
	// DCM criteria: K<Kcrit (then DCM occurs)
	K = 2*33e-6/((*SCvoltage)/(*SCcurrent)/PWM_FREQ);
	Kcrit = dutyCycle*(1-dutyCycle)*(1-dutyCycle);
	if (CCM && (K<Kcrit)){
		sprintf(errorMsg,"DCM criteria detected - switching to DCM...\n");
		errorDisp = true;
		CCM = false;
		initializeDC();
		setDC(dutyCycle);
	}
	
	if (enabled && (((updateDCTimer.check()) && (!shortCircuit)) || (shortCircuitStatus==SC_RECOV))){
		error = min(setpointPower - (*FCpower),10);
		float dutyCyclePrev = dutyCycle;
		pid.update();
		if (dutyCycle > (dutyCyclePrev + DC_CONV_SLEWLIM*.001)){
			dutyCycle = dutyCyclePrev + DC_CONV_SLEWLIM*.001;
		}
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

		// dutyCycle = .5;
		setDC(dutyCycle);
	}
}
void Converter::updateSetpoint(){
	if (statsLag){
		return;
	}
	double deltaT = (double)(micros()-prevTime)/1e6;
	prevTime = micros();
	if (abs((*desPowerIn)-setpointPower)>1){ // off by more than 1
		setpointPower = setpointPower + (5)*min(deltaT,.1)*(((*desPowerIn)>setpointPower)?1:-1);
	}
	else{ // pretty close
		setpointPower = setpointPower + ((*desPowerIn)-setpointPower)*1*min(deltaT,.1);
	}

	if ((setpointPower<desPowerInLowerLimit) && (*desPowerIn<desPowerInLowerLimit) && (*SCvoltage<39)){
		*desPowerIn = desPowerInLowerLimit;
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

void Converter::startShortCircuit(){ // not blocking :)
	startShortCircuit(10);
}
void Converter::startShortCircuit(uint32_t duration){ // not blocking :)
	if (!shortCircuitEnabled){
		return;
	}
	shortCircuit = true;
	shortCircuitDuration = duration;
	shortCircuitStatus = SC_RAMPUP;
	shortCircuitRefTime = micros();
}
void Converter::updateSC(){
	uint32_t t_now = micros();
	switch(shortCircuitStatus){
		case SC_OFF:
			shortCircuit = false;
			break;
		case SC_RAMPUP: // linearly ramp up duty cycle
			if (dutyCycle>.99){
				dutyCycle = 1;
				shortCircuitStatus = SC_HOLD;
				shortCircuitEndTimer.interval(shortCircuitDuration);
				shortCircuitEndTimer.reset();
			}
			dutyCycle = min(1,dutyCycle+50.0*(t_now-shortCircuitRefTime)/1e6);
			break;
		case SC_HOLD: // hold at 100%
			dutyCycle = 1;
			if (shortCircuitEndTimer.check()){
				shortCircuitStatus = SC_RAMPDOWN;
			}
			break;
		case SC_RAMPDOWN: // linearly ramp down duty cycle
			if (dutyCycle<.76){
				shortCircuitStatus = SC_RECOV;
				shortCircuitRecovTimer.reset();
				dutyCycle = 0; // supposedly another thread should take over DC at this point
			} else {
				dutyCycle = max(.75,dutyCycle-50.0*(t_now-shortCircuitRefTime)/1e6);
			}
			break;
		case SC_RECOV: // idk what this is for
			if(shortCircuitRecovTimer.check()){
				shortCircuit=false;
				shortCircuitStatus = SC_OFF;
			}
			break;
	}
	setDC(dutyCycle);
	shortCircuitRefTime = t_now;
}
double Converter::check12V(){
//	stable12V = LPF(stable12V,analogRead(STABLE12V)*3.0/4096*STABLE12_MULT * 12.23/13.15,.8);
//	return stable12V;
	stable12V = 12;
	return 12;
}
