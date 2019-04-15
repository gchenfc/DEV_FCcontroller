// ensure this library description is only included once
#ifndef Communication_h
#define Communication_h

#include "Arduino.h"
#include "Controller.h"
#include "Converter.h"
#include "Supercaps.h"
#include "StatsManager.h"
#include "Constants.h"
#include <i2c_t3.h>
#include <Metro.h>

#define I2Caddress 0x60
#define I2Cspeed 40000

// send/resp: check signals
//	- master send and slave reply to make sure i2c is working
// reg: register read
//	- respond with 32bit signed int
// com: command/acknowledge/failure to execute
//	- master sends command + 32bit signed int (send 0 if N/A)
//	- most commands are setpoints - i.e. try to make SC voltage 35V

#define I2CsendCHECK	0x00
#define I2CrespCHECK	-12345

#define I2CregFCV	0x10
#define I2CregFCI	0x11
#define I2CregSCV	0x12
#define I2CregSCI	0x13
#define I2CregFCP	0x14
#define I2CregSCP	0x15
#define I2CregH2FLOW	0x16
#define I2CregH2TOT	0x17
#define I2CregDC		0x18
#define I2CregFAN	0x19
#define I2CregSTATUS	0x1E
#define I2CregNUM	0x1F

// define PID target
//	ex. SCV means try to match SC voltage to setpoint
#define I2CcomFCV	0x20
#define I2CcomFCI	0x21
#define I2CcomSCV	0x22
#define I2CcomSCI	0x23
#define I2CcomFCP	0x24
#define I2CcomSCP	0x25
#define I2CcomPIDLIM	0x26
#define I2CcomSTART	0x2A
#define I2CcomSTOP	0x2B
#define I2CcomPAUSE	0x2C
#define I2CcomDCM	0x2D
#define I2CcomCCM	0x2E

#define I2C_WRITE_PURGE 0x50
#define I2C_WRITE_SHORT 0x51

#define I2C_WRITE_BOOTUP 0X58
#define I2C_WRITE_BOOTUPSHORTONLY 0x59
#define I2C_WRITE_BOOTUPSMALLPURGE 0x5A // purge for 500ms

volatile bool i2cDoCMD;
volatile bool i2cCMDerror;
volatile uint8_t i2cCMD;
volatile uint8_t i2cCOM;
volatile int32_t i2cCMDval;
uint8_t i2cmem[4];

Metro printStatsTimer = Metro(20);
Metro parseSerialTimer = Metro(100);
// serial
char buf[10];
int bufInd = 0;
bool pollMode = false;

Metro resetButtonTimer = Metro(1000);
bool resetButtonRecent = false;

extern uint32_t startTime;
extern double desPowerIn;

extern FCController FC;
extern Supercaps SC;
extern Converter Conv;
extern StatsManager Stats;

void Comm_setup();
void Comm_update();
void readDIPs();
void printStatsSerial();
void parseSerial();
void i2cReceiveEvent(size_t howMany);
void i2cReadVal(uint8_t cmd);
void i2cmemStore(int32_t val);
void i2cDoCmd(uint8_t cmd, int32_t val);
void i2cRequestEvent();
void i2cDoDriverCMD(uint8_t cmd);

bool buzzersEnabled = false;

Metro DIPcheckTimer = Metro(50);
bool DIP1bounce,DIP2bounce,DIP3bounce,DIP4bounce;

void Comm_setup(){
  #ifdef USE_I2C_BMS
  Wire1.begin(I2C_SLAVE, I2Caddress, I2C_PINS_29_30, I2C_PULLUP_EXT, I2Cspeed);
  pinMode(18,INPUT);
  pinMode(19,INPUT);
  Wire1.onReceive(i2cReceiveEvent);
  Wire1.onRequest(i2cRequestEvent);
  #endif

  i2cDoCMD = false;
  i2cCMDerror = false;
  i2cCMD = 0;
  i2cCOM = 0;
  i2cCMDval = 0;
  memset(i2cmem,0,sizeof(i2cmem));  
  
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  pinMode(LED4,OUTPUT);
  pinMode(DIP1,INPUT);
  pinMode(DIP2,INPUT);
  pinMode(DIP3,INPUT);
  pinMode(DIP4,INPUT);
  pinMode(BUZZ,OUTPUT);
}
void Comm_update(){
	if (i2cDoCMD){
		i2cDoCmd(i2cCOM,i2cCMDval);
	}

	if(printStatsTimer.check() || Conv.shortCircuit){
    printStatsSerial();
  }

  if (parseSerialTimer.check()){
    parseSerial();
  }

  if (DIPcheckTimer.check()){
//    readDIPs();
  }
  
//  buzzersEnabled = digitalReadFast(DIP4);
  buzzersEnabled = true;
  if (buzzersEnabled){
    if (FC.fault || !FC.allGood){
      tone(BUZZ,6000);
    }
    else if (SC.fault || !SC.allGood){
      tone(BUZZ,2000);
    }
//    else if (!Conv.enabled || !Conv.enabled){
//      tone(BUZZ,3000);
//    }
    else{
      noTone(BUZZ);
    }
  }
  else{
    noTone(BUZZ);
  }
}
void readDIPs(){
//  //        3
//  //      4   2
//  //        1
//  // (mounting holes)
//
//  // DO NOT USE BUTTON 2 - it's bad
//
//  if (DIP_PULLUP==0){
//    if (digitalRead(DIP1)){
//      if (!DIP1bounce){
//        desPowerIn = min(MAXDESPOWER,desPowerIn+15);
//      }
//      DIP1bounce = true;
//    }
//    else {
//      DIP1bounce = false;
//    }
//    if (digitalRead(DIP3)){
//      if (!DIP3bounce){
////        desPowerIn = min(desPowerIn+10,MAXDESPOWER);
//        desPowerIn = 15;
//      }
//      DIP3bounce = true;
//    }
//    else {
//      DIP3bounce = false;
//    }
//    
//    if (resetButtonTimer.check()){ // for double click
//      resetButtonRecent = false;
//    }
//    if (!digitalRead(DIP4)){
//      if (!DIP4bounce){
//        if (resetButtonRecent){ // double click
//          FC.bootup();
//          Conv.pause(1100);
//          resetButtonRecent = false;
//        }
//        else{
//          Conv.startShortCircuit(50);
//          resetButtonRecent = true;
//          resetButtonTimer.reset();
//        }
//      }
//      DIP4bounce = true;
//    }
//    else {
//      DIP4bounce = false;
//    }
//  }
//  
//  if (DIP_PULLUP==1){
//    pinMode(DIP1,OUTPUT);
//  //  pinMode(DIP2,OUTPUT);
//    pinMode(DIP3,OUTPUT);
//    pinMode(DIP4,OUTPUT);
//    digitalWrite(DIP1,HIGH);
//  //  digitalWrite(DIP2,HIGH);
//    digitalWrite(DIP3,HIGH);
//    digitalWrite(DIP4,HIGH);
//    pinMode(DIP1,INPUT_PULLUP);
//    if (!digitalRead(DIP1)){
//      if (!DIP1bounce){
//        desPowerIn = max(0,desPowerIn-10);
//      }
//      DIP1bounce = true;
//    }
//    else {
//      DIP1bounce = false;
//    }
//    pinMode(DIP3,INPUT_PULLUP);
//    if (!digitalRead(DIP3)){
//      if (!DIP3bounce){
//        desPowerIn = min(desPowerIn+10,MAXDESPOWER);
//      }
//      DIP3bounce = true;
//    }
//    else {
//      DIP3bounce = false;
//    }
//    pinMode(DIP4,INPUT_PULLUP);
//    if (!digitalRead(DIP4)){
//      if (!DIP4bounce){
//        FC.bootup();
//        Conv.pause(1100);
//      }
//      DIP4bounce = true;
//    }
//    else {
//      DIP4bounce = false;
//    }
//  }
}

void printStatsSerial(){
	//  // plot for PID tuning
	//  Serial.print(error*100,5);
	//  Serial.print('\t');
	//  Serial.print(FCpower,5);
	//  Serial.print('\t');
	//  Serial.print(setpointPower,5);
	//  Serial.print('\t');
	//  Serial.println(dutyCycle*100,3);
  // normal data transmition
  if (pollMode){
    return;
  }
  Serial.print((millis()-startTime)/1000.0,3);
  Serial.print("s\t");
  Serial.print(FC.voltage,2);
  Serial.print("V\t");
  Serial.print(FC.current,3);
  Serial.print("A\t");
  Serial.print(FC.power,3);
  Serial.print("W\t\t");
  Serial.print(SC.voltage,2);
  Serial.print("V\t");
  Serial.print(SC.current,3);
  Serial.print("A\t");
  Serial.print(SC.power,3);
  Serial.print("W\t\t");
  Serial.print(Conv.setpointPower,2);
  Serial.print("W\t");
  Serial.print(Conv.pid.getkI(),3);
  Serial.print("\t");
  Serial.print(Conv.pid.getkP(),3);
  Serial.print("\t");
  Serial.print(Conv.dutyCycle*100,3);
  Serial.print("%\t");
  Serial.print(Stats.converterEff*100,2);
  Serial.print("%\t");
  Serial.print(tempCtoF(FC.temp));
  Serial.print("°F\t");
	//    Serial.print(flowmeter.readVoltage(),5);
	//    Serial.print("V\t");
	//    Serial.print(flowmeter.readFlow(),2);
	//    Serial.print("mg/m\t");
  Serial.println();

  if (FC.errorDisp){
    Serial.print(FC.errorMsg);
    FC.errorDisp = false;
    FC.errorMsg[0] = 0;
  }
  if (Conv.errorDisp){
    Serial.print(Conv.errorMsg);
    Conv.errorDisp = false;
    Conv.errorMsg[0] = 0;
  }
  if (SC.errorDisp){
    Serial.print(SC.errorMsg);
    SC.errorDisp = false;
    SC.errorMsg[0] = 0;
  }
}
void parseSerial(){
  float lim = 0;
  bool tmp;
  if (Serial.available()){
    char c = Serial.read();
    switch(c){
      case 'p':
        FC.emergencyPause();
        break;
      case 'S':
      case 's':
        FC.emergencyShutdown();
        break;
      case 'b':
        FC.bootup();
        Conv.resume();
        Conv.pause(5500+500); // 500ms buffer time
        break;
      case 'r':
        tmp = Conv.shortCircuitEnabled;
        Conv.shortCircuitEnabled = true;
        Conv.startShortCircuit();
        Conv.shortCircuitEnabled = tmp;
        break;
      case 'R':
        Conv.shortCircuitEnabled = !Conv.shortCircuitEnabled;
        break;
      case 'u':
        FC.startPurge();
        break;
      case 'U':
        FC.purgeEnabled = !FC.purgeEnabled;
        break;
      case 'F':
        FC.setFan((float)atof(buf));
        for (int i=0;i<bufInd;i++){
          buf[i] = 0;
        }
        bufInd = 0;
        break;
      case 'L':
        lim = (float)atof(buf);
        Conv.pid.setLimits(0,lim);
        Conv.pid.setILimits(-.15,lim);
        for (int i=0;i<bufInd;i++){
          buf[i] = 0;
        }
        bufInd = 0;
        break;
      case 'W':
        desPowerIn = (float)atof(buf);
        for (int i=0;i<bufInd;i++){
          buf[i] = 0;
        }
        bufInd = 0;
        break;
      case '0':case '1':case '2':case '3':case '4':
      case '5':case '6':case '7':case '8':case '9':
      case '.':case '-':
        buf[bufInd] = c;
        bufInd++;
        break;
      case 'c':
        if (Conv.K<Conv.Kcrit){
          Serial.println("Cannot go into CCM:");
          Serial.print("K = ");
          Serial.print(Conv.K,5);
          Serial.print("\t");
          Serial.print("Kcrit = ");
          Serial.println(Conv.Kcrit,5);
          Serial.println("Staying in DCM");
          break;
        }
        Conv.CCM = true;
        Serial.println("Going into CCM");
//        Conv.initializeDC();
				// dutyCycle = 1-FCvoltage/SCvoltage;
				// setDC(dutyCycle);
        break;
      case 'd':
        Conv.CCM = false;
        Conv.initializeDC();
        break;
      case '(':
        pollMode = false; // to allow print
        printStatsSerial();
        pollMode = true;
      case ')':
        pollMode = false;
    }
  }
}

void i2cReceiveEvent(size_t count) {
//	Serial.println("GOT SOMETHING");
	if(count){
		i2cCMD = Wire1.readByte();
		switch(i2cCMD>>4){
			case (I2CsendCHECK>>4):
				i2cmemStore((int32_t)I2CrespCHECK);
				break;
			case (I2CregFCV>>4):
				i2cReadVal(i2cCMD); // sets i2cmem
				break;
			case (I2CcomFCV>>4): // don't respond
				if (Wire1.available()>=4){
					i2cCMDval = Wire1.readByte();
					i2cCMDval |= Wire1.readByte() << 8;
					i2cCMDval |= Wire1.readByte() << 16;
					i2cCMDval |= Wire1.readByte() << 24;
					i2cCOM = i2cCMD;
					i2cDoCMD = true; // will process during next call to update()
				}
				break;
      case (I2C_WRITE_PURGE>>4):
        i2cDoDriverCMD(i2cCMD);
        break;
		}
	}
}
void i2cReadVal(uint8_t cmd){
	switch(cmd){
		case I2CregFCV:
			i2cmemStore((int32_t) (FC.voltage*1000));
			break;
		case I2CregFCI:
			i2cmemStore((int32_t) (FC.current*1000));
			break;
		case I2CregFCP:
			i2cmemStore((int32_t) (FC.power*1000));
			break;
		case I2CregSCV:
			i2cmemStore((int32_t) (SC.voltage*1000));
			break;
		case I2CregSCI:
			i2cmemStore((int32_t) (SC.current*1000));
			break;
		case I2CregSCP:
			i2cmemStore((int32_t) (SC.power*1000));
			break;
		case I2CregH2FLOW:
			i2cmemStore((int32_t) 0);
			break;
		case I2CregH2TOT:
			i2cmemStore((int32_t) 0);
			break;
		case I2CregDC:
			i2cmemStore((int32_t) (Conv.dutyCycle*1000));
			break;
		case I2CregFAN:
			i2cmemStore((int32_t) (FC.getFan()*1000));
			break;
		case I2CregSTATUS:
			i2cmemStore((int32_t) (Stats.status));
			break;
		case I2CregNUM:
			i2cmemStore((int32_t) (Stats.statusAuxNum));
			break;
	}
}
void i2cmemStore(int32_t value){
	i2cmem[0] = value & 0xFF;
	i2cmem[1] = (value >> 8) & 0xFF;
	i2cmem[2] = (value >> 16) & 0xFF;
	i2cmem[3] = (value >> 24) & 0xFF;
}
void i2cDoCmd(uint8_t cmd,int32_t val){
	switch(cmd){
		case I2CcomFCV:
		case I2CcomFCI:
		case I2CcomSCV:
		case I2CcomSCI:
			// not yet implemented
			break;
		case I2CcomFCP:
			desPowerIn = val/1000.0;
			break;
		case I2CcomSCP:
			// not yet implemented
			break;
		case I2CcomPIDLIM:;
      Conv.pid.setLimits(0,val/1000.0);
      Conv.pid.setILimits(-.15,val/1000.0);
      break;
		case I2CcomSTART:
      FC.bootup();
      Conv.pause(1100);
      break;
		case I2CcomSTOP:
      FC.emergencyShutdown();
      break;
		case I2CcomPAUSE:
			FC.emergencyPause();
			break;
		case I2CcomDCM:
	    Conv.CCM = false;
	    Conv.initializeDC();
	    break;
		case I2CcomCCM:
			if (Conv.K<Conv.Kcrit){
        Serial.println("Cannot go into CCM:");
        Serial.print("K = ");
        Serial.print(Conv.K,5);
        Serial.print("\t");
        Serial.print("Kcrit = ");
        Serial.println(Conv.Kcrit,5);
        Serial.println("Staying in DCM");
        break;
      }
      else{
	      Conv.CCM = true;
	      Conv.initializeDC();
				// dutyCycle = 1-FCvoltage/SCvoltage;
				// setDC(dutyCycle);
	    }
      break;
	}
}
void i2cRequestEvent(void){
	switch (i2cCMD>>4){
		case (I2CsendCHECK>>4):
		case (I2CregFCV>>4):
			Wire1.write(&i2cmem[0],4);
			break;
	}
}
void i2cDoDriverCMD(uint8_t cmd){
  bool tmp;
  switch (cmd){
    case I2C_WRITE_PURGE:
      FC.startPurge();
      break;
    case I2C_WRITE_SHORT:
      tmp = Conv.shortCircuitEnabled;
      Conv.shortCircuitEnabled = true;
      Conv.startShortCircuit();
      Conv.shortCircuitEnabled = tmp;
      break;
    case I2C_WRITE_BOOTUP:
      FC.bootup();
      Conv.resume();
      Conv.pause(5500+500); // 500ms buffer time
      break;
    case I2C_WRITE_BOOTUPSHORTONLY:
      FC.bootupShortOnly();
      Conv.resume();
      Conv.pause(2500+500); // 500ms buffer time
      break;
    case I2C_WRITE_BOOTUPSMALLPURGE:
      FC.bootupSmallPurge();
      Conv.resume();
      Conv.pause(2600+500); // 500ms buffer time
      break;
  }
}

#endif
