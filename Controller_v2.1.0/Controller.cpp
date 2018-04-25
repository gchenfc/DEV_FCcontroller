#include "Controller.h"
#include "Arduino.h"
#include <Metro.h>
#include "PinAssignments.h"
#include "Constants.h" 

void postStartup();
void postShutdown();
bool startingUp = false;

FCController::FCController(){
  // digitalWriteFast(LED4,HIGH);
  // pinMode(LFET,OUTPUT);
  // digitalWriteFast(LFET,HIGH);
  // delay(100);
  // Serial.println("Initializing FC Controller...");
  
  pinMode(TEMP,INPUT);
  pinMode(FAN,OUTPUT);
  pinMode(PURGE,OUTPUT);
  pinMode(SUPPLY,OUTPUT);
}

void FCController::doSafetyChecks(bool shortCircuit,double* setpointPower){
  allGood = true;
  if ((!enabled) || (startingUp) || (paused)){
    digitalWriteFast(LED1,LOW);
    return;
  }
  if (power>130){
    sprintf(errorMsg,"%sFC power too high... "
      "%.3fW\n",errorMsg,power);
    errorDisp = true;
    allGood = false;
    emergencyPause();
  }
  if ((voltage<12) && (voltage!=0) && (!shortCircuit)) {
    *setpointPower = max(*setpointPower-(voltage-11)/1,10);
  }
  if (((voltage<11) && (voltage!=0) && (!shortCircuit)) || (voltage>20)){
    sprintf(errorMsg,"%sFC voltage out of range... "
      "%.3fV\n",errorMsg,voltage);
    errorDisp = true;
    allGood = false;
    emergencyPause();
  }
  if(temp>65){
    sprintf(errorMsg,"%sFC Temperature Too High ("
      "%.1f°C)\n",errorMsg,temp);
    errorDisp = true;
    allGood = false;
    emergencyShutdown();
  }
  if ((-2/3*(current-2.5)-14.75)<1){ // 1V below curve
//    aoeu
  }
  digitalWriteFast(LED1,allGood);
}

// FC management
void FCController::bootup(){
  enabled = true;
  Serial.println("Booting up Fuel Cell");
  analogWrite(FAN,.8*MAXPWM);
  digitalWrite(PURGE,HIGH);
  digitalWrite(SUPPLY,HIGH);
  startingUp = true;
  postStartupTimer.reset();
}
void FCController::postStartup(){
  digitalWrite(PURGE,LOW);
  requestShort = true;
  shortDuration = 50;
  // analogWrite(FAN,fanPrct*MAXPWM); // this will automatically update
  // purgeTimer.reset(); // whatever
}
void FCController::shutdown(){
  digitalWrite(PURGE,HIGH);
  enabled = false;

  shuttingDown = true;
  postShutdownTimer.reset();
}
void postShutdown(){
  digitalWrite(SUPPLY,LOW);
  digitalWrite(PURGE,LOW);
  digitalWrite(FAN,LOW);
}
void FCController::emergencyPause(){
  Serial.println("Fuel cell fault - pausing for 3s");
  fault = true;
  faultDuration = 3000;
  paused = true;
  pausedTimer.interval(faultDuration);
  pausedTimer.reset();
}
void FCController::emergencyShutdown(){
  Serial.println("Fuel cell fualt - SHUTTING DOWN!!!");
  fault = true;
  faultDuration = 0;
  shutdown();
}
void FCController::update(){
  if (startingUp && postStartupTimer.check()){
    postStartup();
    startingUp = false;
  }
  if (shuttingDown && postShutdownTimer.check()){
    postShutdown();
    shuttingDown = false;
  }

  if (!enabled){
    return;
  }

  if (purgeEnabled){
    if(purgeTimer.check()){
      digitalWrite(PURGE,HIGH);
      purgeEndTimer.reset();
    }
    if(purgeEndTimer.check()){
      digitalWrite(PURGE,LOW);
    }
  }
  
  if(fanUpdateTimer.check() && (!startingUp)){
    analogWrite(FAN,fanPrct*MAXPWM);
  }

  if(paused && pausedTimer.check()){
    paused = false;
    fault = false;
  }
}

void FCController::setFan(double prct){
  fanPrct = prct;
}
double FCController::getFan(){
  return fanPrct;
}

