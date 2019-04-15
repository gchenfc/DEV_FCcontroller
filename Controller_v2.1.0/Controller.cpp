#include "Controller.h"
#include "Arduino.h"
#include <Metro.h>
#include "PinAssignments.h"
#include "Constants.h" 

void postStartup();
void postShutdown();
bool startingUp = false;

const double FCController::shortCircuitStartupIntervals[6] = {1, 616, 616, 616, 313, 313};
const double FCController::shortCircuitStartupDurations[6] = {50, 50, 50, 50, 100, 50};

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
    *setpointPower = max(*setpointPower-(12-voltage)/1,10);
  }
  if (((voltage<10.5) && (voltage!=0) && (!(shortCircuit || requestShort))) || (voltage>20)){
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
  // analogWrite(FAN,.8*MAXPWM);
  digitalWrite(SUPPLY,HIGH);
  purgeTimer.reset();

  digitalWrite(PURGE,HIGH);
  postStartupTimer.interval(3000);
  postStartupTimer.reset();
  startingUp = FCController::STARTUP_STATE::STARTUP_PURGE;
}
void FCController::bootupShortOnly(){
  enabled = true;
  Serial.println("Booting up Fuel Cell with shorting only (no purge)");
  // analogWrite(FAN,.8*MAXPWM);
  digitalWrite(SUPPLY,HIGH);
  purgeTimer.reset();

  // digitalWrite(PURGE,HIGH);
  postStartupTimer.interval(1);
  postStartupTimer.reset();
  startingUp = FCController::STARTUP_STATE::STARTUP_PURGE;
}
void FCController::bootupSmallPurge(){
  enabled = true;
  Serial.println("Booting up Fuel Cell with abbreviated purge (100ms)");
  // analogWrite(FAN,.8*MAXPWM);
  digitalWrite(SUPPLY,HIGH);
  purgeTimer.reset();

  digitalWrite(PURGE,HIGH);
  postStartupTimer.interval(100);
  postStartupTimer.reset();
  startingUp = FCController::STARTUP_STATE::STARTUP_PURGE;
}
void FCController::postStartup(){
  switch (startingUp) {
    case STARTUP_DONE:
      postStartupTimer.interval(999999);
      break;
    case STARTUP_PURGE:
      digitalWrite(PURGE,LOW);
      requestShort = true;
      shortDuration = shortCircuitStartupDurations[0];
      postStartupTimer.interval(shortCircuitStartupIntervals[0]);
      postStartupTimer.reset();
      startingUp = STARTUP_POSTPURGE;
      break;
    case STARTUP_POSTPURGE:
      requestShort = true;
      shortDuration = shortCircuitStartupDurations[1];
      postStartupTimer.interval(shortCircuitStartupIntervals[1]);
      postStartupTimer.reset();
      startingUp = STARTUP_SHORT1;
      break;
    case STARTUP_SHORT1:
      requestShort = true;
      shortDuration = shortCircuitStartupDurations[2];
      postStartupTimer.interval(shortCircuitStartupIntervals[2]);
      postStartupTimer.reset();
      startingUp = STARTUP_SHORT2;
      break;
    case STARTUP_SHORT2:
      requestShort = true;
      shortDuration = shortCircuitStartupDurations[3];
      postStartupTimer.interval(shortCircuitStartupIntervals[3]);
      postStartupTimer.reset();
      startingUp = STARTUP_SHORT3;
      break;
    case STARTUP_SHORT3:
      requestShort = true;
      shortDuration = shortCircuitStartupDurations[4];
      postStartupTimer.interval(shortCircuitStartupIntervals[4]);
      postStartupTimer.reset();
      startingUp = STARTUP_SHORT4;
      break;
    case STARTUP_SHORT4:
      requestShort = true;
      shortDuration = shortCircuitStartupDurations[5];
      postStartupTimer.interval(shortCircuitStartupIntervals[5]);
      postStartupTimer.reset();
      startingUp = STARTUP_FANACCEL;
      break;
    case STARTUP_FANACCEL:
      analogWrite(FAN, 0.8*MAXPWM);
      purgeTimer.interval(PURGE_INTERVAL/2); // to offset purge and short
      startingUp = STARTUP_DONE;
      postStartupTimer.interval(10);
      postStartupTimer.reset();
      break;
  }
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
void FCController::startPurge(){
  if (!purgeEnabled){
    return;
  }
  digitalWrite(PURGE,HIGH);
  purgeEndTimer.reset();
}
void FCController::update(){
  if (startingUp && postStartupTimer.check()){
    postStartup();
  }
  if (shuttingDown && postShutdownTimer.check()){
    postShutdown();
    shuttingDown = false;
  }

  if (!enabled){
    return;
  }

  if (purgeEnabled && !startingUp){
    if(purgeTimer.check()){
      startPurge();
      purgeTimer.interval(PURGE_INTERVAL); // to reset the interval after startup offset "hack"
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

