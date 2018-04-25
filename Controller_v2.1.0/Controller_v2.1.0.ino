#include "Controller.h"
#include "Converter.h"
#include "Supercaps.h"
#include "StatsManager.h"
#include "Communication.h"
#include "Metro.h"

uint32_t startTime;
double desPowerIn = 70;

FCController FC = FCController();
Supercaps SC = Supercaps();
Converter Conv = Converter(&startTime,&desPowerIn,
                      &FC.voltage,&FC.current,&FC.power,
                      &SC.voltage,&SC.current,&SC.power);
StatsManager Stats = StatsManager(&Conv.shortCircuit,&FC.temp,
                      &FC.voltage,&FC.current,&FC.power,
                      &SC.voltage,&SC.current,&SC.power);

Metro watchdogTimer = Metro(10);
Metro lapPurgeTimer = Metro(210000);

void setup() {// board setting vars
  // put your setup code here, to run once:
  kickDog();
  
  delay(100);
  Serial.println("Finished Initiazing");
  Serial.begin(115200);
  Serial.println("Beginning setup");
  
  FC.enabled = true;
  SC.enabled = true;
  Conv.enabled = true;
  FC.purgeEnabled = false;
  Conv.shortCircuitEnabled = false;

  Stats.initializeStats();
  Comm_setup();
  
  Conv.initializeDC();
  Conv.setDC();
  if (FC.enabled){
    FC.bootup();
    Conv.pause(1100);
  }
  analogWriteResolution(PWM_RES);
  analogWriteFrequency(FAN, 20000);

  analogReference(EXTERNAL);
  
  startTime = millis();
  
  Serial.println("Finished setup");
  Serial.println("Beginning Loop");
}

void loop() {
  Stats.updateStats();
  
  FC.doSafetyChecks(Conv.shortCircuit,&Conv.setpointPower);
  Conv.doSafetyChecks();
  SC.doSafetyChecks(&Conv.setpointPower);

  if (FC.fault){
    Conv.pause(FC.faultDuration);
    FC.faultDuration = 0;
    FC.fault = false;
  }
  if (SC.fault){
    Conv.pause(SC.faultDuration);
    SC.faultDuration = 0;
    SC.fault = false;
  }
  if (FC.requestShort){
    Conv.startShortCircuit(FC.shortDuration);
    FC.requestShort = false;
    FC.shortDuration = 0;
  }

  FC.update();
  Conv.update();
  Comm_update();

  if (watchdogTimer.check()){
    kickDog();
  }
  
  if(lapPurgeTimer.check()){
    FC.bootup();
    Conv.pause(1100);
    lapPurgeTimer.reset();
  }
}

void kickDog()
{
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
  digitalWriteFast(LED4,!digitalReadFast(LED4));
}
