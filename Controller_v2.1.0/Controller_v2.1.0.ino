#include "Controller.h"
#include "Converter.h"
#include "Supercaps.h"
#include "StatsManager.h"
#include "Communication.h"
#include "Metro.h"

uint32_t startTime;
double desPowerIn = 10;

FCController FC = FCController();
Supercaps SC = Supercaps();
Converter Conv = Converter(&startTime,&desPowerIn,
                      &FC.voltage,&FC.current,&FC.power,
                      &SC.voltage,&SC.current,&SC.power);
StatsManager Stats = StatsManager(&Conv.shortCircuit,&FC.temp,
                      &FC.voltage,&FC.current,&FC.power,
                      &SC.voltage,&SC.current,&SC.power);

Metro watchdogTimer = Metro(10);

void setup() {// board setting vars
  // put your setup code here, to run once:
  kickDog();
  pinMode(LED4, OUTPUT);
  digitalWrite(LED4, HIGH);
  
  delay(100);
  Serial.println("Finished Initiazing");
  Serial.begin(115200);
  Serial.println("Beginning setup");
  
  FC.enabled = false;
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
  
  pinMode(LED1,OUTPUT); //put as input earlier ^^
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  pinMode(LED4,OUTPUT);
  pinMode(DIP1,INPUT);
  pinMode(DIP2,INPUT);
  pinMode(DIP3,INPUT);
  pinMode(DIP4,INPUT);
  
  startTime = millis();
  
  Serial.println("Finished setup");
  Serial.println("Beginning Loop");
  //digitalWriteFast(LED4,HIGH);
}

void loop() {
  Stats.updateStats();
  
  FC.doSafetyChecks(Conv.shortCircuit);
  Conv.doSafetyChecks();
  SC.doSafetyChecks(&desPowerIn);

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

  FC.update();
  Conv.update();
  Comm_update();

  if (watchdogTimer.check()){
    kickDog();
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
