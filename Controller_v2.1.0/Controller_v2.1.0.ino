#include "Controller.h"
#include "Converter.h"
#include "Supercaps.h"
#include "StatsManager.h"
#include "Communication.h"
#include "Metro.h"

uint32_t startTime;
double desPowerIn = 23;

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
  setupWatchdog();
  
  delay(100);
  Serial.println("Finished Initiazing");
  Serial.begin(115200);
  Serial.println("Beginning setup");
  
  kickDog();

  FC.enabled = true;
  SC.enabled = true;
  Conv.enabled = true;
  FC.purgeEnabled = true;
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

  analogReadResolution(12);
  analogReadAveraging(16);
  analogReference(EXTERNAL);
  
  startTime = millis();
  
  Serial.println("Finished setup");
  Serial.println("Beginning Loop");
  kickDog();
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
  
//  if(lapPurgeTimer.check()){
//    FC.bootup();
//    Conv.pause(1100);
//    lapPurgeTimer.reset();
//  }
}

void kickDog()
{
  #if defined(__MKL26Z64__)
    // Teensy LC
    __disable_irq();
    SIM_SRVCOP = 0x55;
    SIM_SRVCOP = 0xAA;
    __enable_irq();
  #elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    // Teensy 3.x
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
  #else
    #error // watchdog not configured - comment out this line if you are ok with no watchdog
  #endif

  digitalWriteFast(LED4,!digitalReadFast(LED4));
}

#if defined(__MKL26Z64__)
extern "C" void startup_early_hook(void) {}
#endif

void setupWatchdog()
{
  #if defined(__MKL26Z64__)
    // Teensy LC
    SIM_COPC = 12; // 1024ms watchdog
    SIM_COPC = 8; // 256ms watchdog
    SIM_COPC = 4; // 32ms watchdog
  #elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    // Teensy 3.x
    // kickDog();
    noInterrupts();                                         // don't allow interrupts while setting up WDOG
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;                         // unlock access to WDOG registers
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);                                   // Need to wait a bit..
    
    // about 0.25 second timeout
    WDOG_TOVALH = 0x001B;
    WDOG_TOVALL = 0x7740;
    
    // This sets prescale clock so that the watchdog timer ticks at 7.2MHz
    WDOG_PRESC  = 0x400;
    
    // Set options to enable WDT. You must always do this as a SINGLE write to WDOG_CTRLH
    WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
        WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
        WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
    interrupts();
  #else
    #error // watchdog not configured
  #endif

  kickDog();
}
