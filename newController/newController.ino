#include <i2c_t3.h>
#include <PID.h>
#include <Metro.h> // schedules events at regular times

#define LFETold 9
#define LFET 4  // don't change this - it's set to FTM1C1
#define HFET 3  // don't change this - it's set to FTM1C0
// technical PWM stuff
#define TPM_C 48000000            // core clock, for calculation only
#define PWM_FREQ 20000            //  PWM frequency [Hz]
#define MODULO (TPM_C / PWM_FREQ) // calculation the modulo for FTM0
// less technical for non-FTM0
#define PWM_SPEED 20000
#define PWM_RES 12
#define MAXPWM 4096

#define SCL1 16
#define SDA1 17
#define ICAP A1
#define VCAP A0

#define FAN 6
#define PURGE 10
#define SUPPLY 5
#define TEMP A9
#define TEMP_RES 1200

#define LED1 0
#define LED2 1
#define LED3 2
#define LED4 7

#define DIP1 21
#define DIP2 22
#define DIP3 13
#define DIP4 12

#define KpPower 0.003
#define KiPower 0.003
#define KdPower -0.000005
#define KpCurrent .02
#define KiCurrent 0.001
#define KdCurrent -0.0001
#define Kp KpPower
#define Ki KiPower
#define Kd KdPower

double FCvoltage = 0.0;
double FCcurrent = 0.0;
double FCpower = 0.0;
double SCvoltage = 0.0;
double SCcurrent = 0.0;
double SCpower = 0.0;
double converterEff = 0.0;

float dutyCycleAdj = 0.0; // not implemented
float dutyCycleBase = 0.0; // not implemented
float dutyCycle = 0.0;
bool CCM = false;
double K = 0;
double Kcrit = 0;

double setpointPower = 0; // constantly updated
double desPowerIn = 10; // stays until user changes
float error = 0.0;
PID pid;
// statistical error calculation
unsigned long errorCount = 0;
float errorMean = 0;
float errorM2 = 0;
long prevTime = 0;

// FC control timers
Metro getTempTimer = Metro(10);
Metro purgeTimer = Metro(10000);
Metro purgeEndTimer = Metro(0);
Metro fanUpdateTimer = Metro(1000);
Metro shortCircuitTimer = Metro(10000000);
Metro shortCircuitEndTimer = Metro(10);
Metro shortCircuitRecovTimer = Metro(10);

// board setting vars
bool FCenabled = true;
bool purgeEnabled = true;
bool shortCircuitEnabled = false;
bool supercapsEnabled = true;

// FC control vars
double FCtemp = 0.0;
double fanPrct = .45;
double purgePrct = 0.0;
bool supplyPrct = 1;
bool shortCircuit = false;

// converter control vars
bool converterEnabled = false;
bool statsLag = false;

// converter control timers
Metro updateDCTimer = Metro(1);
Metro printStatsTimer = Metro(20);
Metro disableTimer = Metro(1000);
Metro statsLagTimer = Metro(200);

long startTime;

// serial
char buf[10];
int bufInd = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.println("Beginning setup");

  // set up PID controller
  pid = PID(&error, &dutyCycle, Kp,Ki,Kd,FORWARD);
  pid.setLimits(0,.75);
  pid.setPLimits(-.25,.25);
  pid.setILimits(-.15,.75);
  pid.setDLimits(-.01,.01);
  initializeDC();
  setDC(dutyCycle);
  analogWriteResolution(PWM_RES);
  analogWriteFrequency(FAN, 20000);

  analogReference(EXTERNAL);
  
  pinMode(TEMP,INPUT);
  pinMode(FAN,OUTPUT);
  pinMode(PURGE,OUTPUT);
  pinMode(SUPPLY,OUTPUT);
  
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  pinMode(LED4,OUTPUT);
  pinMode(DIP1,INPUT);
  pinMode(DIP2,INPUT);
  pinMode(DIP3,INPUT);
  pinMode(DIP4,INPUT);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, 400000);
  INAinit();

  Serial.begin(115200);
  startTime = millis();

  FuelCellBootup();
  
  FCvoltage = readFCvoltage();
  FCcurrent = readFCcurrent();
  SCvoltage = readSCvoltage();
  SCcurrent = readSCcurrent();
  FCpower = FCvoltage*FCcurrent;
  SCpower = SCvoltage*SCcurrent;
  
  Serial.println("Beginning Loop");
  prevTime = micros();
}

void loop() {
  updateStats();
  
  doFuelCellSafetyChecks();
//  doConverterSafetyChecks();
  doSupercapSafetyChecks();

  updateDC();
  
  updateFC();
  
  if(shortCircuit && shortCircuitRecovTimer.check()){
    shortCircuit = false; // to give the voltage a chance to come back up
  }

  if(printStatsTimer.check()){ printStatsSerial(); }
  
  if (Serial.available()){
    char c = Serial.read();
    switch(c){
      case 'p':
        emergencyPause();
        break;
      case 'S':
      case 's':
        emergencyShutdown();
        break;
      case 'b':
        FuelCellBootup();
        break;
      case 'r':
        doShortCircuit();
        break;
      case 'F':
        fanPrct = (float)atof(buf);
        for (int i=0;i<bufInd;i++){
          buf[i] = 0;
        }
        bufInd = 0;
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
        if (K<Kcrit){
          Serial.println("Cannot go into CCM:");
          Serial.print("K = ");
          Serial.print(K,5);
          Serial.print("\t");
          Serial.print("Kcrit = ");
          Serial.println(Kcrit,5);
          Serial.println("Staying in DCM");
          break;
        }
        CCM = true;
        initializeDC();
//        dutyCycle = 1-FCvoltage/SCvoltage;
//        setDC(dutyCycle);
        break;
      case 'd':
        CCM = false;
        initializeDC();
        break;
    }
//    desPowerIn = Serial.parseFloat();
  }

  if (!converterEnabled && disableTimer.check()){
    converterEnabled = true;
    statsLag = true;
    initializeDC();
    statsLagTimer.reset();
    setpointPower = FCpower;
    disableTimer.interval(10000000);
    disableTimer.reset();
  }
}

// safety checks
void doFuelCellSafetyChecks(){
  bool allGood = true;
  if (!FCenabled){
    digitalWriteFast(LED1,LOW);
    return;
  }
  if ((!shortCircuit) && (FCpower>130)){
    Serial.print("FC power too high... ");
    Serial.print(FCpower,3);
    Serial.println("W");
    allGood = false;
    emergencyPause();
  }
  if (((FCvoltage<8) & (FCvoltage!=0) & (!shortCircuit)) || FCvoltage>20){
    Serial.print("FC voltage out of range... ");
    Serial.print(FCvoltage,3);
    Serial.println("V");
    allGood = false;
    emergencyPause();
  }
  if(FCtemp>65){
    Serial.print("FC Temperature Too High (");
    Serial.print(FCtemp,1);
    Serial.println("°C");
    allGood = false;
    emergencyShutdown();
  }
  digitalWriteFast(LED1,allGood);
}
void doConverterSafetyChecks(){
  bool allGood = true;
  if ((FCcurrent>13) && (!shortCircuit)){
    Serial.print("Too much input current - converter pausing... ");
    Serial.print(FCcurrent,3);
    Serial.println("A");
    allGood = false;
    converterPause();
  }
  if (FCvoltage>30){
    Serial.print("Too much input voltage - converter pausing... ");
    Serial.print(FCvoltage,3);
    Serial.println("V");
    allGood = false;
    converterPause();
  }
  if ((FCvoltage<2) && (FCcurrent>1)){ // probably shorted
    Serial.print("Short suspected - converter stopping... ");
    Serial.print(FCvoltage,3);
    Serial.println("V");
    allGood = false;
    converterPause(5000);
  }
  if (SCcurrent>13){
    Serial.print("Too much output current - converter pausing... ");
    Serial.print(SCcurrent,3);
    Serial.println("A");
    allGood = false;
    converterPause();
  }
  if (SCvoltage>40){
    Serial.print("Too much output voltage - converter pausing... ");
    Serial.print(FCvoltage,3);
    Serial.println("V");
    allGood = false;
    converterPause();
  }
  if (converterEnabled && SCvoltage<15){
    Serial.print("INSUFFICIENT INTERNAL DRIVE VOLTAGE ON SC - pausing for 1s... ");
    Serial.print(SCvoltage,3);
    Serial.println("V");
    allGood = false;
    converterPause();
  }
  if (converterEnabled &&
    !shortCircuit &&
    !statsLag &&
    (abs(SCvoltage/FCvoltage - predictedM()) > .1)){ // check CCM
      
    Serial.println("Measurements falling outside expected range... ");
    Serial.print("M = ");
    Serial.print(SCvoltage/FCvoltage,3);
    Serial.print("\tM(D) = ");
    Serial.print(predictedM(),3);
    Serial.println();

    if ((SCvoltage/FCvoltage)>(1/(1-dutyCycle))){ // DCM
      Serial.println("Suspected DCM control error: converter Pausing ... ");
      converterPause();
    }
    else{ // something wrong
      if ((SCvoltage/FCvoltage)<1.1){ // probably 12V regulator malfunctioning
        Serial.println("FC not operational - likely MOSFETs not getting enough power");
        Serial.println("Continuing operation...");
        statsLag = true;
        statsLagTimer.reset();
      }
      else{ 
        if (abs(SCvoltage/FCvoltage - predictedM()) > .15){
          Serial.println("Unknown error - pausing operation");
          converterPause();
        }
        else{
          Serial.println("Unknown error - continuing operation");
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
  if (!converterEnabled){
    allGood = false;
  }
  digitalWriteFast(LED2,allGood);
}
void doSupercapSafetyChecks(){
  bool allGood = true;
  if (!supercapsEnabled){
    digitalWriteFast(LED3,LOW);
    return;
  }
  if (SCvoltage<33/2){
    Serial.print("Supercaps under voltage... ");
    Serial.print(SCvoltage,3);
    Serial.println("V");
    allGood = false;
    desPowerIn=min(desPowerIn*1.1,50); // pseudocode
  }
//  if (SCvoltage>43.2/2){
  if (SCvoltage>18.5){
    Serial.print("Supercaps over voltage... ");
    Serial.print(SCvoltage,3);
    Serial.println("V");
    allGood = false;
    emergencyPause();
  }
  digitalWriteFast(LED3,allGood);
}

// converter management
void converterPause(long duration){
  dutyCycle = 0;
  digitalWrite(HFET,0);
  digitalWrite(LFET,HIGH); // inverted
//  setDC(dutyCycle);
  if (CCM && (K<Kcrit)){
    Serial.println("DCM criteria detected - switching to DCM...");
    CCM = false;
    initializeDC();
  }
  converterEnabled = false;
  disableTimer.interval(duration);
  disableTimer.reset();
  pid.reset();
}
void converterPause(){
  converterPause(1000);
}
void converterShutdown(){
  dutyCycle = 0;
  setpointPower = 0;
  setDC(dutyCycle);
  converterEnabled = false;
  disableTimer.interval(1000000000);
  disableTimer.reset();
}
void converterResume(){
  initializeDC();
  setDC(dutyCycle);
}
void initializeDC(void){
//  if (CCM){
    // see page 775
    FTM1_POL = 0;                  // Positive Polarity 
    FTM1_OUTMASK = 0xFF;           // Use mask to disable outputs
    FTM1_SC = 0x08;                // set system clock as source for FTM0
    FTM1_MOD = MODULO;             // Period register
    FTM1_CNTIN = 0;                // Counter initial value
    FTM1_COMBINE = 0x00000031;     // COMBINE=1, COMP=1, DTEN=1, SYNCEN=1            // page 796  (COMP1 sets complement)
    FTM1_MODE = 0x01;              // Enable FTM0
    FTM1_SYNC = 0x02;              // PWM sync @ max loading point enable
    FTM1_DEADTIME = 0b00<<6;       // DeadTimer prescale systemClk/1                 // page 801
    FTM1_DEADTIME |= 0b000000;     // 1uS DeadTime, max of 63 counts of 48Mhz clock  // page 801
    FTM1_C0V = 0;                  // Combine mode, pulse-width controlled by...
    FTM1_C1V = MODULO/2;           //   odd channel.
    FTM1_SYNC |= 0x80;             // set PWM value update
    FTM1_C0SC = 0x28;              // PWM output, edge aligned, positive signal
    FTM1_C1SC = 0x28;              // PWM output, edge aligned, positive signal
    
    CORE_PIN3_CONFIG = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;    //config teensy output port pins
    CORE_PIN4_CONFIG = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;   //config teensy output port pins

    if (CCM){
      FTM1_OUTMASK = 0x0;            // Turns on PWM output
    }
    else{
      FTM1_OUTMASK = 0x1; // masks high side
    }
//  }
//  else{
//    analogWriteFrequency(LFET,PWM_FREQ);
//    analogWriteFrequency(HFET,PWM_FREQ);
//    analogWriteResolution(PWM_RES);
//    digitalWrite(HFET,0);
//    analogWrite(LFET,MAXPWM);
//  }
  statsLag = true;
  statsLagTimer.reset();
  pid.reset();
//  pid.setScalars(0.0,predictedD(),0.0);
//  pid.update();
}
void setDC(double DC){
  if (DC>.75){
    DC = .75;
  }
  if (DC<0){
    DC = 0;
  }
//  if (CCM){
    FTM1_C1V = MODULO*(1-DC); // sets both low and high side
    FTM1_SYNC |= 0x80;
//  }
//  else{
//    analogWrite(LFET,(1-DC)*MAXPWM);
//    digitalWrite(HFET,0);
//  }
}
void setDC(){
  setDC(dutyCycle);
}
void updateDC(){
//  testingCycleDC()

  if (!converterEnabled){
    dutyCycle = 0;
    setDC(dutyCycle);
    return;
  }
  
  if (!(updateDCTimer.check() && (!shortCircuit))){
    return;
  }
  
//  dutyCycle = 0.3;
//  setDC(dutyCycle);

  error = setpointPower-FCpower;
  pid.update();
  updateSetpoint();
//  dutyCycle = dutyCycleBase+dutyCycleAdj;
  
  // error stats
  if((millis()-startTime)>5000){
    errorCount += 1;
    float delta = error-errorMean;
    errorMean += delta/errorCount;
    float delta2 = error-errorMean;
    errorM2 += delta*delta2;
  }

  // DCM criteria: K<Kcrit (then DCM occurs)
  K = 2*33e-6/(SCvoltage/SCcurrent*1/20e3);
  Kcrit = dutyCycle*(1-dutyCycle)*(1-dutyCycle);

  setDC(dutyCycle);
}
void updateSetpoint(){
  double deltaT = (micros()-prevTime)/1e6;
  prevTime = micros();
  if (abs(desPowerIn-setpointPower)>1){ // off by more than 1
    setpointPower = setpointPower + (10)*min(deltaT,.1)*((desPowerIn>setpointPower)?1:-1);
  }
  else{ // pretty close
    setpointPower = setpointPower + (desPowerIn-setpointPower)*1*min(deltaT,.1);
  }
//  if (abs(error)<1){1
//    dutyCycleBase = LPF(
//      dutyCycleBase,
//      1-(FCvoltage)/((SCpower+setpointPower-FCpower)/SCcurrent),
//      .99);
//  }
//  else{
//    dutyCycleBase = LPF(
//      dutyCycleBase,
//      1-(FCvoltage)/((SCpower+setpointPower-FCpower)/SCcurrent),
//      .9999);
//  }
}
void testingCycleDC(){
  int tConst = 180;
  float minVal = 0;
  float maxVal = 50;
  float stayTime = 15;
  float deltaPower = (maxVal-minVal)/(tConst/2/stayTime);
  if((millis()-startTime)/1000.0>tConst){
    startTime=millis();
  }
  if (((millis()-startTime)/1000.0)<(tConst/2)){
    desPowerIn = (millis()-startTime)/1000.0*((maxVal-minVal)/tConst*2) + minVal;
    desPowerIn = round(desPowerIn/deltaPower)*deltaPower;
  }
  else{
    desPowerIn = 2*(maxVal-minVal) - (millis()-startTime)/1000.0*((maxVal-minVal)/tConst*2) + minVal;
    desPowerIn = round(desPowerIn/deltaPower)*deltaPower;
  }
  if(desPowerIn>11){
    desPowerIn=11;
  }
}
double predictedM(){
  return predictedM(dutyCycle);
}
double predictedM(double D){
  if (K<Kcrit){ // DCM
    return (1+sqrt(1+4*D*D/K))/2;
  }
  else{
    return 1/(1-D);
  }
}
double predictedD(){
  return predictedD(SCvoltage/FCvoltage);
}
double predictedD(double M){
  if (K<Kcrit){ // DCM
    return sqrt(((2*M-1)*(2*M-1)-1)/4*K);
  }
  else{
    return 1-1/M;
  }
}

// FC management
void FuelCellBootup(){
  if (!FCenabled){
    return;
  }
  Serial.println("Booting up Fuel Cell");
  analogWrite(FAN,fanPrct*MAXPWM);
  digitalWrite(PURGE,HIGH);
  digitalWrite(SUPPLY,HIGH);
  delay(1000);
  analogWrite(PURGE,LOW);
  purgeTimer.reset();
}
void emergencyPause(){
  Serial.println("pausing for 5 secs");
  dutyCycle = 0;
  setDC(dutyCycle);
  startTime=millis();
  while((millis()-startTime)<5000){
    // read sensors
    updateStats();
    if(getTempTimer.check()){
      FCtemp = readTemp();
    }
    analogWrite(FAN,0);
    digitalWrite(SUPPLY,0);
    
    if(printStatsTimer.check()){
      printStatsSerial();
    }
  }
  Serial.println("Resuming...");
  startTime = millis();
}
void emergencyShutdown(){
  Serial.println("SHUTTING DOWN!!!");
  dutyCycle=0;
  setDC(dutyCycle);
  digitalWrite(PURGE,HIGH);
  Metro closeValvesTimer = Metro(1000);
  while(true){
    
    // read sensors
    updateStats();
    if(getTempTimer.check()){
      FCtemp = readTemp();
    }
    
    if(printStatsTimer.check()){
      printStatsSerial();
    }
    if(closeValvesTimer.check()){
      digitalWrite(SUPPLY,LOW);
      delay(10);
      digitalWrite(PURGE,LOW);
      analogWrite(FAN,LOW);
      closeValvesTimer.interval(999999);
    }
  }
}

void updateFC(){
  if (!FCenabled){
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
  
  if(shortCircuitEnabled){
    if(shortCircuitTimer.check()){
      doShortCircuit();
    }
  }
  
  if(fanUpdateTimer.check()){
//    fanPrct = .5*sin((millis()-startTime)/1000*6.28)+.5;
//    fanPrct = 1*((millis()-startTime)>5000);
//    fanPrct = .8;
    if (converterEnabled){
      analogWrite(FAN,fanPrct*MAXPWM);
      digitalWrite(SUPPLY,HIGH);
    }
    else{
      analogWrite(FAN,0);
      digitalWrite(SUPPLY,LOW);
    }
  }
}
void doShortCircuit(){ // WARNING - THIS CODE IS BLOCKING, intentional
  shortCircuit = true;
  shortCircuitEndTimer.reset();
  float oldDC = dutyCycle;
  dutyCycle = 1;
  setDC(dutyCycle);
  while(!shortCircuitEndTimer.check()){
    updateStats();
    printStatsSerial(); // so that data collection is more accurate
  }
  dutyCycle = oldDC;
  setDC(dutyCycle);
  shortCircuitRecovTimer.reset();
}

// stats management
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
  Serial.print((millis()-startTime)/1000.0,3);
  Serial.print("s\t");
  Serial.print(FCvoltage,2);
  Serial.print("V\t");
  Serial.print(FCcurrent,3);
  Serial.print("A\t");
  Serial.print(FCpower,3);
  Serial.print("W\t\t");
  Serial.print(SCvoltage,2);
  Serial.print("V\t");
  Serial.print(SCcurrent,3);
  Serial.print("A\t");
  Serial.print(SCpower,3);
  Serial.print("W\t\t");
  Serial.print(setpointPower,2);
  Serial.print("W\t");
  Serial.print(errorM2/(errorCount-1)*1000);
  Serial.print("mW\t");
  Serial.print(dutyCycle*100,3);
  Serial.print("%\t");
  Serial.print(converterEff*100,2);
  Serial.print("%\t");
  Serial.print(tempCtoF(FCtemp));
  Serial.print("°F\t");
//    Serial.print(flowmeter.readVoltage(),5);
//    Serial.print("V\t");
//    Serial.print(flowmeter.readFlow(),2);
//    Serial.print("mg/m\t");
  Serial.println();
}
void updateStats(){
  FCvoltage = LPF(FCvoltage,readFCvoltage(),.9); //LPF(FCvoltage,readFCvoltage(),.99);
  FCcurrent = LPF(FCcurrent,readFCcurrent(),.95); //LPF(FCcurrent,readFCcurrent(),.99);
  SCvoltage = LPF(SCvoltage,readSCvoltage(),.95); //LPF(SCvoltage, readSCvoltage(), .99); // smooth/filter
  SCcurrent = LPF(SCcurrent,readSCcurrent(),.95); //LPF(SCcurrent, readSCcurrent(), .99);
  FCpower = FCvoltage*FCcurrent; //LPF(FCpower,FCvoltage*FCcurrent,.9);
  SCpower = SCvoltage*SCcurrent;
  converterEff = LPF(converterEff,(FCpower<0.01 ? 0 : SCpower/FCpower),.99);
  if(shortCircuit){
    FCpower = 0;
  }
  if(getTempTimer.check()){
    FCtemp = LPF(FCtemp,readTemp(),.95);
  }
}
double LPF(double oldVal, double newVal, double alpha){
  return oldVal*alpha + newVal*(1-alpha);
}
double readSCvoltage(){
  return analogRead(VCAP)*(3.0/1023)*(348/18) * 1.01805; //voltage divider - 10k and 200k
  // 0.0428V calibrated
}
double readSCcurrent(){
  return analogRead(ICAP)*(3.0/1023)/20/0.025 * 0.94896; //INA168 gain is 20 - shunt resistor is 25mO
}
double readFCcurrent(){
  int16_t raw = INAreadReg(0x01) ; //deliberate bad cast! the register is stored as two's complement
  return raw * 0.0000025 / 0.001 * .805/.837;// * 0.81344 + 0.00216; //2.5uV lsb and 10mOhm resistor *fudged*
}
double readFCvoltage(){
  uint16_t raw = INAreadReg(0x02);
  return raw * 0.00125 * 0.9998 * 17.78/17.73; //multiply by 1.25mV LSB
}
double readTemp(){
  double prct = analogRead(TEMP)/1024.0;
  if (prct > .95){
    return 0;
  }
  return 20 + (((1-prct)/prct*TEMP_RES)-1076)/3.8;
}
double tempCtoF(double tempC){
  return tempC*9/5 + 32;
}

// INA226 I2C
void INAinit(){
  Wire.beginTransmission(0x40);
  Wire.write(0x00);//reg select = 0x00
  // 0000 000 001 100 111
  Wire.write(0b0000);//64 averages, 1ms voltage sampling
  Wire.write(0b1100111);//140us current sampling, free running
  Wire.endTransmission();
}
uint16_t INAreadReg(uint8_t reg){
  Wire.beginTransmission(0x40);
  Wire.write(reg);//read from the bus voltage
  Wire.endTransmission();

  Wire.requestFrom(0x40, 2);

  delayMicroseconds(50); // originally 100
  if (Wire.available() < 2)
    return 1337;

  uint16_t resp = (uint16_t)Wire.read() << 8;
  resp |= Wire.read();

  return resp;
}
