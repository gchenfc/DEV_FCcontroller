#include <i2c_t3.h>
#include <SPI.h>
#include <PID.h>
#include <Metro.h> // schedules events at regular times

#define SFET 9
#define SCL1 16
#define SDA1 17
#define ICAP A1
#define VCAP A0

#define FAN 3
#define PURGE 4
#define SUPPLY 5
#define TEMP A6
#define TEMP_RES 1208

#define PWM_SPEED 20000
#define PWM_RES 12
#define MAXPWM 4096

#define KpPower 0.0003
#define KiPower 0.0001
#define KdPower 0.000005
#define KpCurrent .02
#define KiCurrent 0.001
#define KdCurrent 0.0001
#define Kp KpPower
#define Ki KiPower
#define Kd KdPower

double FCvoltage = 0.0;
double FCcurrent = 0.0;
double FCpower = 0.0;
double SCvoltage = 0.0;
double SCcurrent = 0.0;
double SCpower = 0.0;

float dutyCycle = 0.0;

double desPowerIn = 0; // reset each iteration of loop
float error = 0.0;
PID pid;
// statistical error calculation
unsigned long errorCount = 0;
float errorMean = 0;
float errorM2 = 0;

// use these to control purging frequency and duration
Metro updateDCTimer = Metro(1);
Metro printStatsTimer = Metro(20);
Metro getTempTimer = Metro(10);
Metro purgeTimer = Metro(20000);
Metro purgeEndTimer = Metro(0);
Metro fanUpdateTimer = Metro(100);
Metro shortCircuitTimer = Metro(10000);
Metro shortCircuitEndTimer = Metro(0);
Metro shortCircuitRecovTimer = Metro(0);

double FCtemp = 0.0;
double fanPrct = .80;
double purgePrct = 0.0;
bool supplyPrct = 1;
bool shortCircuit = false;

long startTime;

void setup() {
  // put your setup code here, to run once:
  Serial.println("Beginning setup");
  analogWriteFrequency(SFET, PWM_SPEED);
  analogWriteResolution(PWM_RES);
  analogWriteFrequency(FAN, 20000);
  
  pinMode(TEMP,INPUT);
  pinMode(FAN,OUTPUT);
  pinMode(PURGE,OUTPUT);
  pinMode(SUPPLY,OUTPUT);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, 400000);
  INAinit();

  Serial.begin(115200);
  startTime = millis();

  // set up PID controller
  pid = PID(&error, &dutyCycle, Kp,Ki,Kd,FORWARD);
  pid.setLimits(0,.2);
  pid.setPLimits(-.5,.15);
  pid.setILimits(-.15,.15);
  pid.setDLimits(-.01,.01);

  
  analogWrite(SFET,dutyCycle*MAXPWM);
  FuelCellBootup();
  
  FCvoltage = readFCvoltage();
  FCcurrent = readFCcurrent();
  SCvoltage = readSCvoltage();
  SCcurrent = readSCcurrent();
  FCpower = FCvoltage*FCcurrent;
  SCpower = SCvoltage*SCcurrent;
  
  Serial.println("Beginning Loop");
}

void loop() {
  // read sensors
  updateStats();
  
  // Safety checks
  if (SCvoltage>60){
    Serial.println("OVER VOLTAGE...");
    emergencyPause();
  }
  if (FCpower>130){
    Serial.println("FC power too high");
    emergencyPause();
  }
  if (((FCvoltage<8) & (FCvoltage!=0) & (!shortCircuit)) || FCvoltage>20){
    Serial.println("FC voltage out of range");
    emergencyPause();
  }
  if(FCtemp>65){
    Serial.print("FC Temperature Too High (");
    Serial.print(FCtemp,1);
    Serial.println("°C");
    emergencyShutdown();
  }

  // update controls
  if(updateDCTimer.check() & (!shortCircuit)){
    updateDC();
  }

  if(purgeTimer.check()){
    digitalWrite(PURGE,HIGH);
    purgeEndTimer.reset();
  }
  if(purgeEndTimer.check()){
    digitalWrite(PURGE,LOW);
  }

  if(shortCircuitTimer.check()){
    shortCircuit = true;
    doShortCircuit();
    shortCircuitRecovTimer.reset();
  }
  if(shortCircuitRecovTimer.check()){
    shortCircuit = false; // to give the voltage a chance to come back up
  }
  
  if(fanUpdateTimer.check()){
//    fanPrct = .5*sin((millis()-startTime)/1000*6.28)+.5;
//    fanPrct = 1*((millis()-startTime)>5000);
//    fanPrct = .8;
    analogWrite(FAN,fanPrct*MAXPWM);
  }

  // send data
  if(printStatsTimer.check()){
    printStatsSerial();
  }
  
  if (Serial.available()){
    desPowerIn = Serial.parseFloat();
  }
}

void updateDC(){
//  rampDC()

//  dutyCycle = 0.054;

  desPowerIn = 25;
  error = desPowerIn-FCpower;
  pid.update();
  
  // error stats
  if((millis()-startTime)>5000){
    errorCount += 1;
    float delta = error-errorMean;
    errorMean += delta/errorCount;
    float delta2 = error-errorMean;
    errorM2 += delta*delta2;
  }

  analogWrite(SFET,dutyCycle*MAXPWM);
}

void rampDC(){
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

// FC startup and shutdown
void FuelCellBootup(){
  Serial.println("Booting up Fuel Cell");
  analogWrite(FAN,fanPrct*MAXPWM);
  digitalWrite(PURGE,HIGH);
  digitalWrite(SUPPLY,HIGH);
  delay(3000);
  analogWrite(PURGE,LOW);
  purgeTimer.reset();
}
void emergencyPause(){
  Serial.println("pausing for 5 secs");
  dutyCycle = 0;
  digitalWrite(SFET,LOW);
  startTime=millis();
  while((millis()-startTime)<5000){
    
    // read sensors
    updateStats();
    if(getTempTimer.check()){
      FCtemp = readTemp();
    }
    
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
  digitalWrite(SFET,LOW);
  digitalWrite(PURGE,HIGH);
  Metro closeValvesTimer = Metro(3000);
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
      digitalWrite(FAN,LOW);
      closeValvesTimer.interval(999999);
    }
  }
}
void doShortCircuit(){ // WARNING - THIS CODE IS BLOCKING, intentional
  shortCircuitEndTimer.reset();
  float oldDC = dutyCycle;
  dutyCycle = 1;
  analogWrite(SFET,dutyCycle*MAXPWM);
  while(!shortCircuitEndTimer.check()){
    updateStats();
    printStatsSerial(); // so that data collection is more accurate
  }
  dutyCycle = oldDC;
  analogWrite(SFET,dutyCycle*MAXPWM);
  shortCircuitRecovTimer.reset();
}

// stats management
void printStatsSerial(){
//  // plot for PID tuning
//  Serial.print(error*10,5);
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
  Serial.print(desPowerIn,2);
  Serial.print("W\t");
//  Serial.print(errorM2/(errorCount-1)*1000);
//  Serial.print("mW\t");
  Serial.print(dutyCycle*100,3);
  Serial.print("%\t");
  Serial.print((FCpower<0.01 ? 0 : SCpower/FCpower*100),2);
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
  FCcurrent = LPF(FCcurrent,readFCcurrent(),.9); //LPF(FCcurrent,readFCcurrent(),.99);
  SCvoltage = readSCvoltage(); //LPF(SCvoltage, readSCvoltage(), .99); // smooth/filter
  SCcurrent = readSCcurrent(); //LPF(SCcurrent, readSCcurrent(), .99);
  FCpower = FCvoltage*FCcurrent; //LPF(FCpower,FCvoltage*FCcurrent,.9);
  SCpower = SCvoltage*SCcurrent;
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
  return analogRead(VCAP)*(3.3/1023)*(210/10) * 18.07/17.95; //voltage divider - 10k and 200k
  // 0.0428V calibrated
}
double readSCcurrent(){
  return analogRead(ICAP)*(3.3/1023)/20/0.025 * .174/.213; //INA168 gain is 20 - shunt resistor is 25mO
}
double readFCcurrent()
{
  int16_t raw = INAreadReg(0x01); //deliberate bad cast! the register is stored as two's complement
  return raw * 0.0000025 / 0.010 * .371/.381;// * 0.81344 + 0.00216; //2.5uV lsb and 10mOhm resistor *fudged*
}
double readFCvoltage()
{
  uint16_t raw = INAreadReg(0x02);
  return raw * 0.00125 * 15.34/15.43 * 16.58/16.43; //multiply by 1.25mV LSB
}
double readTemp(){
  double prct = analogRead(TEMP)/1024.0;
  if (prct > .95){
    return 0;
  }
  return 20 + ((prct/(1-prct)*TEMP_RES)-1076)/3.8;
}
double tempCtoF(double tempC){
  return tempC*9/5 + 32;
}

// INA I2C
void INAinit()
{
  Wire.beginTransmission(0x40);
  Wire.write(0x00);//reg select = 0x00
  // 0000 000 001 100 111
  Wire.write(0b0000);//64 averages, 1ms voltage sampling
  Wire.write(0b1100111);//140us current sampling, free running
  Wire.endTransmission();
}
uint16_t INAreadReg(uint8_t reg)
{
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
