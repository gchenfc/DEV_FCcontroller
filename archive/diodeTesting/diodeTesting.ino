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

#define Kp .002
#define Ki 0.07
#define Kd 0.00000001

double FCvoltage = 0.0;
double FCcurrent = 0.0;
double FCpower = 0.0;
double SCvoltage = 0.0;
double SCcurrent = 0.0;
double SCpower = 0.0;

float dutyCycle = 0.0;

double desCurrentIn = .1;
float error = 0.0;
PID pid;

// use these to control purging frequency and duration
Metro updateDCTimer = Metro(1);
Metro printStatsTimer = Metro(20);
Metro getTempTimer = Metro(10);
Metro purgeTimer = Metro(20000);
Metro purgeEndTimer = Metro(20);
Metro fanUpdateTimer = Metro(100);
Metro shortCircuitTimer = Metro(10000);
Metro shortCircuitEndTimer = Metro(10);

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
  pid.setPLimits(-.1,.1);
  pid.setILimits(-.5,.5);
  pid.setDLimits(-.001,.001);

  
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

void FuelCellBootup(){
  Serial.println("Booting up Fuel Cell");
  analogWrite(FAN,fanPrct*MAXPWM);
  digitalWrite(PURGE,HIGH);
  digitalWrite(SUPPLY,HIGH);
  delay(3000);
  analogWrite(PURGE,LOW);
  purgeTimer.reset();
}

void loop() {
  // read sensors
  updatePowerStats();
  if(getTempTimer.check()){
    FCtemp = LPF(FCtemp,readTemp(),.9);
  }
  
  // Safety checks
  if (SCvoltage>60){
    Serial.println("OVER VOLTAGE...");
    emergencyPause();
  }
  if (FCpower>130){
    Serial.println("FC power too high");
    emergencyPause();
  }
  if (FCvoltage<8 || FCvoltage>20){
    Serial.println("FC voltage out of range");
    emergencyShutdown();
  }
  if(FCtemp>65){
    Serial.print("FC Temperature Too High (");
    Serial.print(FCtemp,1);
    Serial.println("°C");
    emergencyShutdown();
  }

  // update controls
  error = desCurrentIn-FCcurrent;
  if(updateDCTimer.check()){
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
  }
  if(shortCircuitEndTimer.check()){
    shortCircuit = false;
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
    desCurrentIn = Serial.parseFloat();
  }
}

void emergencyPause(){
  Serial.println("pausing for 5 secs");
  dutyCycle = 0;
  digitalWrite(SFET,LOW);
  startTime=millis();
  while((millis()-startTime)<5000){
    
    // read sensors
    updatePowerStats();
    if(getTempTimer.check()){
      Serial.println(readTemp());
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
    updatePowerStats();
    if(getTempTimer.check()){
      Serial.println(readTemp());
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

void updateDC(){
//  if ((millis()-startTime)/1000.0<10){
//    dutyCycle = 0;
//    analogWrite(SFET,dutyCycle*MAXPWM);
//    return;
//  }
  int tConst = 180;
  float minVal = 0;
  float maxVal = 2;
  float stayTime = 15;
  float deltaCurrent = (maxVal-minVal)/(tConst/2/stayTime);
//  if((millis()-startTime)/1000.0>tConst){
//    startTime=millis();
//  }
  if (((millis()-startTime)/1000.0)<(tConst/2)){
    desCurrentIn = (millis()-startTime)/1000.0*((maxVal-minVal)/tConst*2) + minVal;
    desCurrentIn = round(desCurrentIn/deltaCurrent)*deltaCurrent;
  }
  else{
    desCurrentIn = 2*(maxVal-minVal) - (millis()-startTime)/1000.0*((maxVal-minVal)/tConst*2) + minVal;
    desCurrentIn = round(desCurrentIn/deltaCurrent)*deltaCurrent;
  }
//    dutyCycle=.02;
//    dutyCycle = .1*sin(1*(millis()-startTime)/1000.0/2/3.1415)+.15;
//    desCurrentIn = 1;
//    pid.update();
  if(desCurrentIn>11){
    desCurrentIn=11;
  }
////  dutyCycle = .00;
  desCurrentIn = 1.6;

//  pid.update();
  dutyCycle = 0.054;

  if(shortCircuit){
    dutyCycle = 1;
  }
  analogWrite(SFET,dutyCycle*MAXPWM);
}

void printStatsSerial(){
  Serial.print((millis()-startTime)/1000.0);
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
  Serial.print(desCurrentIn,2);
  Serial.print("A\t");
  Serial.print(dutyCycle*100,2);
  Serial.print("%\t");
  Serial.print(SCpower/FCpower*100,2);
  Serial.print("%\t");
  Serial.print(tempCtoF(FCtemp));
  Serial.print("°F\t");
//    Serial.print(flowmeter.readVoltage(),5);
//    Serial.print("V\t");
//    Serial.print(flowmeter.readFlow(),2);
//    Serial.print("mg/m\t");
  Serial.println();
}
void updatePowerStats(){
  FCvoltage = readFCvoltage(); //LPF(FCvoltage,readFCvoltage(),.99);
  FCcurrent = readFCcurrent(); //LPF(FCcurrent,readFCcurrent(),.99);
  SCvoltage = readSCvoltage(); //LPF(SCvoltage, readSCvoltage(), .99); // smooth/filter
  SCcurrent = readSCcurrent(); //LPF(SCcurrent, readSCcurrent(), .99);
  FCpower = FCvoltage*FCcurrent; //LPF(FCpower,FCvoltage*FCcurrent,.9);
  SCpower = SCvoltage*SCcurrent;
  if(shortCircuit){
    FCpower = 0;
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

void INAinit()
{
  Wire.beginTransmission(0x40);
  Wire.write(0x00);//reg select = 0x00
  Wire.write(0b0111);//64 averages, 1ms voltage sampling
  Wire.write(0b100111);//1ms current sampling, free running
  Wire.endTransmission();
}
uint16_t INAreadReg(uint8_t reg)
{
  Wire.beginTransmission(0x40);
  Wire.write(reg);//read from the bus voltage
  Wire.endTransmission();

  Wire.requestFrom(0x40, 2);

  delayMicroseconds(100);
  if (Wire.available() < 2)
    return 1337;

  uint16_t resp = (uint16_t)Wire.read() << 8;
  resp |= Wire.read();

  return resp;
}
