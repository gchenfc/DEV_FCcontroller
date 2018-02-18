#include <i2c_t3.h>
#include <SPI.h>
#include <PID.h>
#include <Metro.h> // schedules events at regular times
#include "Adafruit_MAX31855.h"
#include "Alicat.h"

#define SFET 9
#define SCL1 16
#define SDA1 17
#define ICAP A1
#define VCAP A0

#define CS_THERMO 10
#define FAN 3
#define PURGE 4
#define SUPPLY 5

#define PWM_SPEED 100000
#define PWM_RES 12
#define MAXPWM 4096

#define Kp .002
#define Ki 0.07
#define Kd 0.0001

double FCvoltage = 0.0;
double FCcurrent = 0.0;
double FCpower = 0.0;
double SCvoltage = 0.0;
double SCcurrent = 0.0;
double SCpower = 0.0;

float dutyCycle = 0.05;

double desPowerIn = 10;
float error = 0.0;
PID pid;

Metro updateDC = Metro(1);
Metro printStats = Metro(10);
Metro getThermo = Metro(1000);
Adafruit_MAX31855 thermocouple(CS_THERMO);

double FCtemp = 0.0;
double fanPrct = 0.0;
double purgePrct = 0.0;
bool supplyPrct = 1;

long startTime;

void setup() {
  // put your setup code here, to run once:
  analogWriteFrequency(SFET, PWM_SPEED);
  analogWriteResolution(PWM_RES);
  analogWriteFrequency(FAN, 20000);
  
  pinMode(CS_THERMO,OUTPUT);
  pinMode(FAN,OUTPUT);
  pinMode(PURGE,OUTPUT);
  pinMode(SUPPLY,OUTPUT);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, 400000);
  INAinit();

  Serial.begin(115200);
  startTime = millis();

  // set up PID controller
  pid = PID(&error, &dutyCycle, Kp,Ki,Kd,FORWARD);
  pid.setLimits(0,.5);
  pid.setPLimits(-1,1);
  pid.setILimits(0,.5);
  pid.setDLimits(-.1,.1);

  analogWrite(SFET,dutyCycle*MAXPWM);
  analogWrite(FAN,fanPrct*MAXPWM);
  analogWrite(PURGE,purgePrct*MAXPWM);
  digitalWrite(SUPPLY,supplyPrct);

  // temporary hack to place flowmeter in second i2c socket
  pinMode(18,OUTPUT);
  digitalWrite(18,LOW);
  analogReference(INTERNAL1V1);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  updatePowerStats();

  // define the control variable here
  error = .05-dutyCycle;
  // update the PID controller
  if(updateDC.check()){
    pid.update();
    analogWrite(SFET,dutyCycle*MAXPWM);
  }

  // TODO
  if(getThermo.check()){
//    boardTemp = thermocouple.readInternal();
//    FCtemp = thermocouple.readCelsius();
  }
  
  if(printStats.check()){
    printStats1();
  }

  if (Serial.available()){
    // process serial command
//    fanPrct = Serial.parseFloat()/100;
//    analogWrite(FAN,fanPrct*MAXPWM);
  }
}

void updatePowerStats(){
  FCvoltage = LPF(FCvoltage,readFCvoltage(),.99);
  FCcurrent = LPF(FCcurrent,readFCcurrent(),.99);
  SCvoltage = LPF(SCvoltage, readSCvoltage(), .99); // smooth/filter
  SCcurrent = LPF(SCcurrent, readSCcurrent(), .99);
  FCpower = LPF(FCpower,FCvoltage*FCcurrent,.9);
  SCpower = SCvoltage*SCcurrent;
}
double LPF(double oldVal, double newVal, double alpha){
  return oldVal*alpha + newVal*(1-alpha);
}
double readSCvoltage(){
  return analogRead(VCAP)*(3.3/1023)*(210/10)-.0428; //voltage divider - 10k and 200k
  // 0.0428V calibrated
}
double readSCcurrent(){
  return analogRead(ICAP)*(3.3/1023)/20/0.025 * 2.25/2.5 + 0.00764; //INA168 gain is 20 - shunt resistor is 25mO
}
double readFCcurrent(){
  int16_t raw = INAreadReg(0x01); //deliberate bad cast! the register is stored as two's complement
  return raw * 0.0000025 / 0.001 * 0.81344 + 0.00216; //2.5uV lsb and 1mOhm resistor *fudged*
}
double readFCvoltage(){
  uint16_t raw = INAreadReg(0x02);
  return raw * 0.00125; //multiply by 1.25mV LSB
}

void printStats1(){
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
  Serial.print("W\t");
  Serial.print(dutyCycle*100,2);
  Serial.print("%\t\t");
  Serial.print(fanPrct*100,2);
  Serial.print("%\t");
//  Serial.print(flowmeter.readVoltage(),5);
//  Serial.print("V\t");
//  Serial.print(flowmeter.readFlow(),2);
//  Serial.print("mg/m\t");
  Serial.println();
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
