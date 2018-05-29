#include "StatsManager.h"
#include <i2c_t3.h>

StatsManager::StatsManager(bool* shortCircuit, double* FCtemp,
              double* FCvoltage, double* FCcurrent, double* FCpower,
              double* SCvoltage, double* SCcurrent, double* SCpower){
  // Serial.println("Initializing Stats Manager...");
  this->shortCircuit = shortCircuit;
  this->FCtemp = FCtemp;
  this->FCvoltage = FCvoltage;
  this->FCcurrent = FCcurrent;
  this->FCpower = FCpower;
  this->SCvoltage = SCvoltage;
  this->SCcurrent = SCcurrent;
  this->SCpower = SCpower;
}
void StatsManager::initializeStats(){
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, 400000);
//  INAinit();

  *FCvoltage = readFCvoltage();
  *FCcurrent = readFCcurrent();
  *SCvoltage = readSCvoltage();
  *SCcurrent = readSCcurrent();
  *FCpower = (*FCvoltage)*(*FCcurrent);
  *SCpower = (*SCvoltage)*(*SCcurrent);
}
void StatsManager::updateStats(){
  *FCvoltage = LPF(*FCvoltage,readFCvoltage(),.95); //LPF(*FCvoltage,readFCvoltage(),.99);
  *FCcurrent = LPF(*FCcurrent,readFCcurrent(),.99); //LPF(*FCcurrent,readFCcurrent(),.99);
  *SCvoltage = LPF(*SCvoltage,readSCvoltage(),.95); //LPF(*SCvoltage, readSCvoltage(), .99); // smooth/filter
  *SCcurrent = LPF(*SCcurrent,readSCcurrent(),.95); //LPF(*SCcurrent, readSCcurrent(), .99);
  *FCpower = LPF(*FCpower,(*FCvoltage)*(*FCcurrent),.9); //LPF(FCpower,FCvoltage*FCcurrent,.9);
  *SCpower = (*SCvoltage)*(*SCcurrent);
  converterEff = LPF(converterEff,((*FCpower)<0.01 ? 0 : (*SCpower)/(*FCpower)),.99);
  if(*shortCircuit){
    *FCpower = 0;
  }
  *FCtemp = LPF(*FCtemp,readTemp(),.99);
}

double LPF(double oldVal, double newVal, double alpha){
  return oldVal*alpha + newVal*(1-alpha);
}
double readSCvoltage(){
  return analogRead(VCAP)*(3.0/1023)*(210/10); //voltage divider - 10k and 200k
  // 0.0428V calibrated
}
double readSCcurrent(){
  return analogRead(ICAP)*(3.0/1023)/20/0.01; //INA168 gain is 20 - shunt resistor is 10mO
}
double readFCcurrent(){
  int16_t raw = INAreadReg(0x01) ; //deliberate bad cast! the register is stored as two's complement
  return raw * 0.0000025 / 0.001 * .805/.837 * 0.577/0.58;// * 0.81344 + 0.00216; //2.5uV lsb and 10mOhm resistor *fudged*
}
double readFCvoltage(){
  uint16_t raw = INAreadReg(0x02);
  return raw * 0.00125 * 0.9998 * 17.78/17.73 * 17.5/17.55; //multiply by 1.25mV LSB
}
double readTemp(){
  double prct = analogRead(TEMP)/1024.0;
  if (prct > .95){
    return 0;
  }
//  return 20 + ((prct/(1-prct)*TEMP_RES)-1076)/3.8;
  return 20;
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
