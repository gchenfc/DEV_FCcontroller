#include <i2c_t3.h>

#define SFET 9
#define SCL1 16
#define SDA1 17
#define ICAP A1
#define VCAP A0

#define maxPWM 4096

double FCvoltage = 0.0;
double FCcurrent = 0.0;
double SCvoltage = 0.0;
double SCcurrent = 0.0;

double desVoltage = 12.0;
double dutyCycle = 0.05;

void setup() {
  // put your setup code here, to run once:
  analogWriteFrequency(SFET, 100000);
  analogWriteResolution(12);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, 400000);
  INAinit();

  Serial.begin(115200);

  analogWrite(SFET,dutyCycle*maxPWM);
  SCvoltage = analogRead(VCAP)*3.3/1023*210/10;
}

void loop() {
  // put your main code here, to run repeatedly:
//  analogWrite(SFET, 0.01*maxPWM);
  SCvoltage = SCvoltage*(1-.01) + readSCvoltage() * (0.01);
  SCcurrent = SCcurrent*(1-.001) + readSCcurrent() * (0.001);
  Serial.print(INAvoltage(),5);
  Serial.print('\t');
  Serial.print(INAcurrent(),5);
  Serial.print('\t');
  Serial.print(dutyCycle,4);
  Serial.print('\t');
  Serial.print(SCvoltage);
  Serial.print('\t');
  Serial.print(SCcurrent);
  Serial.print('\t');
  Serial.print(-(SCvoltage-desVoltage)/300,4);
  Serial.print('\t');
//  Serial.print(dutyCycle*maxPWM);
  Serial.print('\n');
  double newDC = -(readSCvoltage()-desVoltage)/300+dutyCycle;
  if (newDC < 0 or newDC >.5){
    newDC = dutyCycle;
  }
  if(abs(newDC-dutyCycle)>.0001){
    dutyCycle = newDC*.1 + dutyCycle*.9;
    analogWrite(SFET,dutyCycle*maxPWM);
  }
}
double readSCvoltage(){
  return analogRead(VCAP)*(3.3/1023)*(210/10); //voltage divider - 10k and 200k
}
double readSCcurrent(){
  return analogRead(ICAP)*(3.3/1023)/20/0.025; //INA168 gain is 20 - shunt resistor is 25mO
}
double INAcurrent()
{
  int16_t raw = INAreadReg(0x01); //deliberate bad cast! the register is stored as two's complement
  return raw * 0.0000025 / 0.001 * .8 ; //2.5uV lsb and 1mOhm resistor
}

double INAvoltage()
{
  uint16_t raw = INAreadReg(0x02);
  return raw * 0.00125; //multiply by 1.25mV LSB
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
