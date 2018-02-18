#include <i2c_t3.h>
#include <PID.h>
#include <Metro.h> // schedules events at regular times

#define SFET 9
#define SCL1 16
#define SDA1 17
#define ICAP A1
#define VCAP A0

#define maxPWM 4096

#define Kp .1
#define Ki 0.05
#define Kd 0

double FCvoltage = 0.0;
double FCcurrent = 0.0;
double FCpower = 0.0;
double SCvoltage = 0.0;
double SCcurrent = 0.0;
double SCpower = 0.0;

float dutyCycle = 0.05;

double desPowerOut = 2.5;
float error = 0.0;
PID pid;

Metro updateDC = Metro(10);
Metro printStats = Metro(50);

void setup() {
  // put your setup code here, to run once:
  analogWriteFrequency(SFET, 100000);
  analogWriteResolution(12);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, 400000);
  INAinit();

  Serial.begin(115200);

  // set up PID controller
  pid = PID(&error, &dutyCycle, Kp,Ki,Kd,FORWARD);
  pid.setLimits(0,.5);
  pid.setPLimits(-1,1);
  pid.setILimits(-.5,.5);
  pid.setDLimits(-.1,.1);

  analogWrite(SFET,dutyCycle*maxPWM);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  updatePowerStats();
  error = desPowerOut-SCpower;
//  error = 12.0-SCvoltage;

  if(updateDC.check()){
    pid.update();
    analogWrite(SFET,dutyCycle*maxPWM);
  }

  if(printStats.check()){
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
    Serial.print(pid.Pvalue,2);
    Serial.print("\t");
    Serial.print(pid.Ivalue,2);
    Serial.print("\t");
    Serial.print(pid.Dvalue,2);
    Serial.print("\t");
    Serial.println();
    
//      Serial.print(SCpower);
//      Serial.print('\t');
//      Serial.print(dutyCycle*100);
//      Serial.println();
  }
}

void updatePowerStats(){
  FCvoltage = readFCvoltage();
  FCcurrent = readFCcurrent();
  SCvoltage = LPF(SCvoltage, readSCvoltage(), .999); // smooth/filter
  SCcurrent = LPF(SCcurrent, readSCcurrent(), .999);
  FCpower = FCvoltage*FCcurrent;
  SCpower = SCvoltage*SCcurrent;
}
double LPF(double oldVal, double newVal, double alpha){
  return oldVal*alpha + newVal*(1-alpha);
}
double readSCvoltage(){
  return analogRead(VCAP)*(3.3/1023)*(210/10); //voltage divider - 10k and 200k
}
double readSCcurrent(){
  return analogRead(ICAP)*(3.3/1023)/20/0.025; //INA168 gain is 20 - shunt resistor is 25mO
}
double readFCcurrent()
{
  int16_t raw = INAreadReg(0x01); //deliberate bad cast! the register is stored as two's complement
  return raw * 0.0000025 / 0.001; //2.5uV lsb and 1mOhm resistor
}
double readFCvoltage()
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
