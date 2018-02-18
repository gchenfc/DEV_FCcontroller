#include <Metro.h>

Metro printStats = Metro(10);
long startTime = millis();

double FCvoltage,FCcurrent,FCpower;
double SCvoltage,SCcurrent,SCpower;
double dutyCycle,boardTemp,FCtemp,fanPrct;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  double t = (millis()-startTime)/1000.0;
  FCpower = 10 + 5*sin(6.28*t/1);
  SCpower = 9 + 4.7*sin(6.28*t/1);
  if (printStats.check()){
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
    Serial.print(dutyCycle*100,2);
    Serial.print("%\t");
    Serial.print(boardTemp,2);
    Serial.print("°C\t");
    Serial.print(FCtemp,2);
    Serial.print("°C\t");
    Serial.print(fanPrct*100,2);
    Serial.print("%\t");
    Serial.println();
  }
}
