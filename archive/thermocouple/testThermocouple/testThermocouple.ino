#include <SPI.h>
#include "Adafruit_MAX31855.h"

#define CS_THERMO 10


Adafruit_MAX31855 thermocouple(CS_THERMO);

void setup() {
  Serial.begin(115200);
  pinMode(CS_THERMO,OUTPUT);
  // wait for MAX chip to stabilize
  delay(5000);
}

void loop() {

  Serial.print("Internal Temp = ");
  Serial.print(thermocouple.readInternal());
  Serial.print("°C\tFC Temp = ");
  Serial.print(thermocouple.readCelsius());
  Serial.print("°C\n");
  delay(1000);
}
