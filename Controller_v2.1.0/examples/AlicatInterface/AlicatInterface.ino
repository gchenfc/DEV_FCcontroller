/*  Gerry Chen
 *  Alicat Communication Example
 *  Connection Instructions:
 *    with the alicat's 8pin mini-DIN connected according to the color code
 *    in the Alicat manual, make the following connections:
 *      purple to ground
 *      yellow to 7/RX3
 *      red    to 8/TX3
 */


#include <Metro.h>

Metro watchdogTimer = Metro(50);
Metro pollTimer = Metro(100);

void setup() {
  kickDog();
  Serial3.begin(57600,SERIAL_8N1_RXINV_TXINV);
  // check baud rate on Alicat Device
  // RX and TX both inverted - no parity bits
  Serial.begin(115200);
}

void loop() {
  // read from alicat - send to computer serial
  while(Serial3.available()){
    int a = Serial3.read();
    if (a==13){ // Carraige return
      Serial.println(millis());
    }
    else{
      Serial.print(char(a));
    }
  }
  // read from computer - send to alicat (mostly for testing purposes
  while(Serial.available()){
    char a = Serial.read();
    Serial3.write(a);
  }
  // poll from Alicat by writing an 'A' followed by a CR
  //  note: setting the alicat ID to '@' on the Alicat will put it in streaming mode if desired
  if (pollTimer.check()){
    Serial3.write('A');
    Serial3.write('\r');
  }

  // fluff
  if (watchdogTimer.check()){
    kickDog();
  }
  delay(1);
}

void kickDog(){
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
  digitalWriteFast(13,!digitalReadFast(13));
}
