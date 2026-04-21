#include <Arduino.h>
#include "xbee.h"
#include "PID_motors.h"


xbee comms;
Motor motors;

void setup() {
  comms.startup();
  motors.initMotors();
}

void loop() {
//    while (Serial1.available()) {
//    Serial.write(Serial1.read());
//  }
//const double* Position = comms.currentPosition();
//Serial.println(Position[1]);
  motors.update();
  delay(2000);
  //motors.setTurn(10);
  motors.stopMotors();
  delay(1000);
}

