#include <Arduino.h>
#include "xbee.h"
#include "state_machine.h"


xbee comms;
Motor motors;
state_machine state;

void setup() {
  comms.startup();
  motors.initMotors();
  delay(1000);
}

void loop() {
  state.updateStateMachine();
}

//    while (Serial1.available()) {
//    Serial.write(Serial1.read());
//  }
//const double* Position = comms.currentPosition();
//Serial.println(Position[1]);
//  long startTime = millis();
//  while(millis()-startTime < 3000) {
//    motors.update();
//  }
//  
//  motors.stopMotors();
//  delay(1000);