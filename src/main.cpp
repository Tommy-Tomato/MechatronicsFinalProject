#include <Arduino.h>
#include "xbee.h"
#include "state_machine.h"


xbee comms;
Motor motors;
state_machine state;
extern Pixy2 pixy;


void setup() {
  comms.startup();
  motors.initMotors();
  Serial.begin(9600);
  Serial.println("starting...");
  pixy.init();
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
//  

//  
//  motors.stopMotors();
//  delay(1000);