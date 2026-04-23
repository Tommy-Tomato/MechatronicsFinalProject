#include <Arduino.h>
#include "xbeeRadio.h"
#include "state_machine.h"


XBeeRadio comms;
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
  const double* Position = comms.currentPosition();
  Serial.println(Position[1]);
}

