#include <Arduino.h>
#include "xbeeRadio.h"
#include "state_machine.h"


Motor motors;
XBeeRadio comms;
state_machine state;
extern Pixy2 pixy;
state_machine state(comms, motors);


void setup() {
  motors.initMotors();
  pixy.init();
  comms.startup();
  
  Serial.begin(9600);
  Serial.println("starting...");
  
  delay(1000);
}
  
void loop() {
  if (comms.gameStarted()) {
    state.updateStateMachine();
  } else {
    motors.stopMotors();
  }
}

