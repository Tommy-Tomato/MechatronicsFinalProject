#include <Arduino.h>
#include "xbeeRadio.h"
#include "state_machine.h"
#include "Solenoid.h"


Motor motors;
XBeeRadio comms;
Solenoid launcher;
extern Pixy2 pixy;
state_machine state(comms, motors, launcher);


void setup() {
  motors.initMotors();
  pixy.init();
  comms.startup();
  launcher.init();
  
  Serial.begin(9600);
  Serial.println("starting...");
  
  delay(1000);
}
  
void loop() {
   comms.updateXBeePosition();
   if (comms.gameStarted()) {
     state.updateStateMachine();
   } else {
     motors.stopMotors();
   }
}

