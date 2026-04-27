#include <Arduino.h>
#include "xbeeRadio.h"
#include "state_machine.h"
#include "Solenoid.h"


Motor motors;
XBeeRadio comms;
Solenoid launcher;
extern Pixy2 pixy;
state_machine state(comms, motors, launcher);

bool firstTime = true;
long firstTimeStart = 0;

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
    if (firstTime) {
      motors.setBaseSpeed(375);
      firstTimeStart = millis();
      firstTime = false;
    } else if (millis() - firstTimeStart > 2000) {
      motors.setBaseSpeed(80);
      state.firstGrab = false;
    }
     state.updateStateMachine();
   } else {
     motors.stopMotors();
   }
}

