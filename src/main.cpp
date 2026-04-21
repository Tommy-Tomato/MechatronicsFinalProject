#include <Arduino.h>
#include "xbee.h"


xbee comms;
  

void setup() {
  comms.startup();
  
}

void loop() {
 const double* Position = comms.currentPosition();
 Serial.println(Position[1]);
}

