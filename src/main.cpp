#include <Arduino.h>
#include "xbee.h"

xbee comms;
  

void setup() {
  comms.startup();
  
}

void loop() {
 double Position[2] = comms.currentPosition();
 Serial.println(Position[1]);
}
