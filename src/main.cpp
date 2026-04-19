#include <Arduino.h>
#include "xbee.h"

xbee comms;

void setup() {
  
  int result = myFunction(2, 3);
}

void loop() {
  comms.startup();
}
