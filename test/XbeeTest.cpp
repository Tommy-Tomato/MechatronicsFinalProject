#include <Arduino.h>
#include "xbeeRadio.h"


XBeeRadio comms;


void setup() {
  comms.startup();
  Serial.begin(9600);
  Serial.println("starting...");
  delay(1000);
}
  
void loop() {
    const double* curr = comms.currentPosition();
}