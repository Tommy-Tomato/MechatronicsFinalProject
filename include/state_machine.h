#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H


#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <DualMAX14870MotorShield.h>
#include <Arduino.h>
#include <Pixy2.h>
#include "PID_motors.h"

class state_machine {
private:
// Pixy tracking constants
const int SIG_ORANGE;
const int CAM_CENTER_X;
const int CENTER_TOL;

// tune these on the robot
const int FAST_SPEED;
const int SLOW_SPEED;
const float TURN_GAIN;      // deg of heading change per pixel error
const float MAX_TURN_DELTA; // limit heading jump per loop
const int CLOSE_AREA;       // stop / slow when puck is very close
const unsigned long LOST_TIMEOUT;

// enum states
enum State {
  SEARCH_PUCK,FOLLOW_PUCK
};

State state = SEARCH_PUCK;

// ===== tracking variables =====
bool orangeSeen;
int orangeX;
int orangeArea;
unsigned long lastSeenTime;
unsigned long lastSearchUpdate;
bool searchDirRight;

Motor motors;

bool detectOrange();



public:
    state_machine();    
    void updateStateMachine();
    void stateSearchPuck();
    void stateFollowPuck();
};

#endif
