#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <DualMAX14870MotorShield.h>
#include <Pixy2.h>
#include "PID_motors.h"
#include "xbeeRadio.h"
#include <Ultrasonic.h>

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

XBeeRadio& radio;
Motor& motors;

// enum states
enum State {
  SEARCH_PUCK,FOLLOW_PUCK,SHOOT_PUCK
};

State state = SEARCH_PUCK;

// ===== tracking variables =====
bool orangeSeen;
int orangeX;
int orangeArea;
unsigned long lastSeenTime;
unsigned long lastSearchUpdate;
bool searchDirRight;
long lastSpotted;

// === ping and goal var ===
  float pingDistance;
  float goalX, goalY;


bool detectOrange();
float readPing();        
void readGoalFromXBee();
double getHeadingError(double dx, double dy);

double searchYaw;

public:
    state_machine(XBeeRadio& r, Motor& m);    
    void updateStateMachine();
    void stateSearchPuck();
    void stateFollowPuck();
    void stateShootPuck();
};

#endif