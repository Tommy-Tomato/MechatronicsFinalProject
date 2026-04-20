#include <Arduino.h>
#include <Pixy2.h>
#include "PID_motors.h"

extern Pixy2 pixy;

void PID_control();
void stopMotors();
void resetPIDHeading();

// Pixy tracking constants
const int SIG_ORANGE = 1; // signature of Pixy2
const int CAM_CENTER_X = 158;
const int CENTER_TOL = 8;

// tune these on the robot
const int FAST_SPEED = 165;
const int SLOW_SPEED = 120;
const float TURN_GAIN = 0.09;      // deg of heading change per pixel error
const float MAX_TURN_DELTA = 10.0; // limit heading jump per loop
const int CLOSE_AREA = 5200;       // stop / slow when puck is very close
const unsigned long LOST_TIMEOUT = 250;

// enum states
enum State {
  SEARCH_PUCK,FOLLOW_PUCK
};

State state = SEARCH_PUCK;

// ===== tracking variables =====
bool orangeSeen = false;
int orangeX = CAM_CENTER_X;
int orangeArea = 0;
unsigned long lastSeenTime = 0;
unsigned long lastSearchUpdate = 0;
bool searchDirRight = true;

Motor motors;

// functions
bool detectOrange();
void updateStateMachine();
void stateSearchPuck();
void stateFollowPuck();

// detecting orange
bool detectOrange() {
  pixy.ccc.getBlocks();
  orangeSeen = false;

  int bestIndex = -1;
  int bestArea = 0;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == SIG_ORANGE) {
      int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;

      // track biggest orange object
      if (area > bestArea) {
        bestArea = area;
        bestIndex = i;
      }
    }
  }

  if (bestIndex >= 0) {
    orangeSeen = true;
    orangeX = pixy.ccc.blocks[bestIndex].m_x;
    orangeArea = bestArea;
    lastSeenTime = millis();
    return true;
  }

  orangeArea = 0;
  return false;
}

// switches
void updateStateMachine() {
  detectOrange();

  switch (state) {
    case SEARCH_PUCK:
      stateSearchPuck();
      break;

    case FOLLOW_PUCK:
      stateFollowPuck();
      break;
  }
}

void stateSearchPuck() {
  if (orangeSeen) {
    state = FOLLOW_PUCK;
    return;
  }

  // keep moving slowly while sweeping heading
  motors.setBaseSpeed(SLOW_SPEED);

  if (millis() - lastSearchUpdate > 90) {
    if (searchDirRight) {
      motors.adjustHeading(8.0);
    } else {
      motors.adjustHeading(-8.0);
    }

    searchDirRight = !searchDirRight;
    lastSearchUpdate = millis();
  }

  motors.update();
}

void stateFollowPuck() {
  if (!orangeSeen) {
    if (millis() - lastSeenTime > LOST_TIMEOUT) {
      state = SEARCH_PUCK;
    }
    motors.update();
    return;
  }

  int pixelError = orangeX - CAM_CENTER_X;

  // go faster when puck is farther away, slower when very close
  if (orangeArea > CLOSE_AREA) {
    motors.setBaseSpeed(90);;
  } else {
    motors.setBaseSpeed(FAST_SPEED);
  }

  // if nearly centered, drive mostly straight
  if (abs(pixelError) <= CENTER_TOL) {
  float turnDelta = TURN_GAIN * pixelError;

  if (turnDelta > MAX_TURN_DELTA) turnDelta = MAX_TURN_DELTA;
  if (turnDelta < -MAX_TURN_DELTA) turnDelta = -MAX_TURN_DELTA;

  motors.adjustHeading(turnDelta);
  }
  
  motors.update();
}