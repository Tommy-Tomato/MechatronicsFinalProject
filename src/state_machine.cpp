#include <Arduino.h>
#include <Pixy2.h>

extern Pixy2 pixy;

extern double setpoint;
extern int baseSpeed;

void PID_control();
void stopMotors();
void resetPIDHeading();

// Pixy tracking constants
const int SIG_ORANGE = 1;
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

// functions
bool detectOrange();
void updateStateMachine();
void stateSearchPuck();
void stateFollowPuck();
float wrapAngle(float a);

float wrapAngle(float a) {
  while (a >= 360.0) a -= 360.0;
  while (a < 0.0) a += 360.0;
  return a;
}

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
  baseSpeed = SLOW_SPEED;

  if (millis() - lastSearchUpdate > 90) {
    if (searchDirRight) {
      setpoint += 8.0;
    } else {
      setpoint -= 8.0;
    }

    setpoint = wrapAngle(setpoint);
    searchDirRight = !searchDirRight;
    lastSearchUpdate = millis();
  }

  PID_control();
}

void stateFollowPuck() {
  if (!orangeSeen) {
    if (millis() - lastSeenTime > LOST_TIMEOUT) {
      state = SEARCH_PUCK;
    }
    PID_control();
    return;
  }

  int pixelError = orangeX - CAM_CENTER_X;

  // speed logic:
  // go faster when puck is farther away, slower when very close
  if (orangeArea > CLOSE_AREA) {
    baseSpeed = 90;
  } else {
    baseSpeed = FAST_SPEED;
  }

  // if nearly centered, drive mostly straight
  if (abs(pixelError) <= CENTER_TOL) {
    PID_control();
    return;
  }

  // convert camera error into heading correction
  float turnDelta = TURN_GAIN * pixelError;

  if (turnDelta > MAX_TURN_DELTA) turnDelta = MAX_TURN_DELTA;
  if (turnDelta < -MAX_TURN_DELTA) turnDelta = -MAX_TURN_DELTA;

  // IMPORTANT:
  // update setpoint every loop so the robot keeps tracking a moving puck
  setpoint += turnDelta;
  setpoint = wrapAngle(setpoint);

  PID_control();
}