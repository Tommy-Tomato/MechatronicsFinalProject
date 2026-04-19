#include <Arduino.h>
#include "PID_motors.h"

const int SIG_ORANGE = 1; // change signature later
const int CAM_CENTER_X = 158;

const int SEARCH_SPEED = 25;  // searching for puck
const int TURN_STEP = 6;   // how much to adjust heading each loop
const int CENTER_TOL = 15;   // jitter deadband

bool orangeSeen = false;
int orangeX = 0;

bool orangeSeen = false;
int orangeX = 0;

Motor motor;

// enum states
enum State {IDLE, CHASE_PUCK};
State state = IDLE;

// ===== STATE MACHINE =====
void updateStateMachine() {
  switch (state) {
    case IDLE: stateIDLE(); break;
    case CHASE_PUCK: stateChasePuck(); break;
  }
}
  
  // Obstacle avoidance
  if (distancecm > 0 && distancecm < 5) {
    // Double-check we're not seeing a marker right in front
    if (!seesAnyMarker || (seesAnyMarker && distancecm < 2)) {
      setState(REVERSE);
      return;
    }
  }

  // Normal forward driving with PID control
  PID_control();
  delay(20);
}

void stateStopped() {
  stopMotors();
}

void stateChasePuck() {

  if (!detectOrange()) {
    // no puck? :( rotate that ho
    motors.setM1Speed(SEARCH_SPEED);
    motors.setM2Speed(SEARCH_SPEED);
    return;
  }

  // puck detected, adjust heading
  if (orangeX < CAM_CENTER_X - CENTER_TOL) {
    setpoint -= TURN_STEP;   // turn left
  }
  else if (orangeX > CAM_CENTER_X + CENTER_TOL) {
    setpoint += TURN_STEP;   // turn right
  }
  // else: centered, go straight

  if (setpoint >= 360.0) setpoint -= 360.0;
  if (setpoint < 0.0) setpoint += 360.0;

  // PID steering and control
  PID_control();
  delay(20);
}