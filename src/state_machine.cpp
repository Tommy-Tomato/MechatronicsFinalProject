#include <Arduino.h>
#include <Pixy2.h>
#include "PID_motors.h"

const int SIG_ORANGE = 1;
const int camCenter_X = 158;

const int SEARCH_SPEED = 25;
const int TURN_STEP = 6;
const int CENTER_TOL = 15;

bool orangeSeen = false;
int orangeX = 0;

Motor motors;
Pixy2 pixy;

enum State {IDLE, CHASE_PUCK};
State state = IDLE;

bool detectOrange();
void updateStateMachine();
void stateIDLE();
void stateChasePuck();

bool detectOrange() {
  pixy.ccc.getBlocks();
  orangeSeen = false;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == SIG_ORANGE) {
      orangeSeen = true;
      orangeX = pixy.ccc.blocks[i].m_x;
      return true;
    }
  }
  return false;
}

void updateStateMachine() {
  switch (state) {
    case IDLE:
      stateIDLE();
      break;
    case CHASE_PUCK:
      stateChasePuck();
      break;
  }
}

void stateIDLE() {
  motors.stopMotors();
}

void stateChasePuck() {
  if (!detectOrange()) {
    motors.update(SEARCH_SPEED);
    motors.update(SEARCH_SPEED);
    return;
  }

  if (orangeX < camCenter_X - CENTER_TOL) {
    motors.setTurn(-TURN_STEP);
  }
  else if (orangeX > camCenter_X + CENTER_TOL) {
    motors.setTurn(+TURN_STEP);
  }

  if (setpoint >= 360.0) setpoint -= 360.0;
  if (setpoint < 0.0) setpoint += 360.0;

  PID_control();
  delay(20);
}