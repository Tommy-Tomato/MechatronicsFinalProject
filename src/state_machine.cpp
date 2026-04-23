#include "state_machine.h"
#include "xbeeRadio.h"
#include "PID_motors.h"

Pixy2 pixy;

state_machine::state_machine()
    : SIG_ORANGE(1),
      CAM_CENTER_X(158),
      CENTER_TOL(15),
      FAST_SPEED(165),
      SLOW_SPEED(90),
      TURN_GAIN(0.25f),
      MAX_TURN_DELTA(15.0f),
      CLOSE_AREA(5200),
      LOST_TIMEOUT(250),
      state(SEARCH_PUCK),
      orangeSeen(false),
      orangeX(CAM_CENTER_X),
      orangeArea(0),
      lastSeenTime(0),
      lastSearchUpdate(0),
      searchDirRight(true), 
      pingDistance(999.0f), 
      goalX(0), goalY(0){
}

// PING pings and data
const int PINGpin = 3;

float state_machine::readPing() {
  pinMode(PINGpin, OUTPUT);
  digitalWrite(PINGpin, LOW);
  delayMicroseconds(2);
  digitalWrite(PINGpin, HIGH);
  delayMicroseconds(5);
  digitalWrite(PINGpin, LOW);

  pinMode(PINGpin, INPUT);
  long pulseDuration = pulseIn(PINGpin, HIGH, 30000);

  if (pulseDuration == 0) {
    return -1.0f;
  }

  float distance = pulseDuration * 0.0343f / 2.0f;
  return distance;
  Serial.println(distance); // debug
}

// detecting orange
bool state_machine::detectOrange() {
  pixy.ccc.getBlocks();
  orangeSeen = false;

  int bestIndex = -1;
  int bestArea = 0;

  for (uint16_t i = 0; i < pixy.ccc.numBlocks; i++) {
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
    //Serial.println("Orange Seen");
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
void state_machine::updateStateMachine() {
  detectOrange();
  pingDistance = readPing();

  switch (state) {
    case SEARCH_PUCK:
      stateSearchPuck();
      break;

    case FOLLOW_PUCK:
      //Serial.println("following...");
      stateFollowPuck();
      break;

    case SHOOT_PUCK:    
      stateShootPuck();
      break;
  }
}

void state_machine::stateSearchPuck() {
  if (orangeSeen) {
    state = FOLLOW_PUCK;
    return;
  }

  // keep moving slowly while sweeping heading
  motors.setBaseSpeed(SLOW_SPEED);

  if (millis() - lastSearchUpdate > 700) {
    if (searchDirRight) {
      motors.adjustHeading(70);
    } else {
      motors.adjustHeading(-70);
    }

    searchDirRight = !searchDirRight;
    lastSearchUpdate = millis();
  }

  motors.update();
}

void state_machine::stateFollowPuck() {
  if (pingDistance > 0 && pingDistance <= 5.0f) {
    Serial.println("puck is grabbed, going to score");

    const double* goal = radio.leftGoalPosition(); // left or right depending on starting position
    goalX = goal[0];
    goalY = goal[1];

    state = SHOOT_PUCK;
    return;
  }
  
  if (!orangeSeen) {
    if (millis() - lastSeenTime > LOST_TIMEOUT) {
      state = SEARCH_PUCK;
    }
    motors.update();
    return;
  }

  int pixelError = orangeX - CAM_CENTER_X;
  Serial.println(pixelError);

  // go faster when puck is farther away, slower when very close
  if (orangeArea > CLOSE_AREA) {
    motors.setBaseSpeed(90);;
  } else {
    motors.setBaseSpeed(FAST_SPEED);
  }

  // if nearly centered, drive mostly straight
  if (abs(pixelError) >= CENTER_TOL) {
    float turnDelta = -TURN_GAIN * pixelError;

    if (turnDelta > MAX_TURN_DELTA) {turnDelta = MAX_TURN_DELTA;}
    if (turnDelta < -MAX_TURN_DELTA) {turnDelta = -MAX_TURN_DELTA;}

    motors.adjustHeading(turnDelta);
  }
  
  motors.update();
}

void state_machine::stateShootPuck() {
  radio.updateXBeePosition(); // update from xbee

  // robot current position on field
  const double* pos = radio.currentPosition();
  double robotX = pos[0];
  double robotY = pos[1];

  // robot to goal line of movement
  double dx = goalX - robotX;
  double dy = goalY - robotY;

  // distance to goal
  /*double distanceGoal = sqrt(dx * dx + dy * dy);*/

  double desiredHeading = atan2(dy, dx) * 180/ PI; 
  double currentYaw = motors.getYaw(); // robot current heading

  double headingError = desiredHeading - currentYaw; // error of robot facing vs where goal is

  // shortest turn
  if (headingError > 180.0) headingError -= 360.0;
  if (headingError < -180.0) headingError += 360.0;

  float turnDelta = 0.2f * headingError; // steering correction

  if (turnDelta > MAX_TURN_DELTA) turnDelta = MAX_TURN_DELTA; // limit overcorrection
  if (turnDelta < -MAX_TURN_DELTA) turnDelta = -MAX_TURN_DELTA;

  // shoot the puck
  /*if (distanceGoal < 5)_____*/

  motors.setBaseSpeed(FAST_SPEED);
  motors.adjustHeading(turnDelta);
  motors.update();
}