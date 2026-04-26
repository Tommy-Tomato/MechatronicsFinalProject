#include "state_machine.h"
#include "xbeeRadio.h"
#include "PID_motors.h"

Pixy2 pixy;

state_machine::state_machine(XBeeRadio& r, Motor& m)
    : radio(r), 
      motors(m),
      lastSpotted(0),
      SIG_ORANGE(1),
      CAM_CENTER_X(158),
      CENTER_TOL(15),
      FAST_SPEED(170),
      SLOW_SPEED(80),
      TURN_GAIN(0.1f),
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
  delayMicroseconds(10);
  digitalWrite(PINGpin, LOW);

  pinMode(PINGpin, INPUT);

  long pulseDuration = pulseIn(PINGpin, HIGH, 15000);

  if (pulseDuration == 0) {
    return -1.0f;
  }

  float distance = pulseDuration * 0.0343f / 2.0f;

  //Serial.print("ping: ");
  //Serial.println(distance);

  return distance;
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
      lastSpotted = millis();

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
    state = FOLLOW_PUCK;

    return true;
  }

  orangeArea = 0;
  return false;
}

// switches
void state_machine::updateStateMachine() {
  radio.updateXBeePosition();  
  Serial.print("current heading: ");
  Serial.println(motors.getYaw() - motors.headingOffset);

  if (avoidWall()) {
    Serial.println("avoiding walls");  
    return;
  }
  
  
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
    
    motors.stopMotors();
    return;
  }
    if (!motors.turning) {
      if (millis() - lastSpotted > 5000) {

        Serial.println("nothing found, turning...");
        motors.tankTurn();
        lastSpotted = millis();
      } else {
        Serial.println("searching...");
        // keep moving slowly while sweeping heading
        motors.setBaseSpeed(SLOW_SPEED);
        
      }
  }
  motors.update();
}

void state_machine::stateFollowPuck() {
  motors.turning = false;

  if (pingDistance > 0 && pingDistance <= 10.0f) {
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
  radio.updateXBeePosition();

  // ===== robot position =====
  const double* pos = radio.currentPosition();
  double x = pos[0];
  double y = pos[1];

  // ===== vector to goal =====
  double dx = goalX - x;
  double dy = goalY - y;

  // ===== compute goal heading =====
  double goalHeading = atan2(dy, dx) * 180.0 / PI;
  if (goalHeading < 0) goalHeading += 360.0;
  goalHeading = 360.0 - goalHeading;

  // ===== wall avoidance parameters =====
  const int fieldX_min = 8;
  const int fieldX_max = 99;
  const int fieldY_min = 21;
  const int fieldY_max = 210;

  const int wallBuffer = 25;
  const int safeZone   = 35;

  // ===== wall detection =====
  bool inBuffer =
    (x < fieldX_min + wallBuffer) ||
    (x > fieldX_max - wallBuffer) ||
    (y < fieldY_min + wallBuffer) ||
    (y > fieldY_max - wallBuffer);

  // ===== safe zone (fully recovered) =====
  bool wellInside =
    (x > fieldX_min + safeZone) &&
    (x < fieldX_max - safeZone) &&
    (y > fieldY_min + safeZone) &&
    (y < fieldY_max - safeZone);

  static bool avoidingWall = false;

  if (inBuffer) avoidingWall = true;
  if (wellInside) avoidingWall = false;

  // ===== default behavior =====
  double finalHeading = goalHeading;

  if (avoidingWall) {
    double avoidHeading = goalHeading;

    if (x < fieldX_min + wallBuffer) {
      avoidHeading = motors.headingOffset;
    }
    else if (x > fieldX_max - wallBuffer) {
      avoidHeading = motors.headingOffset + 180;
    }
    else if (y < fieldY_min + wallBuffer) {
      avoidHeading = motors.headingOffset + 270;
    }
    else if (y > fieldY_max - wallBuffer) {
      avoidHeading = motors.headingOffset + 90;
    }

    // blend goal + escape direction
    finalHeading = 0.3 * goalHeading + 0.7 * avoidHeading;
    motors.setBaseSpeed(SLOW_SPEED);
  } else {
    motors.setBaseSpeed(FAST_SPEED);
  }

  motors.setHeading(finalHeading);
  motors.update();

  // ===== shoot condition =====
  if (sqrt(dx * dx + dy * dy) < 30) {
    motors.stopMotors();
    delay(2000);
  }

  // ===== puck lost fallback =====
  if (readPing() > 15) {
    Serial.println("puck lost");
    state = SEARCH_PUCK;
  }
}

bool state_machine::avoidWall() {

  static bool avoiding = false;

  const double* pos = radio.currentPosition();
  double x = pos[0];
  double y = pos[1];

  const int fieldX_min = 8;
  const int fieldX_max = 99;
  const int fieldY_min = 21;
  const int fieldY_max = 210;

  const int wallBuffer = 25;
  const int safeZone   = 35;

  bool inBuffer =
    (x < fieldX_min + wallBuffer) ||
    (x > fieldX_max - wallBuffer) ||
    (y < fieldY_min + wallBuffer) ||
    (y > fieldY_max - wallBuffer);

  bool wellInside =
    (x > fieldX_min + safeZone) &&
    (x < fieldX_max - safeZone) &&
    (y > fieldY_min + safeZone) &&
    (y < fieldY_max - safeZone);

  if (inBuffer) avoiding = true;
  if (wellInside) avoiding = false;

  if (!avoiding) return false;

  double desiredYaw = motors.getYaw();

  if (x < fieldX_min + wallBuffer) {
    desiredYaw = motors.headingOffset;
  }
  else if (x > fieldX_max - wallBuffer) {
    desiredYaw = motors.headingOffset + 180;
  }
  else if (y < fieldY_min + wallBuffer) {
    desiredYaw = motors.headingOffset + 270;
  }
  else if (y > fieldY_max - wallBuffer) {
    desiredYaw = motors.headingOffset + 90;
  }

  motors.setBaseSpeed(SLOW_SPEED);
  motors.setHeading(desiredYaw);
  motors.update();

  return true;
}
