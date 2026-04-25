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
    return true;
  }

  orangeArea = 0;
  return false;
}

// switches
void state_machine::updateStateMachine() {
  radio.updateXBeePosition();  

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
    return;
  }

  // if (millis() - lastSpotted > 4000) {

  //   Serial.println("nothing found, turning...");
  //   double target;
  //   target = motors.getYaw() + 180;
  //   motors.setTurn(target);
  //   lastSpotted = millis();

  // } else if (!motors.turning) {
    //Serial.println("searching...");
    // keep moving slowly while sweeping heading
    motors.setBaseSpeed(SLOW_SPEED);

    if (millis() - lastSearchUpdate > 700) {
      
      if (searchDirRight) {
        motors.adjustHeading(20);
      } else {
        motors.adjustHeading(-40);
      }

      searchDirRight = !searchDirRight;
      lastSearchUpdate = millis();
    }
  //}

  motors.update();
}

void state_machine::stateFollowPuck() {
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
  double robotX = pos[0];
  double robotY = pos[1];

  // ===== vector to goal =====
  double dx = goalX - robotX;
  double dy = goalY - robotY;

  // ===== field-centric desired heading =====
  double desiredHeading = (atan2(dy, dx) * 180.0 / PI);
  if (desiredHeading < 0) desiredHeading += 360.0;
  desiredHeading = 360.0 - desiredHeading;

  Serial.print(" | desired: ");
  Serial.print(desiredHeading);


  motors.setBaseSpeed(FAST_SPEED);
  motors.setHeading(desiredHeading);
  motors.update();

  if (sqrt(pow(dx,2)+pow(dy,2)) < 25) {
    //SHOOT!
    motors.stopMotors();
    delay(2000);
  }

  // ===== fallback: puck lost =====
  if (readPing() > 15) {
    Serial.println("puck lost");
    state = SEARCH_PUCK;
  }
}

bool state_machine::avoidWall() {

  

  const double* pos = radio.currentPosition();
  double x = pos[0];
  Serial.print("x: ");
  Serial.print(x);
  double y = pos[1];
  Serial.print("| y: ");
  Serial.println(y);

  // field min and max determined by these recorded coordinates
  // (8,21) bottom left corner
  // (99, 22) bottom right corner
  // (99, 211) top right corner
  // (11, 210) top left corner

  const int fieldX_min = 8;
  const int fieldX_max = 99;
  const int fieldY_min = 21;
  const int fieldY_max = 210;
  
  const int wallBuffer = 13; // change depending on when we want to start moving away from arena walls 
  // (this is the safe zone buffer area)


  double desiredYaw = motors.getYaw();

  // make heading away from closest wall
  if (x < fieldX_min + wallBuffer) {
    desiredYaw = motors.headingOffset; // drive towards +x
    Serial.println("min x");
  }
  else if (x > fieldX_max - wallBuffer) {
    desiredYaw = motors.headingOffset + 180; // drive towards -x
    Serial.println("max x");
  }
  else if (y < fieldY_min + wallBuffer) {
    desiredYaw = motors.headingOffset + 270; // drive toward +y
    Serial.println("min y");
  }
  else if (y > fieldY_max - wallBuffer) {
    desiredYaw = motors.headingOffset + 90; // drive towards -y
    Serial.println("max y");
  }
  else {
    return false;
  }
  motors.setHeading(desiredYaw);
  motors.update();
  return true;
  }