
#include "state_machine.h"

Pixy2 pixy;

// functions



state_machine::state_machine()
    : SIG_ORANGE(1),
      CAM_CENTER_X(158),
      CENTER_TOL(8),
      FAST_SPEED(165),
      SLOW_SPEED(120),
      TURN_GAIN(0.09f),
      MAX_TURN_DELTA(10.0f),
      CLOSE_AREA(5200),
      LOST_TIMEOUT(250),
      state(SEARCH_PUCK),
      orangeSeen(false),
      orangeX(CAM_CENTER_X),
      orangeArea(0),
      lastSeenTime(0),
      lastSearchUpdate(0),
      searchDirRight(true) {
}



// detecting orange
bool state_machine::detectOrange() {
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
    Serial.println("Orange Seen");
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

  switch (state) {
    case SEARCH_PUCK:
      stateSearchPuck();
      break;

    case FOLLOW_PUCK:
      Serial.println("following...");
      stateFollowPuck();
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

void state_machine::stateFollowPuck() {
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