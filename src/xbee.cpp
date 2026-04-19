// NEED TO FIX
// unedited code from lab 4
// variables incorrect

// ===== XBee POSITION GATES ===== DO WE STILL NEED THESE?
//const int START_X_MIN = 45;
//const int START_X_MAX = 53;
//const int START_Y_MIN = 2;
//const int START_Y_MAX = 15;

//const int END_X_MIN = 86;
//const int END_X_MAX = 100;
//const int END_Y_MIN = 18;
//const int END_Y_MAX = 33;

//MOVED VARIABLES TO INTERFACE (xbee.h)
#include "xbee.h"

xbee::xbee() {

}




// Set target goal
    void xbee::updateTargetGoal(bool attackRight) {
    if (attackRight) {
        targetGoal[1] = GOAL_RIGHT[1];
        targetGoal[2] = GOAL_RIGHT[2];
    } else {
        targetGoal[1] = GOAL_LEFT[1];
        targetGoal[2] = GOAL_LEFT[2];
    }
    }



// HELPERS

// boolean to check if inside given coordinates
bool xbee::inBox(int x, int y, int xmin, int xmax, int ymin, int ymax) {
  return (x >= xmin && x <= xmax && y >= ymin && y <= ymax);
}


int xbee::extractDigits(const char* buf, int len, int &pos, int numDigits) {
  int value = 0;
  for (int i = 0; i < numDigits; i++) {
    if (pos >= len) return -1;
    char c = buf[pos++];
    if (c < '0' || c > '9') return -1;
    value = value * 10 + (c - '0');
  }
  return value;
}

bool xbee::parseBroadcast(const char* buf) {
  int len = strlen(buf);
  if (len < 13) return false;
  if (buf[0] != '>') return false;
  if (buf[len - 1] != ';') return false;

  if (buf[len - 3] < '0' || buf[len - 3] > '9') return false;
  if (buf[len - 2] < '0' || buf[len - 2] > '9') return false;

  int txChk = (buf[len - 3] - '0') * 10 + (buf[len - 2] - '0');

  int calcChk = 0;
  for (int i = 0; i < len - 3; i++) {
    calcChk += (unsigned char)buf[i];
  }
  calcChk += ';';
  calcChk %= 64;

  if (calcChk != txChk) return false;

  int pos = 1;

  int mBit = extractDigits(buf, len, pos, 1);
  if (mBit < 0) return false;

  int mTime = extractDigits(buf, len, pos, 4);
  if (mTime < 0) return false;

  int dataEnd = len - 3;
  bool foundSelf = false;

  while (pos + 7 <= dataEnd) {
    char robotLetter = buf[pos++];

    int rx = extractDigits(buf, len, pos, 3);
    if (rx < 0) return false;

    int ry = extractDigits(buf, len, pos, 3);
    if (ry < 0) return false;

    if (robotLetter == ROBOT_ID) {
      currPosition[0] = rx;
      currPosition[1] = ry;
      foundSelf = true;
    }
  }

  return foundSelf;
}

void xbee::processXBeeMessage() {
  if (rxIndex == 0) return;

  rxBuffer[rxIndex] = '\0';

  if (parseBroadcast(rxBuffer)) {
    xbeeHasValidPosition = true;
    Serial.print("XBee position: ");
    Serial.print(currPosition[0]);
    Serial.print(", ");
    Serial.println(currPosition[1]);
  }

  rxIndex = 0;
}

void xbee::updateXBeePosition() {
  while (Serial1.available()) {
    char c = Serial1.read();
    lastRxTime = millis();

    if (c == ';') {
      if (rxIndex < (int)(sizeof(rxBuffer) - 1)) {
        rxBuffer[rxIndex++] = c;
      }
      processXBeeMessage();
    }
    else if (c == '>') {
      rxIndex = 0;
      rxBuffer[rxIndex++] = c;
    }
    else {
      if (rxIndex < (int)(sizeof(rxBuffer) - 1)) {
        rxBuffer[rxIndex++] = c;
      } else {
        rxIndex = 0;
      }
    }
  }

  if (rxIndex > 0 && (millis() - lastRxTime >= RX_TIMEOUT_MS)) {
    processXBeeMessage();
  }
}

//PUBLIC FUNCTIONS
bool xbee::gameStarted() {
  //TODO
  return false;
}

const double* xbee::currentPosition() {
  //TODO
  return 0;
}

const double* xbee::opponentPosition() {
  //TODO
  return 0;
}

const double* xbee::LeftGoalPosition() {
  //TODO
  return GOAL_LEFT;
}
const double* xbee::rightGoalPosition() {
  //TODO
  return GOAL_RIGHT;
}