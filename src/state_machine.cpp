/ ===== STATE MACHINE =====
void updateStateMachine() {
  switch (state) {
    case IDLE: stateIDLE(); break;
    case FORWARD: stateForward(); break;
    case TURN_R: stateTurnR(); break;
    case TURN_L: stateTurnL(); break;
    case REVERSE: stateReverse(); break;
    case ROTATE: stateRotate(); break;
    case STOPPED: stateStopped(); break;
  }
}

void stopMotors() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

void stateTurnR() {
  if (!turning) {
    stopMotors();
    delay(80);
    // first entry into turn state
    double startYaw = getYaw();
    turnGoalYaw = startYaw + 90.0; // turn right 90 degrees
    if (turnGoalYaw >= 360) turnGoalYaw -= 360.0;
    turning = true;
    turnStartTime = millis();
  }
  
  performTurn(turnGoalYaw);
}

void stateTurnL() {
  if (!turning) {
    stopMotors();
    delay(80);
    // first entry into turn state
    double startYaw = getYaw();
    turnGoalYaw = startYaw - 90.0; // Turn left 90 degrees
    if (turnGoalYaw < 0) turnGoalYaw += 360.0;
    turning = true;
    turnStartTime = millis();
  }
  
  performTurn(turnGoalYaw);
}

void stateRotate() {
  if (!turning) {
    stopMotors();
    delay(80);
    double startYaw = getYaw();
    turnGoalYaw = startYaw + 185.0; // Rotate 180 degrees
    if (turnGoalYaw >= 360) turnGoalYaw -= 360.0;
    turning = true;
    turnStartTime = millis();
  }
  
  performTurn(turnGoalYaw);
}

void stateIDLE() {
  stopMotors();

  if (!mazeStarted && atStartZone()) {
    mazeStarted = true;
    resetPIDHeading();
    setState(FORWARD);
  }
}

void stateForward() {
  if (!mazeFinished && atEndZone()) {
    mazeFinished = true;
    setState(STOPPED);
    return;
  }

  // Find the most significant detection
  int primarySignature = 0;
  int largestBlock = 0;
  bool seesAnyMarker = false;
  
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_width > largestBlock) {
      largestBlock = pixy.ccc.blocks[i].m_width;
      primarySignature = pixy.ccc.blocks[i].m_signature;
      seesAnyMarker = true;
    }
  }
  
  // React based on primary signature
  if (seesAnyMarker) {
    if (primarySignature == SIG_RED && distancecm < 16) {
      setState(TURN_L);
      return;
    }
    if (primarySignature == SIG_BLUE && distancecm < 17) {
      setState(TURN_R);
      return;
    }
    if (primarySignature == SIG_GREEN && distancecm < 25) {
      setState(ROTATE);
      return;
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

void stateReverse() {
  unsigned long currentTime = millis();

  // Reset stuck counter if it's been a while since the last reverse
  if (currentTime - firstReverseTime > escapeTimeWindow) {
    reverseCount = 0;
    firstReverseTime = currentTime;
  }

  reverseCount++;

  // Back it up
  motors.setM1Speed(-R_BASE);
  motors.setM2Speed(L_BASE);
  delay(500);
  stopMotors();

  // If we've hit reverse 3 times fast, do a big escape spin
  if (reverseCount >= 3) {
    Serial.println("TRAPPED! Executing 180+ Degree Escape...");
    motors.setM1Speed(-R_BASE_ROTATION);
    motors.setM2Speed(-L_BASE_ROTATION);
    delay(750);
    stopMotors();
    reverseCount = 0;
    resetPIDHeading();
    setState(FORWARD);
  } else {
    // just a small wiggle fix
    motors.setM1Speed(R_BASE_ROTATION);
    motors.setM2Speed(L_BASE_ROTATION);
    delay(300);
    stopMotors();
    resetPIDHeading();
    setState(FORWARD);
  }
}

void stateStopped() {
  stopMotors();
}