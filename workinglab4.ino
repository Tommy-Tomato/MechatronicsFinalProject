#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <DualMAX14870MotorShield.h>

// ================= OBJECTS =================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
DualMAX14870MotorShield motors;

// ================= ENUM =================
// modes for the bot's brain
enum State { IDLE, FORWARD, TURN_R, TURN_L, REVERSE, ROTATE, STOPPED };
State state = IDLE;

void setState(State newState) {
  state = newState;
}

// ================= PID VARIABLES =================
double setpoint = 0.0; // desired yar (captured at startup)

double Kp = 4.5;
double Ki = 0.08;
double Kd = 1.5;

double error = 0;
double previousError = 0;
double integral = 0;
double derivative = 0;

unsigned long previousTime = 0;
unsigned long lastPIDPrint = 0;

// forward driving speed
int baseSpeed = 120;

#include <Pixy2.h>

// ===== XBee POSITION GATES =====
const int START_X_MIN = 45;
const int START_X_MAX = 53;
const int START_Y_MIN = 2;
const int START_Y_MAX = 15;

const int END_X_MIN = 86;
const int END_X_MAX = 100;
const int END_Y_MIN = 18;
const int END_Y_MAX = 33;

bool xbeeHasValidPosition = false;
bool mazeStarted = false;
bool mazeFinished = false;

// latest position from Xbee
int xPos = 0;
int yPos = 0;

char rxBuffer[128];
int rxIndex = 0;
unsigned long lastRxTime = 0;
#define RX_TIMEOUT_MS 5

#define ROBOT_ID 'D'

// ================= TURNING VARIABLES =================
double turnGoalYaw = 0;
bool turning = false;

unsigned long turnStartTime = 0;
const unsigned long turnTimeout = 3000; // timeout if turn takes too long

// Base turn speed
const int TURN_BASE_SPEED = 105;

// Helper functions for turning
double getYaw() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  return orientationData.orientation.x;
}

double angleDiff(double target, double current) {
  double diff = target - current;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  return diff;
}

// ================= OBJECTS =================
Pixy2 pixy;

unsigned long lastSearchTurnTime = 0;
unsigned long forwardStartTime = 0;
int reverseCount = 0; // tracking how many times we backed up to see if we're stuck
unsigned long firstReverseTime = 0;
const unsigned long escapeTimeWindow = 10000; // 10s window to catch if we're looping

// ================= PINS =================
const int trig_signal = 2; // sonic trigger
const int echo_signal = 3; // sonic echo
const int leftIR = A3;
const int rightIR = A5;

// ================= SENSOR VARIABLES =================
int distancecm = 0;
float pulseDuration = 0;

// ================= GLOBAL VARIABLES =================
unsigned long lastCorrectionTime = 0;
const unsigned long correctionDebounce = 3000; // don't spam wall fixes

int searchCounter = 0;
const int searchThreshold = 10; // wait a bit before looking for a gap

// ================= PIXY SIGNATURES =================
const uint8_t SIG_RED = 1, SIG_BLUE = 2, SIG_GREEN = 3;
bool seesRed = false, seesBlue = false, seesGreen = false;

// ================= BALANCED MOTOR CONSTANTS =================
// one motor is beefier than the other so these help keep it straight
const float L_BASE = 142.0;
const float R_BASE = -160.0;
const int L_BASE_ROTATION = 150;
const int R_BASE_ROTATION = -105;

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial);

  if (!bno.begin())
  {
    Serial.println("BNO055 not detected!");
    while (1);
  }

  delay(1000);

  // Capture starting yaw as heading reference
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  setpoint = orientationData.orientation.x;

  previousTime = millis();

  pixy.init();
  pinMode(trig_signal, OUTPUT);
  pinMode(echo_signal, INPUT);

  lastRxTime = millis();
  stopMotors();
}

// ================= LOOP =================
void loop()
{
  /*// ===== DEBUG =====
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(" Error: ");
  Serial.print(error);
  Serial.print(" Correction: ");
  Serial.println(correction);

  delay(20);*/
  updateXBeePosition();
  pingDistancecm();
  updatePixyDetections();
  updateStateMachine();
}

// Reset heading
void resetPIDHeading() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  setpoint = orientationData.orientation.x;
  
  error = 0;
  previousError = 0;
  integral = 0;
  derivative = 0;
  previousTime = millis();
  
  delay(50);
}

void PID_control() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  double yaw = orientationData.orientation.x;
  
  unsigned long currentTime = millis();
  double dt = (currentTime - previousTime) / 1000.0;
  
  if (dt > 0.1) dt = 0.1;
  if (dt <= 0) {
    previousTime = currentTime;
    return;
  }
  
  previousTime = currentTime;
  
  error = setpoint - yaw;
  
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  
  integral += error * dt;
  integral = constrain(integral, -50, 50);
  
  derivative = (error - previousError) / dt;
  derivative = constrain(derivative, -100, 100);
  
  double correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  correction = constrain(correction, -80, 80);
  
  previousError = error;
  
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;
  
  leftSpeed = constrain(leftSpeed, -300, 300);
  rightSpeed = constrain(rightSpeed, -300, 300);
  
  // Apply speeds directly without extra negatives
  motors.setM2Speed(-leftSpeed); // M2 is left motor
  motors.setM1Speed(rightSpeed); // M1 is right motor
}

// Improved turn control with speed scaling (slow when closer to target)
void performTurn(double targetYaw) {
  double currentYaw = getYaw();
  double remaining = angleDiff(targetYaw, currentYaw);
  
  // Target angle reached check
  if (abs(remaining) < 3.0) {
    stopMotors();
    turning = false;
    delay(80);
    resetPIDHeading();
    delay(30);
    setState(FORWARD);
    return;
  }
  
  // Timeout check
  if (millis() - turnStartTime > turnTimeout) {
    stopMotors();
    turning = false;
    delay(80);
    resetPIDHeading();
    delay(30);
    setState(FORWARD);
    return;
  }
  
  // Speed scaling
  int currentSpeed = TURN_BASE_SPEED;
  if (abs(remaining) < 10) {
    currentSpeed = TURN_BASE_SPEED * 0.6;  // Only slow down in last 10 degrees
  }
  
  currentSpeed = max(currentSpeed, 50);
  
  // Apply motor speeds based on turn direction
  if (remaining > 0) {  // Turn right
    motors.setM1Speed(-currentSpeed);
    motors.setM2Speed(-currentSpeed);
  } else {  // Turn left
    motors.setM1Speed(currentSpeed);
    motors.setM2Speed(currentSpeed);
  }
}

// ===== XBEE FUNCTIONS =====
// boolean to check if inside given coordinates
bool inBox(int x, int y, int xmin, int xmax, int ymin, int ymax) {
  return (x >= xmin && x <= xmax && y >= ymin && y <= ymax);
}

// boolean to check whether within start zone
bool atStartZone() {
  return xbeeHasValidPosition &&
         inBox(xPos, yPos, START_X_MIN, START_X_MAX, START_Y_MIN, START_Y_MAX);
}

// boolean to check whether within end zone
bool atEndZone() {
  return xbeeHasValidPosition &&
         inBox(xPos, yPos, END_X_MIN, END_X_MAX, END_Y_MIN, END_Y_MAX);
}

int extractDigits(const char* buf, int len, int &pos, int numDigits) {
  int value = 0;
  for (int i = 0; i < numDigits; i++) {
    if (pos >= len) return -1;
    char c = buf[pos++];
    if (c < '0' || c > '9') return -1;
    value = value * 10 + (c - '0');
  }
  return value;
}

bool parseBroadcast(const char* buf) {
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
      xPos = rx;
      yPos = ry;
      foundSelf = true;
    }
  }

  return foundSelf;
}

void processXBeeMessage() {
  if (rxIndex == 0) return;

  rxBuffer[rxIndex] = '\0';

  if (parseBroadcast(rxBuffer)) {
    xbeeHasValidPosition = true;
    Serial.print("XBee position: ");
    Serial.print(xPos);
    Serial.print(", ");
    Serial.println(yPos);
  }

  rxIndex = 0;
}

void updateXBeePosition() {
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

// ===== STATE MACHINE =====
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

void updatePixyDetections() {
  pixy.ccc.getBlocks();
  seesRed = seesBlue = seesGreen = false;
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == SIG_RED) seesRed = true;
    if (pixy.ccc.blocks[i].m_signature == SIG_BLUE) seesBlue = true;
    if (pixy.ccc.blocks[i].m_signature == SIG_GREEN) seesGreen = true;
  }
}

void pingDistancecm() {
  // standard trigger/echo for the ultrasonic
  digitalWrite(trig_signal, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_signal, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_signal, LOW);
  pulseDuration = pulseIn(echo_signal, HIGH, 30000);
  if (pulseDuration > 0) distancecm = pulseDuration / 58.0;
}