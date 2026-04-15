// FUNCTIONS
void init() {
    // Capture starting yaw as heading reference
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  setpoint = orientationData.orientation.x;
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