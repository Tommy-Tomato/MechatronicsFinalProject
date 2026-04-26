#include "PID_motors.h"

// ================= CONSTRUCTOR =================
Motor::Motor() : bno(55, 0x28, &Wire) {
    // PID gains
    Kp = 7;
    Ki = 0.3;
    Kd = 0.1;

    // PID state
    setpoint = 0;
    error = 0;
    previousError = 0;
    integral = 0;
    derivative = 0;

    previousTime = 0;

    // Motion
    baseSpeed = 120;

    // Turning
    turnStartYaw = 0;
    turning = false;
    turnStartTime = 0;
    accumulatedTurn = 0;
    
    
}

// ================= INITIALIZATION =================
void Motor::initMotors() {
    //initialize IMU
    if (!bno.begin()) {
        Serial.println("BNO055 not detected!");
        while (1);
    }
    bno.setExtCrystalUse(true);

    //initialize Motors
    motors.enableDrivers();
    delay(100);

    // Capture initial heading
    setpoint = getYaw();
    headingOffset = getYaw();
    previousTime = millis();


}

// ================= HELPER FUNCTIONS =================
double Motor::getYaw() {
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
    return event.orientation.x;
}

double Motor::angleDiff(double target, double current) {
    double diff = target - current;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    return diff;
}


// ================= PID CONTROL =================
void Motor::PID_control() {
    double yaw = getYaw() - headingOffset;

    unsigned long currentTime = millis();
    double dt = (currentTime - previousTime) / 1000.0;

    if (dt > 0.1) dt = 0.1;
    if (dt <= 0) {
        previousTime = currentTime;
        return;
    }

    previousTime = currentTime;

    error = angleDiff(setpoint, yaw);

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

    // Apply speeds
    motors.setM2Speed(-leftSpeed); // left motor
    motors.setM1Speed(-rightSpeed); // right motor
}

void Motor::update() {
    if (turning) {
        handleTurn();
    } else {
        PID_control();
    }
}

// ================= RESET HEADING =================
void Motor::resetPIDHeading() {
    setpoint = getYaw() - headingOffset;

    error = 0;
    previousError = 0;
    integral = 0;
    derivative = 0;

    previousTime = millis();

}

// ================= TURN CONTROL =================
void Motor::tankTurn() {

    turnStartYaw = getYaw() - headingOffset;
    turning = true;
    turnStartTime = millis();
    accumulatedTurn = 0;
}

void Motor::handleTurn() {
    double currentYaw = getYaw() - headingOffset;

    double delta = angleDiff(currentYaw, turnStartYaw);
    accumulatedTurn += abs(delta);

    turnStartYaw = currentYaw;

    Serial.println(accumulatedTurn);

    motors.setSpeeds(-100, 100);

    if (accumulatedTurn >= 340) {
        Serial.println("finished full scan");
        turning = false;
        resetPIDHeading();
        stopMotors();
        delay(50);
    }
}

// ================= STOP =================
void Motor::stopMotors() {
    motors.setM1Speed(0);
    motors.setM2Speed(0);
}

void Motor::setBaseSpeed(int speed) {
    baseSpeed = speed;
}

void Motor::adjustHeading(double deltaYaw) {
    setpoint += deltaYaw;

    if (setpoint >= 360.0) setpoint -= 360.0;
    if (setpoint < 0.0) setpoint += 360.0;
}

void Motor::setHeading(double targetYaw) {
    setpoint = targetYaw;
    if (setpoint < 0) setpoint += 360;
    if (setpoint >= 360) setpoint -= 360;
    //Serial.print("setpoint: ");
    //Serial.println(setpoint);
}