#include "PID_motors.h"

// ================= CONSTRUCTOR =================
Motor::Motor() : bno(55, 0x28, &Wire) {
    // PID gains
    Kp = 4.5;
    Ki = 0.08;
    Kd = 1.5;

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
    turnGoalYaw = 0;
    turning = false;
    turnStartTime = 0;
}

// ================= INITIALIZATION =================
void Motor::initMotors() {
    motors.enableDrivers();
    delay(1000);

    // Capture initial heading
    setpoint = getYaw();
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
    double yaw = getYaw();

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
    motors.setM1Speed(rightSpeed); // right motor
}

void Motor::update() {
    PID_control();
}

// ================= RESET HEADING =================
void Motor::resetPIDHeading() {
    setpoint = getYaw();

    error = 0;
    previousError = 0;
    integral = 0;
    derivative = 0;

    previousTime = millis();

    delay(50);
}

// ================= TURN CONTROL =================
void Motor::setTurn(double targetYaw) {
    double currentYaw = getYaw();
    double remaining = angleDiff(targetYaw, currentYaw);

    const int TURN_BASE_SPEED = 105;
    const unsigned long turnTimeout = 3000;

    // Check if finished
    if (abs(remaining) < 3.0) {
        stopMotors();
        turning = false;
        delay(80);
        resetPIDHeading();
        delay(30);
        return;
    }

    // Timeout safety
    if (millis() - turnStartTime > turnTimeout) {
        stopMotors();
        turning = false;
        delay(80);
        resetPIDHeading();
        delay(30);
        return;
    }

    // Speed scaling
    int currentSpeed = TURN_BASE_SPEED;
    if (abs(remaining) < 10) {
        currentSpeed = TURN_BASE_SPEED * 0.6;
    }

    currentSpeed = max(currentSpeed, 50);


    if (remaining > 0) {  // turn right
        motors.setM1Speed(-currentSpeed);
        motors.setM2Speed(currentSpeed);
    } else {              // turn left
        motors.setM1Speed(currentSpeed);
        motors.setM2Speed(-currentSpeed);
    }
}

// ================= STOP =================
void Motor::stopMotors() {
    motors.setM1Speed(0);
    motors.setM2Speed(0);
}
