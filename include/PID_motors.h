#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <DualMAX14870MotorShield.h>

class Motor {
private:
    DualMAX14870MotorShield motors;
    Adafruit_BNO055 bno;

    // PID variables
    double setpoint;
    double Kp, Ki, Kd;
    double error, previousError, integral, derivative;
    unsigned long previousTime;
    int baseSpeed;

    // Turning
    double turnGoalYaw;
    bool turning;
    unsigned long turnStartTime;

    // Helpers
    double getYaw();
    double angleDiff(double target, double current);

public:
    Motor();

    void initMotors();
    void resetPIDHeading();
    void PID_control();

    void performTurn(double targetYaw);
    void stopMotors();
};

#endif
