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
    double turnStartYaw;
    unsigned long turnStartTime;
    double accumulatedTurn;

    // Helpers
    double angleDiff(double target, double current);
    void PID_control();
    void resetPIDHeading();
    void handleTurn();

public:
    Motor();
    bool turning;
    double headingOffset;

    double getYaw();

    void initMotors();
    void tankTurn();
    void stopMotors();
    void update();

    void adjustHeading (double deltaYaw);
    void setBaseSpeed (int speed);
    void setHeading(double targetYaw);
};

#endif
