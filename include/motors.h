#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
    int pin;

public:
    Motor(int motorPin);   // constructor

    void init();
    void forward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void stop();
};

#endif
