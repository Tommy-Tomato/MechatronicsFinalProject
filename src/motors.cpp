#include "motors.h"
#include <Arduino.h>
#include <DualMAX14870MotorShield.h>

const int Motor1EPinA = 2;
const int Motor1EPinB = 3;
const int Motor2EPinA = 18;
const int Motor2EPinB = 19;

int TURN_SPEED = 150;
int STRAIGHT_SPEED = 90;
int CORRECT_SPEED = 85;

Motor::Motor(int motorPin) {
    pin = motorPin;
}

void Motor::init() {
    pinMode(pin, OUTPUT);
}

void Motor::forward(int speed) {
    analogWrite(pin, speed);
}

void Motor::stop() {
    analogWrite(pin, 0);
}
