#ifndef SOLENOID_H
#define SOLENOID_H

#include <Arduino.h>

class Solenoid {
private:
    int solenoidPin;
    int pingPin;

    // Distance sensing
    float puckThresholdCm;

    // Firing control
    unsigned long firePulseMs;
    unsigned long fireCooldownMs;

    bool firing;
    bool readyToFire;
    unsigned long fireStartTime;
    unsigned long lastFireTime;


public:
    // Constructor
    Solenoid();

    // Setup
    void init();

    // Main loop update
    void update();

    // Firing control
    void fireOnce();
    void resetFire();
};

#endif
