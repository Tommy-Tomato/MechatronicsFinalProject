#include "Solenoid.h"

Solenoid::Solenoid(int solPin) {
    solenoidPin = solPin;
    pingPin = 3;

    puckThresholdCm = 5.0;     // puck is considered "captured" within 5 cm
    firePulseMs = 120;         // how long solenoid stays on
    fireCooldownMs = 500;      // delay before it can fire again

    firing = false;
    readyToFire = true;
    fireStartTime = 0;
    lastFireTime = 0;
}

void Solenoid::init() {
    pinMode(solenoidPin, OUTPUT);
    digitalWrite(solenoidPin, LOW);

    pinMode(pingPin, INPUT);
}

void Solenoid::update() {
    unsigned long now = millis();

    // turn solenoid off after pulse time
    if (firing && (now - fireStartTime >= firePulseMs)) {
        digitalWrite(solenoidPin, LOW);
        firing = false;
        lastFireTime = now;
    }

    // re-arm after cooldown
    if (!readyToFire && !firing && (now - lastFireTime >= fireCooldownMs)) {
        readyToFire = true;
    }
}

void Solenoid::triggerPing() {
    pinMode(pingPin, OUTPUT);

    digitalWrite(pingPin, LOW);
    delayMicroseconds(10);

    digitalWrite(pingPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(pingPin, LOW);

    pinMode(pingPin, INPUT);
}

float Solenoid::getDistanceCm() {
    triggerPing();

    unsigned long duration = pulseIn(pingPin, HIGH, 30000); //30 ms timeout if not detected then hasPuck returns falso

    if (duration == 0) {
        return -1.0;
    }

    return (duration * 0.034) / 2.0;
}

bool Solenoid::hasPuck() {
    float distance = getDistanceCm();

    if (distance < 0) {
        return false;
    }

    return (distance <= puckThresholdCm);
}

void Solenoid::fireOnce() {
    unsigned long now = millis();

    // only fire if not already firing and cooldown is done
    if (readyToFire && !firing) {
        digitalWrite(solenoidPin, HIGH);
        firing = true;
        readyToFire = false;
        fireStartTime = now;
    }
}

void Solenoid::resetFire() {
    firing = false;
    readyToFire = true;
    digitalWrite(solenoidPin, LOW);
}