#include "solenoid.h"

Solenoid::Solenoid(int solPin, int pingSensorPin) {
    solenoidPin = solPin;
    pingPin = pingSensorPin;

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

unsigned long Solenoid::readEchoTime() {
    triggerPing();

    unsigned long timeout = micros();

    // wait for echo to go HIGH
    while (digitalRead(pingPin) == LOW) {
        if (micros() - timeout > 30000) {
            return 0;  // timeout
        }
    }

    unsigned long tStart = micros();

    // wait for echo to go LOW
    while (digitalRead(pingPin) == HIGH) {
        if (micros() - tStart > 30000) {
            return 0;  // timeout
        }
    }

    unsigned long tEnd = micros();

    return (tEnd - tStart);
}

float Solenoid::getDistanceCm() {
    unsigned long duration = readEchoTime();

    if (duration == 0) {
        return -1.0;   // no reading / too far / timeout
    }

    // speed of sound ~0.0343 cm/us
    float distance = (duration * 0.0343f) / 2.0f;
    return distance;
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