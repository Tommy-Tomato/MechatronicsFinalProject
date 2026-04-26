#include "Solenoid.h"

Solenoid::Solenoid() {
    solenoidPin = 48;
    firePulseMs = 400;         // how long solenoid stays on
    fireCooldownMs = 1000;      // delay before it can fire again

    firing = false;
    readyToFire = true;
    fireStartTime = 0;
    lastFireTime = 0;
}

void Solenoid::init() {
    pinMode(solenoidPin, OUTPUT);
    digitalWrite(solenoidPin, LOW);
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


void Solenoid::fireOnce() {
    unsigned long now = millis();

    // only fire if not already firing and cooldown is done
    if (readyToFire && !firing) {
        Serial.println("firing!");
        digitalWrite(solenoidPin, HIGH);
        firing = true;
        readyToFire = false;
        fireStartTime = now;
    } else {
        Serial.println("blocked");
    }
}

void Solenoid::resetFire() {
    firing = false;
    readyToFire = true;
    digitalWrite(solenoidPin, LOW);
}