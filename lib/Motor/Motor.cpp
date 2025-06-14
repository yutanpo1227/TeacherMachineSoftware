#include "Motor.h"
#include <Arduino.h>

Motor::Motor(int phPin, int enPin) {
    this->phPin = phPin;
    this->enPin = enPin;
    pinMode(phPin, OUTPUT);
    pinMode(enPin, OUTPUT);
}

void Motor::setSpeed(int speed) {
    if (speed == 0) {
        analogWrite(enPin, 0);
        return;
    }
    if (speed > 0) {
        digitalWrite(phPin, HIGH);
    } else {
        digitalWrite(phPin, LOW);
    }
    analogWrite(enPin, abs(speed));
}