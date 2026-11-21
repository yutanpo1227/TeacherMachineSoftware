#include "Motor.h"
#include <Arduino.h>

Motor::Motor(int phPin, int enPin) {
    this->phPin = phPin;
    this->enPin = enPin;
    pinMode(phPin, OUTPUT);
    pinMode(enPin, OUTPUT);
}

void Motor::setSpeed(int speed) {
    // 符号は車輪の回転方向、絶対値はPWMの強さ。車輪の実際の回転数は計測しない。
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
