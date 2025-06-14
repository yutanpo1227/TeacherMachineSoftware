#include "Debuger.h"

Debuger::Debuger(bool isEnabled) {
    this->isEnabled = isEnabled;
}

void Debuger::begin(int baudRate) {
    Serial.begin(baudRate);
}

void Debuger::printSensorStatus(bool lineSensor, bool gyroSensor) {
    if (!isEnabled) {
        return;
    }
    Serial.print("lineSensor: ");
    Serial.print(lineSensor);
    Serial.print(" gyroSensor: ");
    Serial.println(gyroSensor);
}

void Debuger::println(String message) {
    if (!isEnabled) {
        return;
    }
    Serial.println(message);
}

void Debuger::printValues(int gyroAngle, int lineAngle, float lineVectorMagnitude, int ballAngle, int ballDist) {
    if (!isEnabled) {
        return;
    }
    Serial.print("gyroAngle: ");
    Serial.print(gyroAngle);
    Serial.print(" lineAngle: ");
    Serial.print(lineAngle);
    Serial.print(" lineVectorMagnitude: ");
    Serial.print(lineVectorMagnitude);
    Serial.print(" ballAngle: ");
    Serial.print(ballAngle);
    Serial.print(" ballDist: ");
    Serial.println(ballDist);
}