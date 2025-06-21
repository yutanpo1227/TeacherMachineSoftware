#include "Debugger.h"

Debugger::Debugger(bool isEnabled) {
    enabled = isEnabled;
}

void Debugger::begin(int baudRate) {
    if (enabled) Serial.begin(baudRate);
}

void Debugger::printSensorStatus(bool lineSensor, bool gyroSensor) {
    if (!enabled) {
        return;
    }
    Serial.print("lineSensor: ");
    Serial.print(lineSensor);
    Serial.print(" gyroSensor: ");
    Serial.println(gyroSensor);
}

void Debugger::println(String message) {
    if (!enabled) return;
    
    Serial.print("[DEBUG] ");
    Serial.println(message);
}

void Debugger::printValues(int gyroAngle, int lineAngle, float lineVectorMagnitude, int ballAngle, int ballDist) {
    if (!enabled) {
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