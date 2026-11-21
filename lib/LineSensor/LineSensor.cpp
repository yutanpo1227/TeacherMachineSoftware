#include "LineSensor.h"
#include <Arduino.h>

LineSensor::LineSensor(int startPin, int numSensors, const int* thresholds) {
    this->startPin = startPin;
    this->numSensors = numSensors;
    this->thresholds = thresholds;
    this->mode = true;
    this->previousAngle = 0;
    this->hasPreviousAngle = false;
    this->filteredAngle = 0;
    for (int i = 0; i < numSensors; i++) {
        pinMode(startPin + i, INPUT);
    }
}

void LineSensor::printValues() {
    for (int i = 0; i < numSensors; i++) {
        Serial.print(analogRead(startPin + i));
        Serial.print(" ");
    }
    Serial.println();
}

int LineSensor::readAngle() {
    // 車体の周囲で白線に乗っている側を求める。停止・退避の判断は走行制御側で行う。
    if (!mode) {
        return -1;
    }
    if (!this->checkOnLine()) {
        hasPreviousAngle = false;
        filteredAngle = 0;
        return -1;
    }
    float sumX = 0;
    float sumY = 0;
    for (int i = 0; i < numSensors; i++) {
        int readValue = analogRead(startPin + i);
        if (readValue > thresholds[i]) {
            float angle = i * (360.0 / numSensors) * PI / 180.0;
            sumX += cos(angle);
            sumY += sin(angle);
        }
    }
    
    float rawAngle = atan2(sumY, sumX) * 180 / PI;
    if (rawAngle < 0) {
        rawAngle += 360;  // 0-360度に正規化
    }
    
    // 白線位置が小刻みに揺れても急に進路を変えないよう前回値を使う。
    if (!hasPreviousAngle) {
        // 初回は生の値をそのまま使用
        filteredAngle = rawAngle;
        hasPreviousAngle = true;
    } else {
        // 359度と1度を遠く離れた方向と誤認しないよう、差を±180度に収める。
        float angleDiff = rawAngle - previousAngle;
        if (angleDiff > 180) {
            angleDiff -= 360;
        } else if (angleDiff < -180) {
            angleDiff += 360;
        }
        
        // 急激な変化の場合は前回値を重視
        if (abs(angleDiff) > MAX_ANGLE_JUMP) {
            // 大きな変化は無視し、前回の値を継続
            filteredAngle = previousAngle;
        } else {
            // 円周上の短い差を使う。315度から0度へ変わっても180度側へ飛ばない。
            filteredAngle = previousAngle + ANGLE_FILTER_ALPHA * angleDiff;
            if (filteredAngle < 0) {
                filteredAngle += 360;
            } else if (filteredAngle >= 360) {
                filteredAngle -= 360;
            }
        }
    }
    
    previousAngle = filteredAngle;
    return (int)filteredAngle;
}

void LineSensor::resetFilter() {
    hasPreviousAngle = false;
    filteredAngle = 0;
}

bool LineSensor::checkOnLine() {
    if (!mode) {
        return false;
    }
    for (int i = 0; i < numSensors; i++) {
        if (analogRead(startPin + i) > thresholds[i]) {
            return true;
        }
    }
    return false;
}

void LineSensor::setMode(bool mode) {
    this->mode = mode;
}

float LineSensor::readVectorMagnitude() {
    if (!mode) {
        return 0;
    }
    if (!this->checkOnLine()) {
        return 0;  // ライン検出なしの場合は強度0
    }
    float sumX = 0;
    float sumY = 0;
    float onLineCount = 0;
    for (int i = 0; i < numSensors; i++) {
        int readValue = analogRead(startPin + i);
        if (readValue > thresholds[i]) {
            float angle = i * (360.0 / numSensors) * PI / 180.0;
            sumX += cos(angle);
            sumY += sin(angle);
            onLineCount++;
        }
    }
    // 同じ側に反応が集まると1に近く、反対側にも広がると互いに打ち消され0に近づく。
    float magnitude = sqrt(sumX * sumX + sumY * sumY) / onLineCount;
    return magnitude;
}
