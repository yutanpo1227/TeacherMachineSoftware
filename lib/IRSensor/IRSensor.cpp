#include "IRSensor.h"
#include <Arduino.h>

IRSensor::IRSensor(int startPin, int numSensors) {
    this->startPin = startPin;
    this->numSensors = numSensors;
    for (int i = 0; i < numSensors; i++) {
        pinMode(startPin + i, INPUT);
        firstRead[i] = true;
    }
}

void IRSensor::sampleAllLowDuties(int startPin, int numSensors, uint32_t* lowCount,
                                 uint32_t& totalCount) {
    for (int i = 0; i < numSensors; i++) {
        lowCount[i] = 0;
    }
    totalCount = 0;
    const uint32_t startUs = micros();
    while (static_cast<uint32_t>(micros() - startUs) < kSampleWindowUs) {
        for (int i = 0; i < numSensors; i++) {
            if (digitalRead(startPin + i) == LOW) {
                lowCount[i]++;
            }
        }
        totalCount++;
        delayMicroseconds(kSampleIntervalUs);
    }
}

void IRSensor::updateFilteredDuties() {
    if (numSensors <= 0) {
        return;
    }
    if (numSensors > 16) {
        return;  // 配列上の想定外
    }
    uint32_t lowCount[16];
    uint32_t totalCount = 0;
    sampleAllLowDuties(startPin, numSensors, lowCount, totalCount);
    for (int i = 0; i < numSensors; i++) {
        const float dutyRaw = (totalCount == 0)
                                  ? 0.0f
                                  : (static_cast<float>(lowCount[i]) / static_cast<float>(totalCount));
        if (firstRead[i]) {
            filteredValues[i] = dutyRaw;
            firstRead[i] = false;
        } else {
            filteredValues[i] =
                kDutyEmaAlpha * dutyRaw + (1.0f - kDutyEmaAlpha) * filteredValues[i];
        }
    }
    lastSampleTimeUs_ = micros();
}

void IRSensor::weightedMaxNeighborhoodSum(float& sumX, float& sumY) const {
    float maxValue = 0.0f;
    int maxIndex = 0;
    for (int i = 0; i < numSensors; i++) {
        if (filteredValues[i] > maxValue) {
            maxValue = filteredValues[i];
            maxIndex = i;
        }
    }
    sumX = 0.0f;
    sumY = 0.0f;
    const float n = static_cast<float>(numSensors);
    for (int offset = -2; offset <= 2; offset++) {
        const int index = (maxIndex + offset + numSensors) % numSensors;
        const float a = (static_cast<float>(index) * 360.0f / n) * (PI / 180.0f);
        sumX += cosf(a) * filteredValues[index];
        sumY += sinf(a) * filteredValues[index];
    }
}

int IRSensor::readAngle() {
    updateFilteredDuties();
    if (numSensors <= 0) {
        return 0;
    }
    float sumX;
    float sumY;
    weightedMaxNeighborhoodSum(sumX, sumY);
    float angle = atan2f(sumY, sumX);
    angle = angle * 180.0f / PI;
    if (angle < 0) {
        angle += 360.0f;
    }
    return static_cast<int>(angle);
}

int IRSensor::readDistance() {
    const uint32_t now = micros();
    if (lastSampleTimeUs_ == 0U ||
        static_cast<uint32_t>(now - lastSampleTimeUs_) > kResampleMinIntervalUs) {
        updateFilteredDuties();
    }
    if (numSensors <= 0) {
        return 0;
    }
    float sumX;
    float sumY;
    weightedMaxNeighborhoodSum(sumX, sumY);
    // 相対ベクトル長（max 周囲5本分）。数が変わるとスケールも変わるので main 閾値は要再調整
    float distance = sqrtf(sumX * sumX + sumY * sumY);
    return static_cast<int>(distance * 1000.0f);
}
