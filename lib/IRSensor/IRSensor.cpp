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

int IRSensor::readAngle() {
    updateFilteredDuties();
    if (numSensors <= 0) {
        return 0;
    }
    // 全センサーの EMA 済み LOW 比率
    float sensorValues[16];
    float maxValue = 0;
    int maxIndex = 0;

    for (int i = 0; i < numSensors; i++) {
        sensorValues[i] = filteredValues[i];
        if (sensorValues[i] > maxValue) {
            maxValue = sensorValues[i];
            maxIndex = i;
        }
    }

    float sumX = 0;
    float sumY = 0;

    for (int offset = -1; offset <= 1; offset++) {
        int index = (maxIndex + offset + numSensors) % numSensors;
        float a = index * (360.0f / static_cast<float>(numSensors)) * PI / 180.0f;
        sumX += cosf(a) * sensorValues[index];
        sumY += sinf(a) * sensorValues[index];
    }

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
    float maxValue = 0;
    int maxIndex = 0;

    for (int i = 0; i < numSensors; i++) {
        if (filteredValues[i] > maxValue) {
            maxValue = filteredValues[i];
            maxIndex = i;
        }
    }

    float sumX = 0;
    float sumY = 0;

    for (int offset = -1; offset <= 1; offset++) {
        int index = (maxIndex + offset + numSensors) % numSensors;
        float a = index * (360.0f / static_cast<float>(numSensors)) * PI / 180.0f;
        sumX += cosf(a) * filteredValues[index];
        sumY += sinf(a) * filteredValues[index];
    }
    // 0..1 前後の相対スカラー。旧 pulseIn(µs) ベースの閾値とは量が違うので main 側の閾値は要再調整
    float distance = sqrtf(sumX * sumX + sumY * sumY);
    return static_cast<int>(distance * 1000.0f);
}
