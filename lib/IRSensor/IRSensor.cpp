#include "IRSensor.h"
#include <Arduino.h>

IRSensor::IRSensor(int startPin, int numSensors) {
    this->startPin = startPin;
    this->numSensors = numSensors;
    singlePort_ = true;
    for (int i = 0; i < numSensors; i++) {
        const int pin = startPin + i;
        const uint8_t port = digitalPinToPort(pin);
        regIn_[i] = portInputRegister(port);
        bitIn_[i] = digitalPinToBitMask(pin);
        pinMode(pin, INPUT);
        firstRead[i] = true;
        if (i > 0 && regIn_[i] != regIn_[0]) {
            singlePort_ = false;
        }
    }
    if (numSensors > 0) {
        regInSingle_ = (singlePort_) ? regIn_[0] : nullptr;
    } else {
        regInSingle_ = nullptr;
    }
}

void IRSensor::sampleAllLowDuties(uint32_t* lowCount, uint32_t& totalCount) {
    for (int i = 0; i < numSensors; i++) {
        lowCount[i] = 0;
    }
    totalCount = 0;
    const uint32_t startUs = micros();
    while (static_cast<uint32_t>(micros() - startUs) < kSampleWindowUs) {
        if (regInSingle_ != nullptr) {
            const uint8_t snap = *regInSingle_;
            for (int i = 0; i < numSensors; i++) {
                if ((snap & bitIn_[i]) == 0) {
                    lowCount[i]++;
                }
            }
        } else {
            for (int i = 0; i < numSensors; i++) {
                if ((*(regIn_[i]) & bitIn_[i]) == 0) {
                    lowCount[i]++;
                }
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
    sampleAllLowDuties(lowCount, totalCount);
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

int IRSensor::copyFilteredDuties(float* out, int maxOut) const {
    if (out == nullptr || maxOut <= 0 || numSensors <= 0) {
        return 0;
    }
    const int n = (numSensors < maxOut) ? numSensors : maxOut;
    for (int i = 0; i < n; i++) {
        out[i] = filteredValues[i];
    }
    return n;
}

void IRSensor::weightedRingSum(float& sumX, float& sumY) const {
    sumX = 0.0f;
    sumY = 0.0f;
    const float n = static_cast<float>(numSensors);
    for (int i = 0; i < numSensors; i++) {
        const float a = (static_cast<float>(i) * 360.0f / n) * (PI / 180.0f);
        sumX += cosf(a) * filteredValues[i];
        sumY += sinf(a) * filteredValues[i];
    }
}

int IRSensor::readAngle() {
    updateFilteredDuties();
    if (numSensors <= 0) {
        return 0;
    }
    float sumX;
    float sumY;
    weightedRingSum(sumX, sumY);
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
    weightedRingSum(sumX, sumY);
    // 0..1 前後/本の全本ベクトル長。main 側の距離閾値は要再調整
    float distance = sqrtf(sumX * sumX + sumY * sumY);
    return static_cast<int>(distance * 1000.0f);
}
