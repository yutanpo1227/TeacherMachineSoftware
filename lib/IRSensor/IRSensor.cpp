#include "IRSensor.h"
#include <Arduino.h>
#include <math.h>

IRSensor::IRSensor(int startPin, int numSensors)
    : startPin(startPin),
      numSensors(numSensors) {
    for (int i = 0; i < 16; i++) {
        pulseWidths[i] = 0;
    }

    for (int i = 0; i < numSensors; i++) {
        // TSSP58038は検出時LOWのため、入力をHIGHへ戻す。
        pinMode(startPin + i, INPUT_PULLUP);
    }
}

void IRSensor::measurePulseWidths() {
    // 1周分の受光を取得する。検出できない素子ではそれぞれ最大3ms待つ。
    for (int i = 0; i < numSensors; i++) {
        // TSSP58038は検出時LOWなので、LOWパルスを測定する。
        // パルスがない場合はタイムアウトして0になる。
        pulseWidths[i] = pulseIn(
            startPin + i, LOW, PULSE_TIMEOUT_US);
    }
}

float IRSensor::normalizeAngle(float angle) {
    while (angle < 0.0f) {
        angle += 360.0f;
    }
    while (angle >= 360.0f) {
        angle -= 360.0f;
    }
    return angle;
}

int IRSensor::readAngle() {
    measurePulseWidths();

    // 反応した受光素子の向きをパルス幅で重み付けし、機体から見たボールの方向にする。
    float vectorX = 0.0f;
    float vectorY = 0.0f;
    float totalWeight = 0.0f;

    for (int i = 0; i < numSensors; i++) {
        if (pulseWidths[i] < MIN_PULSE_US) {
            continue;
        }

        const float angle =
            (float)i * (360.0f / (float)numSensors);
        const float angleRad = angle * PI / 180.0f;

        const float weight = (float)pulseWidths[i];
        vectorX += weight * cos(angleRad);
        vectorY += weight * sin(angleRad);
        totalWeight += weight;
    }

    if (totalWeight <= 0.0f) {
        return -1;
    }

    // ボールが見える方向の単位ベクトルを足し合わせ、その合成角を求める。
    // パルス幅は重みとして使うが、この値から実距離は算出していない。
    const float angle = normalizeAngle(
        atan2(vectorY, vectorX) * 180.0f / PI);

    // readAngle()の戻り値はintなので、小数点以下は切り捨てる。
    return (int)angle;
}

int IRSensor::readDistance() {
    // TSSP58038のIR距離は使用しない。
    return -1;
}
