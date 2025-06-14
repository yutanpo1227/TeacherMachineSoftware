#include "IRSensor.h"
#include <Arduino.h>

IRSensor::IRSensor(int startPin, int numSensors) {
    this->startPin = startPin;
    this->numSensors = numSensors;
    for (int i = 0; i < numSensors; i++) {
        pinMode(startPin + i, INPUT);
    }
}

int IRSensor::readAngle() {
    // 全センサーの値を読み取り、フィルタリング
    float sensorValues[16];
    float maxValue = 0;
    int maxIndex = 0;
    
    for (int i = 0; i < numSensors; i++) {
        int rawValue = pulseIn(startPin + i, LOW, 1666);
        
        // RCローパスフィルタ実装（指数移動平均）
        if (firstRead[i]) {
            filteredValues[i] = rawValue;  // 初回はそのまま設定
            firstRead[i] = false;
        } else {
            filteredValues[i] = FILTER_ALPHA * rawValue + (1.0f - FILTER_ALPHA) * filteredValues[i];
        }
        
        sensorValues[i] = filteredValues[i];
        
        // 最大値とそのインデックスを記録
        if (sensorValues[i] > maxValue) {
            maxValue = sensorValues[i];
            maxIndex = i;
        }
    }
    
    // 最大値のセンサーとその隣接2つのセンサーのみを使用
    float sumX = 0;
    float sumY = 0;
    
    for (int offset = -1; offset <= 1; offset++) {
        int index = (maxIndex + offset + numSensors) % numSensors;  // 環状配列として処理
        float angle = index * (360.0 / numSensors) * PI / 180.0;
        sumX += cos(angle) * sensorValues[index];
        sumY += sin(angle) * sensorValues[index];
    }
    
    // 足し合わせたx,y成分から角度を計算
    float angle = atan2(sumY, sumX);
    angle = angle * 180 / PI;
    if (angle < 0) {
        angle += 360;
    }
    return angle;
}

int IRSensor::readDistance() {
    // 全センサーの値を読み取り、フィルタリング
    float sensorValues[16];
    float maxValue = 0;
    int maxIndex = 0;
    
    for (int i = 0; i < numSensors; i++) {
        int rawValue = pulseIn(startPin + i, LOW, 1666);
        
        // RCローパスフィルタ実装（指数移動平均）
        if (firstRead[i]) {
            filteredValues[i] = rawValue;  // 初回はそのまま設定
            firstRead[i] = false;
        } else {
            filteredValues[i] = FILTER_ALPHA * rawValue + (1.0f - FILTER_ALPHA) * filteredValues[i];
        }
        
        sensorValues[i] = filteredValues[i];
        
        // 最大値とそのインデックスを記録
        if (sensorValues[i] > maxValue) {
            maxValue = sensorValues[i];
            maxIndex = i;
        }
    }
    
    // 最大値のセンサーとその隣接4つのセンサーのみを使用
    float sumX = 0;
    float sumY = 0;
    
    for (int offset = -1; offset <= 1; offset++) {
        int index = (maxIndex + offset + numSensors) % numSensors;  // 環状配列として処理
        float angle = index * (360.0 / numSensors) * PI / 180.0;
        sumX += cos(angle) * sensorValues[index];
        sumY += sin(angle) * sensorValues[index];
    }
    
    // 足し合わせたx,y成分からユークリッド距離を計算
    float distance = sqrt(sumX * sumX + sumY * sumY);
    return distance;
}
