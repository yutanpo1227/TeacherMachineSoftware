#include "Debugger.h"

Debugger::Debugger(bool isEnabled) {
    enabled = isEnabled;
}

void Debugger::begin(unsigned long baudRate) {
    if (enabled) Serial.begin(baudRate);
}

void Debugger::printSensorStatus(bool lineSensor, bool gyroSensor) {
    if (!enabled) return;
    
    Serial.println("=== センサーステータス ===");
    Serial.print("ラインセンサー: ");
    Serial.println(lineSensor ? "有効" : "無効");
    Serial.print("ジャイロセンサー: ");
    Serial.println(gyroSensor ? "有効" : "無効");
    Serial.println("======================");
}

void Debugger::println(String message) {
    if (!enabled) return;
    
    Serial.print("[DEBUG] ");
    Serial.println(message);
}

void Debugger::printValues(int gyroAngle, int lineAngle, float lineVectorMagnitude, int ballAngle, int ballDist, int cameraBallAngle) {
    if (!enabled) return;
    // 旧形式の表示。各値は採取順に表示し、時刻付きの一括フレームではない。
    
    Serial.println("=== センサー値 ===");
    Serial.print("ジャイロ角度: ");
    Serial.println(gyroAngle);
    Serial.print("ライン角度: ");
    Serial.println(lineAngle);
    Serial.print("ラインベクトル大きさ: ");
    Serial.println(lineVectorMagnitude);
    Serial.print("ボール角度: ");
    Serial.println(ballAngle);
    Serial.print("ボール距離: ");
    Serial.println(ballDist);
    Serial.print("カメラボール角度: ");
    Serial.println(cameraBallAngle);
    Serial.println("================");
}
