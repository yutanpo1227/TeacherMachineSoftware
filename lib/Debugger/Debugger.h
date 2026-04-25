#ifndef DEBUGGER_H
#define DEBUGGER_H

#include <Arduino.h>

/**
 * デバッグ用クラス
 * シリアル通信を使ってデバッグ情報を出力する
 */
class Debugger {
    public:
        Debugger(bool isEnabled);
        void begin(int baudRate);
        void printSensorStatus(bool lineSensor, bool gyroSensor);
        void println(String message);
        void printValues(int gyroAngle, int lineAngle, float lineVectorMagnitude, int ballAngle, int ballDist);
        /** EMA 後の IR 各 ch の値をカンマ区切りで1行出力 */
        void printIrFilteredDuties(const float* values, int count);
    private:
        bool enabled;
};

#endif
