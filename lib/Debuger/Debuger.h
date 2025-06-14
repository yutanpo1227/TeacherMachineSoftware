#ifndef DEBUGER_H
#define DEBUGER_H

#include <Arduino.h>

/**
 * デバッグ用のクラス
 * mainからのSerial呼び出しを許可しないことで、モニターが荒れるのを防ぐ
 */
class Debuger {
    public:
        Debuger(bool isEnabled);
        void begin(int baudRate);
        void printSensorStatus(bool lineSensor, bool gyroSensor);
        void println(String message);
        void printValues(int gyroAngle, int lineAngle, float lineVectorMagnitude, int ballAngle, int ballDist);
    private:
        bool isEnabled;
};

#endif
