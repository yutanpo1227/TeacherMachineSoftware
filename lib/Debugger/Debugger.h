#ifndef DEBUGGER_H
#define DEBUGGER_H

#include <Arduino.h>

/**
 * デバッグ用クラス
 * USBシリアルへ人が読めるセンサー値を出力する。
 * 構造化テレメトリの送信はmain.cpp側で行う。
 */
class Debugger {
    public:
        Debugger(bool isEnabled);
        // 有効時のみSerialを指定速度で開始する。
        void begin(unsigned long baudRate);
        void printSensorStatus(bool lineSensor, bool gyroSensor);
        void println(String message);
        // 角度は度、未検出は-1。ballDistは現在のIR実装では-1。
        void printValues(int gyroAngle, int lineAngle, float lineVectorMagnitude, int ballAngle, int ballDist, int cameraBallAngle);
    private:
        bool enabled;
};

#endif
