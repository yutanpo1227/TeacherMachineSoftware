#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <Arduino.h>

// 車体の周囲に等間隔で配置したIR受光素子から、発光ボールの方向を求める。
class IRSensor {
    public:
        // startPinから連続するnumSensors本のデジタル入力を使う。
        IRSensor(int startPin, int numSensors);
        // 右0度・前90度・左180度・後ろ270度。反応がなければ-1。
        int readAngle();
        // 現在の受光方式では距離を推定せず、常に-1を返す。
        int readDistance();
    private:
        int startPin;
        int numSensors;

        // 各センサーで測定したLOWパルス幅（マイクロ秒）。
        unsigned long pulseWidths[16];

        // pulseInは各センサーを順番に待つ。タイムアウトも1素子ごとに適用される。
        static constexpr unsigned long PULSE_TIMEOUT_US = 3000;
        static constexpr unsigned long MIN_PULSE_US = 20;

        void measurePulseWidths();
        static float normalizeAngle(float angle);
};


#endif
