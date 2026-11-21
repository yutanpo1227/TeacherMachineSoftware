#ifndef GYROSENSOR_H
#define GYROSENSOR_H

#include <Adafruit_BNO055.h>

// BNO055の方位を読み、機体の旋回・姿勢保持に渡す。
class GyroSensor {
    public:
        GyroSensor();
        // BNO055のヨー角を整数度で返す。無効時は0。起動時にゼロ合わせはしない。
        int readYawAngle();
        void setMode(bool mode);
        // I2C通信を開始する。BNO055を検出できなければエラーを出して待機する。
        void setup();
    private:
        Adafruit_BNO055 bno;
        bool mode;
};

#endif
