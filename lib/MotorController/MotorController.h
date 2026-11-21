#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "Motor.h"

// 機体の移動方向と旋回を4輪へ配分し、白線の停止・退避も適用する。
class MotorController {
    public:
        MotorController(Motor* motor1, Motor* motor2, Motor* motor3, Motor* motor4);
        // directionは機体から見た角度（右0・前90・左180・後ろ270度）、speedはPWM。
        // この2引数版はジャイロ0度に対する補正なしで移動する。
        void moveDirection(int direction, int speed);
        // gyroAngleを0度基準の姿勢誤差として扱う簡易版。
        void moveDirection(int direction, int speed, int gyroAngle);
        // irAngleは採用したボール角、lineAngleは白線角。未検出は-1。
        // headingErrorは「現在方位 - 目標方位」の最短角度差。
        // lineVectorMagnitudeは白線の方向のまとまりで、退避方法の選択に使う。
        void moveDirection(float direction, int speed, int irAngle, float headingError, int lineAngle, float lineVectorMagnitude);
        // 移動せず姿勢だけを補正。実際に指示した符号付きPWMを返す。
        int holdHeading(float headingError, int minPower, int maxPower, float gain);
        void turnRight(int speed);
        void turnLeft(int speed);
        // 全輪のPWMを0にする。delayTimeミリ秒だけ制御ループを待たせる。
        void stop(int delayTime = 100);
    private:
        static int headingPower(float headingError, int minPower, int maxPower, float gain);
        Motor* motor1;
        Motor* motor2;
        Motor* motor3;
        Motor* motor4;
};

#endif
