#include "MotorController.h"
#include "MotorControllerConfig.h"
#include "Motor.h"
#include "GoalHeading.h"
#include <Arduino.h>

MotorController::MotorController(Motor* motor1, Motor* motor2, Motor* motor3, Motor* motor4) {
    this->motor1 = motor1;
    this->motor2 = motor2;
    this->motor3 = motor3;
    this->motor4 = motor4;
}

void MotorController::moveDirection(int direction, int speed) {
    this->moveDirection(direction, speed, 0);
}

void MotorController::moveDirection(int direction, int speed, int gyroAngle) {
    this->moveDirection(direction, speed, -1,
                        GoalHeading::angleDifference(gyroAngle), -1, 0);
}

void MotorController::moveDirection(float direction, int speed, int irAngle, float headingError, int lineAngle, float lineVectorMagnitude) {
    headingError = GoalHeading::angleDifference(headingError);
    if (lineAngle != -1) {
        // 白線を踏んだら、いったん追跡の出力を落とす。
        stop(0);
        if (irAngle >= 0 && abs(GoalHeading::angleDifference(irAngle - lineAngle)) <= LINE_STOP_ANGLE_THRESHOLD) {
            // ボールが白線の方向にあれば、止まったまま追い越さない。
            return;
        }
        if (lineVectorMagnitude < LINE_VECTOR_MAGNITUDE_THRESHOLD) {
            // 複数のセンサーで方向が分散している場合は、検出方向の反対へ退避する。
            direction = lineAngle - 180;
        } else {
            // 元の進行方向に、白線から離れる方向を強めに合成する。
            const float x = cos(direction * PI / 180) - cos(lineAngle * PI / 180) * 1.5;
            const float y = sin(direction * PI / 180) - sin(lineAngle * PI / 180) * 1.5;
            if (x == 0 && y == 0) return;
            direction = atan2(y, x) * 180 / PI;
        }
    }
    const int correctionPower = headingPower(headingError, MIN_CORRECTION_POWER,
                                            MAX_CORRECTION_POWER, HEADING_KP);
    // 並進に同符号のPWMを加えると、その場旋回の成分として4輪に働く。

    // 大きく向きが違う間は旋回のみ。白線検出中は退避移動を優先する。
    if (lineAngle == -1 && abs(headingError) >= 90.0f) {
        if (correctionPower > 0) turnLeft(correctionPower);
        else turnRight(-correctionPower);
        return;
    }
    // 並進方向を4輪それぞれの駆動方向に投影する。斜め移動では一部の車輪が止まる。
    const float speedParams[] = {
        (float)sin((direction - 45) * PI / 180),
        (float)sin((direction - 135) * PI / 180),
        (float)sin((direction + 135) * PI / 180),
        (float)sin((direction + 45) * PI / 180)
    };
    float maxSpeedParam = 0;

    for (float wheel : speedParams) {
        if (abs(wheel) > maxSpeedParam) maxSpeedParam = abs(wheel);
    }

    // 最大速度パラメータを1にするための倍率
    const float ratio = 1 / maxSpeedParam;

    // 基底速度 * 方向パラメータ * 最大速度パラメータ + 姿勢修正パワー
    // 旋回用の余裕を残し、PWM上限を超えないようにする。
    const int translationSpeed = constrain(speed, 0, 255 - abs(correctionPower));
    motor1->setSpeed(translationSpeed * speedParams[0] * ratio + correctionPower);
    motor2->setSpeed(translationSpeed * speedParams[1] * ratio + correctionPower);
    motor3->setSpeed(translationSpeed * speedParams[2] * ratio + correctionPower);
    motor4->setSpeed(translationSpeed * speedParams[3] * ratio + correctionPower);
    // 待機せず次のセンサー取得へ進み、UART受信の滞留を防ぐ。
}

int MotorController::headingPower(float headingError, int minPower, int maxPower, float gain) {
    headingError = GoalHeading::angleDifference(headingError);
    // 正面付近で細かく旋回し続けないよう、不感帯内では補正しない。
    if (abs(headingError) <= HEADING_DEADBAND_DEG) return 0;
    maxPower = constrain(maxPower, 0, 255);
    minPower = constrain(minPower, 0, maxPower);
    const int power = constrain((int)(abs(headingError) * gain), minPower, maxPower);
    return headingError > 0 ? power : -power;
}

int MotorController::holdHeading(float headingError, int minPower, int maxPower, float gain) {
    const int power = headingPower(headingError, minPower, maxPower, gain);
    motor1->setSpeed(power);
    motor2->setSpeed(power);
    motor3->setSpeed(power);
    motor4->setSpeed(power);
    return power;
}

void MotorController::turnRight(int speed) {
    motor1->setSpeed(-speed);
    motor2->setSpeed(-speed);
    motor3->setSpeed(-speed);
    motor4->setSpeed(-speed);
}

void MotorController::turnLeft(int speed) {
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    motor3->setSpeed(speed);
    motor4->setSpeed(speed);
}

void MotorController::stop(int delayTime) {
    motor1->setSpeed(0);
    motor2->setSpeed(0);
    motor3->setSpeed(0);
    motor4->setSpeed(0);
    delay(delayTime);
}
