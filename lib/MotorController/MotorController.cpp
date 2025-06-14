#include "MotorController.h"
#include "Motor.h"

#define MAX_CORRECTION_POWER 50

#define LINE_STOP_ANGLE_THRESHOLD 10
#define LINE_VECTOR_MAGNITUDE_THRESHOLD 0.75

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
    this->moveDirection(direction, speed, gyroAngle, -1, 0);
}

void MotorController::moveDirection(int direction, int speed, int gyroAngle, int lineAngle, float lineVectorMagnitude) {
    if (lineAngle != -1) {
        if (abs(direction - lineAngle) < LINE_STOP_ANGLE_THRESHOLD) {
            this->stop();
            return;
        } else if (lineVectorMagnitude < LINE_VECTOR_MAGNITUDE_THRESHOLD) {
            this->stop();
            direction = lineAngle - 180;
        } else {
            this->stop();
            float component_x = cos(direction * PI / 180) - cos(lineAngle * PI / 180) * 1.5;
            float component_y = sin(direction * PI / 180) - sin(lineAngle * PI / 180) * 1.5;
            if (component_x == 0 && component_y == 0) {
                this->stop();
                return;
            }
            direction = atan2(component_y, component_x) * 180 / PI;
        }
    }
    // 姿勢が左右90度以上ずれている場合はその場で旋回
    if (gyroAngle > 90 && gyroAngle < 180) {
        this->turnLeft(gyroAngle);
        return;
    } else if (gyroAngle < 270 && gyroAngle >= 180) {
        this->turnRight(360 - gyroAngle);
        return;
    }

    // 姿勢が左右90度以内ずれている場合は姿勢修正パワーを計算
    float correctionPowerRatio = 0;
    if (gyroAngle > 0 && gyroAngle < 90) {
        correctionPowerRatio = pow(gyroAngle / 90.0, 2);
    } else if (gyroAngle > 270 && gyroAngle < 360) {
        correctionPowerRatio = -pow((gyroAngle - 360) / 90.0, 2);
    }

    int correctionPower = MAX_CORRECTION_POWER * correctionPowerRatio;

    // モーターの速度パラメータを計算
    float m1SpeedParam = sin((direction-45) * PI / 180);
    float m2SpeedParam = sin((direction-135) * PI / 180);
    float m3SpeedParam = sin((direction+135) * PI / 180);
    float m4SpeedParam = sin((direction+45) * PI / 180);

    float speedParams[4] = {m1SpeedParam, m2SpeedParam, m3SpeedParam, m4SpeedParam};
    float maxSpeedParam = 0;

    for (int i = 0; i < 4; i++) {
        if (abs(speedParams[i]) > maxSpeedParam) {
            maxSpeedParam = abs(speedParams[i]);
        }
    }

    // 最大速度パラメータを1にするための倍率
    float ratio = 1 / maxSpeedParam;

    // 基底速度 * 方向パラメータ * 最大速度パラメータ + 姿勢修正パワー
    motor1->setSpeed(speed * speedParams[0] * ratio + correctionPower);
    motor2->setSpeed(speed * speedParams[1] * ratio + correctionPower);
    motor3->setSpeed(speed * speedParams[2] * ratio + correctionPower);
    motor4->setSpeed(speed * speedParams[3] * ratio + correctionPower);
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

void MotorController::stop() {
    motor1->setSpeed(0);
    motor2->setSpeed(0);
    motor3->setSpeed(0);
    motor4->setSpeed(0);
}