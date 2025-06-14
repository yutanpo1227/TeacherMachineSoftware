#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "Motor.h"
#include "GyroSensor.h"

class MotorController {
    public:
        MotorController(Motor* motor1, Motor* motor2, Motor* motor3, Motor* motor4);
        void moveDirection(int direction, int speed);
        void moveDirection(int direction, int speed, int gyroAngle);
        void moveDirection(int direction, int speed, int gyroAngle, int lineAngle, float lineVectorMagnitude);
        void turnRight(int speed);
        void turnLeft(int speed);
        void stop();
    private:
        Motor* motor1;
        Motor* motor2;
        Motor* motor3;
        Motor* motor4;
};

#endif
