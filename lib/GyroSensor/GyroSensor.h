#ifndef GYROSENSOR_H
#define GYROSENSOR_H

#include <Adafruit_BNO055.h>

class GyroSensor {
    public:
        GyroSensor();
        int readYawAngle();
        void setMode(bool mode);
        void setup();
    private:
        Adafruit_BNO055 bno;
        bool mode;
};

#endif
