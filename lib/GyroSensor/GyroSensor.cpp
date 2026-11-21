#include "GyroSensor.h"
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <Wire.h>


GyroSensor::GyroSensor() {
    bno = Adafruit_BNO055(55, 0x28);
}

void GyroSensor::setup() {
    // 接続を確立できない状態で姿勢制御を始めないよう、失敗時はここで止まる。
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    Serial.println("GyroSensor initialized");
    bno.setExtCrystalUse(true);
}

int GyroSensor::readYawAngle() {
    // BNO055の方位をそのまま返す。電源投入時の機体の向きを0度にする処理はない。
    if (!mode) {
        return 0;
    }
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    return euler.x();
}

void GyroSensor::setMode(bool mode) {
    this->mode = mode;
}
