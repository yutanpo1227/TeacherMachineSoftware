#include "GyroSensor.h"
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <Wire.h>


GyroSensor::GyroSensor() {
    bno = Adafruit_BNO055(55, 0x28);
}

void GyroSensor::setup() {
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
    if (!mode) {
        return 0;
    }
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    return euler.x();
}

void GyroSensor::setMode(bool mode) {
    this->mode = mode;
}