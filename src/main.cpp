#include <Arduino.h>
#include <Wire.h>
#include "LineSensor.h"
#include "IRSensor.h"
#include "MotorController.h"
#include "Motor.h"
#include "GyroSensor.h"
#include "Debugger.h"

// マシンのセットアップ値
#define SPEED 230  // モーターの速度
#define ENABLE_LINE_SENSOR true  // ラインセンサーの有効化
#define ENABLE_GYRO_SENSOR true  // ジャイロセンサーの有効化
#define ENABLE_DEBUGGER false  // デバッグモードの有効化

// 回り込みの設定値（readDistance は TSSP LOW 比率 EMA 由来の相対量×1000。実機で再調整のこと）
#define WRAP_AROUND_BALL_DIST_THRESHOLD 900

// ラインセンサーの閾値
const int LINE_SENSOR_THRESHOLDS[] = {860, 810, 890, 820, 800, 820, 820, 820};

// 8方向移動用の角度配列
const int DIRECTION_EIGHT_ANGLES[] = {0, 30, 90, 135, 180, 225, 270, 315};
const int WRAP_AROUND_ANGLE[] = {315, 345, 90, 225, 250, 300, 0, 225};

// センサーのインスタンス
LineSensor lineSensor = LineSensor(0, 8, LINE_SENSOR_THRESHOLDS);
IRSensor irSensor = IRSensor(22, 8);
GyroSensor gyroSensor = GyroSensor();

// デバッガーのインスタンス
Debugger debugger = Debugger(ENABLE_DEBUGGER);

// 各モーターのインスタンス
Motor motor1 = Motor(3, 2);
Motor motor2 = Motor(6, 5);
Motor motor3 = Motor(8, 7);
Motor motor4 = Motor(12, 11);

// モーターコントローラーのインスタンス
MotorController motorController = MotorController(&motor1, &motor2, &motor3, &motor4);

int correctAngle(int angle);

void setup() {
  debugger.begin(9600);
  Wire.begin();
  gyroSensor.setup();
  // 各センサーを有効にするかどうか
  lineSensor.setMode(ENABLE_LINE_SENSOR);
  gyroSensor.setMode(ENABLE_GYRO_SENSOR);
  debugger.printSensorStatus(ENABLE_LINE_SENSOR, ENABLE_GYRO_SENSOR);
}

void loop() {
  const int lineAngle = lineSensor.readAngle();
  const float lineVectorMagnitude = lineSensor.readVectorMagnitude();
  const int gyroAngle = gyroSensor.readYawAngle();
  const int ballAngle = irSensor.readAngle();
  const int ballDist = irSensor.readDistance();
  debugger.printValues(gyroAngle, lineAngle, lineVectorMagnitude, ballAngle, ballDist);

  // 測った角度を8方向に変換して移動
  const int directionEight = correctAngle(ballAngle);
  // 回り込み距離以下ならボール方向の角度、それ以上なら回り込み角度を使う
  const int moveAngle = ballDist < WRAP_AROUND_BALL_DIST_THRESHOLD ? DIRECTION_EIGHT_ANGLES[directionEight] : WRAP_AROUND_ANGLE[directionEight];
  motorController.moveDirection(moveAngle, SPEED, directionEight * 45, gyroAngle, lineAngle, lineVectorMagnitude);
}

int correctAngle(int angle) {
    if(angle > 330 || angle <= 40){
        return 0;
    } else if(angle > 40 && angle <= 80){
        return 1;
    } else if(angle > 80 && angle <= 110){
        return 2;
    } else if(angle > 110 && angle <= 150){
        return 3;
    } else if(angle > 150 && angle <= 190){
        return 4;
    } else if(angle > 190 && angle <= 230){
        return 5;
    } else if(angle > 230 && angle <= 275){
        return 6;
    } else if(angle > 275 && angle <= 330){
        return 7;
    } else {
        return -1;
    }
}