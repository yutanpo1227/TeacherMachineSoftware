#include <Arduino.h>
#include <Wire.h>
#include "LineSensor.h"
#include "IRSensor.h"
#include "MotorController.h"
#include "Motor.h"
#include "GyroSensor.h"
#include "Debugger.h"

// マシンのセットアップ値
#define SPEED 100  // モーターの速度
#define ENABLE_LINE_SENSOR true  // ラインセンサーの有効化
#define ENABLE_GYRO_SENSOR true  // ジャイロセンサーの有効化
#define ENABLE_DEBUGGER false  // デバッグモードの有効化

// 回り込みの設定値
#define WRAP_AROUND_ANGLE_THRESHOLD 5  // 回り込み角度の閾値
#define WRAP_AROUND_BALL_DIST_THRESHOLD 900  // 回り込み距離の閾値
#define WRAP_AROUND_ANGLE_RATIO 1.5  // 回り込み角度の倍率
#define WRAP_AROUND_ANGLE 70  // 最大回り込み角度

// ラインセンサーの閾値
const int LINE_SENSOR_THRESHOLDS[] = {860, 810, 890, 820, 800, 820, 820, 820};

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

int calcWrapAroundAngle(int ballAngle, int ballDist);

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

  int moveAngle = calcWrapAroundAngle(ballAngle, ballDist);
  // motorController.moveDirection(moveAngle, SPEED, gyroAngle, lineAngle, lineVectorMagnitude);
  motorController.moveDirectionEight(moveAngle, SPEED, gyroAngle, lineAngle, lineVectorMagnitude);
}


// 回り込み角度の計算
int calcWrapAroundAngle(int ballAngle, int ballDist) {
    // ボールの角度の範囲を-180~180に変換
  int correctBallAngle = ballAngle - 90;
  if (correctBallAngle > 180) {
    correctBallAngle = correctBallAngle - 360;
  }

  int moveAngle = correctBallAngle;
  // ボールが近い時は大きく回り込む
  if (ballDist > WRAP_AROUND_BALL_DIST_THRESHOLD) {
    // if (correctBallAngle > WRAP_AROUND_ANGLE_THRESHOLD || correctBallAngle < -WRAP_AROUND_ANGLE_THRESHOLD) {
    //   moveAngle = correctBallAngle * WRAP_AROUND_ANGLE_RATIO;
    // }
    if (correctBallAngle > WRAP_AROUND_ANGLE_THRESHOLD) {
      moveAngle = correctBallAngle + 90;
    } else if (correctBallAngle < -WRAP_AROUND_ANGLE_THRESHOLD) {
      moveAngle = correctBallAngle - 90;
    }
  }
  // ボールの角度を元の座標系に変換
  if (moveAngle < 0) {
    moveAngle = moveAngle + 360;
  }
  moveAngle = moveAngle + 90;
  if (moveAngle > 360) {
    moveAngle = moveAngle - 360;
  }
  if (moveAngle < 0) {
    moveAngle = moveAngle + 360;
  }
  return moveAngle;
}