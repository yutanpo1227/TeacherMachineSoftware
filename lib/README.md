# Teacher Machine ライブラリ

このディレクトリには、Teacher Machineプロジェクトで使用するカスタムライブラリが含まれています。

## ライブラリ構成

### 1. LineSensor
ラインセンサーの制御を行うライブラリです。16個のセンサーから線の位置と方向を検出します。

### 2. GyroSensor
ジャイロセンサー(MPU6050)の制御を行うライブラリです。ロボットの傾きや回転を検出します。

### 3. BallSensor
ボールセンサーの制御を行うライブラリです。40個のセンサーからボールの位置を検出します。

### 4. Motor
モータードライバの制御を行うライブラリです。4つのモーターを独立して制御できます。

### 5. MotorControl
モーターの協調制御を行うライブラリです。全方向移動の計算を行います。

### 6. Debugger
デバッグ用のライブラリです。シリアル通信を使ってセンサーの値やシステムの状態を出力します。

## 使用方法

各ライブラリは `src/main.cpp` でインクルードして使用します。

```cpp
#include "LineSensor.h"
#include "GyroSensor.h"
#include "BallSensor.h"
#include "Motor.h"
#include "MotorControl.h"
#include "Debugger.h"
```

## ディレクトリ構造

```
lib/
|--LineSensor/
|  |--LineSensor.h
|  |--LineSensor.cpp
|--GyroSensor/
|  |--GyroSensor.h
|  |--GyroSensor.cpp
|--BallSensor/
|  |--BallSensor.h
|  |--BallSensor.cpp
|--Motor/
|  |--Motor.h
|  |--Motor.cpp
|--MotorControl/
|  |--MotorControl.h
|  |--MotorControl.cpp
|--Debugger
|  |  |- Debugger.h
|  |  |- Debugger.cpp
|--README.md
```

## 初期化例

```cpp
#include "libraries.h"
#include "Motor.h"
#include "LineSensor.h"
#include "GyroSensor.h"
#include "BallSensor.h"
#include "MotorControl.h"
#include "Debugger.h"

LineSensor lineSensor;
GyroSensor gyroSensor;
BallSensor ballSensor;
Motor motor;
MotorControl motorControl;
Debugger debug;

void setup() {
  lineSensor.begin();
  gyroSensor.begin();
  ballSensor.begin();
  motor.begin();
  debug.begin(9600);
}
```

各ライブラリの詳細な使用方法については、各ヘッダーファイルのコメントを参照してください。
