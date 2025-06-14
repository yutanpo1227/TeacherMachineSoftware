# プロジェクトライブラリ README

このディレクトリには、プロジェクト専用のライブラリが含まれています。
PlatformIOはこれらを静的ライブラリとしてコンパイルし、実行ファイルにリンクします。

## 自作ライブラリ一覧

### 1. GyroSensor
**説明**: Adafruit BNO055ジャイロセンサーを使用した姿勢検出ライブラリ
**機能**:
- Yaw角の読み取り
- I2C通信によるBNO055制御
- センサーのモード設定
**使用例**: ロボットの姿勢制御、方向補正

### 2. IRSensor  
**説明**: IRセンサーアレイを使用したボール検出ライブラリ
**機能**:
- 複数IRセンサーからの角度計算
- 距離測定
- RCローパスフィルタによるノイズ除去
- 最大値センサー周辺のみを使用した精密検出
**使用例**: サッカーロボットのボール追跡

### 3. MotorController
**説明**: 4輪オムニホイールロボット用モーター制御ライブラリ
**機能**:
- 全方向移動制御
- 旋回動作
- ジャイロセンサー連携による姿勢修正
- 三角関数を使用した方向制御
**使用例**: オムニホイールロボットの移動制御

### 4. Motor
**説明**: 単一DCモーターの制御ライブラリ
**機能**:
- PWM制御による速度調整
- 正転・逆転制御
**使用例**: MotorControllerから使用される基本モーター制御

### 5. LineSensor
**説明**: ライン検出用ライブラリ
**機能**:
- 複数センサーからのライン角度検出
- ライン上判定
- しきい値による白/黒判定
**使用例**: ライン回避

### 6. Debuger
**説明**: シリアル通信を使用したデバッグ出力ライブラリ
**機能**:
- 可変長引数対応のprint関数
- センサー状態表示
- 値の組み合わせ表示
**使用例**: 開発時のデバッグ、センサー値監視

## ライブラリ構造

```
|--lib
|  |--GyroSensor
|  |  |- GyroSensor.h
|  |  |- GyroSensor.cpp
|  |--IRSensor
|  |  |- IRSensor.h
|  |  |- IRSensor.cpp
|  |--MotorController
|  |  |- MotorController.h
|  |  |- MotorController.cpp
|  |--Motor
|  |  |- Motor.h
|  |  |- Motor.cpp
|  |--LineSensor
|  |  |- LineSensor.h
|  |  |- LineSensor.cpp
|  |--Debuger
|  |  |- Debuger.h
|  |  |- Debuger.cpp
|  |- README --> このファイル
```

## 使用例

```cpp
#include "GyroSensor.h"
#include "IRSensor.h" 
#include "MotorController.h"
#include "Debuger.h"

GyroSensor gyro;
IRSensor irSensor(22, 8);
MotorController motorController(&motor1, &motor2, &motor3, &motor4);
Debuger debug;

void setup() {
    debug.begin(9600);
    gyro.setup();
    gyro.setMode(true);
}

void loop() {
    int gyroAngle = gyro.readYawAngle();
    int ballAngle = irSensor.readAngle();
    int ballDist = irSensor.readDistance();
    
    debug.printValues(gyroAngle, 0, ballAngle, ballDist);
    motorController.moveDirection(ballAngle, 127, gyroAngle);
}
```

---

## PlatformIO Library Dependency Finder

PlatformIOライブラリ依存関係ファインダーは、プロジェクトのソースファイルをスキャンして、
依存ライブラリを自動的に見つけます。

詳細情報:
- https://docs.platformio.org/page/librarymanager/ldf.html
