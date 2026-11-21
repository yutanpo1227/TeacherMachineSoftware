# V字ミラーカメラと攻守制御

守備の導入手順は [DEFENSE.md](DEFENSE.md) を参照してください。

`V-mirror.py` はMaixPy v1のUnitV-M12用です。
前後のミラー映像からオレンジ色のボール、黄色ゴール、青色ゴールを検出します。
実画像に合わせ、画像の右半分を前方、左半分を後方として扱います。
ここでの左右は物理的なミラーの位置ではなく、画像上の位置です。
オフェンス設定では相手ゴールに機体を向けながら、カメラ優先のボール追跡と既存の回り込みを実行します。

## 書き込みと設定

1. `V-mirror.py` をカメラへ書き込みます。
2. `src/RobotConfig.h` の `ATTACK_GOAL_IS_YELLOW` を設定します。`true` は黄色へ攻撃、`false` は青色へ攻撃です。前後半の切り替え時には変更してください。
3. Arduino側を `pio run -t upload` で書き込みます。

カメラとArduinoで対応する通信形式を使用してください。UARTは115200bps、8N1です。
`ENABLE_CAMERA=true`、`ENABLE_GYRO_SENSOR=true` で使用してください。
`USE_CAMERA_BALL_ANGLE=true` が初期値なので、ボール追跡にはカメラの検出角度を優先します。
カメラ未検出時にはIR角度を使います。
従来のIRだけでボール追跡する場合は、この設定を `false` に変更します。
`ENABLE_CAMERA=false` のオフェンスではジャイロの0度を姿勢目標にします。ディフェンスではカメラ通信がないため通常の走行を停止します。

## ゴール方向の維持

カメラ角度は機体正面が90度、後方が270度です。
新しい映像で攻撃ゴールを検出したときに、現在のジャイロ方位から目標方位を更新します。
ゴールが見えない間は、その方位を `GOAL_HOLD_MS`（初期値1500ms）だけ維持します。
最初のゴール検出前と保持期限後は、従来のジャイロ0度を目標にしてボール追跡を続けます。
再検出すればゴール方向を目標に戻します。
ゴールが側方の死角に入っても、自動探索の旋回は行いません。
ボールを見失った場合も従来どおり停止しますが、白線検出中は退避を優先します。

## 姿勢制御だけを検証する

`src/RobotConfig.h` の `SPEED=0` にすると姿勢検証モードになります。
このモードではボール検出を必要とせず、白線による停止と退避も通さず、旋回だけを指示します。
移動速度は常に0です。
通常のボール追跡へ戻すときは `SPEED` を正の値に変更します。

TESTモードの旋回出力は `src/RobotConfig.h` の次の設定で調整します。
通常走行の旋回設定には影響しません。

```cpp
constexpr int TEST_HEADING_MIN_POWER = 60;
constexpr int TEST_HEADING_MAX_POWER = 100;
constexpr float TEST_HEADING_KP = 2.0f;
```

出力は角度差の絶対値に `TEST_HEADING_KP` を掛け、最小60から最大100の範囲に収めます。
以前の最小20、最大50より強く旋回する設定です。
動き出しの強さは `TEST_HEADING_MIN_POWER`、最大の強さは `TEST_HEADING_MAX_POWER` で調整できます。
PWMの範囲は0〜255で、正面から±3度以内では出力を0にします。

Arduinoへ書き込んだ後、USBシリアルモニターを115200bpsで開きます。

```sh
pio device monitor --baud 115200
```

標準設定（`ENABLE_DEBUGGER=true`）では約250msごとに `telemetry` 行が出ます。姿勢検証中は `state=TEST` です。

- `camera=1` はArduinoが200ms以内に有効なカメラフレームを受信した状態です。`camera=0` が続く場合は、カメラのTX、MegaのRX1（19番）、共通GND、両側の通信速度を確認します。
- `yellow`、`blue`、`ownRear` はゴール方向で、未検出は-1です。`bottomGap` は後方ゴール下端までの画像上の画素距離です。
- `yaw` はジャイロ方位、`error` は姿勢の目標との差、`ir` と `cameraBall` はボールの検出角です。

従来の `heading-test` 行にある受信バイト数・不正フレーム数などは、現行の構造化ログでは送信していません。カメラ側のログは `tx=V3,...` で確認できます。詳しいUSBシリアルの手順は [実機調査スキル](../.agents/skills/rcj-robot-diagnostics/references/arduino-serial.md) を参照してください。

## 実機での確認順

1. カメラのログ `tx=V3,...` の黄色・青色の角度を確認し、`YELLOW_THRESHOLDS` と `BLUE_THRESHOLDS` を会場の色に合わせます。これらはLAB色空間の閾値です。現場の照明・ゴールの色と実際の検出結果を照合してください。対象がないときは `-1.0` になることを確認してください。
2. ゴールを正面、左、右、後方へ置き、角度が約90度、90度より大きい値、90度より小さい値、約270度になることを確認します。後方の左右は前方と異なるため、実際の機体方位で確認してください。
3. 前後が逆なら `ROI_FRONT` と `ROI_BACK`、左右が逆なら `FRONT_ANGLE_SIGN` と `BACK_ANGLE_SIGN` を調整します。正面位置は `FRONT_CENTER_POS` と `BACK_CENTER_POS`、視野角は `MIRROR_VIEW_FOV_DEG` で調整します。
4. 最初は `SPEED=0` の姿勢検証モードで、攻撃ゴールを左右へ動かして、その方向へ機体が旋回することを確認します。その後 `SPEED` を小さな正の値にしてボール追跡を確認します。
5. カメラの角度が正しいのに反対へ旋回する場合は、BNO055の方位の増加方向を確認して `GOAL_CAMERA_TO_YAW_SIGN` を反転します。初期値は、カメラ角度が反時計回り、ジャイロ方位が時計回りという仮定です。
6. オフェンス設定でボールを検出できる状態で攻撃ゴールを隠し、約1.5秒後にジャイロ0度へ目標が戻ることと、再検出でゴール方向へ向くことを確認します。オフェンスでは白線の位置とボールの方向に応じて停止または退避します。

旋回の強さは `lib/MotorController/MotorControllerConfig.h` の `HEADING_KP`、`MIN_CORRECTION_POWER`、`MAX_CORRECTION_POWER` で調整します。
正面から±3度以内では旋回を止め、90度以上のずれでは移動を止めて旋回します。
オフェンスでは白線を検出すると停止または退避します。守備ではカメラのゴール下端を使います。
守備と一時的な攻撃への切り替えは [守備の設定手順](DEFENSE.md) を参照してください。
自動得点判定はありません。

## 通信形式

現行カメラはV3を1フレーム1行で送信します。
角度は前90度・後270度、未検出は-1です。
ゴール幅はミラー画像のy方向の端点を角度へ換算した見かけの幅です。

```text
V3,球角度,球ピクセル数,球ミラー,黄前角,黄前幅,黄後角,黄後幅,青前角,青前幅,青後角,青後幅,黄後下端gap,青後下端gap
V3,90.2,400,1,90.0,35.0,270.0,-1.0,-1.0,-1.0,-1.0,-1.0,144,-1
```

球ミラーは0=未検出、1=前方、2=後方です。
ゴールが画像端で切れた場合、角度は送り、幅は-1にします。
前後のゴール情報は独立して送ります。
オフェンス用の代表ゴール角は、前方に検出があれば前方、なければ後方を使います。

末尾の2項目は画像中央から後方ゴール下端までの画素距離で、未検出は-1です。
受信側は旧V1・V2も受け取れますが、下端までの画素距離はありません。
**守備を使うにはカメラとArduinoの両方を更新してください。**
以前のArduinoはV3を受け取れません。
UARTは115200bps/8N1です。
V3は64バイトを超えるため、PlatformIOでArduinoの受信バッファを256バイトに設定しています。
Arduino IDE/CLIで別途ビルドする場合も、コアを含め同じ設定が必要です。

受信側は最新の有効行を使い、不正な行は破棄します。
カメラ情報は最終受信から200msで無効になり、ゴール方位の保持期限1500msは別に計算します。

## ソフトウェア検証

機体用のビルド：

```sh
pio run
```

角度境界、保持期限、時刻の周回、UART異常、モーター出力範囲のホストテスト：

```sh
c++ -std=c++11 -Wall -Wextra -Werror \
  -Itest/host/stubs -Ilib/Camera -Ilib/GoalHeading \
  -Ilib/DefenseController -Ilib/LineSensor -Ilib/Motor -Ilib/MotorController \
  test/host/goal_control_test.cpp lib/Camera/Camera.cpp lib/GoalHeading/GoalHeading.cpp \
  lib/DefenseController/*.cpp lib/LineSensor/LineSensor.cpp lib/Motor/Motor.cpp lib/MotorController/MotorController.cpp \
  -o /tmp/teacher-goal-control-test
/tmp/teacher-goal-control-test
```

カメラの前後検出と送信処理のテストは、uvで環境を作成して実行します。
追加パッケージは不要です。

```sh
uv venv /tmp/teacher-camera-validation-venv
PYTHONDONTWRITEBYTECODE=1 /tmp/teacher-camera-validation-venv/bin/python \
  -m unittest discover -s test/host -p 'test_*.py' -v
```

ホストテストはMaixPyの実機APIを置き換えて検証します。
実際の色の判別、ミラーの視野、処理速度、機体の旋回方向は実機で確認してください。
