# TeacherMachineSoftware

ロボカップジュニア・サッカー ライトウェイト用のロボット制御ソフトです。Arduino Megaが4輪のオムニホイールを駆動します。V字ミラー付きカメラで前後を見て、IRセンサーでボール、白線センサーで床のライン、BNO055で機体の向きを測ります。MacではPITCHデバッガーで観測値を確認できます。

初めてこの機体を見る方向けに、[説明書の目次](docs/README.md)からロボット全体・カメラ・IR・白線・姿勢・モーターの動作をたどれます。

## 使用デバイス

| 部品 | 使用しているもの | 主な役割 |
| --- | --- | --- |
| マイコン | Arduino Mega Pro Mini | センサーを読み、4輪のモーターを制御する |
| カメラ | [M5Stack UnitV AI Camera（OV7740、M12レンズ版）](https://www.switch-science.com/products/9182) | V字ミラー越しにボールとゴールを検出する |
| IR受光素子 | TSSP58038、8個。センサー基板は自作 | 赤外線を出すボールの方向を検出する |
| 白線センサー | 自作基板と反射センサー、8個 | 床の白線の方向を検出する |
| 姿勢センサー | BNO055 | 機体の方位を取得する |
| モーター | IG22 DCモーター、4個 | 車輪を駆動する。減速比・詳細型番は未確認 |
| モータードライバー | DRV8874、4個 | Arduinoの指令でモーターの出力と回転方向を制御する |
| 基板・車輪 | 自作メイン基板、自作モーター基板、自作オムニホイール4個 | センサーとモーターを接続し、全方向へ移動する |

ハードウェア構成の基になった情報は[TeacherMachineの開発記事](https://y-blog-livid.vercel.app/posts/teacher-machine)にあります。記事公開後に追加されたカメラと変更されたIR受光素子を上の一覧へ反映しています。現在のソフトウェアはIRの方向を求め、距離は算出していません。

## 設定して動かす

1. カメラ側にはMaixPy IDEから [V-mirror.py](camera/V-mirror.py) を書き込みます。前後の映像・色閾値・配線は [カメラの設定](camera/README.md) を参照してください。
2. Arduino側の設定は [src/RobotConfig.h](src/RobotConfig.h) にあります。`ROBOT_ROLE` は `OFFENSE` または `DEFENSE`、`ATTACK_GOAL_IS_YELLOW` は黄色へ攻めるなら `true`、青へ攻めるなら `false` です。守備の横速度・攻撃時間は [DefenseConfig.h](lib/DefenseController/DefenseConfig.h)、通常走行の姿勢補正・白線判定は [MotorControllerConfig.h](lib/MotorController/MotorControllerConfig.h) にあります。カメラの色閾値と視野はカメラ側の [V-mirror.py](camera/V-mirror.py) で設定します。
3. ArduinoをUSBで接続し、リポジトリのルートで `pio run` を実行します。書き込みは `pio run -t upload` です。ポートの指定が必要なら `--upload-port <ポート名>` を追加します。
4. 姿勢だけを確認するときは `SPEED` を `0` にすると、車体を移動させずに旋回だけを試せます。通常走行に戻すときは正の値にします。出力はPWM指令で、タイヤの回転数を表す値ではありません。

## 実機の動きとコードの対応

| 実機で見える動き | コード |
| --- | --- |
| 相手ゴールへ機体を向け、ボールが正面なら直進、横や後ろなら回り込む | [main.cpp](src/main.cpp) の `runOffense()`。カメラ優先で、未検出ならIRへ切り替えます。正面での直進の保持と移動方向の変化は [攻撃の補助処理](lib/DefenseController/) で調整します。 |
| 自ゴールを後ろに見ながら、ボールに合わせて左右に移動する | [DefenseController.h](lib/DefenseController/DefenseController.h) と `runDefense()`。守備中はIR優先で、白線は表示用に観測します。 |
| ゴール前で距離を調整する | [GoalEdgeGuard.h](lib/DefenseController/GoalEdgeGuard.h) と [GoalReturn.h](lib/DefenseController/GoalReturn.h)。カメラ画像のゴール下端を基準に、近すぎれば前進、遠すぎれば後退を横移動に合成します。 |
| ゴールを見失い、その場で向き直ってから後退する | [BlindReturnAlignment.h](lib/DefenseController/BlindReturnAlignment.h) と `runDefense()`。カメラ通信中はジャイロ0°へ整列してから左右を確認しつつ後退します。守備中に通信が切れた場合は停止します。 |
| 守備から一時攻撃に切り替える | [DefenseController.h](lib/DefenseController/DefenseController.h) と `runTimedAttack()`。前方カメラにボールが5秒映ると、最大10秒間、通常攻撃と同じ処理で走ります。条件を満たす場合は開始直後に短時間の白線離脱が入ります。 |
| 白線で停止・退避する、旋回しながら並進する | [MotorController.cpp](lib/MotorController/MotorController.cpp)。動きの指令と姿勢補正を各車輪のPWMへ変換します。 |

`loop()` はセンサー取得、表示用データの送信、モードに応じた走行の順です。機体から見た角度は右0°、前90°、左180°、後ろ270°。ジャイロの0°はセンサーの方位基準です。守備の閾値と遷移条件は [守備の仕様](camera/DEFENSE.md) にまとめています。

## デバッグ

[PITCHデバッガー](robot-debugger/README.md) を起動し、ArduinoのUSBポートを選んで115200bpsで接続します。ボールと白線は**方向**、`bottomGap` はゴールまでの距離の目安になる**画素値**です。カメラとArduinoのUARTは、このUSB接続とは別の経路です。PlatformIOのシリアルモニターなどとポートを共有できない場合は、先に片方を閉じます。

このプロジェクトでCodexを使うときは、[実機調査スキル](.agents/skills/rcj-robot-diagnostics/SKILL.md) にセンサー・通信・走行の切り分け手順があります。

## 検証

リポジトリのルートで実行します。Arduinoへの書き込みなしで、ビルドと主要な判定を検証できます。

```sh
pio run
clang++ -std=c++11 -Wall -Wextra -Werror \
  -Itest/host/stubs -Ilib/Camera -Ilib/GoalHeading -Ilib/DefenseController \
  -Ilib/Motor -Ilib/MotorController -Ilib/LineSensor test/host/goal_control_test.cpp \
  lib/Camera/Camera.cpp lib/GoalHeading/GoalHeading.cpp lib/DefenseController/*.cpp \
  lib/Motor/Motor.cpp lib/MotorController/MotorController.cpp lib/LineSensor/LineSensor.cpp \
  -o /tmp/teacher-goal-control-test
/tmp/teacher-goal-control-test
uv run --no-project python -m unittest discover -s test/host -p 'test_camera.py'
npm --prefix robot-debugger test
```
