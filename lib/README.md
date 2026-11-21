# ロボット制御ライブラリ

Arduino Megaのセンサー入力から、ボール・ゴールに対する走行を決め、4輪のPWMを出力するライブラリです。各クラスの公開する機能と入力値は `.h`、計算や制御の実装は `.cpp` にあります。組み合わせ方とピン番号は [src/main.cpp](../src/main.cpp) を参照してください。

| ディレクトリ | 実機での役割 |
| --- | --- |
| [Camera](Camera/) | カメラからSerial1でV1〜V3の行を受信し、ボール・ゴールの方向や後方ゴール下端の画素距離を取得する。 |
| [IRSensor](IRSensor/) | 8個のIR受光素子のパルスから、ボールの方向を求める。距離は使用せず `-1` を返す。 |
| [LineSensor](LineSensor/) | 8個の反射センサーの閾値から、白線の方向と検出ベクトルの大きさを求める。 |
| [GyroSensor](GyroSensor/) | BNO055から機体の方位を読む。起動時の向きを自動で0度にはしない。 |
| [GoalHeading](GoalHeading/) | 検出したゴール角とジャイロから、目標方位と姿勢のずれを計算する。 |
| [DefenseController](DefenseController/) | 守備の横追従、前方カメラによる一時攻撃、ゴール距離・白線・攻撃時の方向の補助判定を行う。 |
| [Motor](Motor/) | 1輪分の回転方向とPWMをモータードライバーへ出力する。 |
| [MotorController](MotorController/) | 移動方向・姿勢補正・白線回避を合成し、4輪に指令する。 |
| [Debugger](Debugger/) | USBシリアルへのセンサー情報や初期化状態の表示を補助する。現在の構造化テレメトリは `src/main.cpp` で送信する。 |

機体から見た方向角は右0度、前90度、左180度、後ろ270度です。角度の `-1` は未検出を表します。ゴール下端の `bottomGap` は画像上の画素数です。PWM指令は0〜255で、実測した回転数ではありません。

通常走行時の姿勢補正と白線判定の調整値は [MotorControllerConfig.h](MotorController/MotorControllerConfig.h) にあります。

守備の状態遷移と調整値は [守備の説明](../camera/DEFENSE.md)、ビルドとテストは [プロジェクトのREADME](../README.md) を参照してください。
