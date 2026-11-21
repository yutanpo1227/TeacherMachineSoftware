#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>
#include "DefenseConfig.h" // 守備の横速度・攻撃時間などはDefenseConfigの既定値で設定。

// 試合の役割と攻撃するゴール色。前後半の入れ替えに合わせて設定する。
enum class RobotRole { OFFENSE, DEFENSE };
constexpr RobotRole ROBOT_ROLE = RobotRole::DEFENSE;
#define ATTACK_GOAL_IS_YELLOW true // trueは黄色へ攻撃、falseは青色へ攻撃。

// 走行の最大PWM指令（0〜255）。0にするとボールを追わず姿勢制御だけを試す。
#define SPEED 230
constexpr bool HEADING_TEST_MODE = SPEED == 0;
// 姿勢テスト時の旋回PWM上下限と、姿勢誤差1度あたりの比例係数。
constexpr int TEST_HEADING_MIN_POWER = 60;
constexpr int TEST_HEADING_MAX_POWER = 100;
constexpr float TEST_HEADING_KP = 2.0f;

// センサーとUSBログの有効・無効。守備中のラインは表示用に測定する。
#define ENABLE_LINE_SENSOR true
#define ENABLE_GYRO_SENSOR true
#define ENABLE_DEBUGGER true
constexpr bool ENABLE_TELEMETRY = ENABLE_DEBUGGER;
#define ENABLE_CAMERA true
#define USE_CAMERA_BALL_ANGLE true // 攻撃時はカメラ優先。未検出時はIRへ切替。

// カメラの角度をジャイロ方位へ変換する符号と、見失った目標の保持時間（ms）。
constexpr float GOAL_CAMERA_TO_YAW_SIGN = -1.0f;
constexpr uint32_t GOAL_HOLD_MS = 1500;

// ゴール下端までの画素距離。視野・取り付け角によって実距離との関係は変わる。
constexpr int GOAL_EDGE_NEAR_GAP = 60;  // 近すぎる判定。校正時は機体中心から約20cm。
constexpr int GOAL_EDGE_CLEAR_GAP = 66; // この画素値まで離れると近接判定を解除。
constexpr int GOAL_EDGE_ESCAPE_POWER = 180; // 近すぎる際に前進するPWM指令。
constexpr int ATTACK_LINE_EXIT_POWER = 150; // 守備から攻撃へ移った際の白線離脱PWM。
constexpr int ATTACK_LINE_EXIT_MAX_GAP = 120; // 白線離脱を許可するゴール下端の画素距離上限。
constexpr int GOAL_HOME_TARGET_GAP = 66; // 守備時に戻りたいゴール下端の画素距離。
constexpr int GOAL_HOME_START_MARGIN = 4; // 目標からこの画素数を超えて離れたとき後退を開始。
constexpr int GOAL_RETURN_POWER = 210; // ゴールを見ながら守備位置へ戻る後退PWM指令。
constexpr int BLIND_RETURN_POWER = 180; // ゴール未検出で探索しながら後退するPWM指令。
constexpr int RETURN_TURN_MIN_POWER = 85; // 復帰・探索で停止中に旋回する最小PWM。
constexpr int RETURN_TURN_MAX_POWER = 130; // 同じ旋回の最大PWM。
constexpr float RETURN_TURN_KP = 2.0f; // 同じ旋回の姿勢誤差1度あたりの比例係数。

// 8個の白線センサーごとのアナログ値閾値。各値を超えると白線と判定する。
const int LINE_SENSOR_THRESHOLDS[] = {860, 810, 890, 820, 800, 820, 820, 820};

#endif
