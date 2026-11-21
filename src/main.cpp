#include <Arduino.h>
#include <Wire.h>
#include "LineSensor.h"
#include "IRSensor.h"
#include "MotorController.h"
#include "Motor.h"
#include "GyroSensor.h"
#include "Debugger.h"
#include "Camera.h"
#include "GoalHeading.h"
#include "DefenseController.h"
#include "GoalEdgeGuard.h"
#include "GoalReturn.h"
#include "BlindReturnAlignment.h"
#include "AttackLineExit.h"
#include "OffenseDirectionSmoother.h"
#include "OffenseStraightApproach.h"
#include "RobotConfig.h"

// 守備の調整値はDefenseConfigの初期値を使う。
const DefenseConfig DEFENSE_CONFIG = DefenseConfig();
DefenseController defense(DEFENSE_CONFIG);

GoalReturn goalReturn(GOAL_HOME_TARGET_GAP, GOAL_HOME_START_MARGIN);
BlindReturnAlignment blindReturnAlignment;
AttackLineExit attackLineExit;
OffenseDirectionSmoother offenseDirectionSmoother;
OffenseStraightApproach offenseStraightApproach;
GoalEdgeGuard goalEdgeGuard(GOAL_EDGE_NEAR_GAP, GOAL_EDGE_CLEAR_GAP);
GoalEdgeGuard::State goalEdgeState = GoalEdgeGuard::State::UNCALIBRATED;

// センサーのピン配置。ライン8個はA0から、IR受光素子8個はデジタル22番から接続する。
LineSensor lineSensor = LineSensor(0, 8, LINE_SENSOR_THRESHOLDS);
// 守備中の観測で、攻撃用ラインセンサーの角度フィルターを変更しない。
LineSensor debugLineSensor = LineSensor(0, 8, LINE_SENSOR_THRESHOLDS);
IRSensor irSensor = IRSensor(22, 8);
GyroSensor gyroSensor = GyroSensor();

// カメラのインスタンス
Camera camera = Camera();
GoalHeading goalHeading(GOAL_HOLD_MS, GOAL_CAMERA_TO_YAW_SIGN);
GoalHeading defenseHeading(GOAL_HOLD_MS, GOAL_CAMERA_TO_YAW_SIGN);

// デバッガーのインスタンス
Debugger debugger = Debugger(ENABLE_DEBUGGER);

// 各モーターのピンは (回転方向PH, PWM出力EN)。4輪の並進・旋回はMotorControllerが合成する。
Motor motor1 = Motor(3, 2);
Motor motor2 = Motor(6, 5);
Motor motor3 = Motor(8, 7);
Motor motor4 = Motor(12, 11);

// モーターコントローラーのインスタンス
MotorController motorController = MotorController(&motor1, &motor2, &motor3, &motor4);

float calculateWrapAroundAngle(float ballAngle);
void runOffense(int ballAngle, float headingError, int lineAngle, float lineVectorMagnitude);
void flushTelemetry();
void queueTelemetry(const char *role, const char *state, int yaw, int ball, int ir, int cameraBall, int line,
                    int lineStrength, int yellow, int blue, int ownRear, int cameraOk, int error,
                    int bottomGap, int lineUsed, int attackYellow, int lineEnabled);

void setup() {
    debugger.begin(115200);
    if (!ENABLE_DEBUGGER && (HEADING_TEST_MODE || ROBOT_ROLE == RobotRole::DEFENSE)) {
        Serial.begin(115200);
    }
    Wire.begin();
    if (ENABLE_GYRO_SENSOR) {
        gyroSensor.setup();
    }
    // 各センサーを有効にするかどうか
    lineSensor.setMode(ENABLE_LINE_SENSOR);
    debugLineSensor.setMode(ENABLE_LINE_SENSOR);
    gyroSensor.setMode(ENABLE_GYRO_SENSOR);
    debugger.printSensorStatus(ENABLE_LINE_SENSOR, ENABLE_GYRO_SENSOR);
    if (ENABLE_CAMERA) {
        camera.setup();
    }
}

// 1ループで読んだ入力をまとめ、走行と表示で同じボール角・姿勢を使う。
struct ControlFrame {
    uint32_t nowMs; // millis()の経過時間。連続検出や攻撃時間の判定に使う。
    bool defenseMode;
    int lineAngle; // 機体から見た白線の方向。-1は未検出、守備中も制御上は-1。
    float lineVectorMagnitude;
    int gyroAngle; // ジャイロの方位。電源投入時の機体方向で自動的にゼロにはならない。
    int irBallAngle;
    int cameraBallAngle;
    bool newCameraFrame;
    bool defenseHeadingValid;
    float defenseError; // 自ゴールを後方に向けるための姿勢のずれ。
    float offenseError; // 相手ゴールを前方に向けるための姿勢のずれ。
    bool useGoalHeading;
    int ballAngle; // 現在の役割で採用した角度。守備ではIRを優先する。
    int offenseBallAngle; // 攻撃で採用する角度。設定が有効ならカメラを優先する。
};

// 各センサーを1回読み、同じ時点の制御入力としてまとめる。
// 方向角は機体の右0度・前90度・左180度・後270度。未検出は-1。
ControlFrame readControlFrame() {
    const bool defenseMode = ROBOT_ROLE == RobotRole::DEFENSE;
    int lineAngle;
    float lineVectorMagnitude;
    if (defenseMode) {
        // 守備ではユーザー指定によりラインを使わない。
        lineAngle = -1;
        lineVectorMagnitude = 0;
    } else {
        lineAngle = lineSensor.readAngle();
        lineVectorMagnitude = lineSensor.readVectorMagnitude();
    }
    const int gyroAngle = gyroSensor.readYawAngle();
    const int irBallAngle = irSensor.readAngle();
    const bool newCameraFrame = ENABLE_CAMERA && camera.update();
    const int cameraBallAngle = ENABLE_CAMERA ? (int)camera.ballAngle() : -1;
    const uint32_t nowMs = millis();
    if (newCameraFrame) {
        // 未検出(-1)では更新しない。同じフレームで保持期限を延長しない。
        goalHeading.observe(camera.goalAngle(ATTACK_GOAL_IS_YELLOW), gyroAngle, nowMs);
        // 端が見切れていても、映っているゴール矩形の中心を追う。
        const float rearGoal = camera.goalViewAngle(!ATTACK_GOAL_IS_YELLOW, true);
        if (rearGoal >= 0) {
            defenseHeading.observe(rearGoal, gyroAngle, nowMs, 270.0f);
        }
    }
    const bool defenseHeadingValid = ENABLE_CAMERA && defenseHeading.isValid(nowMs);
    const float defenseError = ENABLE_GYRO_SENSOR
                                   ? (defenseHeadingValid ? defenseHeading.error(gyroAngle)
                                                          : GoalHeading::angleDifference(gyroAngle))
                                   : 0;
    // 姿勢検証では守備なら自ゴールを後ろへ、攻撃なら相手ゴールを前へ向ける。
    const bool useGoalHeading =
        ENABLE_CAMERA && (defenseMode ? defenseHeadingValid : goalHeading.isValid(nowMs));
    const float offenseError = ENABLE_GYRO_SENSOR
                                   ? (goalHeading.isValid(nowMs) ? goalHeading.error(gyroAngle)
                                                                 : GoalHeading::angleDifference(gyroAngle))
                                   : 0;

    // 守備では発光ボールを直接見るIRを優先し、未検出ならカメラを使う。
    // 攻撃では従来どおりカメラ優先設定に従う。
    const bool useCameraBallAngle =
        ENABLE_CAMERA && cameraBallAngle != -1 && (defenseMode ? irBallAngle < 0 : USE_CAMERA_BALL_ANGLE);
    const int ballAngle = useCameraBallAngle ? cameraBallAngle : irBallAngle;
    const int offenseBallAngle =
        ENABLE_CAMERA && USE_CAMERA_BALL_ANGLE && cameraBallAngle != -1 ? cameraBallAngle : irBallAngle;

    return {nowMs,        defenseMode,     lineAngle,      lineVectorMagnitude, gyroAngle,
            irBallAngle,  cameraBallAngle, newCameraFrame, defenseHeadingValid, defenseError,
            offenseError, useGoalHeading,  ballAngle,      offenseBallAngle};
}

// USBに表示用の状態を送り、PITCHデバッガーでセンサーと制御状態を確認できるようにする。
void publishTelemetry(const ControlFrame &frame) {
    const auto nowMs = frame.nowMs;
    const auto defenseMode = frame.defenseMode;
    const auto lineAngle = frame.lineAngle;
    const auto lineVectorMagnitude = frame.lineVectorMagnitude;
    const auto gyroAngle = frame.gyroAngle;
    const auto irBallAngle = frame.irBallAngle;
    const auto cameraBallAngle = frame.cameraBallAngle;
    const auto ballAngle = frame.ballAngle;
    const auto offenseBallAngle = frame.offenseBallAngle;
    const float headingError = defenseMode ? frame.defenseError : frame.offenseError;
    // 表示用だけの測定は約4Hz。守備中の白線判定には一切使わない。
    static uint32_t lastTelemetryMs = 0;
    if (ENABLE_TELEMETRY && (uint32_t)(nowMs - lastTelemetryMs) >= 250) {
        lastTelemetryMs = nowMs;
        const int observedLine = defenseMode ? debugLineSensor.readAngle() : lineAngle;
        const float observedMagnitude =
            defenseMode ? debugLineSensor.readVectorMagnitude() : lineVectorMagnitude;
        const bool attack = defenseMode && defense.state() == DefenseController::State::ATTACK;
        const bool cameraOk = ENABLE_CAMERA && camera.isReceiving();
        const int ownRear = cameraOk ? (int)camera.goalViewAngle(!ATTACK_GOAL_IS_YELLOW, true) : -1;
        const char *state = HEADING_TEST_MODE ? "TEST"
                            : !defenseMode    ? "OFFENSE"
                            : attack          ? "ATTACK"
                            : !cameraOk       ? "CAMERA_LOST"
                            : ownRear < 0     ? "GOAL_MISSING"
                                              : defense.stateName();
        const float displayError = attack ? frame.offenseError : headingError;
        queueTelemetry(defenseMode ? "DEFENSE" : "OFFENSE", state, gyroAngle,
                       attack ? offenseBallAngle : ballAngle, irBallAngle, cameraBallAngle, observedLine,
                       (int)(observedMagnitude * 100.0f), cameraOk ? (int)camera.goalAngle(true) : -1,
                       cameraOk ? (int)camera.goalAngle(false) : -1, ownRear, cameraOk ? 1 : 0,
                       (int)displayError, cameraOk ? camera.rearGoalBottomGap(!ATTACK_GOAL_IS_YELLOW) : -1,
                       ENABLE_LINE_SENSOR && (!defenseMode || attack) ? 1 : 0, ATTACK_GOAL_IS_YELLOW ? 1 : 0,
                       ENABLE_LINE_SENSOR ? 1 : 0);
    }
}

// SPEEDが0のとき、タイヤに並進を指示せずゴール方向への向き直りだけを試す。
void runHeadingTest(const ControlFrame &frame) {
    const auto gyroAngle = frame.gyroAngle;
    const auto irBallAngle = frame.irBallAngle;
    const auto lineAngle = frame.lineAngle;
    const auto useGoalHeading = frame.useGoalHeading;
    const float headingError = frame.defenseMode ? frame.defenseError : frame.offenseError;
    // ボール未検出による停止や白線の退避を通さず、旋回だけを検証する。
    const int turnPower = motorController.holdHeading(headingError, TEST_HEADING_MIN_POWER,
                                                      TEST_HEADING_MAX_POWER, TEST_HEADING_KP);
    static uint32_t lastLogMs = 0;
    const uint32_t now = millis();
    if (!ENABLE_TELEMETRY && (uint32_t)(now - lastLogMs) >= 200) {
        lastLogMs = now;
        Serial.print(F("heading-test cam="));
        Serial.print(ENABLE_CAMERA && camera.isReceiving() ? F("OK") : F("NO_DATA"));
        Serial.print(F(" rx="));
        Serial.print(camera.receivedBytes());
        Serial.print(F(" valid="));
        Serial.print(camera.validFrames());
        Serial.print(F(" bad="));
        Serial.print(camera.invalidFrames());
        Serial.print(F(" yellow="));
        Serial.print(camera.goalAngle(true), 1);
        Serial.print(F(" blue="));
        Serial.print(camera.goalAngle(false), 1);
        Serial.print(F(" ownRear="));
        Serial.print(camera.goalViewAngle(!ATTACK_GOAL_IS_YELLOW, true), 1);
        Serial.print(F(" span="));
        Serial.print(camera.goalSpan(!ATTACK_GOAL_IS_YELLOW, true), 1);
        Serial.print(F(" bottomGap="));
        Serial.print(camera.rearGoalBottomGap(!ATTACK_GOAL_IS_YELLOW));
        Serial.print(F(" source="));
        Serial.print(useGoalHeading ? F("GOAL") : F("ZERO"));
        Serial.print(F(" yaw="));
        Serial.print(gyroAngle);
        Serial.print(F(" error="));
        Serial.print(headingError, 1);
        Serial.print(F(" turn="));
        Serial.print(turnPower);
        Serial.print(F(" ir="));
        Serial.print(irBallAngle);
        Serial.print(F(" line="));
        Serial.print(lineAngle);
        Serial.print(F(" wire=["));
        Serial.print(camera.lastReceivedLine());
        Serial.println(F("]"));
    }
}

// 守備から一時攻撃へ。白線からの離脱を終えたら通常OFFENSEと同じ経路を通る。
bool runTimedAttack(const ControlFrame &frame) {
    const uint32_t nowMs = frame.nowMs;
    const float defenseError = frame.defenseError;
    // ボールの採用元がIRでも、前方カメラでの連続検出だけで攻撃に移る。
    const bool wasAttacking = defense.state() == DefenseController::State::ATTACK;
    defense.observeFrontBall(nowMs, ENABLE_CAMERA && frame.cameraBallAngle >= 0 && camera.ballView() == 1);
    // 攻撃中はOFFENSEと同じ走行を使い、制限時間まで守備側の判定を通さない。
    if (defense.state() != DefenseController::State::ATTACK)
        return false;
    blindReturnAlignment.reset();
    DefenseInput tick;
    tick.now = nowMs;
    // 攻撃が終わったループから、守備側のゴール確認と復帰へ進む。
    if (!defense.update(tick).attack)
        return false;

    // 守備中は制御用センサーを読まないため、前回の攻撃時の方向を持ち越さない。
    if (!wasAttacking) {
        lineSensor.resetFilter();
    }
    const int lineAngle = lineSensor.readAngle();
    const float lineVectorMagnitude = lineSensor.readVectorMagnitude();
    const float ownRear = camera.goalViewAngle(!ATTACK_GOAL_IS_YELLOW, true);
    const int gap = camera.rearGoalBottomGap(!ATTACK_GOAL_IS_YELLOW);
    // 一時的な白線離脱は、自ゴールが機体後方の近くに見え、向きが合っている場合に限る。
    const bool goalBehindNearby = ENABLE_CAMERA && camera.isReceiving() && ownRear >= 180 && ownRear < 360 &&
                                  abs(GoalHeading::angleDifference(ownRear - 270.0f)) <= 20.0f && gap >= 3 &&
                                  gap <= ATTACK_LINE_EXIT_MAX_GAP && ENABLE_GYRO_SENSOR &&
                                  frame.defenseHeadingValid && abs(defenseError) <= 20.0f;
    if (!wasAttacking)
        attackLineExit.begin(nowMs, lineAngle >= 0, goalBehindNearby);
    if (attackLineExit.update(nowMs, lineAngle >= 0, goalBehindNearby)) {
        offenseDirectionSmoother.reset();
        offenseStraightApproach.reset();
        // ゴールから離れる向きへ。白線を抜けた瞬間または時間切れで通常処理に戻る。
        motorController.moveDirection(90.0f, min(ATTACK_LINE_EXIT_POWER, SPEED), -1, defenseError, -1, 0);
        return true;
    }
    runOffense(frame.offenseBallAngle, frame.offenseError, lineAngle, lineVectorMagnitude);
    return true;
}

// 守備の優先順は、時限攻撃、カメラ通信断、ゴール探索、近接退避、通常の横追従。
void runDefense(const ControlFrame &frame) {
    const auto nowMs = frame.nowMs;
    const auto gyroAngle = frame.gyroAngle;
    const auto cameraBallAngle = frame.cameraBallAngle;
    const auto newCameraFrame = frame.newCameraFrame;
    const auto defenseHeadingValid = frame.defenseHeadingValid;
    const auto defenseError = frame.defenseError;
    const auto ballAngle = frame.ballAngle;
    const auto irBallAngle = frame.irBallAngle;
    if (runTimedAttack(frame))
        return;
    attackLineExit.reset();
    offenseDirectionSmoother.reset();
    offenseStraightApproach.reset();
    const float rearGoal = ENABLE_CAMERA ? camera.goalViewAngle(!ATTACK_GOAL_IS_YELLOW, true) : -1;
    const int bottomGap = ENABLE_CAMERA ? camera.rearGoalBottomGap(!ATTACK_GOAL_IS_YELLOW) : -1;
    if (newCameraFrame)
        goalEdgeState = goalEdgeGuard.update(bottomGap);
    if (!camera.isReceiving()) {
        blindReturnAlignment.reset();
        // USBではなく、カメラからのUARTが途切れたので走行を停止する。
        defense.retreat();
        motorController.stop(0);
        return;
    }
    static bool scanning = false;
    static uint32_t scanStart = 0;
    // 自ゴールが見えない間は、その場で0度へ整列してから後退する。
    if (rearGoal < 0) {
        defense.retreat();
        goalReturn.observe(-1, false);
        const float zeroError = GoalHeading::angleDifference(gyroAngle);
        if (!ENABLE_GYRO_SENSOR) {
            blindReturnAlignment.reset();
            scanning = false;
            motorController.stop(0);
        } else if (!blindReturnAlignment.update(gyroAngle, nowMs)) {
            scanning = false;
            motorController.holdHeading(zeroError, RETURN_TURN_MIN_POWER, RETURN_TURN_MAX_POWER,
                                        RETURN_TURN_KP);
        } else {
            if (!scanning) {
                scanning = true;
                scanStart = nowMs;
            }
            // 0度を中心に左右15度を交互に見ながら、0度基準の後方へ戻る。
            const float scanTarget = ((uint32_t)(nowMs - scanStart) / 1500) % 2 ? 15.0f : -15.0f;
            const float scanError = GoalHeading::angleDifference(gyroAngle - scanTarget);
            motorController.moveDirection(270.0f + zeroError, BLIND_RETURN_POWER, -1, scanError, -1, 0);
        }
        static uint32_t lastMissingLog = 0;
        if (!ENABLE_TELEMETRY && (uint32_t)(nowMs - lastMissingLog) >= 250) {
            lastMissingLog = nowMs;
            Serial.print(F("defense state=GOAL_MISSING yaw="));
            Serial.print(gyroAngle);
            Serial.print(F(" camera="));
            Serial.println(camera.isReceiving() ? 1 : 0);
        }
        return;
    }
    scanning = false;
    blindReturnAlignment.reset();
    // 下端が測れない場合はゴールまでの前後位置を決められないので並進しない。
    const bool edgeUnavailable = !goalEdgeGuard.calibrated() || bottomGap < 3 || bottomGap > 160 ||
                                 goalEdgeState == GoalEdgeGuard::State::UNKNOWN ||
                                 goalEdgeState == GoalEdgeGuard::State::UNCALIBRATED;
    DefenseInput input;
    input.now = nowMs;
    input.ballAngle = ballAngle;
    input.rearGoalAngle = rearGoal;
    input.rearGoalSpan = ENABLE_CAMERA ? camera.goalSpan(!ATTACK_GOAL_IS_YELLOW, true) : -1;
    input.yaw = gyroAngle;
    input.headingError = defenseError;
    input.headingValid = ENABLE_GYRO_SENSOR && defenseHeadingValid;
    // 近すぎる場合はボールへ横移動しながらゴールから離れる前進を加える。
    if (edgeUnavailable || goalEdgeState == GoalEdgeGuard::State::TOO_CLOSE) {
        defense.retreat();
        goalReturn.observe(-1, false);
        if (!edgeUnavailable && ENABLE_GYRO_SENSOR && defenseHeadingValid && abs(defenseError) <= 20.0f) {
            input.homePositionReady = false;
            const DefenseMove lateral = defense.update(input);
            float direction;
            int power;
            GoalReturn::escapeForward(lateral.direction, min(lateral.speed, SPEED), GOAL_EDGE_ESCAPE_POWER,
                                      min(DEFENSE_CONFIG.lateralSpeed, SPEED), direction, power);
            motorController.moveDirection(direction, power, -1, defenseError, -1, 0);
        } else {
            motorController.holdHeading(defenseError, RETURN_TURN_MIN_POWER, RETURN_TURN_MAX_POWER,
                                        RETURN_TURN_KP);
        }
        return;
    }
    const bool returnEligible =
        rearGoal >= 180 && rearGoal < 360 && input.headingValid && abs(defenseError) <= 20.0f;
    if (newCameraFrame)
        goalReturn.observe(bottomGap, returnEligible);
    input.homePositionReady = goalReturn.atHome(bottomGap);
    const DefenseMove command = defense.update(input);
    // ボールに合わせて横へ動きながら、離れすぎた分だけゴールへ戻る。
    float moveDirection = command.direction;
    int movePower = min(command.speed, SPEED);
    const bool returnTowardGoal = returnEligible && goalReturn.shouldReturn();
    GoalReturn::combine(moveDirection, movePower, returnTowardGoal, GOAL_RETURN_POWER, bottomGap,
                        GOAL_HOME_TARGET_GAP, min(DEFENSE_CONFIG.lateralSpeed, SPEED), moveDirection,
                        movePower);
    static uint32_t lastDefenseLog = 0;
    if (!ENABLE_TELEMETRY && (uint32_t)(nowMs - lastDefenseLog) >= 250) {
        lastDefenseLog = nowMs;
        Serial.print(F("defense state="));
        Serial.print(defense.stateName());
        Serial.print(F(" ownRear="));
        Serial.print(input.rearGoalAngle, 1);
        Serial.print(F(" span="));
        Serial.print(input.rearGoalSpan, 1);
        Serial.print(F(" ball="));
        Serial.print(ballAngle);
        Serial.print(F(" ir="));
        Serial.print(irBallAngle);
        Serial.print(F(" cameraBall="));
        Serial.print(cameraBallAngle);
        Serial.print(F(" frontPixels="));
        Serial.print(camera.ballView() == 1 ? camera.ballPixels() : 0, 0);
        Serial.print(F(" yaw="));
        Serial.print(gyroAngle);
        Serial.print(F(" headingValid="));
        Serial.print(defenseHeadingValid ? 1 : 0);
        Serial.print(F(" error="));
        Serial.print(defenseError, 1);
        Serial.print(F(" homeOffset="));
        Serial.print(defense.homeOffsetForLog(), 1);
        Serial.print(F(" lateralLimited="));
        Serial.print(defense.lateralLimitedForLog() ? 1 : 0);
        Serial.print(F(" homeReady="));
        Serial.print(input.homePositionReady ? 1 : 0);
        Serial.print(F(" attackWaitMs="));
        Serial.print(defense.attackWaitForLog(nowMs));
        Serial.print(F(" line="));
        Serial.print(frame.lineAngle);
        Serial.print(F(" bottomGap="));
        Serial.print(bottomGap);
        Serial.print(F(" return="));
        Serial.print(returnTowardGoal ? 1 : 0);
        Serial.print(F(" move="));
        Serial.print(moveDirection, 1);
        Serial.print(F(" power="));
        Serial.println(movePower);
    }
    if (movePower > 0) {
        motorController.moveDirection(moveDirection, movePower, -1, defenseError, -1, 0);
    } else {
        if (defense.state() == DefenseController::State::RETURN)
            motorController.holdHeading(defenseError, RETURN_TURN_MIN_POWER, RETURN_TURN_MAX_POWER,
                                        RETURN_TURN_KP);
        else
            motorController.holdHeading(defenseError, 20, 50, 0.8f);
    }
}

// 取得 → 状態表示 → 姿勢テスト/守備/攻撃の順に、停止せず繰り返す。
void loop() {
    flushTelemetry();
    const ControlFrame frame = readControlFrame();
    publishTelemetry(frame);
    if (HEADING_TEST_MODE) {
        runHeadingTest(frame);
    } else if (frame.defenseMode) {
        runDefense(frame);
    } else {
        runOffense(frame.offenseBallAngle, frame.offenseError, frame.lineAngle, frame.lineVectorMagnitude);
    }
}

// 最大64バイトのUART送信バッファが埋まっても制御ループを止めない。
static char telemetryBuffer[256];
static size_t telemetryLength = 0;
static size_t telemetryOffset = 0;

void flushTelemetry() {
    if (!ENABLE_TELEMETRY || telemetryOffset >= telemetryLength)
        return;
    const int available = Serial.availableForWrite();
    if (available <= 0)
        return;
    const size_t remaining = telemetryLength - telemetryOffset;
    const size_t count = remaining < (size_t)available ? remaining : (size_t)available;
    telemetryOffset += Serial.write((const uint8_t *)telemetryBuffer + telemetryOffset, count);
}

void queueTelemetry(const char *role, const char *state, int yaw, int ball, int ir, int cameraBall, int line,
                    int lineStrength, int yellow, int blue, int ownRear, int cameraOk, int error,
                    int bottomGap, int lineUsed, int attackYellow, int lineEnabled) {
    // 前の行が送信途中なら破棄せず、行が混ざらないよう新しいサンプルを見送る。
    if (telemetryOffset < telemetryLength)
        return;
    const int length = snprintf(
        telemetryBuffer, sizeof(telemetryBuffer),
        "telemetry role=%s state=%s yaw=%d ball=%d ir=%d cameraBall=%d line=%d lineStrength=%d yellow=%d "
        "blue=%d ownRear=%d camera=%d error=%d bottomGap=%d lineUsed=%d attackYellow=%d lineEnabled=%d\n",
        role, state, yaw, ball, ir, cameraBall, line, lineStrength, yellow, blue, ownRear, cameraOk, error,
        bottomGap, lineUsed, attackYellow, lineEnabled);
    if (length <= 0 || length >= (int)sizeof(telemetryBuffer))
        return;
    telemetryLength = (size_t)length;
    telemetryOffset = 0;
    flushTelemetry();
}

// ボール方位から移動方向を作り、相手ゴールへの姿勢を保ちながら攻撃する。
// ディフェンスからの一時攻撃も、この関数と通常OFFENSEを共有する。
void runOffense(int ballAngle, float headingError, int lineAngle, float lineVectorMagnitude) {
    float moveAngle = calculateWrapAroundAngle((float)ballAngle);
    if (moveAngle == -1) {
        offenseDirectionSmoother.reset();
        offenseStraightApproach.reset();
        // ボールを見失っても、白線上からの退避を優先する。
        if (lineAngle != -1) {
            motorController.moveDirection((float)(lineAngle + 180), 100, -1, 0.0f, lineAngle,
                                          lineVectorMagnitude);
        } else {
            motorController.stop(10);
        }
        return;
    }
    // 白線上では通常の退避・停止を最優先し、古い移動指令を残さない。
    if (lineAngle != -1) {
        offenseDirectionSmoother.reset();
        offenseStraightApproach.reset();
    } else {
        moveAngle = offenseStraightApproach.target(ballAngle, moveAngle);
        moveAngle = offenseDirectionSmoother.update(moveAngle, millis());
    }
    // 直進・回り込みの指令に、ゴールへの旋回と白線処理を車輪側で重ねる。
    motorController.moveDirection(moveAngle, SPEED, ballAngle, headingError, lineAngle, lineVectorMagnitude);
}

float calculateWrapAroundAngle(float ballAngle) {
    if (ballAngle < 0.0f || ballAngle > 360.0f) {
        return -1.0f;
    }

    // 入力は機体から見たボールの方角、出力は車体の移動方向。
    // 90度は前進。各点の間は円周上の短い側でつなぎ、0度をまたいでも急反転しない。
    static constexpr float ballBearings[] = {0, 60, 95, 130, 170, 210, 252.5f, 302.5f, 360};
    static constexpr float moveBearings[] = {315, 345, 90, 225, 250, 300, 0, 225, 315};
    unsigned int segment = 0;
    while (segment < 7 && ballAngle >= ballBearings[segment + 1])
        ++segment;
    const float ratio =
        (ballAngle - ballBearings[segment]) / (ballBearings[segment + 1] - ballBearings[segment]);
    const float delta = GoalHeading::angleDifference(moveBearings[segment + 1] - moveBearings[segment]);
    const float direction = moveBearings[segment] + delta * ratio;
    if (direction < 0)
        return direction + 360;
    if (direction >= 360)
        return direction - 360;
    return direction;
}
