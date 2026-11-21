#include <cassert>
#include <iostream>
#include <limits>
#include "Camera.h"
#include "GoalHeading.h"
#include "GoalEdgeGuard.h"
#include "GoalReturn.h"
#include "BlindReturnAlignment.h"
#include "AttackLineExit.h"
#include "OffenseDirectionSmoother.h"
#include "OffenseStraightApproach.h"
#include "MotorController.h"
#include "LineSensor.h"
#include "DefenseController.h"

uint32_t testMillis = 0;
int testPwm[32] = {};
int testDirection[32] = {};
int testAnalog[32] = {};
FakeSerial Serial1;
FakeSerial Serial;

static void near(float actual, float expected) {
    assert(std::abs(actual - expected) < 0.01f);
}

static void testHeading() {
    GoalHeading heading(1500, -1);
    assert(!heading.isValid(0));
    heading.observe(90, 359, 100);
    near(heading.error(1), 2);  // 359度から1度をまたぐ。
    heading.observe(120, 359, 200);
    near(heading.error(359), 30);  // 左のゴールへ左旋回。
    heading.observe(60, 1, 300);
    near(heading.error(1), -30);
    heading.observe(270, 10, 400);
    near(std::abs(heading.error(10)), 180);
    heading.observe(-1, 10, 1900);  // 未検出で期限を延ばさない。
    assert(heading.isValid(1900));
    assert(!heading.isValid(1901));
    heading.observe(90, 42, 2000);  // 再検出。
    near(heading.error(42), 0);
    const uint32_t last = std::numeric_limits<uint32_t>::max() - 100;
    heading.observe(90, 10, last);
    assert(heading.isValid(100));  // millis()の周回。
    assert(!heading.isValid(1500));
    GoalHeading held(1500, -1);
    held.observe(100, 20, 0);
    held.refresh(1400);
    near(held.error(20), 10);
    assert(held.isValid(2900));
    assert(!held.isValid(2901));
    assert(held.hasReference());
    held.refresh(3000); // 見失った後も直前の目標を再利用。
    near(held.error(20), 10);
    assert(held.isValid(3000));
    GoalHeading rear(1500, -1);
    rear.observe(270, 359, 0, 270);
    near(rear.error(1), 2);
    rear.observe(290, 10, 1, 270);
    near(rear.error(10), 20);
    rear.observe(250, 10, 2, 270);
    near(rear.error(10), -20);
    GoalHeading reversed(1500, 1);
    reversed.observe(120, 0, 0);
    near(reversed.error(0), -30);
}

static void testGoalDistance() {
    OffenseStraightApproach front;
    near(front.target(90, 75), 90);
    near(front.target(82, 50), 90);
    near(front.target(79, 40), 90); // 直進中は±12度まで維持。
    near(front.target(78, 40), 90);
    near(front.target(77, 40), 77.5f); // 外へ出たら回り込みへ徐々に移る。
    near(front.target(79, 40), 82.5f); // 再び±8度に入るまで直進へ戻らない。
    near(front.target(82, 40), 90);
    near(front.target(118, 150), 150);
    front.reset();
    near(front.target(100, 110), 92);
    OffenseDirectionSmoother smoother;
    near(smoother.update(350, 0), 350);
    near(smoother.update(10, 25), 2); // 360度境界は近い方向へ回る。
    near(smoother.update(10, 50), 10);
    near(smoother.update(100, 100), 34); // 50msでは24度まで。
    near(smoother.update(190, 500), 190); // 長時間中断後は履歴を使わない。
    smoother.reset();
    near(smoother.update(270, 501), 270);
    near(smoother.update(180, 551), 246);
    smoother.reset();
    near(smoother.update(80, UINT32_MAX - 10), 80);
    near(smoother.update(180, 9), 89.6f); // millisの桁あふれでも経過時間を使う。
    AttackLineExit lineExit;
    lineExit.begin(0, true, true);
    assert(lineExit.update(0, true, true));
    assert(lineExit.update(699, true, true));
    assert(!lineExit.update(700, true, true));
    assert(!lineExit.update(701, true, true)); // 同じ攻撃中は再開しない。
    lineExit.begin(1000, true, true);
    assert(!lineExit.update(1100, false, true));
    assert(!lineExit.update(1150, true, true));
    lineExit.begin(2000, true, true);
    assert(!lineExit.update(2100, true, false));
    lineExit.begin(3000, true, false);
    assert(!lineExit.update(3001, true, true));
    lineExit.begin(4000, false, true);
    assert(!lineExit.update(4001, true, true));
    lineExit.begin(UINT32_MAX - 300, true, true);
    assert(lineExit.update(100, true, true));
    assert(!lineExit.update(400, true, true));
    BlindReturnAlignment alignment;
    assert(!alignment.update(30, 0));
    assert(!alignment.update(359, 10));
    assert(!alignment.update(4, 209));
    assert(!alignment.update(6, 210)); // 許容範囲を出たら計り直す。
    assert(!alignment.update(355, 220));
    assert(!alignment.update(0, 419));
    assert(alignment.update(1, 420));
    assert(alignment.update(15, 500)); // 整列後は左右確認を継続する。
    alignment.reset();
    assert(!alignment.update(15, 600)); // 再検出後の喪失時は再整列。
    GoalEdgeGuard guard(60, 66);
    using Edge = GoalEdgeGuard::State;
    assert(guard.update(90) == Edge::SAFE);
    assert(guard.update(60) == Edge::SAFE);
    assert(guard.update(61) == Edge::SAFE); // 一瞬の検出は補正しない。
    assert(guard.update(60) == Edge::SAFE);
    assert(guard.update(60) == Edge::SAFE);
    assert(guard.update(60) == Edge::TOO_CLOSE);
    assert(guard.update(66) == Edge::TOO_CLOSE);
    assert(guard.update(66) == Edge::SAFE);
    assert(guard.update(-1) == Edge::UNKNOWN);
    GoalReturn back(70, 6);
    back.observe(77, true); back.observe(77, true);
    assert(!back.shouldReturn());
    back.observe(77, true); assert(back.shouldReturn());
    back.observe(70, true); assert(!back.shouldReturn());
    back.observe(-1, false); assert(!back.shouldReturn());
    float y = -100;
    GoalReturn::prioritizeLateral(-160, y); near(y, -40);
    y = -100;
    GoalReturn::prioritizeLateral(160, y); near(y, -40);
    y = -80;
    GoalReturn::prioritizeLateral(0, y); near(y, -80);
    y = 40;
    GoalReturn::prioritizeLateral(-160, y); near(y, 40);
    float direction; int speed;
    near(GoalReturn::returnRatio(66, 66), 0.5f);
    near(GoalReturn::returnRatio(110, 66), 0.7f);
    GoalReturn::combine(0, 230, true, 180, 110, 66, 230, direction, speed);
    assert(direction > 324 && direction < 326 && speed == 230);
    // 車輪正規化後も後退と横移動の両方を確保する。
    const float rad = direction * 0.01745329252f;
    const float total = fabsf(cosf(rad)) + fabsf(sinf(rad));
    assert(speed * cosf(rad) / total > 130);
    assert(-speed * sinf(rad) / total > 90);
    GoalReturn::combine(180, 230, true, 180, 110, 66, 230, direction, speed);
    assert(direction > 214 && direction < 216 && speed == 230);
    GoalReturn::combine(0, 230, false, 180, 110, 66, 230, direction, speed);
    near(direction, 0); assert(speed == 230);
    GoalReturn::combine(90, 0, true, 180, 110, 66, 230, direction, speed);
    near(direction, 270); assert(speed == 180);
    GoalReturn::combine(90, 0, false, 180, 110, 66, 230, direction, speed);
    assert(speed == 0);
    GoalReturn::escapeForward(0, 230, 180, 230, direction, speed);
    assert(direction > 37 && direction < 39 && speed == 230);
    GoalReturn::escapeForward(180, 230, 180, 230, direction, speed);
    assert(direction > 141 && direction < 143 && speed == 230);
    GoalReturn::escapeForward(90, 0, 180, 230, direction, speed);
    near(direction, 90); assert(speed == 180);
}

static void testCamera() {
    Camera camera;
    camera.setup();
    assert(Serial1.baud == 115200);
    assert(camera.receivedBytes() == 0);
    assert(camera.validFrames() == 0);
    assert(camera.invalidFrames() == 0);
    near(camera.ballAngle(), -1);
    Serial1.input = "V1,87.4,120.5,-1\n";
    assert(camera.update());
    near(camera.ballAngle(), 87.4);
    near(camera.goalAngle(true), 120.5);
    near(camera.goalAngle(false), -1);
    assert(camera.receivedBytes() == std::string("V1,87.4,120.5,-1\n").size());
    assert(camera.validFrames() == 1);
    assert(camera.invalidFrames() == 0);
    assert(std::string(camera.lastReceivedLine()) == "V1,87.4,120.5,-1");
    assert(!camera.update());
    Serial1.input = "V1,1,90,270\nV1,2,-1,280\n";
    assert(camera.update());
    near(camera.ballAngle(), 2);  // 古い行ではなく最後の行。
    near(camera.goalAngle(true), -1);
    near(camera.goalAngle(false), 280);
    Serial1.input = "V1,3,95";
    assert(!camera.update());
    Serial1.input = ",275\r\n";
    assert(camera.update());
    near(camera.ballAngle(), 3);
    for (const char* invalid : {"junk\n", "90\n", "V1,x,90,270\n",
             "V1,4,nan,270\n", "V1,4,inf,270\n", "V1,4,360,270\n",
             "V1,4,-2,270\n", "V1,4,,270\n", "V1,4,90,270,1\n"}) {
        Serial1.input = invalid;
        const uint32_t before = camera.invalidFrames();
        assert(!camera.update());
        assert(camera.invalidFrames() == before + 1);
        near(camera.ballAngle(), 3);
    }
    testMillis += 201;
    near(camera.ballAngle(), -1);
    near(camera.goalAngle(false), -1);
    Serial1.input = std::string(200, 'a') + "V1,4,90,270\nV1,-1,90,270\n";
    assert(camera.update());  // 長すぎる行を破棄して次の行で復帰。
    near(camera.ballAngle(), -1);
    near(camera.goalAngle(true), 90);
    Serial1.input = "broken\x1b[31m\n";
    assert(!camera.update());
    assert(std::string(camera.lastReceivedLine()) == "broken.[31m");

    GoalHeading heading(1500, -1);
    heading.observe(camera.goalAngle(false), 0, testMillis);
    near(std::abs(heading.error(0)), 180);  // 青ゴールを選択。
    const uint32_t seen = testMillis;
    Serial1.input = "V1,90,-1,-1\n";
    assert(camera.update());
    heading.observe(camera.goalAngle(false), 0, seen + 1000);
    assert(!heading.isValid(seen + 1501));
}

static void testCameraV2() {
    Camera camera;
    camera.setup();
    Serial1.input = "V2,91,400,1,90,40,270,20,-1,-1,275,45\n";
    assert(camera.update());
    near(camera.ballPixels(), 400);
    assert(camera.ballView() == 1);
    near(camera.goalViewAngle(true, true), 270);
    near(camera.goalSpan(false, true), 45);
    near(camera.goalAngle(true), 90);
    for (const char* invalid : {
        "V2,91,400,3,90,40,270,20,-1,-1,275,45\n",
        "V2,-1,400,1,90,40,270,20,-1,-1,275,45\n",
        "V2,91,400,1,90,40,270,20,-1,20,275,45\n",
        "V2,91,400,1,90,nan,270,20,-1,-1,275,45\n",
        "V2,91,400,1,90,40,270,20,-1,-1,275,45,0\n"}) {
        Serial1.input = invalid;
        assert(!camera.update());
        near(camera.goalSpan(false, true), 45);
    }
    testMillis += 201;
    near(camera.goalViewAngle(false, true), -1);
    near(camera.goalSpan(false, true), -1);
    assert(camera.ballView() == 0);
    Serial1.input = "V1,90,90,270\n";
    assert(camera.update());
    near(camera.goalSpan(false, true), -1);
    near(camera.goalViewAngle(false, true), -1);
}

static void testCameraV3() {
    Camera camera;
    camera.setup();
    Serial1.input = "V3,91,400,1,90,40,270,-1,-1,-1,275,45,116,90\n";
    assert(camera.update());
    assert(camera.rearGoalBottomGap(true) == 116);
    assert(camera.rearGoalBottomGap(false) == 90);
    for (const char* invalid : {
        "V3,91,400,1,90,40,270,-1,-1,-1,275,45,161,90\n",
        "V3,91,400,1,90,40,-1,-1,-1,-1,275,45,116,90\n",
        "V3,91,400,1,90,40,270,-1,-1,-1,275,45,3.5,90\n"}) {
        Serial1.input = invalid;
        assert(!camera.update());
        assert(camera.rearGoalBottomGap(true) == 116);
    }
    testMillis += 201;
    assert(camera.rearGoalBottomGap(true) == -1);
    Serial1.input = "V2,91,400,1,90,40,270,-1,-1,-1,275,45\n";
    assert(camera.update());
    assert(camera.rearGoalBottomGap(true) == -1);
}

static void testDefense() {
    DefenseConfig config;
    DefenseInput in;
    in.rearGoalAngle = 270;
    in.headingValid = true;
    DefenseController keeper(config);
    in.ballAngle = 120;
    DefenseMove move = keeper.update(in);
    near(move.direction, 180); assert(move.speed >= config.lateralMinSpeed && !move.attack);
    in.ballAngle = 60;
    move = keeper.update(in);
    near(move.direction, 0); assert(move.speed > 0);
    in.ballAngle = 197;
    move = keeper.update(in);
    assert(move.direction > 180 && move.direction < 270 && move.speed > 0);
    in.ballAngle = 220;
    move = keeper.update(in);
    assert(move.direction > 180 && move.direction < 270 && move.speed > 0);
    in.ballAngle = 270;
    move = keeper.update(in);
    assert(move.direction > 180 && move.direction < 270 && move.speed > 0);
    in.ballAngle = 275; // 真後ろ付近では直前の左側を保持。
    move = keeper.update(in);
    assert(move.direction > 180 && move.direction < 270 && move.speed > 0);
    in.ballAngle = 343;
    move = keeper.update(in);
    assert(move.direction > 270 && move.direction < 360 && move.speed > 0);
    in.ballAngle = 270;
    move = keeper.update(in);
    assert(move.direction > 270 && move.direction < 360 && move.speed > 0);
    in.ballAngle = 90;
    assert(keeper.update(in).speed == 0);
    in.ballAngle = 93;
    assert(keeper.update(in).speed == 0); // 停止中は4度まで動かない。
    in.ballAngle = 94;
    assert(keeper.update(in).speed >= config.lateralMinSpeed);
    in.ballAngle = 93;
    assert(keeper.update(in).speed >= config.lateralMinSpeed); // 追従中は2度まで維持。
    in.ballAngle = 92;
    assert(keeper.update(in).speed == 0);
    in.ballAngle = 110;
    assert(keeper.update(in).speed == config.lateralSpeed);
    in.rearGoalAngle = -1;
    assert(keeper.update(in).speed == 0);
    assert(keeper.state() == DefenseController::State::RETURN);
    in.rearGoalAngle = 270; in.ballAngle = 60; in.now = 100;
    move = keeper.update(in);
    assert(move.direction == 0 && move.speed >= config.lateralMinSpeed);

    DefenseController challenger(config);
    in.rearGoalAngle = 270;
    in.ballAngle = 90;
    in.now = 0; challenger.observeFrontBall(in.now, true); assert(!challenger.update(in).attack);
    in.now = config.frontWaitMs - 1; challenger.observeFrontBall(in.now, true); assert(!challenger.update(in).attack);
    in.now = config.frontWaitMs; challenger.observeFrontBall(in.now, true); assert(challenger.update(in).attack);
    in.ballAngle = -1;
    in.rearGoalAngle = -1;
    in.now += 401; assert(challenger.update(in).attack);
    in.now = config.frontWaitMs + config.attackMaxMs - 1;
    assert(challenger.update(in).attack);
    in.now = config.frontWaitMs + config.attackMaxMs;
    assert(!challenger.update(in).attack);
    assert(challenger.state() == DefenseController::State::RETURN);

    DefenseController boundary(config);
    DefenseInput edge;
    edge.rearGoalAngle = 270;
    edge.headingValid = true;
    edge.ballAngle = 120;
    assert(boundary.update(edge).speed > 0); // 初期方向を記憶。
    edge.yaw = -40;
    assert(boundary.update(edge).speed == 0);
    assert(boundary.lateralLimitedForLog());
    near(boundary.homeOffsetForLog(), -40);
    edge.ballAngle = 60;
    assert(boundary.update(edge).speed > 0); // 反対向きには戻れる。
    assert(!boundary.lateralLimitedForLog());

    // 復帰中でも前方カメラの連続検出だけで攻撃へ移る。
    DefenseController recovered(config);
    DefenseInput ready;
    ready.rearGoalAngle = 270;
    ready.headingValid = true;
    ready.ballAngle = 110; // 新しい±20度の境界。
    recovered.update(ready);
    recovered.retreat();
    ready.yaw = 10;
    ready.now = 100; recovered.update(ready);
    ready.now = 600; recovered.update(ready);
    assert(recovered.state() == DefenseController::State::GUARD);
    ready.now = 601; recovered.observeFrontBall(ready.now, true); assert(!recovered.update(ready).attack);
    ready.now = 5600; recovered.observeFrontBall(ready.now, true); assert(!recovered.update(ready).attack);
    ready.now = 5601; recovered.observeFrontBall(ready.now, true); assert(recovered.update(ready).attack);

    // ゴール・姿勢・IRの値は開始条件に含めない。
    DefenseController independent(config);
    independent.retreat();
    independent.observeFrontBall(0, true);
    independent.retreat(); // ゴール未検出時の処理でも計時を継続する。
    independent.observeFrontBall(5000, true);
    assert(independent.state() == DefenseController::State::ATTACK);
    DefenseController lost(config);
    lost.observeFrontBall(0, true);
    lost.observeFrontBall(4900, true);
    lost.observeFrontBall(4950, false);
    lost.observeFrontBall(5000, true);
    lost.observeFrontBall(9999, true);
    assert(lost.state() != DefenseController::State::ATTACK);
    lost.observeFrontBall(10000, true);
    assert(lost.state() == DefenseController::State::ATTACK);
}

static void testLineAngleWrap() {
    const int thresholds[8] = {500, 500, 500, 500, 500, 500, 500, 500};
    LineSensor line(0, 8, thresholds);
    for (int i = 0; i < 8; ++i) testAnalog[i] = 0;

    testAnalog[7] = 900; // 315度から0度へ変化。
    assert(line.readAngle() >= 314);
    testAnalog[7] = 0;
    testAnalog[0] = 900;
    const int forward = line.readAngle();
    assert(forward >= 345 && forward <= 348);

    testAnalog[0] = 0;
    assert(line.readAngle() == -1); // 未検出で履歴をリセット。
    testAnalog[0] = 900;
    assert(line.readAngle() == 0);
    testAnalog[0] = 0;
    testAnalog[7] = 900; // 0度から315度へ変化。
    const int reverse = line.readAngle();
    assert(reverse >= 327 && reverse <= 329);

    for (int i = 0; i < 8; ++i) testAnalog[i] = 0;
    assert(line.readAngle() == -1);
    testAnalog[0] = testAnalog[7] = 900; // 337.5度から22.5度へ。
    const int leftOfZero = line.readAngle();
    assert(leftOfZero >= 337 && leftOfZero <= 338);
    testAnalog[7] = 0;
    testAnalog[1] = 900;
    const int acrossZero = line.readAngle();
    assert(acrossZero >= 8 && acrossZero <= 10);

    for (int i = 0; i < 8; ++i) testAnalog[i] = 0;
    assert(line.readAngle() == -1);
    testAnalog[0] = testAnalog[1] = 900; // 22.5度から337.5度へ。
    const int rightOfZero = line.readAngle();
    assert(rightOfZero >= 22 && rightOfZero <= 23);
    testAnalog[1] = 0;
    testAnalog[7] = 900;
    const int backAcrossZero = line.readAngle();
    assert(backAcrossZero >= 350 && backAcrossZero <= 352);
    // 守備中に場所が変わったとき、前回と反対側でも再計測できる。
    line.resetFilter();
    for (int i = 0; i < 8; ++i) testAnalog[i] = 0;
    testAnalog[4] = 900;
    assert(line.readAngle() == 180);
    for (int i = 0; i < 8; ++i) testAnalog[i] = 0;
}

static void testMotors() {
    Motor a(1, 2), b(3, 4), c(5, 6), d(7, 8);
    MotorController motors(&a, &b, &c, &d);
    assert(motors.holdHeading(30, 60, 100, 2.0f) == 60);
    for (int pin : {1, 3, 5, 7}) assert(testDirection[pin] == HIGH);
    for (int pin : {2, 4, 6, 8}) assert(testPwm[pin] == 60);
    assert(motors.holdHeading(-90, 60, 100, 2.0f) == -100);
    for (int pin : {1, 3, 5, 7}) assert(testDirection[pin] == LOW);
    for (int pin : {2, 4, 6, 8}) assert(testPwm[pin] == 100);
    assert(motors.holdHeading(3, 60, 100, 2.0f) == 0);
    for (int pin : {2, 4, 6, 8}) assert(testPwm[pin] == 0);
    assert(motors.holdHeading(350, 60, 100, 2.0f) == -60);
    assert(motors.holdHeading(180, 300, 500, 3.0f) == 255);
    // 姿勢検証ではボール角なし・移動速度0でも旋回する。
    motors.moveDirection(90.0f, 0, -1, 30.0f, -1, 0);
    for (int pin : {1, 3, 5, 7}) assert(testDirection[pin] == HIGH);
    for (int pin : {2, 4, 6, 8}) assert(testPwm[pin] == 24);
    motors.moveDirection(90.0f, 0, -1, -30.0f, -1, 0);
    for (int pin : {1, 3, 5, 7}) assert(testDirection[pin] == LOW);
    for (int pin : {2, 4, 6, 8}) assert(testPwm[pin] == 24);
    motors.moveDirection(90.0f, 0, -1, 0.0f, -1, 0);
    for (int pin : {2, 4, 6, 8}) assert(testPwm[pin] == 0);
    motors.moveDirection(90.0f, 230, 90, 180.0f, -1, 0);
    for (int pin : {1, 3, 5, 7}) assert(testDirection[pin] == HIGH);
    for (int pin : {2, 4, 6, 8}) assert(testPwm[pin] == 50);
    motors.moveDirection(90.0f, 230, 90, -180.0f, -1, 0);
    for (int pin : {1, 3, 5, 7}) assert(testDirection[pin] == LOW);
    for (int pin : {2, 4, 6, 8}) assert(testPwm[pin] == 50);
    motors.moveDirection(90.0f, 230, 90, 0.0f, -1, 0);
    const int base = testPwm[2];
    motors.moveDirection(90.0f, 230, 90, 2.5f, -1, 0);
    assert(testPwm[2] == base);  // 正面付近の不感帯。
    for (int direction = 0; direction < 360; direction += 5) {
        for (float error : {-89.0f, -30.0f, 0.0f, 30.0f, 89.0f}) {
            motors.moveDirection((float)direction, 255, 90, error, -1, 0);
            for (int pin : {2, 4, 6, 8}) assert(testPwm[pin] <= 255);
        }
    }
    const uint32_t before = testMillis;
    motors.moveDirection(180.0f, 100, 359, 0.0f, 1, 1);
    for (int pin : {2, 4, 6, 8}) assert(testPwm[pin] == 0);
    assert(testMillis == before);  // 白線処理で通信を止めない。
    motors.moveDirection(270.0f, 100, -1, 0.0f, 90, 0.5f);
    assert(testPwm[2] > 0);  // ボール未検出でも退避。
    assert(testMillis == before);
}

int main() {
    testHeading();
    testGoalDistance();
    testCamera();
    testCameraV2();
    testCameraV3();
    testDefense();
    testLineAngleWrap();
    testMotors();
    std::cout << "Goal heading, UART and motor tests passed\n";
}
