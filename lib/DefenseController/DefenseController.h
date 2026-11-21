#ifndef DEFENSE_CONTROLLER_H
#define DEFENSE_CONTROLLER_H

#include <stdint.h>
#include <math.h>
#include "GoalHeading.h"

#include "DefenseConfig.h"

struct DefenseInput {
    uint32_t now = 0;
    // 機体から見た方向。0度が右、90度が前、270度が後ろ、-1が未検出。
    float ballAngle = -1;
    float rearGoalAngle = -1; // 後方ミラーで見えた自ゴールの方向。-1は未検出。
    float rearGoalSpan = -1; // 自ゴールの見かけの幅。-1は使用できない。
    float yaw = 0; // ジャイロの方位。ボール角とは別の基準。
    float headingError = 0; // 自ゴールを後ろへ向ける目標方位との差。
    bool headingValid = false; // 目標方位が有効ならtrue。
    bool homePositionReady = true; // 下端から見た前後位置が待機範囲か。
};

struct DefenseMove {
    // 車輪への並進指令。旋回の補正はモーター側で合成する。
    float direction = 90;
    int speed = 0;
    bool attack = false;
};

class DefenseController {
public:
    // GUARDは守備、ATTACKは一時攻撃、RETURNは守備位置への復帰。
    enum class State { GUARD, ATTACK, RETURN };
    explicit DefenseController(const DefenseConfig& config);
    State state() const;
    float homeOffsetForLog() const;
    bool lateralLimitedForLog() const;
    uint32_t attackWaitForLog(uint32_t now) const; // 前方ボールの連続検出時間。単位ms。
    // 毎ループ呼ぶ。前方カメラで一定時間見え続けたらATTACKへ移り、見失えば計時をやり直す。
    void observeFrontBall(uint32_t now, bool detected);
    const char* stateName() const;
    // 復帰へ移り、待機位置に戻って安定したかを数え直す。
    // 前方カメラの連続検出は observeFrontBall() が独立に管理する。
    void retreat();

    // 現在の観測から次の並進方向とPWMを返す。attack=trueのときは呼び出し元が攻撃処理を実行する。
    DefenseMove update(const DefenseInput& in);
private:
    float limitLateral(float power, float homeOffset);

    float lateralPower(float ballAngle) const;
    static bool elapsed(bool condition, bool& timing, uint32_t& start,
                        uint32_t now, uint32_t duration);
    DefenseConfig config;
    State currentState = State::GUARD;
    bool hasHomeYaw = false;
    bool lateralLimited = false;
    bool lateralTracking = false;
    int8_t ballSide = 1;
    float lastHomeOffset = 0;
    bool hasRange = false, rangeBlocked = false;
    float lastRangeSpan = 0;
    uint32_t lastRangeMs = 0;
    bool visibleTiming = false, stableTiming = false;
    float homeYaw = 0;
    uint32_t visibleStart = 0, stableStart = 0;
    uint32_t attackStart = 0;
};
#endif
