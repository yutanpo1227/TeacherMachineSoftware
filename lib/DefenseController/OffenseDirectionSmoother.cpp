#include "OffenseDirectionSmoother.h"
#include "DefenseConfig.h"

float OffenseDirectionSmoother::update(float target, uint32_t now) {
    if (!valid || (uint32_t)(now - lastMs) > MotionTuning::OFFENSE_DIRECTION_RESET_MS) {
        direction = normalize(target);
        lastMs = now;
        valid = true;
        return direction;
    }
    // 経過時間が短いループでは変化量も小さくし、処理周期による差を減らす。
    const float limit = MotionTuning::OFFENSE_DIRECTION_RATE_DEG_PER_SEC * (uint32_t)(now - lastMs) / 1000.0f;
    float change = GoalHeading::angleDifference(target - direction);
    if (change > limit)
        change = limit;
    if (change < -limit)
        change = -limit;
    direction = normalize(direction + change);
    lastMs = now;
    return direction;
}

void OffenseDirectionSmoother::reset() { valid = false; }

float OffenseDirectionSmoother::normalize(float value) {
    while (value < 0)
        value += 360;
    while (value >= 360)
        value -= 360;
    return value;
}
