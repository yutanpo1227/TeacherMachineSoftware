#include "GoalReturn.h"

GoalReturn::GoalReturn(int targetGap, int startMargin)
    : targetGap(targetGap), startMargin(startMargin), returning(false), farFrames(0) {}

void GoalReturn::observe(int gap, bool valid) {
    if (!valid || gap < 3 || gap > 160) {
        returning = false;
        farFrames = 0;
        return;
    }
    if (gap <= targetGap) {
        returning = false;
        farFrames = 0;
    } else if (gap > targetGap + startMargin) {
        if (farFrames < 3)
            ++farFrames;
        if (farFrames >= 3)
            returning = true;
    } else {
        // 復帰開始前は3フレーム連続を要求する。復帰中は目標まで進む。
        farFrames = 0;
    }
}

bool GoalReturn::shouldReturn() const { return returning; }

bool GoalReturn::atHome(int gap) const { return gap >= 3 && gap <= targetGap + startMargin && !returning; }

void GoalReturn::prioritizeLateral(float x, float &y, float maxRearRatio) {
    const float lateral = fabsf(x);
    if (lateral >= 80.0f && y < -lateral * maxRearRatio)
        y = -lateral * maxRearRatio;
}

float GoalReturn::returnRatio(int gap, int targetGap) {
    const float progress = (gap - targetGap) / 44.0f;
    if (progress <= 0)
        return 0.5f;
    if (progress >= 1)
        return 0.7f;
    return 0.5f + progress * 0.2f;
}

void GoalReturn::combine(float direction, int speed, bool returning, int returnPower, int gap, int targetGap,
                         int limit, float &resultDirection, int &resultSpeed) {
    // 右を+x、前を+yとする。ゴールへの後退は-yとして横追従に重ねる。
    const float radians = 0.01745329252f;
    const float x = cosf(direction * radians) * speed;
    float y = sinf(direction * radians) * speed;
    if (returning)
        y -= returnPower;
    prioritizeLateral(x, y, returning ? returnRatio(gap, targetGap) : 0.25f);
    // 斜め移動で必要な車輪の最大出力を求め、全車輪の上限を超えないようにする。
    const float wheelDemand = fabsf(x) + fabsf(y);
    if (wheelDemand < 0.01f) {
        resultDirection = direction;
        resultSpeed = 0;
        return;
    }
    resultDirection = atan2f(y, x) * 57.295779513f;
    if (resultDirection < 0)
        resultDirection += 360;
    resultSpeed = (int)(wheelDemand > limit ? limit : wheelDemand);
}

void GoalReturn::escapeForward(float direction, int speed, int forwardPower, int limit,
                               float &resultDirection, int &resultSpeed) {
    const float x = cosf(direction * 0.01745329252f) * speed;
    const float y = forwardPower;
    const float wheelDemand = fabsf(x) + y;
    if (wheelDemand < 0.01f) {
        resultSpeed = 0;
        resultDirection = direction;
        return;
    }
    resultDirection = atan2f(y, x) * 57.295779513f;
    resultSpeed = (int)(wheelDemand > limit ? limit : wheelDemand);
}
