#include "OffenseStraightApproach.h"
#include "DefenseConfig.h"

float OffenseStraightApproach::target(float ballAngle, float wrapAngle) {
    const float offset = fabsf(GoalHeading::angleDifference(ballAngle - 90.0f));
    if (straight) {
        if (offset > MotionTuning::OFFENSE_STRAIGHT_EXIT_DEG)
            straight = false;
    } else if (offset <= MotionTuning::OFFENSE_STRAIGHT_ENTER_DEG)
        straight = true;
    if (straight)
        return 90.0f;
    if (offset >= MotionTuning::OFFENSE_BLEND_END_DEG)
        return wrapAngle;
    // ±8〜28度で従来の回り込み方向へ滑らかにつなぐ。
    const float ratio = (offset - MotionTuning::OFFENSE_STRAIGHT_ENTER_DEG) /
                        (MotionTuning::OFFENSE_BLEND_END_DEG - MotionTuning::OFFENSE_STRAIGHT_ENTER_DEG);
    float result = 90.0f + GoalHeading::angleDifference(wrapAngle - 90.0f) * ratio;
    if (result < 0)
        result += 360.0f;
    if (result >= 360)
        result -= 360.0f;
    return result;
}

void OffenseStraightApproach::reset() { straight = false; }
