#include "BlindReturnAlignment.h"
#include "DefenseConfig.h"

bool BlindReturnAlignment::update(float yaw, uint32_t now) {
    if (aligned)
        return true;
    if (fabsf(GoalHeading::angleDifference(yaw)) > MotionTuning::BLIND_ALIGN_TOLERANCE_DEG) {
        timing = false;
        return false;
    }
    if (!timing) {
        timing = true;
        start = now;
        return false;
    }
    if ((uint32_t)(now - start) < MotionTuning::BLIND_ALIGN_STABLE_MS)
        return false;
    aligned = true;
    return true;
}

void BlindReturnAlignment::reset() {
    aligned = false;
    timing = false;
}
