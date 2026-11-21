#include "GoalHeading.h"

GoalHeading::GoalHeading(uint32_t holdMs, float cameraToYawSign)
    : holdMs(holdMs), cameraToYawSign(cameraToYawSign), hasTarget(false), targetYaw(0), lastSeenMs(0) {}

float GoalHeading::angleDifference(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void GoalHeading::observe(float goalAngle, float currentYaw, uint32_t now, float referenceBearing) {
    if (!(goalAngle >= 0.0f && goalAngle < 360.0f)) return;
    // カメラから見たずれを現在のジャイロ方位に足し、目標方位として保持する。
    const float offset = angleDifference(goalAngle - referenceBearing);
    targetYaw = angleDifference(currentYaw + cameraToYawSign * offset);
    lastSeenMs = now;
    hasTarget = true;
}

bool GoalHeading::hasReference() const { return hasTarget; }

void GoalHeading::refresh(uint32_t now) {
    if (hasTarget) lastSeenMs = now;
}

bool GoalHeading::isValid(uint32_t now) const {
    return hasTarget && (uint32_t)(now - lastSeenMs) <= holdMs;
}

float GoalHeading::error(float currentYaw) const {
    return angleDifference(currentYaw - targetYaw);
}
