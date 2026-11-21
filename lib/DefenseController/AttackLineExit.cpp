#include "AttackLineExit.h"
#include "DefenseConfig.h"

void AttackLineExit::begin(uint32_t now, bool onLine, bool goalBehindNearby) {
    active = onLine && goalBehindNearby;
    start = now;
}

bool AttackLineExit::update(uint32_t now, bool onLine, bool goalBehindNearby) {
    if (active && (!onLine || !goalBehindNearby || (uint32_t)(now - start) >= MotionTuning::ATTACK_LINE_EXIT_MS))
        active = false;
    return active;
}

void AttackLineExit::reset() { active = false; }
