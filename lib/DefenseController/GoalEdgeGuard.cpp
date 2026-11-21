#include "GoalEdgeGuard.h"

GoalEdgeGuard::GoalEdgeGuard(int nearGap, int clearGap)
    : nearGap(nearGap), clearGap(clearGap), tooClose(false), nearFrames(0), clearFrames(0) {}

bool GoalEdgeGuard::calibrated() const {
    return nearGap >= 3 && nearGap <= 160 && clearGap >= 3 && clearGap <= 160 &&
           (nearGap - clearGap >= 3 || clearGap - nearGap >= 3);
}

GoalEdgeGuard::State GoalEdgeGuard::update(int gap) {
    if (!calibrated())
        return State::UNCALIBRATED;
    if (gap < 3 || gap > 160) {
        nearFrames = clearFrames = 0;
        return State::UNKNOWN;
    }
    const bool closerIsSmaller = nearGap < clearGap;
    // カメラの取り付け方によって、接近時に画素値が増える場合と減る場合に対応する。
    const bool near = closerIsSmaller ? gap <= nearGap : gap >= nearGap;
    const bool clear = closerIsSmaller ? gap >= clearGap : gap <= clearGap;
    if (near) {
        // 単発の色検出ノイズで前進しないよう、接近を3フレーム連続で確認する。
        if (nearFrames < 3)
            ++nearFrames;
        if (nearFrames >= 3)
            tooClose = true;
        clearFrames = 0;
    } else if (tooClose && clear) {
        // 少し離れた状態が2フレーム続くと通常の守備へ戻す。
        nearFrames = 0;
        if (++clearFrames >= 2) {
            tooClose = false;
            clearFrames = 0;
        }
    } else {
        nearFrames = clearFrames = 0;
    }
    return tooClose ? State::TOO_CLOSE : State::SAFE;
}
