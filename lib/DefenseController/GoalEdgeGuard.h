#ifndef GOAL_EDGE_GUARD_H
#define GOAL_EDGE_GUARD_H

// 実機で測定した2つの画素値で、ゴール接近を判定する。
class GoalEdgeGuard {
public:
    // UNKNOWNは画素値が取得できない状態、TOO_CLOSEはゴールから前へ離れる必要がある状態。
    enum class State { UNCALIBRATED, UNKNOWN, SAFE, TOO_CLOSE };
    // nearGapとclearGapは、実機を所定位置に置いて得た画像上の画素値。
    GoalEdgeGuard(int nearGap, int clearGap);
    bool calibrated() const;
    // 新しいカメラフレームごとに呼ぶ。接近は3フレーム、解除は2フレームの連続判定。
    State update(int gap);
private:
    int nearGap, clearGap;
    bool tooClose;
    int nearFrames;
    int clearFrames;
};
#endif
