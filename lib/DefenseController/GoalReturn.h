#ifndef GOAL_RETURN_H
#define GOAL_RETURN_H

#include <math.h>

// ゴール下端の位置が、校正した待機位置から離れた場合の復帰を管理する。
class GoalReturn {
public:
    // targetGapは目標位置、startMarginは後退を始めるまで許容する画素差。
    GoalReturn(int targetGap, int startMargin);
    // 新しいカメラフレームごとに呼ぶ。無効値なら古い距離に基づく後退を取り消す。
    void observe(int gap, bool valid);
    // 目標より離れた値が3フレーム続くとtrueになり、目標位置まで後退を継続する。
    bool shouldReturn() const;
    // 復帰動作中でなく、画素値が待機範囲にあればtrue。
    bool atHome(int gap) const;
    // 横へボールを追うときは後退を弱め、斜め移動で車輪出力が
    // 極端に小さくなるのを抑える。横移動が小さいときは後退を優先する。
    static void prioritizeLateral(float x, float& y, float maxRearRatio = 0.25f);
    // 画像上のゴール下端が目標から離れるほど後退を強める。
    // 画素値は距離そのものではないため、両端は実機で調整する。
    static float returnRatio(int gap, int targetGap);
    // 成分を先に合成し、後退の制限後に車輪の上限へ合わせる。
    // MotorController は最大の車輪成分を speed に正規化するため、
    // 斜め移動時は |x|+|y| を速度として渡す。
    static void combine(float direction, int speed, bool returning, int returnPower,
                        int gap, int targetGap, int limit, float& resultDirection, int& resultSpeed);
    // ゴールに近いときは横追従を残しつつ、前方への退避を合成する。
    static void escapeForward(float direction, int speed, int forwardPower, int limit,
                              float& resultDirection, int& resultSpeed);
private:
    int targetGap, startMargin;
    bool returning;
    int farFrames;
};
#endif
