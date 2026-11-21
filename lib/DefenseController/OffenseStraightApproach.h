#ifndef OFFENSE_STRAIGHT_APPROACH_H
#define OFFENSE_STRAIGHT_APPROACH_H

#include <math.h>
#include "GoalHeading.h"

// 正面付近は直進を維持し、その外側で回り込みへ徐々に移る。
class OffenseStraightApproach {
public:
    // ballAngleは機体から見たボール方向、wrapAngleは元の回り込み方向。
    // 正面±8度に入ると90度を返し、±12度を超えるまで直進状態を保持する。
    float target(float ballAngle, float wrapAngle);
    // 白線処理やボール未検出などで直進状態を解除する。
    void reset();
private:
    bool straight = false;
};
#endif
