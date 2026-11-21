#ifndef OFFENSE_DIRECTION_SMOOTHER_H
#define OFFENSE_DIRECTION_SMOOTHER_H

#include <stdint.h>
#include "GoalHeading.h"

// 円周上の最短方向へ、経過時間に応じて移動指令を変化させる。
class OffenseDirectionSmoother {
public:
    // targetは目標移動角度、nowはmillis()。角度の変化を最大480度/秒に制限する。
    // 前回更新から250msを超えた場合は古い指令を使わず、目標角から再開する。
    float update(float target, uint32_t now);
    // 白線処理やボール未検出などで通常の追跡を離れたときに履歴を消す。
    void reset();
private:
    static float normalize(float value);
    bool valid = false;
    float direction = 0;
    uint32_t lastMs = 0;
};
#endif
