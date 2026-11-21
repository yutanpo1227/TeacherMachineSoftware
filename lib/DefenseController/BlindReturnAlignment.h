#ifndef BLIND_RETURN_ALIGNMENT_H
#define BLIND_RETURN_ALIGNMENT_H

#include <stdint.h>
#include <math.h>
#include "GoalHeading.h"

// 未検出から復帰するとき、0度付近で一定時間安定してから後退する。
class BlindReturnAlignment {
public:
    // yawはジャイロ方位、nowはmillis()。0度±5度を200ms維持するとtrueを返す。
    // trueになった後は左右確認中も維持し、resetされるまで再判定しない。
    bool update(float yaw, uint32_t now);
    // ゴール再検出や通信断後の再探索では、もう一度0度への整列から始める。
    void reset();
private:
    bool aligned = false;
    bool timing = false;
    uint32_t start = 0;
};
#endif
