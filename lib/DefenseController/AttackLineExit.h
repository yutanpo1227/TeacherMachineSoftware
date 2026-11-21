#ifndef ATTACK_LINE_EXIT_H
#define ATTACK_LINE_EXIT_H

#include <stdint.h>

// 守備から攻撃に切り替わる瞬間の白線から、時間を限って離れる。
class AttackLineExit {
public:
    // 攻撃へ切り替わった瞬間に一度だけ呼び、既に白線を踏み自ゴールが後方近くにある場合に開始する。
    void begin(uint32_t now, bool onLine, bool goalBehindNearby);
    // 離脱を続ける間true。白線から離れる、ゴール条件が外れる、700ms経過のいずれかでfalse。
    bool update(uint32_t now, bool onLine, bool goalBehindNearby);
    // 攻撃を離れたら状態を破棄する。次回の攻撃ではbeginから判定し直す。
    void reset();
private:
    bool active = false;
    uint32_t start = 0;
};
#endif
