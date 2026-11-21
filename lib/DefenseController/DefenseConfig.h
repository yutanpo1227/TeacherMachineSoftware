#ifndef DEFENSE_CONFIG_H
#define DEFENSE_CONFIG_H

#include <stdint.h>

// 攻守で共有する動作の調整値。角度は度、時間はミリ秒。
namespace MotionTuning {
constexpr uint32_t ATTACK_LINE_EXIT_MS = 700; // 攻撃開始時、白線から前進して抜ける最長時間。
constexpr float BLIND_ALIGN_TOLERANCE_DEG = 5.0f; // ゴール未検出時、0度に向いたとみなす誤差。
constexpr uint32_t BLIND_ALIGN_STABLE_MS = 200; // 誤差内で維持してから後退を始める時間。
constexpr uint32_t OFFENSE_DIRECTION_RESET_MS = 250; // 指令が途切れた際に移動方向の履歴を捨てる時間。
constexpr float OFFENSE_DIRECTION_RATE_DEG_PER_SEC = 480.0f; // 攻撃時の移動方向が変化する最大速度。
constexpr float OFFENSE_STRAIGHT_ENTER_DEG = 8.0f; // 正面との角度差がここまで小さくなると直進する。
constexpr float OFFENSE_STRAIGHT_EXIT_DEG = 12.0f; // 直進中にこの角度差を超えると回り込みへ移る。
constexpr float OFFENSE_BLEND_END_DEG = 28.0f; // この角度差から従来の回り込み方向をそのまま使う。
}

// ゴールの見かけの幅は、実際の安全な待機位置で校正する。
struct DefenseConfig {
    // ゴールが画面いっぱいの現在のミラーでは距離を測らず、横守備する。
    // ゴール全体を安定して撮影・校正できる場合のみtrueにする。
    bool useGoalSpanForDepth = false;
    float homeSpan = 45.0f; // 幅で距離を判断するときの待機位置でのゴールの角幅（度）。
    float spanTolerance = 4.0f; // 待機位置とみなす角幅の許容差（度）。
    float maxHomeOffset = 30.0f; // 起動時のゴール方向を基準とした横移動の角度上限。
    int lateralSpeed = 230; // 横移動の最大PWM。
    int lateralMinSpeed = 170; // 小さい方向差でも動き出せる強さ。
    int depthSpeed = 80; // ゴールの幅を使った前後移動のPWM。
    uint32_t frontWaitMs = 5000; // 前方カメラでボールを連続検出する時間。
    uint32_t attackMaxMs = 10000; // 一時攻撃の最長時間。経過後はRETURNへ移る。
    uint32_t homeStableMs = 500; // 待機位置と姿勢が整った状態を保つ時間。
};

#endif
