#ifndef MOTOR_CONTROLLER_CONFIG_H
#define MOTOR_CONTROLLER_CONFIG_H

// 通常走行中に姿勢を直す旋回成分のPWM上限・下限（0〜255）。
// 下限を上げると回り始めやすくなり、上限を上げると大きなずれを強く補正する。
constexpr int MAX_CORRECTION_POWER = 50;
constexpr int MIN_CORRECTION_POWER = 20;

// 姿勢誤差1度あたりの旋回PWM。上下限の範囲に収めてから車輪へ配分する。
constexpr float HEADING_KP = 0.8f;
// 姿勢誤差がこの角度（度）以内なら旋回せず、正面付近の揺れを防ぐ。
constexpr float HEADING_DEADBAND_DEG = 3.0f;

// ボールと白線の角度差がこの値（度）以内なら、白線を越えて追わずに停止する。
constexpr int LINE_STOP_ANGLE_THRESHOLD = 45;
// 白線ベクトルの大きさ（0〜1）の境界。小さいと白線の反対側へ退避する。
constexpr double LINE_VECTOR_MAGNITUDE_THRESHOLD = 0.75;

#endif
