#ifndef GOAL_HEADING_H
#define GOAL_HEADING_H

#include <stdint.h>

// カメラの機体相対角をBNO055の方位へ変換し、一時的な未検出を補う。
class GoalHeading {
    public:
        // holdMsは見失った後の目標保持時間。符号はカメラ角とジャイロ方位の向きを合わせる。
        GoalHeading(uint32_t holdMs, float cameraToYawSign);

        // 角度差を-180〜180度に収め、0度をまたいでも最短の向きを求める。
        static float angleDifference(float angle);

        // goalAngleは機体から見たゴール方向、referenceBearingは目標とする機体上の方向。
        // currentYawと合わせて、ゴールに向くためのジャイロ目標方位を記録する。
        void observe(float goalAngle, float currentYaw, uint32_t now, float referenceBearing = 90.0f);

        bool hasReference() const;

        // 方向が測れなくてもゴールが見えている間、直前の目標方位を維持する。
        void refresh(uint32_t now);

        // 最後の観測から保持時間内ならtrue。nowはmillis()。
        bool isValid(uint32_t now) const;

        // 正なら左旋回、負なら右旋回する（既存モーター配線の規約）。
        float error(float currentYaw) const;

    private:
        uint32_t holdMs;
        float cameraToYawSign;
        bool hasTarget;
        float targetYaw;
        uint32_t lastSeenMs;
};

#endif
