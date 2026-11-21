#include "DefenseController.h"

DefenseController::DefenseController(const DefenseConfig &config) : config(config) {}

DefenseController::State DefenseController::state() const { return currentState; }

float DefenseController::homeOffsetForLog() const { return lastHomeOffset; }

bool DefenseController::lateralLimitedForLog() const { return lateralLimited; }

const char *DefenseController::stateName() const {
    return currentState == State::GUARD ? "GUARD" : currentState == State::ATTACK ? "ATTACK" : "RETURN";
}

uint32_t DefenseController::attackWaitForLog(uint32_t now) const {
    return visibleTiming ? (uint32_t)(now - visibleStart) : 0;
}

void DefenseController::observeFrontBall(uint32_t now, bool detected) {
    // カメラの連続検出が途切れると、次の検出から設定時間を数え直す。
    if (currentState == State::ATTACK || !detected) {
        visibleTiming = false;
        return;
    }
    if (!visibleTiming) {
        visibleTiming = true;
        visibleStart = now;
        return;
    }
    if ((uint32_t)(now - visibleStart) >= config.frontWaitMs) {
        currentState = State::ATTACK;
        attackStart = now;
        visibleTiming = stableTiming = false;
        hasRange = rangeBlocked = false;
    }
}

void DefenseController::retreat() {
    currentState = State::RETURN;
    stableTiming = false;
}

DefenseMove DefenseController::update(const DefenseInput &in) {
    DefenseMove move;
    lateralLimited = false;
    const bool ballVisible = in.ballAngle >= 0 && in.ballAngle < 360;
    if (currentState == State::ATTACK) {
        // 攻撃中の実際の走行は呼び出し元のOFFENSE処理へ任せる。
        if ((uint32_t)(in.now - attackStart) >= config.attackMaxMs) {
            retreat();
        } else {
            move.attack = true;
            return move;
        }
    }
    const bool rearValid = in.rearGoalAngle >= 180 && in.rearGoalAngle < 360;
    if (in.rearGoalSpan < 0)
        hasRange = rangeBlocked = false;
    bool rangeValid = config.useGoalSpanForDepth && rearValid && in.rearGoalSpan > 0;
    // 短時間で幅が25%を超えて減った場合は、遮蔽の疑いとして停止する。
    // 色検出だけでは、ゆっくりした遮蔽や初見の遮蔽までは判定できない。
    if (rangeValid) {
        if (hasRange && (uint32_t)(in.now - lastRangeMs) <= 200 && in.rearGoalSpan < lastRangeSpan * 0.75f)
            rangeBlocked = true;
        if (rangeBlocked && in.rearGoalSpan < lastRangeSpan * 0.75f) {
            rangeValid = false;
        } else {
            rangeBlocked = false;
            hasRange = true;
            lastRangeSpan = in.rearGoalSpan;
            lastRangeMs = in.now;
        }
    }
    // 自ゴールをロボット後方へ向ける姿勢が、およそ20度以内なら整列済みとする。
    const bool aligned = in.headingValid && fabsf(in.headingError) <= 20;
    // 幅が見切れている場合、起動位置をゴール前と仮定して横守備する。
    // その場合は前後位置を推定できないので、前後移動は指示しない。
    const bool homeDepth =
        rearValid && (!rangeValid || fabsf(in.rearGoalSpan - config.homeSpan) <= config.spanTolerance);
    // 起動時に安全なゴール中央前へ配置し、その向きを中央の目印にする。
    if (!hasHomeYaw && homeDepth && aligned) {
        homeYaw = GoalHeading::angleDifference(in.yaw - in.headingError);
        hasHomeYaw = true;
    }
    const float homeOffset =
        hasHomeYaw ? GoalHeading::angleDifference(in.yaw - in.headingError - homeYaw) : 0;
    lastHomeOffset = homeOffset;
    // 距離情報が途切れた場合、古い距離で後退しない。
    if (!rearValid || rangeBlocked) {
        lateralTracking = false;
        // 後方から消えたら復帰状態へ。前方ミラーに移った自ゴールも再捕捉可能にする。
        if (!rearValid)
            retreat();
        stableTiming = false;
        return move;
    }
    const bool rear = ballVisible && in.ballAngle >= 180;
    const bool directlyBehind = rear && fabsf(in.ballAngle - 270) <= 8;
    // 正面近くの揺れで停止と発進を繰り返さない。
    const float frontOffset = fabsf(in.ballAngle - 90);
    if (!ballVisible || frontOffset <= 2)
        lateralTracking = false;
    else if (frontOffset >= 4)
        lateralTracking = true;
    // 真後ろ付近で検出角が揺れても、回り込む側を頻繁に切り替えない。
    if (ballVisible && !directlyBehind && frontOffset >= 5)
        ballSide = cosf(in.ballAngle * 0.01745329252f) < 0 ? -1 : 1;

    // ゴールに向き直る途中でも、ボールが見えていれば横に追う。
    // 距離の補正や、見失ったときの中央寄せは姿勢が整ってから行う。
    float x = 0;
    if (lateralTracking && (!aligned || hasHomeYaw)) {
        x = directlyBehind ? ballSide * config.lateralMinSpeed : lateralPower(in.ballAngle);
    } else if (aligned && hasHomeYaw && !ballVisible && fabsf(homeOffset) > 5) {
        x = -homeOffset * 3;
    }
    x = limitLateral(x, homeOffset);
    if (!aligned) {
        stableTiming = false;
        if (fabsf(x) >= 1) {
            move.direction = x > 0 ? 0.0f : 180.0f;
            move.speed = (int)fabsf(x);
        }
        return move;
    }
    if (currentState == State::RETURN &&
        // ゴール方向は横位置や見切れでも変わるため、起動方位との一致を
        // 復帰完了条件にしない。姿勢の整列は上のaligned判定で確認済み。
        elapsed(homeDepth && in.homePositionReady, stableTiming, stableStart, in.now, config.homeStableMs)) {
        currentState = State::GUARD;
        stableTiming = false;
    }
    // 後方のボールへは横に回りつつ少し後退。近接・未検出時は
    // main.cppのゴール距離ガードがこの指令より優先される。
    float y = rear && !lateralLimited ? sinf(in.ballAngle * 0.01745329252f) * 60.0f : 0;
    if (rangeValid && !homeDepth)
        y = in.rearGoalSpan > config.homeSpan ? config.depthSpeed : -config.depthSpeed;
    // 横成分xと前後成分yを、モーターコントローラーが使う角度とPWMへ変換する。
    const float magnitude = sqrtf(x * x + y * y);
    if (magnitude >= 1) {
        move.direction = atan2f(y, x) * 57.295779513f;
        if (move.direction < 0)
            move.direction += 360;
        move.speed = (int)(magnitude > config.lateralSpeed ? config.lateralSpeed : magnitude);
    }
    return move;
}

float DefenseController::limitLateral(float power, float homeOffset) {
    // ゴール中央を基準に、外へ出る指令だけを止めて内側へは戻れるようにする。
    if ((homeOffset >= config.maxHomeOffset && power > 0) ||
        (homeOffset <= -config.maxHomeOffset && power < 0)) {
        lateralLimited = true;
        return 0;
    }
    if (power > config.lateralSpeed)
        return config.lateralSpeed;
    if (power < -config.lateralSpeed)
        return -config.lateralSpeed;
    return power;
}

float DefenseController::lateralPower(float ballAngle) const {
    // 正面から約20度で最大速度になり、ボールに素早く追従する。
    float x = cosf(ballAngle * 0.01745329252f) * config.lateralSpeed / 0.3420201f;
    if (fabsf(x) < config.lateralMinSpeed)
        x = x < 0 ? -config.lateralMinSpeed : config.lateralMinSpeed;
    return x;
}

bool DefenseController::elapsed(bool condition, bool &timing, uint32_t &start, uint32_t now,
                                uint32_t duration) {
    if (!condition) {
        timing = false;
        return false;
    }
    if (!timing) {
        timing = true;
        start = now;
    }
    return (uint32_t)(now - start) >= duration;
}
