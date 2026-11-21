#include "Camera.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

Camera::Camera() {
    resetReception();
}

void Camera::setup() {
    Serial1.begin(115200);  // camera/V-mirror.py と一致させる。
    resetReception();
}

// 起動・再初期化の直後は、次に受信するまでボールもゴールも未検出とする。
void Camera::resetReception() {
    receivedLength = 0;
    discardLine = false;
    hasFrame = false;
    lastBallAngle = lastYellowAngle = lastBlueAngle = -1;
    for (int i = 0; i < 4; ++i) viewAngles[i] = viewSpans[i] = -1;
    rearBottomGaps[0] = rearBottomGaps[1] = -1;
    lastBallPixels = 0;
    lastBallView = 0;
    lastUpdateMs = 0;
    byteCount = validCount = invalidCount = 0;
    diagnosticLine[0] = '\0';
}

static bool validAngle(float value) {
    // -1は未検出、それ以外は機体基準の角度として扱う。
    return value == -1 || (value >= 0 && value < 360);
}

bool Camera::parseFrame() {
    // V1は角度のみ、V2は前後のミラー別、V3はゴール下端の画素距離も持つ。
    const bool v3 = strncmp(receivedData, "V3,", 3) == 0;
    const bool v2 = v3 || strncmp(receivedData, "V2,", 3) == 0;
    if (!v2 && strncmp(receivedData, "V1,", 3) != 0) return false;
    const int count = v3 ? 13 : (v2 ? 11 : 3);
    float values[13];
    char* cursor = receivedData + 3;
    for (int i = 0; i < count; ++i) {
        char* end;
        values[i] = strtod(cursor, &end);
        if (end == cursor || !isfinite(values[i]) ||
            *end != (i + 1 < count ? ',' : '\0')) return false;
        cursor = end + 1;
    }
    // 壊れた行や途中で切れた行を採用しない。異常時は前回の有効値を保持する。
    if (!validAngle(values[0])) return false;
    if (v2) {
        if (values[1] < 0 || values[1] > 76800 ||
            !(values[2] == 0 || values[2] == 1 || values[2] == 2)) return false;
        if ((values[0] == -1) != (values[2] == 0) ||
            (values[2] == 0 && values[1] != 0)) return false;
        for (int i = 3; i < 11; i += 2) {
            if (!validAngle(values[i]) ||
                !(values[i+1] == -1 || (values[i+1] > 0 && values[i+1] <= 180)) ||
                (values[i] == -1 && values[i+1] != -1)) return false;
        }
        if (v3) {
            for (int i = 11; i < 13; ++i) {
                if (!(values[i] == -1 || (values[i] >= 3 && values[i] <= 160 &&
                      values[i] == (int)values[i]))) return false;
                if (values[i] >= 0 && values[i == 11 ? 5 : 9] < 0) return false;
            }
        }
    } else if (!validAngle(values[1]) || !validAngle(values[2])) return false;
    // 全項目を検証してから採用。旧形式を受けたら距離情報も消す。
    rearBottomGaps[0] = v3 ? (int)values[11] : -1;
    rearBottomGaps[1] = v3 ? (int)values[12] : -1;
    lastBallAngle = values[0];
    lastBallPixels = v2 ? values[1] : 0;
    lastBallView = v2 ? (int)values[2] : 0;
    for (int i = 0; i < 4; ++i) {
        viewAngles[i] = v2 ? values[3 + i*2] : -1;
        viewSpans[i] = v2 ? values[4 + i*2] : -1;
    }
    // 色だけを指定した照会は前方優先。守備側は後方ミラーを明示して照会する。
    lastYellowAngle = v2 ? (viewAngles[0] >= 0 ? viewAngles[0] : viewAngles[1]) : values[1];
    lastBlueAngle = v2 ? (viewAngles[2] >= 0 ? viewAngles[2] : viewAngles[3]) : values[2];
    lastUpdateMs = millis();
    hasFrame = true;
    return true;
}

bool Camera::update() {
    // loop()が呼ばれるたびに溜まった文字を改行まで読み、最新の有効行を使う。
    // 1行を丸ごと待たないため、カメラが送信途中でも車輪の制御を続けられる。
    bool updated = false;
    while (Serial1.available() > 0) {
        const char c = Serial1.read();
        ++byteCount;
        if (c == '\n') {
            // USBログ用には制御文字を除き、受信データをそのまま実行しない。
            for (uint8_t i = 0; i < receivedLength; ++i) {
                const char value = receivedData[i];
                diagnosticLine[i] = value >= 32 && value <= 126 ? value : '.';
            }
            diagnosticLine[receivedLength] = '\0';
            if (!discardLine) {
                receivedData[receivedLength] = '\0';
                if (parseFrame()) {
                    updated = true;
                    ++validCount;
                } else {
                    ++invalidCount;
                }
            } else {
                ++invalidCount;
            }
            receivedLength = 0;
            discardLine = false;
        } else if (c != '\r' && !discardLine) {
            if (receivedLength < sizeof(receivedData) - 1) {
                receivedData[receivedLength++] = c;
            } else {
                // バッファに収まらない行は次の改行まで捨てる。途中の断片を誤検出にしない。
                discardLine = true;
            }
        }
    }
    return updated;
}

bool Camera::isFresh() const {
    // カメラの通信が止まった状態で、最後に見えた方向を使って走り続けない。
    return hasFrame && (uint32_t)(millis() - lastUpdateMs) <= CAMERA_TIMEOUT_MS;
}

float Camera::ballAngle() const {
    return isFresh() ? lastBallAngle : -1;
}

float Camera::goalAngle(bool yellow) const {
    return isFresh() ? (yellow ? lastYellowAngle : lastBlueAngle) : -1;
}

int Camera::readBallAngle() {
    update();
    return (int)ballAngle();
}

float Camera::goalViewAngle(bool yellow, bool rear) const {
    return isFresh() ? viewAngles[(yellow ? 0 : 2) + (rear ? 1 : 0)] : -1;
}
float Camera::goalSpan(bool yellow, bool rear) const {
    return isFresh() ? viewSpans[(yellow ? 0 : 2) + (rear ? 1 : 0)] : -1;
}
float Camera::ballPixels() const { return isFresh() ? lastBallPixels : 0; }
int Camera::ballView() const { return isFresh() ? lastBallView : 0; }

int Camera::rearGoalBottomGap(bool yellow) const {
    return isFresh() ? rearBottomGaps[yellow ? 0 : 1] : -1;
}
