#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>

// MaixPyカメラからSerial1で届く検出結果を読み、走行制御へ渡す窓口。
// 機体基準角度は右0°、前90°、左180°、後ろ270°。
class Camera {
    public:
        Camera();
        void setup(); // MegaのSerial1を115200bpsで開始し、古い検出値を消す。
        // 受信済みの全行を消費し、最後の有効フレームを採用する。
        bool update();
        int readBallAngle();
        // 200ms以内に有効なフレームがない場合、角度は-1（未検出）を返す。
        float ballAngle() const;
        float goalAngle(bool yellow) const; // 同色の前後が見える場合は前方の角度を返す。
        // rear=true: 後方ミラー。V1では前後別情報を使えない。
        float goalViewAngle(bool yellow, bool rear) const;
        float goalSpan(bool yellow, bool rear) const; // ゴールの見かけの角幅。端切れ等では-1。
        int rearGoalBottomGap(bool yellow) const; // 画像中央から下端までの画素数、-1は無効。
        float ballPixels() const; // 採用したボール領域の画素数。実距離ではない。
        int ballView() const; // 0=未検出、1=前、2=後

        // 通信の診断用。受信バイトと有効・無効フレームを別々に数える。
        bool isReceiving() const { return isFresh(); }
        uint32_t receivedBytes() const { return byteCount; }
        uint32_t validFrames() const { return validCount; }
        uint32_t invalidFrames() const { return invalidCount; }
        const char* lastReceivedLine() const { return diagnosticLine; }
    private:
        static constexpr uint32_t CAMERA_TIMEOUT_MS = 200;
        char receivedData[128];
        char diagnosticLine[128];
        uint32_t byteCount;
        uint32_t validCount;
        uint32_t invalidCount;
        uint8_t receivedLength;
        bool discardLine;
        bool hasFrame;
        float lastBallAngle;
        float lastYellowAngle;
        float lastBlueAngle;
        float viewAngles[4]; // 黄色前・黄後・青前・青後
        float viewSpans[4];
        int rearBottomGaps[2];
        float lastBallPixels;
        int lastBallView;
        uint32_t lastUpdateMs;
        void resetReception();
        bool parseFrame();
        bool isFresh() const;
};

#endif
