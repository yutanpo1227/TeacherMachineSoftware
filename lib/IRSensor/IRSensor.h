#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <Arduino.h>

class IRSensor {
   public:
    IRSensor(int startPin, int numSensors);
    int readAngle();
    int readDistance();
    int getNumSensors() const { return numSensors; }
    /** 直近の update 後の EMA 済み LOW 比を out へ。戻り値: 格納した本数。 */
    int copyFilteredDuties(float* out, int maxOut) const;

   private:
    void updateFilteredDuties();
    static void sampleAllLowDuties(int startPin, int numSensors, uint32_t* lowCount, uint32_t& totalCount);
    // 全センサーの強度を取り付け角で重み付き合成（角度・距離のベクトル和）
    void weightedRingSum(float& sumX, float& sumY) const;

    int startPin;
    int numSensors;
    float filteredValues[16];
    bool firstRead[16];
    uint32_t lastSampleTimeUs_ = 0;
    // readAngle の直後の readDistance で 20ms 窓を二重に採らないための併用閾値（us）
    static constexpr uint32_t kResampleMinIntervalUs = 25000;
    // TSSP4038: 40kHz/1.2kHz 的変調をデジタルOUTの LOW 比率で相対化
    static constexpr uint32_t kSampleWindowUs = 20000;   // 20ms
    static constexpr uint16_t kSampleIntervalUs = 80;    // 12.5kHz
    static constexpr float kDutyEmaAlpha = 0.25f;      // 0..1
};

#endif