#ifndef IRSENSOR_H
#define IRSENSOR_H

class IRSensor {
    public:
        IRSensor(int startPin, int numSensors);
        int readAngle();
        int readDistance();
    private:
        int startPin;
        int numSensors;
        float filteredValues[16];  // フィルタ済み値を保存
        bool firstRead[16];        // 初回読み取りフラグ
        static constexpr float FILTER_ALPHA = 0.8f;  // フィルタ係数（0-1）
};


#endif