#ifndef LINESENSOR_H
#define LINESENSOR_H

class LineSensor {
    public:
        LineSensor(int startPin, int numSensors, const int* thresholds);
        int readAngle();
        float readVectorMagnitude();
        bool checkOnLine();
        void setMode(bool mode);
        void printValues();
    private:
        int startPin;
        int numSensors;
        const int* thresholds;
        bool mode;
        float previousAngle;        // 前回の角度
        bool hasPreviousAngle;     // 前回角度の有効性
        float filteredAngle;       // フィルタ済み角度
        static constexpr float ANGLE_FILTER_ALPHA = 0.7f;  // 角度フィルタ係数
        static constexpr float MAX_ANGLE_JUMP = 60.0f;     // 許容最大角度変化
};
#endif
