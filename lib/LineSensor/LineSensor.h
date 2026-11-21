#ifndef LINESENSOR_H
#define LINESENSOR_H

// 車体周囲の反射センサーを個別の閾値と比較し、白線の方向を求める。
class LineSensor {
    public:
        // thresholds[i]はstartPin+iの判定値。配列はこのオブジェクトの使用中保持する。
        LineSensor(int startPin, int numSensors, const int* thresholds);
        // 白線を検出した方向を0〜359度で返す。未検出・無効なら-1。
        int readAngle();
        // 長時間読み取らなかった後は、以前の白線方向を引き継がずに再計測する。
        void resetFilter();
        // 検出方向のまとまりを0〜1で表す。白線の距離や明るさそのものではない。
        float readVectorMagnitude();
        // いずれかのセンサーが閾値を超えればtrue。
        bool checkOnLine();
        // falseなら白線判定を無効にする。
        void setMode(bool mode);
        // 各センサーの生のアナログ値をUSBシリアルへ出す調整用関数。
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
