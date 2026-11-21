#ifndef MOTOR_H
#define MOTOR_H

// モータードライバー1チャンネルへ回転方向とPWMを出す。
class Motor {
    public:
        // phPinは方向、enPinはPWM出力に使うピン。
        Motor(int phPin, int enPin);
        // 符号が回転方向、絶対値がPWM。0なら停止。呼び出し側で0〜255に収める。
        void setSpeed(int speed);
    private:
        int phPin;
        int enPin;
};


#endif
