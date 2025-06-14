#ifndef MOTOR_H
#define MOTOR_H

class Motor {
    public:
        Motor(int phPin, int enPin);
        void setSpeed(int speed);
    private:
        int phPin;
        int enPin;
};


#endif
