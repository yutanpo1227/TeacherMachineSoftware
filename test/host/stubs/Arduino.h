#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <algorithm>
#define PI 3.14159265358979323846
#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0
using std::abs;
template <typename T> T constrain(T value, T low, T high) {
    return std::min(std::max(value, low), high);
}
extern uint32_t testMillis;
extern int testPwm[32];
extern int testDirection[32];
extern int testAnalog[32];
inline unsigned long millis() { return testMillis; }
inline void delay(unsigned long ms) { testMillis += ms; }
inline void pinMode(int, int) {}
inline int analogRead(int pin) { return testAnalog[pin]; }
inline void analogWrite(int pin, int value) { testPwm[pin] = value; }
inline void digitalWrite(int pin, int value) { testDirection[pin] = value; }
class FakeSerial {
    public:
        std::string input;
        unsigned long baud = 0;
        void begin(unsigned long rate) { baud = rate; }
        template<typename T> void print(T) {}
        void println() {}
        int available() const { return input.size(); }
        char read() {
            const char c = input[0];
            input.erase(0, 1);
            return c;
        }
};
extern FakeSerial Serial1;
extern FakeSerial Serial;
#endif
