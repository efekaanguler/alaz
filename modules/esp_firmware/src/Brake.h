#pragma once
#include <Arduino.h>

class Brake {
public:
    Brake(uint8_t dir_pin, uint8_t pwm_pin);
    void begin();
    
    // Direction: 0 (push/brake), 1 (pull/release)
    // Speed: 0 to 255
    void setMotor(int direction, int speed);
    void stop();

private:
    uint8_t _dir_pin;
    uint8_t _pwm_pin;

    static constexpr int PWM_FREQ = 1000;
    static constexpr int PWM_RES_BITS = 8;
    static constexpr int PWM_MAX = 255;
};
