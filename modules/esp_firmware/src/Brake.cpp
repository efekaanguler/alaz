#include "Brake.h"

Brake::Brake(uint8_t dir_pin, uint8_t pwm_pin)
    : _dir_pin(dir_pin), _pwm_pin(pwm_pin) {
}

void Brake::begin() {
    pinMode(_dir_pin, OUTPUT);
    // ESP32 Arduino Core 3.x ledc configuration
    ledcAttach(_pwm_pin, PWM_FREQ, PWM_RES_BITS);
    stop();
}

void Brake::setMotor(int direction, int speed) {
    digitalWrite(_dir_pin, direction ? HIGH : LOW);
    speed = constrain(speed, 0, PWM_MAX);
    ledcWrite(_pwm_pin, speed);
}

void Brake::stop() {
    setMotor(0, 0);
}
