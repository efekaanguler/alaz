#pragma once
#include <Arduino.h>
#include <Wire.h>

class Throttle {
public:
    Throttle(uint8_t sda_pin, uint8_t scl_pin, uint8_t relay_brake, uint8_t relay_low, uint8_t relay_reverse);
    void begin();
    
    // Set voltage output to motor controller DAC
    bool setVoltage(float voltage);
    
    // Relay Controls
    void setDirection(bool reverse);
    void setSpeedMode(bool lowSpeed);
    void setMotorBrake(bool engaged);
    
    // Emergency
    void emergencyStop(const char* reason);

    // Constants
    static constexpr float IDLE_V = 0.90f;
    static constexpr float MAX_FORWARD_V = 3.00f;
    static constexpr float MAX_REVERSE_V = 2.30f;
    static constexpr float DAC_MAX_V = 4.60f;

private:
    uint8_t _sda_pin;
    uint8_t _scl_pin;
    uint8_t _relay_brake;
    uint8_t _relay_low;
    uint8_t _relay_reverse;
    uint8_t _pcfAddress;
    float _currentVoltage;

    bool findPCF8591();
    void setNCContact(uint8_t pin, bool closed);
};
