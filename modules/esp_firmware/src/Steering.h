#pragma once
#include <Arduino.h>
#include <AccelStepper.h>

class Steering {
public:
    Steering(uint8_t ena_pin, uint8_t pul_pin, uint8_t dir_pin);
    void begin();
    void update();
    void setTargetAngle(float angle_deg);
    void enable(bool state);
    void stop();

private:
    uint8_t _ena_pin;
    AccelStepper _stepper;

    // Constants from original code
    static constexpr int32_t PULSES_PER_MOTOR_REV = 800;
    static constexpr float MOTOR_REVS_PER_WHEEL_REV = 1.0f;
    static constexpr float CONTROL_SCALE = 0.50f;
    static constexpr float MAX_STEPPER_SPEED = 750.0f;
    static constexpr float STEPPER_ACCELERATION = 450.0f;
    static constexpr uint32_t STEP_PULSE_WIDTH_US = 10;
};
