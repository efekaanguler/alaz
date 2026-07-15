#include "Steering.h"

Steering::Steering(uint8_t ena_pin, uint8_t pul_pin, uint8_t dir_pin)
    : _ena_pin(ena_pin),
      _stepper(AccelStepper::DRIVER, pul_pin, dir_pin) {
}

void Steering::begin() {
    pinMode(_ena_pin, OUTPUT);
    digitalWrite(_ena_pin, HIGH); // Disable by default (Active LOW)

    _stepper.setPinsInverted(false, false, false);
    _stepper.setMinPulseWidth(STEP_PULSE_WIDTH_US);
    _stepper.setMaxSpeed(MAX_STEPPER_SPEED);
    _stepper.setAcceleration(STEPPER_ACCELERATION);
    
    // Set current position as center (0)
    _stepper.setCurrentPosition(0);
}

void Steering::update() {
    _stepper.run();
}

void Steering::setTargetAngle(float angle_deg) {
    // Convert angle to stepper pulses
    float motorRevolutions = (angle_deg / 360.0f) * MOTOR_REVS_PER_WHEEL_REV * CONTROL_SCALE;
    int32_t targetSteps = static_cast<int32_t>(round(motorRevolutions * PULSES_PER_MOTOR_REV));
    _stepper.moveTo(targetSteps);
}

void Steering::enable(bool state) {
    // ENA is Active LOW
    digitalWrite(_ena_pin, state ? LOW : HIGH);
}

void Steering::stop() {
    _stepper.stop();
}
