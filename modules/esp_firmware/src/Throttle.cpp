#include "Throttle.h"

constexpr uint8_t RELAY_COIL_ON  = LOW;
constexpr uint8_t RELAY_COIL_OFF = HIGH;

Throttle::Throttle(uint8_t sda_pin, uint8_t scl_pin, uint8_t relay_brake, uint8_t relay_low, uint8_t relay_reverse)
    : _sda_pin(sda_pin), _scl_pin(scl_pin), _relay_brake(relay_brake), 
      _relay_low(relay_low), _relay_reverse(relay_reverse), _pcfAddress(0), _currentVoltage(IDLE_V) {
}

void Throttle::begin() {
    pinMode(_relay_brake, OUTPUT);
    pinMode(_relay_low, OUTPUT);
    pinMode(_relay_reverse, OUTPUT);

    // Safe default states
    setMotorBrake(true);
    setSpeedMode(true);
    setDirection(false);

    Wire.begin(_sda_pin, _scl_pin);
    Wire.setClock(100000);

    if (!findPCF8591()) {
        Serial.println("ERROR: PCF8591 (Throttle DAC) not detected!");
    } else {
        setVoltage(IDLE_V);
    }
}

bool Throttle::findPCF8591() {
    for (uint8_t address = 0x48; address <= 0x4F; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            _pcfAddress = address;
            return true;
        }
    }
    return false;
}

void Throttle::setNCContact(uint8_t pin, bool closed) {
    digitalWrite(pin, closed ? RELAY_COIL_OFF : RELAY_COIL_ON);
}

void Throttle::setDirection(bool reverse) {
    // Reverse relay -> Active = REVERSE
    setNCContact(_relay_reverse, reverse);
}

void Throttle::setSpeedMode(bool lowSpeed) {
    // Low speed relay -> Active = LOW
    setNCContact(_relay_low, lowSpeed);
}

void Throttle::setMotorBrake(bool engaged) {
    // Brake relay -> Active = BRAKED
    setNCContact(_relay_brake, engaged);
}

bool Throttle::setVoltage(float voltage) {
    if (_pcfAddress == 0) return false;
    
    voltage = constrain(voltage, 0.0f, DAC_MAX_V);
    if (abs(voltage - _currentVoltage) < 0.01f) return true;
    uint8_t dacValue = static_cast<uint8_t>(round((voltage / DAC_MAX_V) * 255.0f));

    Wire.beginTransmission(_pcfAddress);
    Wire.write(0x40);  // Enable DAC output
    Wire.write(dacValue);
    
    if (Wire.endTransmission() != 0) return false;
    
    _currentVoltage = voltage;
    return true;
}

void Throttle::emergencyStop(const char* reason) {
    Serial.printf("THROTTLE E-STOP: %s\n", reason);
    setVoltage(IDLE_V);
    setMotorBrake(true);
    setSpeedMode(true);
    setDirection(false);
}
