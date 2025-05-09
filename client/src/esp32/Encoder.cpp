#include "Encoder.hpp"

Encoder::Encoder(uint8_t address) : i2cAddress(address) {}

void Encoder::init() {
    as5600.begin(i2cAddress);
}

uint16_t Encoder::get_angle() {
    return as5600.readAngle(); // Assuming 'readAngle' is the correct method in AS5600
}
