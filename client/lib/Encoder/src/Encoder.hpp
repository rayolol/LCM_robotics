#pragma once 

#include <AS5600.h>


class Encoder {
public:
    Encoder(uint8_t i2cAddress);
    void init();
    uint16_t get_angle();
private:
    uint8_t i2cAddress;
    AS5600 as5600;
};
