#pragma once
#include <AccelStepper.h>
#include <Wire.h>
#include <string>
#include <Encoder.hpp>
#include <PID_v1.h>

class Motor {
public:
    Motor(
        int stepPin,
        int dirPin,
        int motorNumber,
        int enablePin = 0,
        int stepPerRev = 1600,
        uint8_t i2cAddress = 0
    );

    void begin();
    void update();
    void setTargetAngle(int angle);
    double getCurrentAngle();
    double readEncoderAngle();

    std::string name;
    // Make these public for debugging
    int stepPin;
    int dirPin;

private:
    int enablePin;
    int motorId;
    int stepPerRev;
    double stepPerDegreeRatio;
    float currentPosition = 0;
    float targetPosition = 0;
    double input, output, setpoint;

    bool hasEncoder;
    uint8_t i2cAddress;

    PID pid;
    AccelStepper stepper;
    Encoder encoder;
};
