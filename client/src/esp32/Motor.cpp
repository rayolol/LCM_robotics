#include "Motor.hpp"
#include <Arduino.h>


Motor::Motor(int stepPin, int dirPin, int motorNumber, int enablePin, int stepPerRev, uint8_t i2cAddress)
    : stepPerRev(stepPerRev),
      stepPin(stepPin),
      dirPin(dirPin),
      enablePin(enablePin),
      motorId(motorNumber),
      stepper(AccelStepper::DRIVER, stepPin, dirPin),
      hasEncoder(i2cAddress != 0),
      i2cAddress(i2cAddress),
      encoder(i2cAddress),
      pid(&input, &output, &setpoint, 0.1, 0.01, 0.1, DIRECT)
{
    name = "Motor" + std::to_string(motorNumber);
    stepPerDegreeRatio = 360/stepPerRev; // Calculate steps per degree
}

void Motor::begin() {
    // Skip initialization if pins are invalid (0)
    if (stepPin == 0 || dirPin == 0) {
        Serial.printf("Motor %s: Skipping initialization due to invalid pins (step=%d, dir=%d)\n",
                     name.c_str(), stepPin, dirPin);
        return;
    }

    // Configure pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    if (enablePin != 0) {
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, LOW); // Enable the motor driver (LOW is typically enabled)
    }

    // Configure stepper parameters
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(500);

    Serial.printf("Motor %s: Configured with maxSpeed=1000, accel=500\n", name.c_str());

    if (hasEncoder) {
        Serial.printf("Motor %s: Initializing encoder at address 0x%02X\n", name.c_str(), i2cAddress);
        try {
            encoder.init();
            pid.SetMode(AUTOMATIC);
            pid.SetOutputLimits(-500, 500); // Set reasonable output limits for the PID controller

            setpoint = readEncoderAngle(); // Set the initial setpoint to the encoder angle
            input = setpoint; // Set the initial input to the encoder angle
            stepper.setCurrentPosition((long)(input * 10)); // Set the current position to the encoder angle

            Serial.printf("Motor %s: Encoder initialized, initial angle: %.2f\n",
                         name.c_str(), setpoint);
        } catch (...) {
            Serial.printf("Motor %s: Error initializing encoder\n", name.c_str());
            hasEncoder = false; // Fallback to non-encoder mode
        }
    } else {
        Serial.printf("Motor %s: No encoder, setting home position to 0\n", name.c_str());
        stepper.setCurrentPosition(0);
    }
}

double Motor::readEncoderAngle() {
    uint16_t raw = encoder.get_angle(); // Assuming get_angle() returns the angle in degrees
    return (raw / 4095.0) * 360.0; // Convert raw value to degrees (assuming 12-bit resolution)
}

void Motor::update() {
    // Skip update if pins are invalid
    if (stepPin == 0 || dirPin == 0) {
        return;
    }

    try {
        if(hasEncoder) {
            input = readEncoderAngle(); // Read the encoder angle
            pid.Compute(); // Compute the PID output
            stepper.setSpeed(output); // Set the speed based on PID output
            stepper.runSpeed(); // Run the motor at the computed speed
        } else {
            stepper.run();
        }
    } catch (...) {
        // If there's an error, just log it and continue
        // This prevents one motor error from crashing the whole system
        Serial.printf("Motor %s: Error in update()\n", name.c_str());
    }
}

void Motor::setTargetAngle(int angle) {
    // Skip if pins are invalid
    if (stepPin == 0 || dirPin == 0) {
        return;
    }

    try {
        Serial.printf("Motor %s: Setting target angle to %d degrees\n", name.c_str(), angle);

        setpoint = angle;
        long steps = (long)(angle * stepPerDegreeRatio); // Calculate steps based on stepPerRev

        Serial.printf("Motor %s: Target steps = %ld\n", name.c_str(), steps);

        if (hasEncoder) {
            stepper.moveTo(steps); // Move to the target angle in steps
        } else {
            stepper.moveTo(steps); // Move to the target angle directly
        }
    } catch (...) {
        Serial.printf("Motor %s: Error in setTargetAngle()\n", name.c_str());
    }
}

double Motor::getCurrentAngle() {
    if (hasEncoder) {
        return readEncoderAngle(); // Return the encoder angle
    } else {
        return stepper.currentPosition() / this->stepPerRev * 360.0; // Convert steps to degrees
    }
}
