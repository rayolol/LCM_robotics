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
    stepPerDegreeRatio = (1 / stepPerRev) * 360; // Calculate steps per degree
}

void Motor::begin() {
    //pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW); // Enable the motor driver

    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(500);

    if (hasEncoder) {
        encoder.init();
        pid.SetMode(AUTOMATIC);

        setpoint = readEncoderAngle(); // Set the initial setpoint to the encoder angle
        input =setpoint; // Set the initial input to the encoder angle
        stepper.setCurrentPosition((long)(input * 10)); // Set the current position to the encoder angl
    } else {
        // TODO: Implement homing logic for motors without encoders
        stepper.setCurrentPosition(0);
    }
}

double Motor::readEncoderAngle() {
    uint16_t raw = encoder.get_angle(); // Assuming get_angle() returns the angle in degrees
    return (raw / 4095.0) * 360.0; // Convert raw value to degrees (assuming 12-bit resolution)
}

void Motor::update() {
    if(hasEncoder) {
        input = readEncoderAngle(); // Read the encoder angle
        pid.Compute(); // Compute the PID output
        stepper.setSpeed(output); // Set the speed based on PID output
        stepper.runSpeed(); // Run the motor at the computed speed
    } else {
        stepper.run();
    }
}

void Motor::setTargetAngle(int angle) {
    setpoint = angle;
    long steps = (long)(angle / 360 * 1600);//assumes 1600 steps per revolution

    if (hasEncoder) {
        stepper.moveTo(steps); // Move to the target angle in steps
    } else {
        stepper.moveTo(angle); // Move to the target angle directly
    }
}

double Motor::getCurrentAngle() {
    if (hasEncoder) {
        return readEncoderAngle(); // Return the encoder angle
    } else {
        return stepper.currentPosition() / 1600.0 * 360.0; // Convert steps to degrees
    }
}
