#include <Arduino.h>
#include <SPI.h>
#include "Motor.hpp"

#define SS 5

Motor motors[6] = {
    Motor(33, 32, 1),
    Motor(26, 25, 2),
    Motor(14, 12, 3),
    Motor(34, 35, 4),
    Motor(18, 19, 5),
    Motor(21, 23, 6)
};

void parseSPI() {
  uint8_t data[64] = {0}; // Buffer to store received data
  if (digitalRead(SS) == LOW) {
    // SPI data is being received
    for (int i = 0; i < 8; i++) {
      // Read 8 bits of data
      data[i] = SPI.transfer(0x00); // Dummy transfer to read data
      Serial.print(data[i], HEX);
      Serial.print(" ");
    } 
  }

  if (data[0] == 0xAA) {
    // Start byte detected
    Serial.println("Start byte detected");
  } else {
    Serial.println("No start byte detected");
    return;
  }

  uint8_t payloadSize = data[1]; // Length byte
  if (payloadSize != (6 * sizeof(float) +  6 * sizeof(float))) {
    Serial.println("Invalid payload size");
    return;
  }

  uint8_t command = data[2]; // Command byte
  if (command != 0x01) {
    Serial.println("Invalid command");
    return;
  }
  float angles[6], speeds[6];

  memcpy(angles, data + 3, sizeof(float) * 6);
  memcpy(speeds, data + 3 + sizeof(float) * 6, sizeof(float) * 6);
  Serial.println("Received angles and speeds:");

  uint8_t checksum = 0;
  int fullPayloadSize = 3 + sizeof(float) * 6 + sizeof(float) * 6;
  for (size_t i = 0; i < payloadSize + 3; i++) {
    checksum += data[i];
  }

  if (checksum != data[fullPayloadSize]) {
    Serial.println("Checksum error");
    return;
  }

  Serial.println("Checksum OK");
  for (int i = 0; i < 6; i++) {
    // Print received angles and speeds
    Serial.printf("Motor %d: angle=%.2f, speed=%.2f\n", i+1, angles[i], speeds[i]);
    
    if (motors[i].stepPin != 0 && motors[i].dirPin != 0) {
      motors[i].setTargetAngle(angles[i]);
    }
          
  }
}


void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(SS, INPUT); // Set SS pin as input with pull-up resistor
  Serial.println("SPI Receiver Initialized");

  Serial.println("Initializing motors...");
  for (int i = 0; i < 6; i++) {
    Serial.printf("Initializing Motor %d (pins: step=%d, dir=%d)...\n",
                  i+1, motors[i].stepPin, motors[i].dirPin);
    motors[i].begin();
    Serial.printf("Motor %s initialized successfully\n", motors[i].name.c_str());
  }
}

