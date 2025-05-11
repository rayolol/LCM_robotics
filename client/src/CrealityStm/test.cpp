#define STEP_PIN 2
#define DIR_PIN 3

#include <Arduino.h>
#include <SPI.h>

volatile int16_t motorSpeed = 0;

void stepperTask() {
  static bool dirSet = false;
  if (motorSpeed == 0) return;

  if (!dirSet) {
    digitalWrite(DIR_PIN, motorSpeed > 0 ? HIGH : LOW);
    dirSet = true;
  }

  int delayMicros = abs(1000000 / motorSpeed);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delayMicros - 5);
}

void onSpiReceive() {
  static uint8_t state = 0;
  static uint8_t jointID;
  static int16_t value;

  uint8_t byte = SPDR;
  switch (state) {
    case 0: jointID = byte; state++; break;
    case 1: value = byte << 8; state++; break;
    case 2:
      value |= byte;
      if (jointID == 0x01) motorSpeed = value;
      state = 0;
      break;
  }
}

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  SPI.attachInterrupt();
  SPCR |= _BV(SPE);
}

void loop() {
  stepperTask();
}
ISR(SPI_STC_vect) {
  onSpiReceive();
}
