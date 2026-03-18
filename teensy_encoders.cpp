#include <Arduino.h>
#include "globals.h"

// ======================================================
// ENCODER PIN DEFINITIONS
// ======================================================

#define LEFT_ENA_PIN   2
#define LEFT_ENB_PIN   3
#define RIGHT_ENA_PIN  4
#define RIGHT_ENB_PIN  5


// ======================================================
// INTERNAL STATE (LAST QUADRATURE STATE)
// ======================================================

static volatile uint8_t lastStateL = 0;
static volatile uint8_t lastStateR = 0;


// ======================================================
// LEFT ENCODER ISR
// ======================================================
// Full quadrature decoding using state transitions
// ======================================================

void leftISR() {

  uint8_t A = digitalRead(LEFT_ENA_PIN);
  uint8_t B = digitalRead(LEFT_ENB_PIN);

  uint8_t state = (A << 1) | B;
  uint8_t combined = (lastStateL << 2) | state;

  // Forward transitions
  if (combined == 0b0001 || combined == 0b0111 ||
      combined == 0b1110 || combined == 0b1000) {
    leftEncoderTicks++;
  }
  // Reverse transitions
  else if (combined == 0b0010 || combined == 0b0100 ||
           combined == 0b1101 || combined == 0b1011) {
    leftEncoderTicks--;
  }

  lastStateL = state;
}


// ======================================================
// RIGHT ENCODER ISR
// ======================================================

void rightISR() {

  uint8_t A = digitalRead(RIGHT_ENA_PIN);
  uint8_t B = digitalRead(RIGHT_ENB_PIN);

  uint8_t state = (A << 1) | B;
  uint8_t combined = (lastStateR << 2) | state;

  // Forward transitions
  if (combined == 0b0001 || combined == 0b0111 ||
      combined == 0b1110 || combined == 0b1000) {
    rightEncoderTicks++;
  }
  // Reverse transitions
  else if (combined == 0b0010 || combined == 0b0100 ||
           combined == 0b1101 || combined == 0b1011) {
    rightEncoderTicks--;
  }

  lastStateR = state;
}


// ======================================================
// INITIALIZATION
// ======================================================

void encodersInit() {

  // ---------------- Pin Setup ----------------
  pinMode(LEFT_ENA_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENB_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENA_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENB_PIN, INPUT_PULLUP);

  // ---------------- Initialize State ----------------
  lastStateL = (digitalRead(LEFT_ENA_PIN) << 1) |
               digitalRead(LEFT_ENB_PIN);

  lastStateR = (digitalRead(RIGHT_ENA_PIN) << 1) |
               digitalRead(RIGHT_ENB_PIN);

  // ---------------- Attach Interrupts ----------------
  attachInterrupt(digitalPinToInterrupt(LEFT_ENA_PIN),  leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENB_PIN),  leftISR,  CHANGE);

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENA_PIN), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENB_PIN), rightISR, CHANGE);
}
