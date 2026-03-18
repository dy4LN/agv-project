#include <Arduino.h>
#include "globals.h"

// ======================================================
// ACTUATOR PIN DEFINITIONS
// ======================================================

#define ACT_IN1_PIN 7
#define ACT_IN2_PIN 8
#define ACT_EN_PIN  9


// ======================================================
// FORWARD DECLARATIONS (LOCAL HELPERS)
// ======================================================

static void actuatorExtend();
static void actuatorRetract();
static void actuatorStop();


// ======================================================
// INITIALIZATION
// ======================================================

void actuatorInit() {

  pinMode(ACT_IN1_PIN, OUTPUT);
  pinMode(ACT_IN2_PIN, OUTPUT);
  pinMode(ACT_EN_PIN, OUTPUT);

  actuatorStop();
}


// ======================================================
// LOW-LEVEL ACTUATOR CONTROL
// ======================================================
// IN1 / IN2 set direction
// EN enables motion
// ======================================================

static void actuatorExtend() {
  digitalWrite(ACT_IN1_PIN, LOW);
  digitalWrite(ACT_IN2_PIN, HIGH);
  digitalWrite(ACT_EN_PIN, HIGH);
}

static void actuatorRetract() {
  digitalWrite(ACT_IN1_PIN, HIGH);
  digitalWrite(ACT_IN2_PIN, LOW);
  digitalWrite(ACT_EN_PIN, HIGH);
}

static void actuatorStop() {
  digitalWrite(ACT_IN1_PIN, LOW);
  digitalWrite(ACT_IN2_PIN, LOW);
  digitalWrite(ACT_EN_PIN, LOW);
}


// ======================================================
// UPDATE LOOP
// ======================================================
// Uses global actuatorState:
//   1  → extend
//  -1  → retract
//   0  → stop
// ======================================================

void actuatorUpdate() {

  if (actuatorState == 1) {
    actuatorExtend();
  }
  else if (actuatorState == -1) {
    actuatorRetract();
  }
  else {
    actuatorStop();
  }
}
