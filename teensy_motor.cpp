#include <Arduino.h>
#include "globals.h"
#include "motors.h"

// ======================================================
// MOTOR DRIVER PINS
// ======================================================

#define AN1_PIN 11   // Left motor PWM
#define AN2_PIN 10   // Right motor PWM
#define IN1_PIN 13   // Left motor direction
#define IN2_PIN 12   // Right motor direction

#define PWM_MAX 255
#define MIN_ACTIVE_PWM 25
#define I_CLAMP 180.0f


// ======================================================
// UTILITY FUNCTIONS
// ======================================================

// Clamp value symmetrically around zero
static float clampAbs(float v, float maxAbs) {
  if (v > maxAbs) return maxAbs;
  if (v < -maxAbs) return -maxAbs;
  return v;
}


// Convert encoder ticks → RPM
static float ticksToRPM(long dTicks, float dt) {
  return ((float)dTicks / dt) * (60.0f / CPR);
}


// ======================================================
// INITIALIZATION
// ======================================================

void motorsInit() {
  pinMode(AN1_PIN, OUTPUT);
  pinMode(AN2_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  stopMotors();
}


// ======================================================
// SAFETY / RESET
// ======================================================

void stopMotors() {
  analogWrite(AN1_PIN, 0);
  analogWrite(AN2_PIN, 0);
}


void resetPI() {
  eIntL = 0.0f;
  eIntR = 0.0f;
}


// ======================================================
// MAIN VELOCITY CONTROL LOOP
// ======================================================
// dt: loop period (seconds)
// ======================================================

void updateVelocityControl(float dt) {

  static long prevL = 0;
  static long prevR = 0;

  // ----------------------------------------------------
  // 1. ENCODER DELTA
  // ----------------------------------------------------
  long currL = leftEncoderTicks;
  long currR = rightEncoderTicks;

  long dL = currL - prevL;
  long dR = currR - prevR;

  prevL = currL;
  prevR = currR;

  // ----------------------------------------------------
  // 2. MEASURE RPM (SIGNED)
  // ----------------------------------------------------
  measuredRPM_L = ticksToRPM(dL, dt);
  measuredRPM_R = ticksToRPM(dR, dt);

  // ----------------------------------------------------
  // 3. TARGETS
  // ----------------------------------------------------
  float tgtL = targetRPM_L;
  float tgtR = targetRPM_R;

  // ----------------------------------------------------
  // 4. ERROR
  // ----------------------------------------------------
  float eL = tgtL - measuredRPM_L;
  float eR = tgtR - measuredRPM_R;

  // ----------------------------------------------------
  // 5. PI CONTROL (ANTI-WINDUP)
  // ----------------------------------------------------
  float uL = KpL * eL + KiL * eIntL;
  float uR = KpR * eR + KiR * eIntR;

  // Detect saturation BEFORE integration
  bool satL = (uL >= PWM_MAX) || (uL <= -PWM_MAX);
  bool satR = (uR >= PWM_MAX) || (uR <= -PWM_MAX);

  // Integrate only if not saturated
  if (!satL) eIntL += eL * dt;
  if (!satR) eIntR += eR * dt;

  // Clamp integrator (anti-windup)
  eIntL = clampAbs(eIntL, I_CLAMP);
  eIntR = clampAbs(eIntR, I_CLAMP);

  // Recompute control after integration
  uL = KpL * eL + KiL * eIntL;
  uR = KpR * eR + KiR * eIntR;

  // Final output clamp
  uL = constrain(uL, -PWM_MAX, PWM_MAX);
  uR = constrain(uR, -PWM_MAX, PWM_MAX);

  // ----------------------------------------------------
  // 6. DIRECTION CONTROL
  // ----------------------------------------------------
  // INx = LOW  → forward
  // INx = HIGH → reverse

  digitalWrite(IN1_PIN, (uL >= 0) ? LOW : HIGH);
  digitalWrite(IN2_PIN, (uR >= 0) ? LOW : HIGH);

  // ----------------------------------------------------
  // 7. PWM OUTPUT
  // ----------------------------------------------------
  pwmL = abs((int)uL);
  pwmR = abs((int)uR);

  // Enforce minimum active PWM if command is nonzero
  if (abs(tgtL) > 0 && pwmL > 0 && pwmL < MIN_ACTIVE_PWM)
    pwmL = MIN_ACTIVE_PWM;

  if (abs(tgtR) > 0 && pwmR > 0 && pwmR < MIN_ACTIVE_PWM)
    pwmR = MIN_ACTIVE_PWM;

  analogWrite(AN1_PIN, pwmL);
  analogWrite(AN2_PIN, pwmR);
}
