#include <Arduino.h>
#include "globals.h"

// ======================================================
// INTERNAL STATE
// ======================================================

static long prevL = 0;
static long prevR = 0;

// IMU frequency tracking
static unsigned long prevImuCount = 0;
static unsigned long lastImuRateTime = 0;
static float imuHz = 0.0f;


// ======================================================
// INITIALIZATION
// ======================================================

void telemetryInit() {
  prevL = leftEncoderTicks;
  prevR = rightEncoderTicks;
  lastImuRateTime = millis();
}


// ======================================================
// UTILITY
// ======================================================

// Convert encoder ticks → RPM
static float ticksToRPM(long dTicks, float dt) {
  return ((float)dTicks / dt) * (60.0f / CPR);
}


// ======================================================
// MAIN TELEMETRY UPDATE
// ======================================================
// dt: control loop period (seconds)
// ======================================================

void telemetryUpdate(float dt) {

  // ----------------------------------------------------
  // TIMESTAMP
  // ----------------------------------------------------
  unsigned long ts_us = micros();

  // ----------------------------------------------------
  // ENCODER DELTA
  // ----------------------------------------------------
  long currL = leftEncoderTicks;
  long currR = rightEncoderTicks;

  long dL = currL - prevL;
  long dR = currR - prevR;

  prevL = currL;
  prevR = currR;

  // ----------------------------------------------------
  // RPM (SIGNED)
  // ----------------------------------------------------
  float rpmL_signed = ticksToRPM(dL, dt);
  float rpmR_signed = ticksToRPM(dR, dt);

  // ----------------------------------------------------
  // ENCODER RATE (TICKS / SEC)
  // ----------------------------------------------------
  float encHzL = (float)dL / dt;
  float encHzR = (float)dR / dt;

  // ----------------------------------------------------
  // VELOCITY (DIFFERENTIAL DRIVE)
  // ----------------------------------------------------
  float vL = (rpmL_signed / 60.0f) * (2.0f * PI * WHEEL_RADIUS_M);
  float vR = (rpmR_signed / 60.0f) * (2.0f * PI * WHEEL_RADIUS_M);

  float linearVelocity  = 0.5f * (vL + vR);
  float angularVelocity = (vR - vL) / WHEEL_BASE_M;

  // ----------------------------------------------------
  // IMU UPDATE RATE (Hz)
  // ----------------------------------------------------
  unsigned long now = millis();

  if (now - lastImuRateTime >= 1000) {
    unsigned long imuDelta = imuUpdateCount - prevImuCount;
    imuHz = (float)imuDelta;
    prevImuCount = imuUpdateCount;
    lastImuRateTime = now;
  }

  // ----------------------------------------------------
  // TELEMETRY STREAM (CSV)
  // ----------------------------------------------------
  // FORMAT:
  // TEL,
  // rpmL,rpmR,
  // encHzL,encHzR,
  // targetL,targetR,
  // pwmL,pwmR,
  // linearVel,angularVel,
  // imuHz,
  // actuatorState,
  // timestamp_us

  Serial.print("TEL,");

  Serial.print(rpmL_signed, 3); Serial.print(",");
  Serial.print(rpmR_signed, 3); Serial.print(",");

  Serial.print(encHzL, 1);      Serial.print(",");
  Serial.print(encHzR, 1);      Serial.print(",");

  Serial.print(targetRPM_L, 2); Serial.print(",");
  Serial.print(targetRPM_R, 2); Serial.print(",");

  Serial.print(pwmL);           Serial.print(",");
  Serial.print(pwmR);           Serial.print(",");

  Serial.print(linearVelocity, 4);  Serial.print(",");
  Serial.print(angularVelocity, 4); Serial.print(",");

  Serial.print(imuHz, 1);       Serial.print(",");
  Serial.print(actuatorState);  Serial.print(",");
  Serial.println(ts_us);
}
