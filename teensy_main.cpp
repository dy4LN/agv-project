#include "globals.h"
#include "motors.h"
#include "encoders.h"
#include "serial_control.h"
#include "telemetry.h"
#include "actuator.h"
#include "imu.h"

// =====================================================
// CONTROL LOOP CONFIG
// =====================================================

#define CONTROL_PERIOD_MS 20   // 50 Hz motor control loop


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(460800);

  // ---------------- Hardware Init ----------------
  motorsInit();
  encodersInit();
  actuatorInit();
  imuInit();

  // ---------------- Interfaces ----------------
  serialInit();
  telemetryInit();
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  // =================================================
  // 1. SERIAL COMMAND HANDLING (cmd_vel, actuator, etc.)
  // =================================================
  serialUpdate();

  // =================================================
  // 2. ACTUATOR CONTROL (fork lift, etc.)
  // =================================================
  actuatorUpdate();

  // =================================================
  // 3. IMU STREAMING (runs at its own rate internally)
  // =================================================
  imuUpdate();

  // =================================================
  // 4. VELOCITY CONTROL LOOP (FIXED RATE ~50 Hz)
  // =================================================
  static unsigned long lastCtrlTime = 0;
  unsigned long now = millis();

  if (now - lastCtrlTime >= CONTROL_PERIOD_MS) {

    float dt = (now - lastCtrlTime) / 1000.0f;  // seconds
    lastCtrlTime = now;

    // ---- Closed-loop motor control ----
    updateVelocityControl(dt);

    // ---- Telemetry feedback ----
    telemetryUpdate(dt);
  }

  // Allow background tasks (USB, etc.)
  yield();
}
