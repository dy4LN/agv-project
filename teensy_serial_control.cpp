#include <Arduino.h>
#include "globals.h"
#include "motors.h"

// ======================================================
// SERIAL RX BUFFER
// ======================================================

static String rxLine = "";


// ======================================================
// INIT
// ======================================================

void serialInit() {
  lastCommandTime = millis();
}


// ======================================================
// UTILITY: CLAMP
// ======================================================

static float clamp(float v, float minV, float maxV) {
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return v;
}


// ======================================================
// SERIAL UPDATE LOOP
// ======================================================
// Expected format from Jetson:
//   "RPM <rpmL> <rpmR> <lift>\n"
// Example:
//   RPM 50.0 50.0 1
// ======================================================

void serialUpdate() {

  // ================= READ SERIAL STREAM =================
  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {

      rxLine.trim();

      // ---------------- COMMAND PARSE ----------------
      if (rxLine.startsWith("RPM")) {

        float rpmL, rpmR;
        int lift;

        int parsed = sscanf(
          rxLine.c_str(),
          "RPM %f %f %d",
          &rpmL,
          &rpmR,
          &lift
        );

        if (parsed == 3) {

          // ---------------- CLAMP MAX RPM ----------------
          rpmL = clamp(rpmL, -MAX_ALLOWED_RPM, MAX_ALLOWED_RPM);
          rpmR = clamp(rpmR, -MAX_ALLOWED_RPM, MAX_ALLOWED_RPM);

          // ---------------- APPLY MIN DEADZONE ----------------
          if (abs(rpmL) > 0 && abs(rpmL) < MIN_ALLOWED_RPM)
            rpmL = (rpmL > 0 ? MIN_ALLOWED_RPM : -MIN_ALLOWED_RPM);

          if (abs(rpmR) > 0 && abs(rpmR) < MIN_ALLOWED_RPM)
            rpmR = (rpmR > 0 ? MIN_ALLOWED_RPM : -MIN_ALLOWED_RPM);

          // ---------------- UPDATE GLOBAL STATE ----------------
          targetRPM_L = rpmL;
          targetRPM_R = rpmR;
          actuatorState = lift;

          lastCommandTime = millis();
        }
      }

      // Clear buffer after processing line
      rxLine = "";
    }
    else {
      rxLine += c;
    }
  }

  // ================= WATCHDOG FAILSAFE =================
  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {

    targetRPM_L = 0.0f;
    targetRPM_R = 0.0f;
    actuatorState = 0;

    // Reset controller to avoid windup after timeout
    resetPI();
  }
}
