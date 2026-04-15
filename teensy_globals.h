#pragma once
#include <Arduino.h>

// ======================================================
// GLOBAL SHARED STATE
// ======================================================
// NOTE:
// These variables are shared across multiple modules.
// Many are marked `volatile` because they are updated
// in ISRs or asynchronous contexts (e.g., encoders).
// ======================================================


// ======================================================
// MOTOR TARGETS
// ======================================================

// Target RPM received from Jetson
extern volatile float targetRPM_L;
extern volatile float targetRPM_R;

// Measured wheel RPM (from encoders)
extern volatile float measuredRPM_L;
extern volatile float measuredRPM_R;


// ======================================================
// RPM LIMITS
// ======================================================

extern float MAX_ALLOWED_RPM;
extern float MIN_ALLOWED_RPM;


// ======================================================
// AGV PHYSICAL PARAMETERS
// ======================================================

extern float WHEEL_DIAMETER_M;
extern float WHEEL_RADIUS_M;
extern float WHEEL_BASE_M;


// ======================================================
// PI CONTROLLER (LEFT / RIGHT WHEELS)
// ======================================================

// Gains
extern float KpL, KiL;
extern float KpR, KiR;

// Integral error terms
extern float eIntL;
extern float eIntR;

// Output PWM values
extern int pwmL;
extern int pwmR;


// ======================================================
// ENCODERS
// ======================================================

// Counts per revolution
extern float CPR;

// Encoder tick counters (updated in ISR)
extern volatile long leftEncoderTicks;
extern volatile long rightEncoderTicks;

// Previous tick values (for delta computation)
extern volatile long prevLeftTicks;
extern volatile long prevRightTicks;


// ======================================================
// ACTUATOR (FORK CONTROL)
// ======================================================

// -1 = retract, 0 = stop, 1 = extend
extern volatile int actuatorState;


// ======================================================
// COMMAND WATCHDOG
// ======================================================

// Last time a valid command was received (ms)
extern volatile unsigned long lastCommandTime;

// Timeout threshold (ms)
extern const unsigned long COMMAND_TIMEOUT_MS;


// ======================================================
// TELEMETRY / DEBUG
// ======================================================

// IMU update counter (used for rate estimation)
extern volatile unsigned long imuUpdateCount;

// General telemetry loop counter
extern volatile unsigned long telemetryCounter;
