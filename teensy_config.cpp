#include "globals.h"

// ======================================================
// MOTOR TARGETS
// ======================================================
// Set by Jetson (via serial), consumed by control loop

volatile float targetRPM_L = 0.0f;
volatile float targetRPM_R = 0.0f;

// Measured wheel speeds (computed from encoders)
volatile float measuredRPM_L = 0.0f;
volatile float measuredRPM_R = 0.0f;


// ======================================================
// RPM LIMITS
// ======================================================

float MAX_ALLOWED_RPM = 160.0f;
float MIN_ALLOWED_RPM = 30.0f;


// ======================================================
// AGV PHYSICAL PARAMETERS
// ======================================================

float WHEEL_DIAMETER_M = 0.066675f;
float WHEEL_RADIUS_M   = 0.0333375f;
float WHEEL_BASE_M     = 0.1190625f;


// ======================================================
// PI CONTROLLER (LEFT / RIGHT)
// ======================================================

// Gains
float KpL = 0.8f;
float KiL = 3.0f;

float KpR = 0.8f;
float KiR = 3.0f;

// Integral accumulators
float eIntL = 0.0f;
float eIntR = 0.0f;

// PWM outputs (final motor commands)
int pwmL = 0;
int pwmR = 0;


// ======================================================
// ENCODERS
// ======================================================

// Counts per revolution
float CPR = 1760.0f;

// Tick counters (updated in ISR)
volatile long leftEncoderTicks  = 0;
volatile long rightEncoderTicks = 0;

// Previous tick values (for delta calculations)
volatile long prevLeftTicks  = 0;
volatile long prevRightTicks = 0;


// ======================================================
// ACTUATOR (FORK CONTROL)
// ======================================================

// -1 = retract, 0 = stop, 1 = extend
volatile int actuatorState = 0;


// ======================================================
// COMMAND WATCHDOG
// ======================================================

// Last time a valid command was received (ms)
volatile unsigned long lastCommandTime = 0;

// Timeout threshold (ms)
const unsigned long COMMAND_TIMEOUT_MS = 200;


// ======================================================
// TELEMETRY / DEBUG
// ======================================================

// IMU update counter (used for rate monitoring)
volatile unsigned long imuUpdateCount = 0;

// General telemetry loop counter
volatile unsigned long telemetryCounter = 0;
