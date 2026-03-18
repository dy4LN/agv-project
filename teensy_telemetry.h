#pragma once

// ======================================================
// TELEMETRY INTERFACE
// ======================================================
//
// Handles:
// - Initialization of telemetry system
// - Periodic transmission of robot state (RPM, IMU, etc.)
// ======================================================


// Initialize telemetry system (counters, timing, etc.)
void telemetryInit();


// Update and transmit telemetry data
// dt: time since last update (seconds)
void telemetryUpdate(float dt);
