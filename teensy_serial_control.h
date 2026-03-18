#pragma once

// ======================================================
// SERIAL CONTROL INTERFACE
// ======================================================
//
// Handles communication with Jetson:
// - Receives velocity / actuator commands
// - Updates global targets (RPM, actuator state)
// ======================================================

void serialInit();
void serialUpdate();
