#pragma once

// ======================================================
// ACTUATOR INTERFACE (FORK CONTROL)
// ======================================================
//
// Handles:
// - Initialization of actuator hardware
// - Applying actuatorState commands (-1, 0, 1)
// ======================================================


// Initialize actuator pins / driver
void actuatorInit();


// Update actuator based on actuatorState:
//  -1 → retract
//   0 → stop
//   1 → extend
void actuatorUpdate();
