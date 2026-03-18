#pragma once
#include "globals.h"

// ======================================================
// MOTOR CONTROL INTERFACE
// ======================================================
//
// Handles:
// - Motor initialization
// - Closed-loop velocity control (PI)
// - Safety stop + integrator reset
// ======================================================


// Initialize motor driver pins and PWM
void motorsInit();


// Immediately stop both motors (sets PWM = 0)
void stopMotors();


// Reset PI integrators (prevents windup)
void resetPI();


// Dual-wheel velocity control update
// dt: time since last update (seconds)
void updateVelocityControl(float dt);
