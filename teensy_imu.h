#pragma once
#include "globals.h"

// ======================================================
// IMU INTERFACE
// ======================================================
//
// Handles:
// - IMU initialization (I2C + sensor setup)
// - Periodic IMU updates / data streaming
// ======================================================


// Initialize IMU hardware (e.g., BNO055)
void imuInit();


// Update IMU readings and publish/track state
void imuUpdate();
