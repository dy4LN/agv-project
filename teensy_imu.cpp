#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "imu.h"

// ======================================================
// IMU CONFIGURATION
// ======================================================

#define BNO055_ADDR   0x28
#define IMU_PERIOD_MS 10   // 100 Hz update rate

static Adafruit_BNO055 bno(55, BNO055_ADDR);
static unsigned long lastImuTime = 0;


// ======================================================
// INITIALIZATION
// ======================================================

void imuInit() {

  Wire.begin();

  if (!bno.begin()) {
    // Hard fail if IMU not detected
    while (1) {}
  }

  // Use external crystal for better stability
  bno.setExtCrystalUse(true);

  lastImuTime = millis();
}


// ======================================================
// UPDATE LOOP
// ======================================================

void imuUpdate() {

  // Maintain fixed update rate (~100 Hz)
  if (millis() - lastImuTime < IMU_PERIOD_MS)
    return;

  lastImuTime = millis();

  // Track update frequency (for telemetry/debug)
  imuUpdateCount++;

  // ----------------------------------------------------
  // READ SENSOR DATA
  // ----------------------------------------------------
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion quat = bno.getQuat();

  // ----------------------------------------------------
  // SERIAL TELEMETRY (CSV FORMAT)
  // ----------------------------------------------------
  // Format:
  // IMU, ax, ay, az, gx, gy, gz, qx, qy, qz, qw

  Serial.print("IMU,");

  Serial.print(accel.x(), 3); Serial.print(",");
  Serial.print(accel.y(), 3); Serial.print(",");
  Serial.print(accel.z(), 3); Serial.print(",");

  Serial.print(gyro.x(), 3);  Serial.print(",");
  Serial.print(gyro.y(), 3);  Serial.print(",");
  Serial.print(gyro.z(), 3);  Serial.print(",");

  Serial.print(quat.x(), 6);  Serial.print(",");
  Serial.print(quat.y(), 6);  Serial.print(",");
  Serial.print(quat.z(), 6);  Serial.print(",");
  Serial.println(quat.w(), 6);
}
