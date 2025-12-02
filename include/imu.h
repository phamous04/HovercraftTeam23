#ifndef IMU_H
#define IMU_H

#include <stdint.h>

// Public IMU API

// Initialize MPU-6050 (I2C + registers)
void imu_init(void);

// Calibrate gyro Z bias (keep device still while running)
void imu_calibrate(void);

// Update IMU state (call frequently, non-blocking)
void imu_update(void);

// Get current yaw angle (degrees, -180 to +180)
float imu_get_yaw(void);

// Reset yaw angle to 0
void imu_reset_yaw(void);

// Get raw gyro Z (deg/s, UNCORRECTED)
float imu_get_gyro_z(void);

// Get acceleration magnitude (m/s^2)
float imu_get_accel_total(void);

#endif // IMU_H
