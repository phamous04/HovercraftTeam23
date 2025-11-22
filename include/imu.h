#ifndef IMU_H
#define IMU_H

#include <stdint.h>

// MPU-6050 I2C address (AD0 = GND)
#define IMU_ADDR 0x68

void imu_init();
void imu_read();
void imu_calibrate();
void imu_update();

float imu_get_yaw();
void imu_reset_yaw();
float imu_get_gyro_z();

#endif
