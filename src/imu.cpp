#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include "imu.h"
#include "i2c.h"
#include "uart.h"      // For debugging prints (optional but you’re already using it)

// =========================
// Internal IMU state
// =========================
float gz_offset   = 0.0f;    // Gyro Z bias (deg/s)
float gyro_z_raw  = 0.0f;    // Latest gyro Z reading (deg/s)
float gyro_z      = 0.0f;    // Calibrated gyro Z (deg/s)
#define IMU_UPDATE_INTERVAL_US  10000
uint32_t last_imu_time = 0;

float yaw_angle   = 0.0f;    // Integrated yaw angle (degrees)

// For ±500°/s: 65.5 LSB / (deg/s)
float gyro_scale  = 65.5f;


void imu_init()
{
    uart_print("Initializing IMU...\r\n");

    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x6B);
    i2c_write(0x00);
    i2c_stop();

    _delay_ms(10);

    // Set gyro
    i2c_start();
    i2c_write(IMU_ADDR << 1);
    i2c_write(0x1B);
    i2c_write(1 << 3); // Set gyro to ±500°/s
    i2c_stop();

    // Set accelerometer
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1C);     // ACCEL_CONFIG
    i2c_write(1 << 3);   // ±4g
    i2c_stop();
}

void imu_read() {
    uint8_t data[14];

    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x3B);
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 1);
    for (uint8_t i = 0; i < 13; i++) {
        data[i] = i2c_read_ack(); 
    }
    data[13] = i2c_read_nack();
    i2c_stop();

    gyro_z_raw = (int16_t)((data[12] << 8) | data[13]);

    gyro_z = (float)gyro_z_raw / gyro_scale;
}

void imu_calibrate() {
    uint16_t samples = 1000;
    float gz_sum = 0.0f;

    uart_print("Calibrating IMU...\r\n");

    for (uint16_t i = 0; i < samples; i++) {
        imu_read();
        gz_sum += gyro_z;
        _delay_ms(2);
    }

    gz_offset = gz_sum / samples;

    uart_print("IMU Calibration complete. Gyro Z offset: ");
    uart_print_float(gz_offset);
    uart_print("\r\n");
}

void imu_update() {
    uint32_t now = micros();

    if (now - last_imu_time < IMU_UPDATE_INTERVAL_US)
        return;

    last_imu_time = now;

    imu_read();
    float corrected_gz = gyro_z - gz_offset;
    float dt_sec = IMU_UPDATE_INTERVAL_US / 1000000.0f;

    // Low-pass filter on gyro
    static float gyro_filtered = 0.0f;
    gyro_filtered = 0.9f * gyro_filtered + 0.1f * corrected_gz;

    yaw_angle += corrected_gz * dt_sec;

    if (yaw_angle > 180.0f) yaw_angle -= 360.0f;
    if (yaw_angle < -180.0f) yaw_angle += 360.0f;
}

float imu_get_yaw() { 
    return yaw_angle;
}

void imu_reset_yaw() {
    yaw_angle = 0;
}

float imu_get_gyro_z() {
    return gyro_z;
}