#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

#include "imu.h"
#include "i2c.h"



#define IMU_ADDR            0x68

// MPU-6050 registers
#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B

#define GYRO_SENS           65.5f
#define ACCEL_SENS          8192.0f
#define GRAVITY             9.81f

#define IMU_UPDATE_INTERVAL_US   10000UL

#define GYRO_LP_ALPHA       0.4f

#define ACC_MAG_TOL_G       0.15f
#define GZ_STATIONARY_THR   1.0f
#define BIAS_ADAPT_RATE     0.001f

#define IMU_DEBUG           0


// =========================
// Internal state
// =========================

static volatile float imu_yaw_deg      = 0.0f;   // yaw angle (deg)
static volatile float gyro_z_dps       = 0.0f;   // raw gyro Z in deg/s
static volatile float gyro_z_bias      = 0.0f;   // bias in deg/s
static volatile float accel_total_mps2 = 0.0f;   // accel magnitude (m/s^2)

static float gyro_z_filtered = 0.0f;            // filtered gyro Z (deg/s)

static uint32_t last_imu_time_us = 0;
static uint8_t  imu_first_update = 1;


// =========================
// Local helpers
// =========================

static void mpu_write_reg(uint8_t reg, uint8_t val)
{
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);   // write
    i2c_write(reg);
    i2c_write(val);
    i2c_stop();
}

static void mpu_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(reg);

    i2c_start();
    i2c_write((IMU_ADDR << 1) | 1);

    for (uint8_t i = 0; i < len; i++)
    {
        if (i < (len - 1))
            buf[i] = i2c_read_ack();
        else
            buf[i] = i2c_read_nack();
    }

    i2c_stop();
}


// =========================
// Public functions
// =========================

void imu_init(void)
{
    i2c_init();

#if IMU_DEBUG
    uart_print("IMU: init...\r\n");
#endif

    mpu_write_reg(REG_PWR_MGMT_1, 0x00);
    _delay_ms(100);

    mpu_write_reg(REG_SMPLRT_DIV, 0x07);

    mpu_write_reg(REG_CONFIG, 0x03);

    mpu_write_reg(REG_GYRO_CONFIG, (1 << 3));

    mpu_write_reg(REG_ACCEL_CONFIG, (1 << 3));

    imu_yaw_deg      = 0.0f;
    gyro_z_bias      = 0.0f;
    gyro_z_filtered  = 0.0f;
    last_imu_time_us = 0;
    imu_first_update = 1;

#if IMU_DEBUG
    uart_print("IMU: init done.\r\n");
#endif
}

void imu_calibrate(void)
{
    const uint16_t samples = 200;
    float sum_gz = 0.0f;

#if IMU_DEBUG
    uart_print("IMU: calibration, keep hovercraft STILL...\r\n");
#endif

    for (uint16_t i = 0; i < samples; i++)
    {
        uint8_t d[14];
        mpu_read_bytes(REG_ACCEL_XOUT_H, d, 14);

        int16_t gz_raw = (int16_t)((d[12] << 8) | d[13]);
        float   gz_dps = (float)gz_raw / GYRO_SENS;

        sum_gz += gz_dps;

        _delay_ms(5);
    }

    gyro_z_bias = sum_gz / (float)samples;

#if IMU_DEBUG
    uart_print("IMU: calibration done. Gyro Z bias = ");
    uart_print_float(gyro_z_bias);
    uart_print("\r\n");
#endif
}

// Core update
void imu_update(void)
{
    uint32_t now = micros();

    if (imu_first_update)
    {
        last_imu_time_us = now;
        imu_first_update = 0;
        return;
    }

    uint32_t dt_us = now - last_imu_time_us;

    if (dt_us < IMU_UPDATE_INTERVAL_US)
        return;

    last_imu_time_us = now;
    float dt_sec = dt_us / 1000000.0f;

    uint8_t d[14];
    mpu_read_bytes(REG_ACCEL_XOUT_H, d, 14);

    int16_t ax_raw = (int16_t)((d[0] << 8) | d[1]);
    int16_t ay_raw = (int16_t)((d[2] << 8) | d[3]);
    int16_t az_raw = (int16_t)((d[4] << 8) | d[5]);

    int16_t gz_raw = (int16_t)((d[12] << 8) | d[13]);

    float ax = ((float)ax_raw / ACCEL_SENS) * GRAVITY;
    float ay = ((float)ay_raw / ACCEL_SENS) * GRAVITY;
    float az = ((float)az_raw / ACCEL_SENS) * GRAVITY;

    float acc_mag = sqrtf(ax * ax + ay * ay + az * az);
    accel_total_mps2 = acc_mag;

    gyro_z_dps = (float)gz_raw / GYRO_SENS;
    float gz_corrected = gyro_z_dps - gyro_z_bias;

    gyro_z_filtered = gyro_z_filtered + GYRO_LP_ALPHA * (gz_corrected - gyro_z_filtered);

    imu_yaw_deg += gyro_z_filtered * dt_sec;

    if (imu_yaw_deg > 180.0f)  imu_yaw_deg -= 360.0f;
    if (imu_yaw_deg < -180.0f) imu_yaw_deg += 360.0f;

    float acc_err_g = fabsf((acc_mag / GRAVITY) - 1.0f);
    if (acc_err_g < ACC_MAG_TOL_G && fabsf(gz_corrected) < GZ_STATIONARY_THR)
    {
        // Move bias slightly towards current corrected value (which should be ~0)
        gyro_z_bias += BIAS_ADAPT_RATE * gz_corrected;
    }

#if IMU_DEBUG
    uart_print("IMU: dt_us=");
    uart_print_float((float)dt_us);
    uart_print(" yaw=");
    uart_print_float(imu_yaw_deg);
    uart_print(" gz=");
    uart_print_float(gyro_z_dps);
    uart_print(" bias=");
    uart_print_float(gyro_z_bias);
    uart_print("\r\n");
#endif
}

float imu_get_yaw(void)
{
    return imu_yaw_deg;
}

void imu_reset_yaw(void)
{
    imu_yaw_deg = 0.0f;
}

float imu_get_gyro_z(void)
{
    return gyro_z_dps;
}

float imu_get_accel_total(void)
{
    return accel_total_mps2;
}
