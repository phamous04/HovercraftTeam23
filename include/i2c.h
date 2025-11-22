#ifndef I2C_H
#define I2C_H

#include <stdint.h>

// Initialize I2C (TWI)
void i2c_init();

// Start condition
void i2c_start();

// Repeated start condition
void i2c_restart();

// Stop condition
void i2c_stop();

// Write 8-bit data
void i2c_write(uint8_t data);

// Read byte with ACK
uint8_t i2c_read_ack();

// Read byte with NACK
uint8_t i2c_read_nack();

#endif
