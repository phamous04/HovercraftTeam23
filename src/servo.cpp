#include <avr/io.h>
#include <avr/interrupt.h>
#include "servo.h"

#define SERVO_PULSE_CENTER_US   1500.0f
#define SERVO_PULSE_RANGE_US    600.0f

static inline uint16_t us_to_ticks(float us) {
    return (uint16_t)(us * 2.0f + 0.5f);
}

void servo_init(void) {
    DDRB |= (1 << PB1);

    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << WGM12) | (1 << CS11);

    ICR1 = 40000;  // 20ms period

    OCR1A = 3000;
}

void setServoAngle(float angle) {
    if (angle > 180.0f) angle = 180.0f;
    if (angle < -180.0f) angle = -180.0f;

    float pulse_us = SERVO_PULSE_CENTER_US + (angle * (SERVO_PULSE_RANGE_US / 90.0f));


    uint16_t ticks = us_to_ticks(pulse_us);

    OCR1A = ticks;
}