#include <avr/io.h>
#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "us_sensor.h"

// =========================================================
// HARD-REALTIME SAFE ULTRASONIC DRIVER (MICROSECOND BASED)
// Front: TRIG = PB5, ECHO = PD3 (INT1)
// Right: TRIG = PB3, ECHO = PD2 (INT0)
// =========================================================

// ---- PINS ----
#define FRONT_TRIG_PIN  PB5
#define RIGHT_TRIG_PIN  PB3

#define FRONT_ECHO_PIN  PD3
#define RIGHT_ECHO_PIN  PD2

// ---- TIMESTAMPS ----
volatile uint32_t front_start_us = 0;
volatile uint32_t front_end_us   = 0;

volatile uint32_t right_start_us = 0;
volatile uint32_t right_end_us   = 0;

// ---- OUTPUT ----
uint16_t front_cm = 9999;
uint16_t right_cm = 9999;


// =========================================================
// INIT
// =========================================================

void us_init(void)
{
    // TRIG PINS OUTPUT
    DDRB |= (1 << FRONT_TRIG_PIN) | (1 << RIGHT_TRIG_PIN);
    PORTB &= ~((1 << FRONT_TRIG_PIN) | (1 << RIGHT_TRIG_PIN));

    // ECHO PINS INPUT
    DDRD &= ~((1 << FRONT_ECHO_PIN) | (1 << RIGHT_ECHO_PIN));
    PORTD &= ~((1 << FRONT_ECHO_PIN) | (1 << RIGHT_ECHO_PIN));

    // External interrupts on any edge
    EICRA = (1 << ISC10) | (1 << ISC00);
    EIMSK = (1 << INT1) | (1 << INT0);

    front_start_us = front_end_us = 0;
    right_start_us = right_end_us = 0;
}


// =========================================================
// ISR HANDLERS (TIMESTAMP ONLY)
// =========================================================

ISR(INT1_vect)
{
    if (PIND & (1 << FRONT_ECHO_PIN))
        front_start_us = micros();
    else
        front_end_us = micros();
}

ISR(INT0_vect)
{
    if (PIND & (1 << RIGHT_ECHO_PIN))
        right_start_us = micros();
    else
        right_end_us = micros();
}


// =========================================================
// TRIGGERS
// =========================================================

static void trig_front(void)
{
    PORTB |= (1 << FRONT_TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << FRONT_TRIG_PIN);
}

static void trig_right(void)
{
    PORTB |= (1 << RIGHT_TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << RIGHT_TRIG_PIN);
}


// =========================================================
// UPDATE FUNCTION (ALL MATH HERE)
// =========================================================

void us_update(void)
{
    // ----- FRONT -----
    trig_front();
    _delay_ms(25);

    if (front_end_us > front_start_us)
    {
        uint32_t dt_us = front_end_us - front_start_us;
        front_cm = dt_us / 58;      // standard HC-SR04 conversion
    }
    else
        front_cm = 9999;

    front_start_us = front_end_us = 0;


    // ----- RIGHT -----
    trig_right();
    _delay_ms(25);

    if (right_end_us > right_start_us)
    {
        uint32_t dt_us = right_end_us - right_start_us;
        right_cm = dt_us / 58;
    }
    else
        right_cm = 9999;

    right_start_us = right_end_us = 0;
}
