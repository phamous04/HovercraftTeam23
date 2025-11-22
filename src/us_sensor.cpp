#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "us_sensor.h"

// Trigger Pins
#define FRONT_LEFT_TRIG_PIN PB5
#define RIGHT_TRIG_PIN      PB3

// Echo Pins
#define FRONT_ECHO_PIN  PB0
#define LEFT_ECHO_PIN   PD3
#define RIGHT_ECHO_PIN  PD2

// Timing Variables
volatile uint16_t left_start = 0, left_end = 0;
volatile uint16_t right_start = 0, right_end = 0;

volatile uint8_t left_done = 0;
volatile uint8_t right_done = 0;

// Initialize Ultrasonic Sensors
void us_init() {
    // Triggers
    DDRB |= (1 << FRONT_LEFT_TRIG_PIN);
    DDRB |= (1 << RIGHT_TRIG_PIN);

    // Echos
    DDRB &= ~(1 << FRONT_ECHO_PIN);
    DDRD &= ~(1 << LEFT_ECHO_PIN);
    DDRB &= ~(1 << RIGHT_ECHO_PIN);

    // Timer1 setup
    TCCR1A = 0;
    TCCR1B = (1<<CS11);

    // Left sensor interrupt INT1 on PD3
    EICRA |= (1 << ISC10);
    EIMSK |= (1 << INT1);

    // Right sensor interrupt INT0 on PD2
    EICRA |= (1 << ISC00);
    EIMSK |= (1 << INT0);
}

// Trigger Functions
void trig_front_left() {
    PORTB |= (1<<FRONT_LEFT_TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1<<FRONT_LEFT_TRIG_PIN);
}

void trig_right() {
    PORTB |= (1<<RIGHT_TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1<<RIGHT_TRIG_PIN);
}

// Interrupt for Left and Right
ISR(INT1_vect) {
    if (PIND & (1<<LEFT_ECHO_PIN)) {
        left_start = TCNT1;
        left_done = 0;
    } else {
        left_end = TCNT1;
        left_done = 1;
    }
}

ISR (INT0_vect) {
    if (PIND & (1<<RIGHT_ECHO_PIN)) {
        right_start = TCNT1;
        right_done = 0;
    } else {
        right_end = TCNT1;
        right_done = 1;
    }
}

// Distance Calculation
float calc_distance(uint16_t start, uint16_t end)
{
    uint16_t dt = (end >= start) ? (end - start)
                : (0xFFFF - start + end);

    return dt * 0.008575f;
}

// Read Sensors
float read_front_sensor() {
    uint16_t start, end;

    // Reset timer + trigger
    TCNT1 = 0;
    trig_front_left();

    // Wait for rising edge
    uint16_t timeout = 0;
    while (!(PINB & (1<<FRONT_ECHO_PIN)))
    {
        if (++timeout > 30000) return -1;
        _delay_us(1);
    }
    start = TCNT1;

    // Wait for falling edge
    while (PINB & (1<<FRONT_ECHO_PIN))
    {
        if (++timeout > 60000) return -1;
        _delay_us(1);
    }
    end = TCNT1;

    return calc_distance(start, end);
}

float read_left_sensor() {
    left_done = 0;
    TCNT1 = 0;
    trig_front_left();

    uint16_t timeout = 0;
    while (!left_done)
    {
        if (++timeout > 30000) return -1;
        _delay_us(1);
    }

    return calc_distance(left_start, left_end);
}

float read_right_sensor()
{
    right_done = 0;
    TCNT1 = 0;
    trig_right();

    uint16_t timeout = 0;
    while (!right_done)
    {
        if (++timeout > 30000) return -1;
        _delay_us(1);
    }

    return calc_distance(right_start, right_end);
}