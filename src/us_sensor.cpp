#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "us_sensor.h"

#include <time.h>

// Trigger Pins (Right : P6, Left : P13, Front : P10)
#define FRONT_LEFT_TRIG_PIN PB5
#define RIGHT_TRIG_PIN      PB3

// Echo Pins
#define FRONT_ECHO_PIN  PB0
#define LEFT_ECHO_PIN   PD3 // INT1
#define RIGHT_ECHO_PIN  PD2 // INT0

// Timing Variables
volatile uint8_t left_state = 0;
volatile uint8_t right_state = 0;
volatile uint8_t front_state = 0;

volatile uint16_t left_last_cm = 0;
volatile uint16_t right_last_cm = 0;
volatile uint16_t front_last_cm = 0;

uint16_t left_cm = 0;
uint16_t right_cm = 0;
uint16_t front_cm = 0;

// Initialize Ultrasonic Sensors
void us_init() {
    // Triggers
    DDRB |= (1 << FRONT_LEFT_TRIG_PIN);
    DDRB |= (1 << RIGHT_TRIG_PIN);

    // Echos
    DDRB &= ~(1 << FRONT_ECHO_PIN);
    DDRD &= ~(1 << LEFT_ECHO_PIN);
    DDRD &= ~(1 << RIGHT_ECHO_PIN);

    // Ensures no pullups
    PORTB &= ~(1 << FRONT_ECHO_PIN);
    PORTD &= ~(1 << LEFT_ECHO_PIN);
    PORTD &= ~(1 << RIGHT_ECHO_PIN);

    // Timer1 setup
    TCCR1A = 0;
    TCCR1B = (1 << CS11);

    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0);

    // Left sensor interrupt INT1 on PD3, Right sensor interrupt INT0 on PD2
    EICRA |= (1 << ISC10) | (1 << ISC00);
    EICRA &= ~((1 << ISC01) | (1 << ISC11));

    EIMSK |= (1 << INT1) | (1 << INT0);

    left_state = 0;
    right_state = 0;
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

void us_trig(char s) {
    if (s == 'f') {
        trig_front_left();
    }
    else if (s == 'l') {
        trig_front_left();
    }
    else if (s == 'r'){
        trig_right();
    }
}

// ticks -> cm with Timer1 @ F_CPU/8 (16MHz -> 2MHz -> 0.5us/tick)
// distance (cm) ≈ ticks / 116
uint16_t ticks_to_cm(uint16_t ticks)
{
    return ticks / 116;
}

// Interrupt for Left and Right
ISR(INT1_vect) {
    uint8_t high = PIND & (1<<LEFT_ECHO_PIN);

    if (left_state == 0) {
        if (high) {
            TCNT1 = 0;
            left_state = 1;
        }
    } else {
        if (!high) {
            uint16_t ticks = TCNT1;
            left_last_cm = ticks_to_cm(ticks);
            left_state = 0;
        }
    }
}

ISR (INT0_vect) {
    uint8_t high = PIND & (1<<RIGHT_ECHO_PIN);

    if (right_state == 0) {
        if (high) {
            TCNT1 = 0;
            right_state = 1;
        }
    } else {
        if (!high) {
            uint16_t ticks = TCNT1;
            right_last_cm = ticks_to_cm(ticks);
            right_state = 0;
        }
    }
}

ISR (PCINT0_vect) {
    uint8_t high = PINB & (1 << FRONT_ECHO_PIN);

    if (front_state == 0) {
        if (high) {
            TCNT1 = 0;
            front_state = 1;
        }
    } else {
        if (!high) {
            uint16_t ticks = TCNT1;
            front_last_cm = ticks_to_cm(ticks);
            front_state = 0;
        }
    }
}

// uint16_t front_distance() {
//     trig_front_left();
//
//     uint16_t timeout = 30000;
//     uint8_t prev = PINB & (1 << FRONT_ECHO_PIN);
//     uint16_t ticks = 0;
//
//     while (timeout--) {
//         uint8_t now = PINB & (1 << FRONT_ECHO_PIN);
//         if (!prev && now) {
//             TCNT1 = 0;
//             break;
//         }
//         prev = now;
//         _delay_us(2);
//     }
//     if (timeout == 0) return 0;
//
//     TCNT1 = 0;
//
//     timeout = 30000;
//     while (timeout--) {
//         uint8_t now = PINB & (1 << FRONT_ECHO_PIN);
//         if (prev && !now) {
//             ticks = TCNT1;
//             break;
//         }
//         prev = now;
//         _delay_us(2);
//     }
//     if (timeout == 0) return 0;
//
//     return ticks_to_cm(TCNT1);
// }


void us_update() {
    // Front
    trig_front_left();
    _delay_ms(60);
    front_cm = front_last_cm;

    // Left
    trig_front_left();
    _delay_ms(60);
    left_cm = left_last_cm;

    // Right
    trig_right();
    _delay_ms(60);
    right_cm = right_last_cm;
}
