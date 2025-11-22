//#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "servo.h"

#define SERVO_PIN PB1
volatile uint16_t servo_high_time = 3;
volatile uint16_t servo_counter = 0;

void servo_init() {
    DDRB |= (1 << SERVO_PIN);

    // Timer2 → CTC mode
    TCCR2A = (1 << WGM21);      
    TCCR2B = (1 << CS22);
    OCR2A = 124;

    TIMSK2 = (1 << OCIE2A);

    // Start Centered
    servo_high_time = 3;
}

void setServoAngle(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    servo_high_time = 2 + (angle * 2.0f / 180.0f);
}

ISR(TIMER2_COMPA_vect) {
    servo_counter++;

    if (servo_counter <= servo_high_time) {
        PORTB |= (1 << SERVO_PIN);   // HIGH
    } else {
        PORTB &= ~(1 << SERVO_PIN);  // LOW
    }

    if (servo_counter >= 40) {  // 40 × 0.5ms = 20 ms
        servo_counter = 0;
    }
}

void left_turn() {
    setServoAngle(0);
}

void right_turn() {
    setServoAngle(180);
}