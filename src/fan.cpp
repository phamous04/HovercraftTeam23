#include <Arduino.h>
#include <avr/io.h>
#include "fan.h"

#define LIFT_FAN_PIN PD5
#define THRUST_FAN_PIN PD6


void fan_init() {
    DDRD |= (1 << LIFT_FAN_PIN);
    DDRD |= (1 << THRUST_FAN_PIN);

    TCCR0A = 0;
    TCCR0B = 0;

    TCCR0A |= (1 << WGM00) | (1 << WGM01);
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1);

    TCCR0B |= (1 << CS01) | (1 << CS00);

    OCR0A = 0;
    OCR0B = 0;
}

void setThrustFan(uint8_t speed) {
    OCR0A = speed;
}

void setLiftFan(uint8_t speed) {
    OCR0B = speed;
}

