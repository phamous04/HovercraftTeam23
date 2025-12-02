#include <Arduino.h>
#include <avr/io.h>
#include "fan.h"

#define LIFT_FAN_PIN PD5
#define THRUST_FAN_PIN PD6  // P4


void fan_init() {
    DDRD |= (1 << LIFT_FAN_PIN) | (1 << THRUST_FAN_PIN);
    PORTD &= ~(1 << LIFT_FAN_PIN);


    TCCR0A |= (1 << WGM01) | (1 << WGM00);  // Fast PWM mode
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1);  // Clear OC0A on compare match
    TCCR0B |= (1 << CS01);    // Set prescaler to 8
    OCR0A = 0;                // Thrust
    OCR0B = 0;                // Lift
}

void setThrustFan(uint8_t speed) {
    OCR0A = speed;
}

void setLiftFanOn() {
    OCR0B = 255;
}

void setLiftFanOff() {
    OCR0B = 0;
}


