#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include "ir_sensor.h"


#define IR_PIN 2 // P14
#define IR_BAR_THRESHOLD  400
uint16_t ir_raw = 0;


void ir_init() {
    ADMUX = (1 << REFS0);

    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    ADMUX = (ADMUX & 0xF0) | (IR_PIN & 0x0F);
}

void ir_update() {
    ADMUX = (ADMUX & 0xF0) | (IR_PIN & 0x0F);

    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC))
        ;
    ir_raw = ADC;
}

uint16_t ir_get_distance() {
    return ir_raw;
}

uint8_t ir_detect() {
    return (ir_raw > IR_BAR_THRESHOLD) ? 1 : 0;
}

uint16_t ir_get_cm() {
    float v = (float)ir_raw * (5.0 / 1023.0);
    float d = 27.86 * pow(v, -1.15);
    return (uint16_t)d;
}

