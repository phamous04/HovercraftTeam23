#ifndef US_SENSOR_H
#define US_SENSOR_H

#include <avr/io.h>

extern uint16_t front_cm;
extern uint16_t left_cm;
extern uint16_t right_cm;

void us_init();

void trig_front_left();

void us_trig(char s);
void us_update();

#endif
