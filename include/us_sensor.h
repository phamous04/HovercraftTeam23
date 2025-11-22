#ifndef US_SENSOR_H
#define US_SENSOR_H

#include <stdint.h>


void us_init();

float read_front_sensor();
float read_left_sensor();
float read_right_sensor();

#endif
