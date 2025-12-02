#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdint.h>

void ir_init();
uint16_t ir_read();
void ir_update();
uint16_t ir_get_distance();
uint8_t ir_detect();
uint16_t ir_get_cm();

#endif
