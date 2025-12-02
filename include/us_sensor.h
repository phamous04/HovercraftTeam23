#ifndef US_SENSOR_H
#define US_SENSOR_H

#include <stdint.h>

// Global distance variables
extern uint16_t front_cm;
extern uint16_t right_cm;

// Function prototypes
void us_init(void);
void us_update(void);
void trig_front(void);
void trig_right(void);

#endif