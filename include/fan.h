#ifndef FAN_H
#define FAN_H

#include <stdint.h>

void fan_init();

void setThrustFan(uint8_t speed);
void setLiftFanOn();
void setLiftFanOff();


#endif
