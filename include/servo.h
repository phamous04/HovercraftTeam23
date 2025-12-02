#ifndef SERVO_H
#define SERVO_H

#include <avr/io.h>

void servo_init(void);
void setServoAngle(float angle);

#endif