#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>


void servo_init();
void setServoAngle(float angle);
void left_turn();
void right_turn();

#endif
