#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init();
void uart_transmit(char data);
void uart_print_char(char c);
void uart_print(const char* str);
void uart_println(const char* str);
void uart_print_int(uint16_t num);
void uart_print_float(float value);

#endif
