#include <avr/io.h>
#include <stdlib.h>
#include "uart.h"


void uart_init() {
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

void uart_transmit(char data) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void uart_print_char(char c) {
    uart_transmit(c);
}

void uart_print(const char* str) {
    while (*str) uart_transmit(*str++);
}

void uart_println(const char* str) {
    uart_print(str);
    uart_print("\r\n");
}

void uart_print_int(uint16_t num) {
    char b[10];
    itoa(num, b, 10);
    uart_print(b);
}

void uart_print_float(float value) {
    char buffer[20];
    dtostrf(value, 0, 3, buffer);  
    uart_print(buffer);
}
