#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>

void uart_init(uint32_t baudrate);
void uart_write_byte(uint8_t data);
void uart_write(const uint8_t *buffer, size_t length);
void uart_print(const char *str);
void uart_println(const char *str);

#endif /* UART_H */
