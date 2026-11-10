#include "uart.h"

#define SYSCLK_FREQ_HZ  54000000UL

/* Base Addresses (STM32F746ZG RM0385) */
#define RCC_BASE    0x40023800UL
#define GPIOD_BASE  0x40020C00UL
#define USART3_BASE 0x40004800UL

/* Register Pointers */
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x40UL))

#define GPIOD_MODER (*(volatile uint32_t *)(GPIOD_BASE + 0x00UL))
#define GPIOD_AFRH  (*(volatile uint32_t *)(GPIOD_BASE + 0x24UL))

#define USART3_CR1  (*(volatile uint32_t *)(USART3_BASE + 0x00UL))
#define USART3_BRR  (*(volatile uint32_t *)(USART3_BASE + 0x0CUL))
#define USART3_ISR  (*(volatile uint32_t *)(USART3_BASE + 0x1CUL))
#define USART3_TDR  (*(volatile uint32_t *)(USART3_BASE + 0x28UL))

/* Bit Definitions */
#define RCC_AHB1ENR_GPIODEN  (1UL << 3)
#define RCC_APB1ENR_USART3EN (1UL << 18)
#define USART_CR1_UE         (1UL << 0)
#define USART_CR1_TE         (1UL << 3)
#define USART_ISR_TXE        (1UL << 7)

void uart_init(uint32_t baudrate) {
    /* 1. Enable GPIOD and USART3 peripheral clocks */
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC_APB1ENR |= RCC_APB1ENR_USART3EN;

    /* Dummy read to ensure clock enable completes */
    volatile uint32_t tmpreg = RCC_APB1ENR;
    (void)tmpreg;

    /* 2. Configure PD8 (TX) and PD9 (RX) as Alternate Function (AF7) */
    GPIOD_MODER &= ~((3UL << (8 * 2)) | (3UL << (9 * 2)));
    GPIOD_MODER |=  ((2UL << (8 * 2)) | (2UL << (9 * 2)));

    GPIOD_AFRH &= ~((0xFUL << ((8 - 8) * 4)) | (0xFUL << ((9 - 8) * 4)));
    GPIOD_AFRH |=  ((7UL   << ((8 - 8) * 4)) | (7UL   << ((9 - 8) * 4)));

    /* 3. Configure USART3 baud rate and enable Transmitter + USART */
    USART3_CR1 = 0;
    USART3_BRR = (SYSCLK_FREQ_HZ + (baudrate / 2U)) / baudrate;
    USART3_CR1 = USART_CR1_TE | USART_CR1_UE;
}

void uart_write_byte(uint8_t data) {
    while (!(USART3_ISR & USART_ISR_TXE)) {
        /* Wait until transmit data register is empty */
    }
    USART3_TDR = data;
}

void uart_write(const uint8_t *buffer, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        uart_write_byte(buffer[i]);
    }
}

void uart_print(const char *str) {
    while (*str) {
        uart_write_byte((uint8_t)(*str++));
    }
}

void uart_println(const char *str) {
    uart_print(str);
    uart_write_byte('\r');
    uart_write_byte('\n');
}
