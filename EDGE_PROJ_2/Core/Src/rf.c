#include "rf.h"

#define SYSCLK_FREQ_HZ          108000000UL
#define RH_ASK_START_SYMBOL     0xB38U
#define RH_ASK_RX_SAMPLES_BIT   8U
#define RH_ASK_RX_RAMP_LEN      160U
#define RH_ASK_RAMP_INC         (RH_ASK_RX_RAMP_LEN / RH_ASK_RX_SAMPLES_BIT)
#define RH_ASK_RAMP_TRANSITION  (RH_ASK_RX_RAMP_LEN / 2U)
#define RH_ASK_RAMP_ADJUST      9U
#define RH_ASK_RAMP_INC_RETARD  (RH_ASK_RAMP_INC - RH_ASK_RAMP_ADJUST)
#define RH_ASK_RAMP_INC_ADVANCE (RH_ASK_RAMP_INC + RH_ASK_RAMP_ADJUST)

/* Base Addresses (STM32F746ZG RM0385 & Cortex-M7 PM0253) */
#define RCC_BASE    0x40023800UL
#define GPIOA_BASE  0x40020000UL
#define TIM2_BASE   0x40000000UL
#define NVIC_BASE   0xE000E100UL

/* Register Pointers */
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x40UL))

#define GPIOA_MODER (*(volatile uint32_t *)(GPIOA_BASE + 0x00UL))
#define GPIOA_IDR   (*(volatile uint32_t *)(GPIOA_BASE + 0x10UL))

#define TIM2_CR1    (*(volatile uint32_t *)(TIM2_BASE + 0x00UL))
#define TIM2_DIER   (*(volatile uint32_t *)(TIM2_BASE + 0x0CUL))
#define TIM2_SR     (*(volatile uint32_t *)(TIM2_BASE + 0x10UL))
#define TIM2_CNT    (*(volatile uint32_t *)(TIM2_BASE + 0x24UL))
#define TIM2_PSC    (*(volatile uint32_t *)(TIM2_BASE + 0x28UL))
#define TIM2_ARR    (*(volatile uint32_t *)(TIM2_BASE + 0x2CUL))

/* NVIC Registers (TIM2_IRQn = 28) */
#define NVIC_ISER0  (*(volatile uint32_t *)(NVIC_BASE + 0x000UL))
#define NVIC_IPR28  (*(volatile uint8_t  *)(NVIC_BASE + 0x300UL + 28UL))

/* Bit Definitions */
#define RCC_AHB1ENR_GPIOAEN (1UL << 0)
#define RCC_APB1ENR_TIM2EN  (1UL << 0)
#define GPIO_IDR_ID0        (1UL << 0)
#define TIM_CR1_CEN         (1UL << 0)
#define TIM_DIER_UIE        (1UL << 0)
#define TIM_SR_UIF          (1UL << 0)
#define TIM2_IRQn_BIT       (1UL << 28)

/* 4b/6b decoding table used by RadioHead RH_ASK */
static const uint8_t rf_symbols[16] = {
    0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
    0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

static volatile uint8_t  rx_buf[RH_ASK_MAX_PAYLOAD_LEN];
static volatile uint8_t  rx_buf_len     = 0;
static volatile bool     rx_buf_full    = false;
static volatile bool     rx_buf_valid   = false;
static volatile bool     rx_active      = false;
static volatile bool     rx_last_sample = false;
static volatile uint8_t  rx_integrator  = 0;
static volatile uint8_t  rx_pll_ramp    = 0;
static volatile uint16_t rx_bits        = 0;
static volatile uint8_t  rx_bit_count   = 0;
static volatile uint8_t  rx_count       = 0;

static uint16_t crc_ccitt_update(uint16_t crc, uint8_t data) {
    data ^= (uint8_t)(crc & 0xFFU);
    data ^= (uint8_t)(data << 4);
    return ((((uint16_t)data << 8) | (crc >> 8)) ^
            (uint8_t)(data >> 4) ^
            ((uint16_t)data << 3));
}

static uint8_t symbol_to_nibble(uint8_t symbol) {
    for (uint8_t i = (symbol >> 2) & 8U, count = 8U; count--; ++i) {
        if (symbol == rf_symbols[i]) {
            return i;
        }
    }
    return 0;
}

static void validate_rx_buf(void) {
    uint16_t crc = 0xFFFFU;
    for (uint8_t i = 0; i < rx_buf_len; ++i) {
        crc = crc_ccitt_update(crc, rx_buf[i]);
    }
    if (crc != 0xF0B8U) {
        rx_buf_full  = false;
        rx_buf_valid = false;
        return;
    }
    rx_buf_valid = true;
}

bool rf_init(uint16_t speed_bps) {
    if (speed_bps == 0 || speed_bps > 10000U) {
        return false;
    }

    /* 1. Configure PA0 as Digital Input for RF DATA */
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA_MODER &= ~(3UL << (0 * 2)); /* Input mode (00) */

    /* 2. Configure TIM2 for 8x bit-rate oversampling */
    RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;
    volatile uint32_t tmpreg = RCC_APB1ENR;
    (void)tmpreg;

    TIM2_CR1  = 0;
    TIM2_PSC  = 0;
    TIM2_ARR  = (SYSCLK_FREQ_HZ / ((uint32_t)speed_bps * RH_ASK_RX_SAMPLES_BIT)) - 1UL;
    TIM2_CNT  = 0;
    TIM2_DIER |= TIM_DIER_UIE;

    /* Configure NVIC for TIM2 (IRQ 28) */
    NVIC_IPR28 = (1U << 4); /* Priority 1 (4 priority bits implemented on STM32F7) */
    NVIC_ISER0 = TIM2_IRQn_BIT;

    TIM2_CR1 |= TIM_CR1_CEN;
    return true;
}

void TIM2_IRQHandler(void) {
    if (TIM2_SR & TIM_SR_UIF) {
        TIM2_SR &= ~TIM_SR_UIF;

        /* Sample PA0 */
        bool rx_sample = (GPIOA_IDR & GPIO_IDR_ID0) != 0U;
        if (rx_sample) {
            rx_integrator++;
        }

        if (rx_sample != rx_last_sample) {
            rx_pll_ramp += (rx_pll_ramp < RH_ASK_RAMP_TRANSITION)
                               ? RH_ASK_RAMP_INC_RETARD
                               : RH_ASK_RAMP_INC_ADVANCE;
            rx_last_sample = rx_sample;
        } else {
            rx_pll_ramp += RH_ASK_RAMP_INC;
        }

        if (rx_pll_ramp >= RH_ASK_RX_RAMP_LEN) {
            rx_bits >>= 1;
            if (rx_integrator >= 5U) {
                rx_bits |= 0x800U;
            }
            rx_pll_ramp -= RH_ASK_RX_RAMP_LEN;
            rx_integrator = 0;

            if (rx_active) {
                if (++rx_bit_count >= 12U) {
                    uint8_t this_byte = (uint8_t)((symbol_to_nibble((uint8_t)(rx_bits & 0x3FU)) << 4) |
                                                   symbol_to_nibble((uint8_t)(rx_bits >> 6)));
                    if (rx_buf_len == 0U) {
                        rx_count = this_byte;
                        if (rx_count < 7U || rx_count > RH_ASK_MAX_PAYLOAD_LEN) {
                            rx_active = false;
                            return;
                        }
                    }
                    rx_buf[rx_buf_len++] = this_byte;
                    if (rx_buf_len >= rx_count) {
                        rx_active   = false;
                        rx_buf_full = true;
                    }
                    rx_bit_count = 0;
                }
            } else if (rx_bits == RH_ASK_START_SYMBOL) {
                rx_active    = true;
                rx_bit_count = 0;
                rx_buf_len   = 0;
            }
        }
    }
}

bool rf_recv(uint8_t *buf, uint8_t *len) {
    if (!rx_buf_full) {
        return false;
    }

    if (!rx_buf_valid) {
        validate_rx_buf();
        if (!rx_buf_valid) {
            return false;
        }
    }

    /* Payload length subtracts length byte (1), headers (4), and CRC (2) */
    uint8_t msg_len = rx_buf_len - RH_ASK_HEADER_LEN - 3U;
    if (*len > msg_len) {
        *len = msg_len;
    }

    for (uint8_t i = 0; i < *len; ++i) {
        buf[i] = rx_buf[i + RH_ASK_HEADER_LEN + 1U];
    }

    rx_buf_valid = false;
    rx_buf_full  = false;
    return true;
}
