#include "stm32f7xx_hal.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "ssd1306.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

extern "C" {
    #include "uart.h"
    #include "rf.h"

    void Error_Handler(void) {
        __disable_irq();
        while (1) {
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Board User LEDs (PB0: Green, PB7: Blue, PB14: Red)                         */
/* -------------------------------------------------------------------------- */
#define LED_GREEN_PIN    GPIO_PIN_0
#define LED_BLUE_PIN     GPIO_PIN_7
#define LED_RED_PIN      GPIO_PIN_14
#define LED_GPIO_PORT    GPIOB

/* -------------------------------------------------------------------------- */
/* Edge Impulse Feature Buffer Allocation                                     */
/* -------------------------------------------------------------------------- */
static float raw_features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static size_t feature_idx = 0;
static char last_detected_label[32] = {0};

static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, raw_features + offset, length * sizeof(float));
    return 0;
}

/* -------------------------------------------------------------------------- */
/* Clock & Hardware Initializations                                           */
/* -------------------------------------------------------------------------- */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Internal HSI oscillator (16 MHz) configured for 216 MHz SysClock
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 432;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 432 / 2 = 216 MHz
    RCC_OscInitStruct.PLL.PLLQ = 9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        return;
    }

    // Activate Over-Drive to reach 216 MHz
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        return;
    }

    // Bus clocks: HCLK = 216 MHz, APB1 = 54 MHz, APB2 = 108 MHz
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; // 54 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; // 108 MHz (SPI1 Base)
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
        return;
    }
}

static void LED_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    GPIO_InitStruct.Pin = LED_GREEN_PIN | LED_BLUE_PIN | LED_RED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GREEN_PIN | LED_BLUE_PIN | LED_RED_PIN, GPIO_PIN_RESET);
}

static void Display_Event(const char* label, float confidence) {
    char footer_buf[32];
    snprintf(footer_buf, sizeof(footer_buf), "CONF: %d%%", (int)(confidence * 100.0f));

    ssd1306_clear();
    ssd1306_draw_text(0, 0, FONT_SMALL, "CLASSIFIER RESULT:");
    ssd1306_draw_text(0, 18, FONT_LARGE, label);
    ssd1306_draw_text(0, 48, FONT_SMALL, footer_buf);
    ssd1306_update();
}

static void Set_Class_LED(const char* label) {
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GREEN_PIN | LED_BLUE_PIN | LED_RED_PIN, GPIO_PIN_RESET);

    if (strcmp(label, "walking") == 0) {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GREEN_PIN, GPIO_PIN_SET);  // PB0 (Green)
    } else if (strcmp(label, "standing") == 0) {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE_PIN, GPIO_PIN_SET);   // PB7 (Blue)
    } else if (strcmp(label, "sitting") == 0) {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_SET);    // PB14 (Red)
    }
}

/* -------------------------------------------------------------------------- */
/* Classification Pipeline                                                    */
/* -------------------------------------------------------------------------- */
static void parse_csv_and_classify(char *csv_string) {
    char *token = strtok(csv_string, ",");
    while (token != NULL && feature_idx < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        raw_features[feature_idx++] = strtof(token, NULL);
        token = strtok(NULL, ",");
    }

    if (feature_idx >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        signal_t signal;
        signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        signal.get_data = &raw_feature_get_data;

        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res == EI_IMPULSE_OK) {
            float max_val = 0.0f;
            const char* predicted_label = "";

            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                if (result.classification[ix].value > max_val) {
                    max_val = result.classification[ix].value;
                    predicted_label = result.classification[ix].label;
                }
            }

            if (max_val > 0.60f) {
                Set_Class_LED(predicted_label);

                // Update screen only when classification state changes
                if (strcmp(predicted_label, last_detected_label) != 0) {
                    strncpy(last_detected_label, predicted_label, sizeof(last_detected_label) - 1);
                    Display_Event(predicted_label, max_val);
                }

                uart_print("Detected: ");
                uart_print(predicted_label);
                uart_println("");
            }
        }

        feature_idx = 0;
    }
}

/* -------------------------------------------------------------------------- */
/* Application Entry Point                                                    */
/* -------------------------------------------------------------------------- */
extern "C" int main(void) {
    /* 1. Enable Cortex-M7 L1 Caches */
    SCB_EnableICache();
    SCB_EnableDCache();

    /* 2. Configure Hardware Subsystems */
    HAL_Init();
    SystemClock_Config(); // Run core at 216 MHz
    HAL_InitTick(TICK_INT_PRIORITY); // Resynchronize SysTick to 216 MHz!
    LED_Init();
    uart_init(9600);

    /* 3. Initialize Hardware SPI1 and Display */
    ssd1306_init();

    /* 4. Display System Boot Screen */
    ssd1306_clear();
    ssd1306_draw_text(0, 0, FONT_SMALL, "EDGE AI INFERENCE");
    ssd1306_draw_text(0, 18, FONT_LARGE, "SYS OK");
    ssd1306_draw_text(0, 48, FONT_SMALL, "AWAITING SENSORS...");
    ssd1306_update();

    /* 5. Initialize RF Receiver */
    if (!rf_init(4000)) {
        uart_println("RF initialization failed");
        ssd1306_clear();
        ssd1306_draw_text(0, 0, FONT_SMALL, "ERROR:");
        ssd1306_draw_text(0, 18, FONT_LARGE, "RF FAIL");
        ssd1306_update();
        while (true) {
            HAL_Delay(1000);
        }
    }

    uart_println("System Initialized at 216 MHz. Awaiting RF sensor data...");

    while (true) {
        uint8_t buffer[RH_ASK_MAX_MESSAGE_LEN];
        uint8_t length = sizeof(buffer);

        if (rf_recv(buffer, &length)) {
            uint8_t commas  = 0;
            bool    digit   = false;
            bool    point   = false;
            bool    atStart = true;
            bool    valid   = (length > 0U);

            for (uint8_t i = 0; (i < length) && valid; ++i) {
                char c = (char)buffer[i];
                if (c >= '0' && c <= '9') {
                    digit   = true;
                    atStart = false;
                } else if (c == '-' && atStart) {
                    atStart = false;
                } else if (c == '.' && !point) {
                    point   = true;
                    atStart = false;
                } else if (c == ',' && digit) {
                    ++commas;
                    digit   = false;
                    point   = false;
                    atStart = true;
                } else {
                    valid = false;
                }
            }

            if (valid && digit && (commas == 6U)) {
                char csv_line[RH_ASK_MAX_MESSAGE_LEN + 1];
                memcpy(csv_line, (char*)buffer, length);
                csv_line[length] = '\0';

                parse_csv_and_classify(csv_line);
            }
        } else {
            __NOP(); // Yield core cycle
        }
    }

    return 0;
}
