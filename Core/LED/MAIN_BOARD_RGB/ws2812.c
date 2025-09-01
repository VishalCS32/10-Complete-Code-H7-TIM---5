/* ws2812.c */
#include "../MAIN_BOARD_RGB/ws2812.h"
#include "targets.h"
#include "main.h" // For printf
#include <stdbool.h>

// External dependencies defined in other modules (e.g., tim.c, dma.c)
extern TIM_HandleTypeDef WS2812_TIMER; // Timer handle (htim3)
extern DMA_HandleTypeDef WS2812_DMA; // DMA handle (hdma_tim3_ch2)

volatile uint8_t data_sent_flag = 0; // DMA completion flag
static uint32_t pwm_buffer[BUFFER_SIZE]; // PWM duty cycle buffer
static uint8_t led_data[LED_COUNT][3]; // GRB data for each LED
static float global_brightness = 1.0; // Global brightness (0.0 to 1.0)
static volatile uint32_t led_on_time = 0; // Duration for LED to stay on (ms)
static volatile uint32_t led_start_time = 0; // Start time of LED on state
static volatile bool led_active = false; // LED state flag

void WS2812_Init(TIM_HandleTypeDef *htim) {
    // Initialize PWM buffer with reset pulse
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        pwm_buffer[i] = DUTY_RESET;
    }
    led_active = false;
    led_on_time = 0;
    led_start_time = 0;
    printf("WS2812 Initialized\n");
}

void WS2812_SetGlobalBrightness(float brightness) {
    // Clamp brightness between 0.0 and 1.0
    if (brightness < 0.0) brightness = 0.0;
    if (brightness > 1.0) brightness = 1.0;
    global_brightness = brightness;
}

void WS2812_SetColor(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness) {
    if (led_index < LED_COUNT) {
        // Clamp brightness between 0.0 and 1.0
        if (brightness < 0.0) brightness = 0.0;
        if (brightness > 1.0) brightness = 1.0;
        // Apply brightness scaling (combine with global brightness)
        float total_brightness = brightness * global_brightness;
        led_data[led_index][0] = (uint8_t)(green * total_brightness); // GRB order
        led_data[led_index][1] = (uint8_t)(red * total_brightness);
        led_data[led_index][2] = (uint8_t)(blue * total_brightness);
//        printf("WS2812 SetColor: LED %lu, R=%d, G=%d, B=%d, Brightness=%.2f\n",
//               led_index, led_data[led_index][1], led_data[led_index][0], led_data[led_index][2], brightness);
    }
}

void WS2812_Send(void) {
    uint32_t buffer_index = 0;

    // Clear buffer to ensure no stale data
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        pwm_buffer[i] = DUTY_RESET;
    }

    // Convert LED data to PWM duty cycles
    for (uint32_t led = 0; led < LED_COUNT; led++) {
        for (uint32_t color = 0; color < 3; color++) { // GRB
            for (int8_t bit = 7; bit >= 0; bit--) {
                if (led_data[led][color] & (1 << bit)) {
                    pwm_buffer[buffer_index++] = DUTY_1; // Logical 1
                } else {
                    pwm_buffer[buffer_index++] = DUTY_0; // Logical 0
                }
            }
        }
    }

    // Stop previous DMA and timer
    HAL_TIM_PWM_Stop_DMA(&WS2812_TIMER, WS2812_TIMER_CHANNEL);
    HAL_TIM_Base_Stop(&WS2812_TIMER);

    // Ensure DMA interrupt is enabled
    HAL_NVIC_SetPriority(WS2812_DMA_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(WS2812_DMA_IRQn);

    // Reset flag and start new DMA transfer
    data_sent_flag = 0;
    HAL_TIM_Base_Start(&WS2812_TIMER);
    HAL_TIM_PWM_Start_DMA(&WS2812_TIMER, WS2812_TIMER_CHANNEL, pwm_buffer, BUFFER_SIZE);
//    printf("WS2812 Send Started\n");
}

void main_led(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness) {
    WS2812_SetColor(led_index, red, green, blue, brightness);
    WS2812_Send();
    led_active = true;
    led_start_time = HAL_GetTick();
    led_on_time = 100; // Default duration (100ms)
//    printf("main_led: LED %lu set, duration=%lu ms\n", led_index, led_on_time);
}

void WS2812_Update(void) {
    if (led_active && data_sent_flag) {
        if ((HAL_GetTick() - led_start_time) >= led_on_time) {
            // Turn off LED by setting color to black
            WS2812_SetColor(0, 0, 0, 0, 1.0);
            WS2812_Send();
            led_active = false;
//            printf("WS2812 LED turned off\n");
        }
    }
}

__weak void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        HAL_TIM_PWM_Stop_DMA(&WS2812_TIMER, WS2812_TIMER_CHANNEL);
        HAL_TIM_Base_Stop(&WS2812_TIMER);
        data_sent_flag = 1;
//        printf("WS2812 DMA Transfer Complete\n");
    }
}
