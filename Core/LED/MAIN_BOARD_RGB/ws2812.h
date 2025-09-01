/* ws2812.h */
#ifndef WS2812_H
#define WS2812_H

#include "stm32h7xx_hal.h"

#define LED_COUNT 1 // Number of LEDs
#define BITS_PER_LED 24 // 24 bits per LED (GRB)
#define BUFFER_SIZE (LED_COUNT * BITS_PER_LED + 50) // PWM buffer + reset pulse
#define DUTY_0 80 // ~32% duty (0.4 µs / 1.25 µs * 250)
#define DUTY_1 160 // ~64% duty (0.8 µs / 1.25 µs * 250)
#define DUTY_RESET 0 // Low for reset

extern volatile uint8_t data_sent_flag; // DMA completion flag

void WS2812_Init(TIM_HandleTypeDef *htim);
void WS2812_SetColor(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness);
void WS2812_SetGlobalBrightness(float brightness);
void WS2812_Send(void);
void main_led(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness);
void WS2812_Update(void);

#endif
