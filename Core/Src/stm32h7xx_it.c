/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t uart4_rx_flag = 0;
uint8_t uart4_rx_data = 0;

uint8_t ibus_rx_buf[32];
uint8_t ibus_rx_cplt_flag = 0;

uint8_t uart7_rx_data = 0;

uint8_t uart6_rx_flag = 0;
uint8_t uart6_rx_data = 0;

char uart6_rx_buffer[10]; // Size for "cal_accel" (9 chars + null)
uint8_t uart6_rx_index;

uint16_t tim7_1ms_flag = 0;
uint16_t tim7_2ms_flag = 0;
uint16_t tim7_20ms_flag = 0;
uint16_t tim7_50ms_flag = 0;
uint16_t tim7_100ms_flag = 0;
uint16_t tim7_200ms_flag = 0;
uint16_t tim7_1000ms_flag = 0;

#define CMD_BUFFER_SIZE 64
#define CMD_TIMEOUT_MS  200  // reset after 200ms of inactivity

volatile char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t cmd_index = 0;
volatile bool cmd_receiving = false;
volatile uint32_t last_char_time = 0; // in ms

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_tim3_ch2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim3_ch2);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

	static unsigned char cnt = 0;
	//    static uint8_t ibus_rx_buf[32];

	if (LL_USART_IsActiveFlag_RXNE(UART4)) {
		uart4_rx_data = LL_USART_ReceiveData8(UART4);
		uart4_rx_flag = 1;

		switch (cnt) {
		case 0:
			if (uart4_rx_data == 0x20) {
				ibus_rx_buf[cnt] = uart4_rx_data;
				cnt++;
			}
			break;
		case 1:
			if (uart4_rx_data == 0x40) {
				ibus_rx_buf[cnt] = uart4_rx_data;
				cnt++;
			} else {
				cnt = 0; // Reset if invalid header
			}
			break;
		case 31:
			ibus_rx_buf[cnt] = uart4_rx_data;
			cnt = 0;
			ibus_rx_cplt_flag = 1;
			// Send raw packet as hexadecimal to USART6
			//	                char buffer[100];
			//	                int len = snprintf(buffer, sizeof(buffer), "");
			//	                for (uint8_t i = 0; i < 32; i++) {
			//	                    len += snprintf(buffer + len, sizeof(buffer) - len, "%02X ", ibus_rx_buf[i]);
			//	                }
			//	                len += snprintf(buffer + len, sizeof(buffer) - len, "\r\n");
			//
			//	                for (uint8_t i = 0; i < len; i++) {
			//	                    while (!LL_USART_IsActiveFlag_TXE(USART6)) {}
			//	//                    LL_USART_TransmitData8(USART6, buffer[i]);
			//	                }
			//	                while (!LL_USART_IsActiveFlag_TC(USART6)) {}
			break;
		default:
			ibus_rx_buf[cnt] = uart4_rx_data;
			cnt++;
			break;
		}
	}

	// Handle UART errors
	if (LL_USART_IsActiveFlag_ORE(UART4) || LL_USART_IsActiveFlag_FE(UART4) || LL_USART_IsActiveFlag_NE(UART4)) {
		LL_USART_ClearFlag_ORE(UART4);
		LL_USART_ClearFlag_FE(UART4);
		LL_USART_ClearFlag_NE(UART4);
		cnt = 0; // Reset buffer on error
		//        LL_USART_TransmitData8(USART6, "\r\n");
		//	        char error_msg[] = "\r\n";
		//	        for (uint8_t i = 0; error_msg[i] != '\0'; i++) {
		//	            while (!LL_USART_IsActiveFlag_TXE(USART6)) {}
		//	//            LL_USART_TransmitData8(USART6, error_msg[i]);
		//	        }
		//	        while (!LL_USART_IsActiveFlag_TC(USART6)) {}

	}

  /* USER CODE END UART4_IRQn 0 */
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	static uint16_t tim7_1ms_count    = 0;
	static uint16_t tim7_2ms_count    = 0;
	static uint16_t tim7_20ms_count   = 0;
	static uint16_t tim7_50ms_count   = 0;
	static uint16_t tim7_100ms_count  = 0;
	static uint16_t tim7_200ms_count  = 0;
	static uint16_t tim7_1000ms_count = 0;

	if (LL_TIM_IsActiveFlag_UPDATE(TIM7))
	{
		LL_TIM_ClearFlag_UPDATE(TIM7);

		if (++tim7_1ms_count >= 1) {
			tim7_1ms_count = 0;
			tim7_1ms_flag = 1;
		}

		if (++tim7_2ms_count >= 2) {
			tim7_2ms_count = 0;
			tim7_2ms_flag = 1;
		}

		if (++tim7_20ms_count >= 20) {
			tim7_20ms_count = 0;
			tim7_20ms_flag = 1;
		}

		if (++tim7_50ms_count >= 50) {
			tim7_50ms_count = 0;
			tim7_50ms_flag = 1;
		}

		if (++tim7_100ms_count >= 100) {
			tim7_100ms_count = 0;
			tim7_100ms_flag = 1;
		}

		if (++tim7_200ms_count >= 200) {
			tim7_200ms_count = 0;
			tim7_200ms_flag = 1;
		}

		if (++tim7_1000ms_count >= 1000) {
			tim7_1000ms_count = 0;
			tim7_1000ms_flag = 1;
		}
	}

  /* USER CODE END TIM7_IRQn 0 */
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

	if (LL_USART_IsActiveFlag_RXNE(USART6)) {
		char c = LL_USART_ReceiveData8(USART6);
		last_char_time = HAL_GetTick();  // update timestamp

		if (!cmd_receiving) {
			if (c == '[') {
				cmd_receiving = true;
				cmd_index = 0;
			}
		} else {
			if (c == ']') {
				if (cmd_index < CMD_BUFFER_SIZE) {
					cmd_buffer[cmd_index] = '\0';  // null-terminate
					process_command((char*)cmd_buffer);  // parse the command
				}
				cmd_receiving = false;
				cmd_index = 0;
			} else {
				if (cmd_index < CMD_BUFFER_SIZE - 1) {
					cmd_buffer[cmd_index++] = c;
				} else {
					// Buffer overflow, reset
					cmd_receiving = false;
					cmd_index = 0;
				}
			}
		}
	}

  /* USER CODE END USART6_IRQn 0 */
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
