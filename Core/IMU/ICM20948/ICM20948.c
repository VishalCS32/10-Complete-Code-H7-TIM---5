///*
// * ICM20948.c
// *
// *  Created on: Sep 5, 2025
// *      Author: vishal
// */
//
//
//#include "ICM20948.h"
//#include <stdio.h>
//
//Struct_ICM20948 ICM20948;
//
//void ICM20948_GPIO_SPI_Initialization(void)
//{
//
//	LL_SPI_InitTypeDef SPI_InitStruct = {0};
//
//	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//	/* Peripheral clock enable */
//	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
//
//	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
//	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
//
//	/**SPI2 GPIO Configuration
//	  PC1   ------> SPI2_MOSI
//	  PC2_C   ------> SPI2_MISO
//	  PB13   ------> SPI2_SCK
//	 */
//
//	GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2;
//	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
//	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//	GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
//	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
//	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//
//	LL_GPIO_SetOutputPin(ICM20948_SPI_CS_PORT, ICM20948_SPI_CS_PIN); // Start with CS high
//
//	GPIO_InitStruct.Pin = ICM20948_SPI_CS_PIN;
//	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//	LL_GPIO_Init(ICM20948_SPI_CS_PORT, &GPIO_InitStruct);
//
//	/* Interrupt Pin */
//	GPIO_InitStruct.Pin = ICM20948_INT1_PIN;
//	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
//	LL_GPIO_Init(ICM20948_INT1_PORT, &GPIO_InitStruct);
//
//	/* STM32H7 SPI3 configuration - Fixed for proper LL driver usage */
//	// Disable SPI first
//	LL_SPI_Disable(ICM20948_SPI_CHANNEL);
//
//	// Configure SPI3 - STM32H7 style
//	LL_SPI_SetBaudRatePrescaler(ICM20948_SPI_CHANNEL, LL_SPI_BAUDRATEPRESCALER_DIV32);
//	LL_SPI_SetTransferDirection(ICM20948_SPI_CHANNEL, LL_SPI_FULL_DUPLEX);
//	LL_SPI_SetClockPhase(ICM20948_SPI_CHANNEL, LL_SPI_PHASE_2EDGE);
//	LL_SPI_SetClockPolarity(ICM20948_SPI_CHANNEL, LL_SPI_POLARITY_HIGH);
//	LL_SPI_SetTransferBitOrder(ICM20948_SPI_CHANNEL, LL_SPI_MSB_FIRST);
//	LL_SPI_SetDataWidth(ICM20948_SPI_CHANNEL, LL_SPI_DATAWIDTH_8BIT);
//	LL_SPI_SetNSSMode(ICM20948_SPI_CHANNEL, LL_SPI_NSS_SOFT);
//	LL_SPI_SetMode(ICM20948_SPI_CHANNEL, LL_SPI_MODE_MASTER);
//
//	// STM32H7 specific settings
//	LL_SPI_SetFIFOThreshold(ICM20948_SPI_CHANNEL, LL_SPI_FIFO_TH_01DATA);
//
//	// Enable SPI
//	LL_SPI_Enable(ICM20948_SPI_CHANNEL);
//
//	// Start SPI (STM32H7 requirement)
//	LL_SPI_StartMasterTransfer(ICM20948_SPI_CHANNEL);
//
//	CHIP_DESELECT(ICM20948);
//
//	printf("SPI3 initialized for STM32H7\n");
//
//
//}
