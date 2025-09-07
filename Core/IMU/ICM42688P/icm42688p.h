/**
 * ICM42688P.h
 * @author Converted from ChrisP's ICM20602 driver
 *
 * This library source code has been created for STM32H7. Only supports SPI.
 * Converted from ICM20602 to ICM42688P with SPI3 interface.
 *
 * Development environment specifics:
 * STM32CubeIDE (H7 Compatible)
 * STM32CubeH7 FW
 * STM32H7 LL Driver(SPI) and HAL Driver(RCC for HAL_Delay() function)
 *
 * ICM42688P is a newer generation IMU with enhanced features
 * Key differences from ICM20602:
 * - Different register map
 * - Bank-based register organization
 * - Enhanced filtering options
 * - Higher performance capabilities
 */

#ifndef _ICM42688P_H
#define _ICM42688P_H

#include "main.h"

/*
ICM-42688P SPI Operational Features
1. Data is delivered MSB first and LSB last
2. Data is latched on the rising edge of SPC
3. Data should be transitioned on the falling edge of SPC
4. The maximum frequency of SPC is 24MHz (higher than ICM20602)
5. SPI read and write operations are completed in 16 or more clock cycles (two or more bytes)
6. The first bit of the first byte contains the Read/Write bit: Read (1) or Write (0)
7. The following 7 bits contain the Register Address
*/

/**
 * @brief Definition for connected to SPI3 (STM32H7)
 */
#define ICM42688P_SPI_CHANNEL		SPI3

// SPI3 GPIO Configuration for STM32H7
#define ICM42688P_SPI_SCLK_PIN		LL_GPIO_PIN_10
#define ICM42688P_SPI_SCLK_PORT		GPIOC
#define ICM42688P_SPI_SCLK_CLK		LL_AHB4_GRP1_PERIPH_GPIOC

#define ICM42688P_SPI_MISO_PIN		LL_GPIO_PIN_11
#define ICM42688P_SPI_MISO_PORT		GPIOC
#define ICM42688P_SPI_MISO_CLK		LL_AHB4_GRP1_PERIPH_GPIOC

#define ICM42688P_SPI_MOSI_PIN		LL_GPIO_PIN_12
#define ICM42688P_SPI_MOSI_PORT		GPIOC
#define ICM42688P_SPI_MOSI_CLK		LL_AHB4_GRP1_PERIPH_GPIOC

#define ICM42688P_SPI_CS_PIN		LL_GPIO_PIN_15
#define ICM42688P_SPI_CS_PORT		GPIOA
#define ICM42688P_SPI_CS_CLK		LL_AHB4_GRP1_PERIPH_GPIOA

#define ICM42688P_INT1_PIN			LL_GPIO_PIN_4
#define ICM42688P_INT1_PORT			GPIOE
#define ICM42688P_INT1_CLK			LL_AHB4_GRP1_PERIPH_GPIOE

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define CHIP_SELECT(ICM42688P)		LL_GPIO_ResetOutputPin(ICM42688P_SPI_CS_PORT, ICM42688P_SPI_CS_PIN)
#define CHIP_DESELECT(ICM42688P)	LL_GPIO_SetOutputPin(ICM42688P_SPI_CS_PORT, ICM42688P_SPI_CS_PIN)
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ICM-42688P Register Map (Bank 0 - User Bank)
 * ICM42688P uses bank-based register organization
 */

// Bank 0 Registers (User Bank)
#define ICM42688P_DEVICE_CONFIG     0x11
#define ICM42688P_DRIVE_CONFIG      0x13
#define ICM42688P_INT_CONFIG        0x14
#define ICM42688P_FIFO_CONFIG       0x16
#define ICM42688P_TEMP_DATA1        0x1D
#define ICM42688P_TEMP_DATA0        0x1E
#define ICM42688P_ACCEL_DATA_X1     0x1F
#define ICM42688P_ACCEL_DATA_X0     0x20
#define ICM42688P_ACCEL_DATA_Y1     0x21
#define ICM42688P_ACCEL_DATA_Y0     0x22
#define ICM42688P_ACCEL_DATA_Z1     0x23
#define ICM42688P_ACCEL_DATA_Z0     0x24
#define ICM42688P_GYRO_DATA_X1      0x25
#define ICM42688P_GYRO_DATA_X0      0x26
#define ICM42688P_GYRO_DATA_Y1      0x27
#define ICM42688P_GYRO_DATA_Y0      0x28
#define ICM42688P_GYRO_DATA_Z1      0x29
#define ICM42688P_GYRO_DATA_Z0      0x2A
#define ICM42688P_TMST_FSYNCH       0x2B
#define ICM42688P_TMST_FSYNCL       0x2C
#define ICM42688P_INT_STATUS        0x2D
#define ICM42688P_FIFO_COUNTH       0x2E
#define ICM42688P_FIFO_COUNTL       0x2F
#define ICM42688P_FIFO_DATA         0x30
#define ICM42688P_APEX_DATA0        0x31
#define ICM42688P_APEX_DATA1        0x32
#define ICM42688P_APEX_DATA2        0x33
#define ICM42688P_APEX_DATA3        0x34
#define ICM42688P_APEX_DATA4        0x35
#define ICM42688P_APEX_DATA5        0x36
#define ICM42688P_INT_STATUS2       0x37
#define ICM42688P_INT_STATUS3       0x38
#define ICM42688P_SIGNAL_PATH_RESET 0x4B
#define ICM42688P_INTF_CONFIG0      0x4C
#define ICM42688P_INTF_CONFIG1      0x4D
#define ICM42688P_PWR_MGMT0         0x4E
#define ICM42688P_GYRO_CONFIG0      0x4F
#define ICM42688P_ACCEL_CONFIG0     0x50
#define ICM42688P_GYRO_CONFIG1      0x51
#define ICM42688P_GYRO_ACCEL_CONFIG0 0x52
#define ICM42688P_ACCEL_CONFIG1     0x53
#define ICM42688P_TMST_CONFIG       0x54
#define ICM42688P_APEX_CONFIG0      0x56
#define ICM42688P_SMD_CONFIG        0x57
#define ICM42688P_FIFO_CONFIG1      0x5F
#define ICM42688P_FIFO_CONFIG2      0x60
#define ICM42688P_FIFO_CONFIG3      0x61
#define ICM42688P_FSYNC_CONFIG      0x62
#define ICM42688P_INT_CONFIG0       0x63
#define ICM42688P_INT_CONFIG1       0x64
#define ICM42688P_INT_SOURCE0       0x65
#define ICM42688P_INT_SOURCE1       0x66
#define ICM42688P_INT_SOURCE3       0x68
#define ICM42688P_INT_SOURCE4       0x69
#define ICM42688P_FIFO_LOST_PKT0    0x6C
#define ICM42688P_FIFO_LOST_PKT1    0x6D
#define ICM42688P_SELF_TEST_CONFIG  0x70
#define ICM42688P_WHO_AM_I          0x75  // Should return 0x47
#define ICM42688P_REG_BANK_SEL      0x76

// Bank Select Values
#define ICM42688P_BANK_SEL_0        0x00
#define ICM42688P_BANK_SEL_1        0x01
#define ICM42688P_BANK_SEL_2        0x02
#define ICM42688P_BANK_SEL_3        0x03
#define ICM42688P_BANK_SEL_4        0x04

// Configuration Values
#define ICM42688P_WHO_AM_I_VALUE    0x47

// Power Management
#define ICM42688P_PWR_MGMT0_TEMP_DIS_MASK   0x20
#define ICM42688P_PWR_MGMT0_IDLE_MASK       0x10
#define ICM42688P_PWR_MGMT0_GYRO_MODE_MASK  0x0C
#define ICM42688P_PWR_MGMT0_ACCEL_MODE_MASK 0x03

// Gyro and Accel Modes
#define ICM42688P_PWR_MGMT0_GYRO_MODE_OFF   0x00
#define ICM42688P_PWR_MGMT0_GYRO_MODE_STBY  0x04
#define ICM42688P_PWR_MGMT0_GYRO_MODE_LN    0x0C

#define ICM42688P_PWR_MGMT0_ACCEL_MODE_OFF  0x00
#define ICM42688P_PWR_MGMT0_ACCEL_MODE_LP   0x02
#define ICM42688P_PWR_MGMT0_ACCEL_MODE_LN   0x03

// Gyro Full Scale Select
#define ICM42688P_GYRO_FS_SEL_2000DPS       0x00
#define ICM42688P_GYRO_FS_SEL_1000DPS       0x01
#define ICM42688P_GYRO_FS_SEL_500DPS        0x02
#define ICM42688P_GYRO_FS_SEL_250DPS        0x03
#define ICM42688P_GYRO_FS_SEL_125DPS        0x04
#define ICM42688P_GYRO_FS_SEL_62_5DPS       0x05
#define ICM42688P_GYRO_FS_SEL_31_25DPS      0x06
#define ICM42688P_GYRO_FS_SEL_15_625DPS     0x07

// Accel Full Scale Select
#define ICM42688P_ACCEL_FS_SEL_16G          0x00
#define ICM42688P_ACCEL_FS_SEL_8G           0x01
#define ICM42688P_ACCEL_FS_SEL_4G           0x02
#define ICM42688P_ACCEL_FS_SEL_2G           0x03

// ODR (Output Data Rate) Settings
#define ICM42688P_ODR_32KHZ                 0x01
#define ICM42688P_ODR_16KHZ                 0x02
#define ICM42688P_ODR_8KHZ                  0x03
#define ICM42688P_ODR_4KHZ                  0x04
#define ICM42688P_ODR_2KHZ                  0x05
#define ICM42688P_ODR_1KHZ                  0x06
#define ICM42688P_ODR_200HZ                 0x07
#define ICM42688P_ODR_100HZ                 0x08
#define ICM42688P_ODR_50HZ                  0x09
#define ICM42688P_ODR_25HZ                  0x0A
#define ICM42688P_ODR_12_5HZ                0x0B

/**
 * @brief ICM42688P structure definition.
 */
typedef struct _ICM42688P{
	int16_t acc_x_raw;
	int16_t acc_y_raw;
	int16_t acc_z_raw;
	int16_t temperature_raw;
	int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;

	float gyro_x_rad, gyro_y_rad, gyro_z_rad;

	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float temperature;

	// Gyro offsets
	float gyro_offset_x;
	float gyro_offset_y;
	float gyro_offset_z;
}Struct_ICM42688P;

/**
 * @brief ICM42688P external variables
 */
extern Struct_ICM42688P ICM42688P;
extern int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset;

/**
 * @brief ICM42688P function prototypes
 */
void ICM42688P_GPIO_SPI_Initialization(void);
int ICM42688P_Initialization(void);
void ICM42688P_Get6AxisRawData(int16_t* accel, int16_t* gyro);
void ICM42688P_Get3AxisGyroRawData(int16_t* gyro);
void ICM42688P_Get3AxisAccRawData(int16_t* accel);
int ICM42688P_DataReady(void);
void ICM42688P_SelectBank(uint8_t bank);
uint8_t ICM42688P_ReadByte(uint8_t reg_addr);
void ICM42688P_WriteByte(uint8_t reg_addr, uint8_t val);
void ICM42688P_ReadBytes(uint8_t reg_addr, uint8_t len, uint8_t* data);

// Additional helper functions
float ICM42688P_GyroRawToDPS(int16_t raw_data);
float ICM42688P_AccelRawToG(int16_t raw_data);
float ICM42688P_TempRawToCelsius(int16_t raw_data);
int16_t ICM42688P_GetTemperatureRaw(void);
void ICM42688P_UpdateAllData(void);


// DMA Configuration for SPI3
#define ICM42688P_DMA_RX_STREAM     DMA1_Stream2
#define ICM42688P_DMA_TX_STREAM     DMA1_Stream3
#define ICM42688P_DMA_RX_CHANNEL    LL_DMA_CHANNEL_1
#define ICM42688P_DMA_TX_CHANNEL    LL_DMA_CHANNEL_2
#define ICM42688P_DMA_RX_IRQn       DMA1_Stream2_IRQn
#define ICM42688P_DMA_TX_IRQn       DMA1_Stream3_IRQn

// Buffer sizes
#define ICM42688P_DMA_BUFFER_SIZE   16  // Buffer for 12-byte data + address + dummy bytes

// Function prototypes for DMA
void ICM42688P_DMA_Init(void);
void ICM42688P_ReadBytesDMA(uint8_t reg_addr, uint8_t len, uint8_t* data);
void ICM42688P_Get6AxisRawDataDMA(int16_t* accel, int16_t* gyro);
void ICM42688P_DMA_TransferCompleteCallback(void);
void ICM42688P_DMA_TransferErrorCallback(void);

void ICM42688P_Calibrate(uint16_t sample_count);



#endif
