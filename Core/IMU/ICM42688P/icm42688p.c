#include "icm42688p.h"
#include "stdio.h"

Struct_ICM42688P ICM42688P;

// DMA buffers
static uint8_t dma_tx_buffer[ICM42688P_DMA_BUFFER_SIZE];
static uint8_t dma_rx_buffer[ICM42688P_DMA_BUFFER_SIZE];
static volatile uint8_t dma_transfer_complete = 0;
static volatile uint8_t dma_transfer_error = 0;

void ICM42688P_GPIO_SPI_Initialization(void)
{
	LL_SPI_InitTypeDef SPI_InitStruct = {0};
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable for STM32H7 */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

	/* GPIO clock enable for STM32H7 */
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);

	/**SPI3 GPIO Configuration for STM32H7
	 * PC10  ------> SPI3_SCK
	 * PC11  ------> SPI3_MISO
	 * PC12  ------> SPI3_MOSI
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6; // SPI3 alternate function for STM32H7
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**ICM42688P GPIO Control Configuration
	 * PA15  ------> ICM42688P_SPI_CS_PIN (output)
	 * PC8   ------> ICM42688P_INT1_PIN (input)
	 */

	/* Chip Select Pin */
	LL_GPIO_SetOutputPin(ICM42688P_SPI_CS_PORT, ICM42688P_SPI_CS_PIN); // Start with CS high

	GPIO_InitStruct.Pin = ICM42688P_SPI_CS_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(ICM42688P_SPI_CS_PORT, &GPIO_InitStruct);

	/* Interrupt Pin */
	GPIO_InitStruct.Pin = ICM42688P_INT1_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(ICM42688P_INT1_PORT, &GPIO_InitStruct);

	/* STM32H7 SPI3 configuration - Fixed for proper LL driver usage */
	// Disable SPI first
	LL_SPI_Disable(ICM42688P_SPI_CHANNEL);

	// Configure SPI3 - STM32H7 style
	LL_SPI_SetBaudRatePrescaler(ICM42688P_SPI_CHANNEL, LL_SPI_BAUDRATEPRESCALER_DIV32);
	LL_SPI_SetTransferDirection(ICM42688P_SPI_CHANNEL, LL_SPI_FULL_DUPLEX);
	LL_SPI_SetClockPhase(ICM42688P_SPI_CHANNEL, LL_SPI_PHASE_2EDGE);
	LL_SPI_SetClockPolarity(ICM42688P_SPI_CHANNEL, LL_SPI_POLARITY_HIGH);
	LL_SPI_SetTransferBitOrder(ICM42688P_SPI_CHANNEL, LL_SPI_MSB_FIRST);
	LL_SPI_SetDataWidth(ICM42688P_SPI_CHANNEL, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetNSSMode(ICM42688P_SPI_CHANNEL, LL_SPI_NSS_SOFT);
	LL_SPI_SetMode(ICM42688P_SPI_CHANNEL, LL_SPI_MODE_MASTER);

	// STM32H7 specific settings
	LL_SPI_SetFIFOThreshold(ICM42688P_SPI_CHANNEL, LL_SPI_FIFO_TH_01DATA);

	// Enable SPI
	LL_SPI_Enable(ICM42688P_SPI_CHANNEL);

	// Start SPI (STM32H7 requirement)
	LL_SPI_StartMasterTransfer(ICM42688P_SPI_CHANNEL);

	CHIP_DESELECT(ICM42688P);

	printf("SPI3 initialized for STM32H7\n");
}



uint8_t SPI3_SendByte(uint8_t data)
{
	while(LL_SPI_IsActiveFlag_TXP(ICM42688P_SPI_CHANNEL)==RESET);
	LL_SPI_TransmitData8(ICM42688P_SPI_CHANNEL, data);

	while(LL_SPI_IsActiveFlag_RXP(ICM42688P_SPI_CHANNEL)==RESET);
	return LL_SPI_ReceiveData8(ICM42688P_SPI_CHANNEL);
}

//////////////////////////////////////////////////////////////

void ICM42688P_SelectBank(uint8_t bank)
{
	printf("Selecting bank %d\n", bank);
	ICM42688P_WriteByte(ICM42688P_REG_BANK_SEL, bank);
	HAL_Delay(1); // Small delay after bank selection
}

uint8_t ICM42688P_ReadByte(uint8_t reg_addr)
{
	uint8_t val;

	CHIP_SELECT(ICM42688P);
	SPI3_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	val = SPI3_SendByte(0x00); //Send DUMMY to read data
	CHIP_DESELECT(ICM42688P);

//	printf("0x%02x\n", val);
	return val;
}

void ICM42688P_ReadBytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	uint8_t i = 0;

	CHIP_SELECT(ICM42688P);
	SPI3_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	while(i < len)
	{
		data[i++] = SPI3_SendByte(0x00); //Send DUMMY to read data
	}
	CHIP_DESELECT(ICM42688P);
}

void ICM42688P_WriteByte(uint8_t reg_addr, uint8_t val)
{
	CHIP_SELECT(ICM42688P);
	SPI3_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	SPI3_SendByte(val); //Send Data to write
	CHIP_DESELECT(ICM42688P);
}

void ICM42688P_WriteBytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	uint8_t i = 0;
	CHIP_SELECT(ICM42688P);
	SPI3_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	while(i < len)
	{
		SPI3_SendByte(data[i++]); //Send Data to write
	}
	CHIP_DESELECT(ICM42688P);
}

int ICM42688P_Initialization(void)
{
	uint8_t who_am_i = 0;
	uint8_t retry_count = 0;

	ICM42688P_GPIO_SPI_Initialization();

	printf("Checking ICM42688P...\n");

	// Give some time for the sensor to power up
	HAL_Delay(100);

	// Try to communicate multiple times
	for(retry_count = 0; retry_count < 5; retry_count++)
	{
		printf("Attempt %d: ", retry_count + 1);

		// Select Bank 0 (User Bank)
		ICM42688P_SelectBank(ICM42688P_BANK_SEL_0);
		HAL_Delay(10);

		// Check WHO_AM_I (0x75)
		who_am_i = ICM42688P_ReadByte(ICM42688P_WHO_AM_I);

		printf("Read WHO_AM_I = 0x%02x\n", who_am_i);

		// WHO_AM_I should be 0x47 for ICM42688P
		if(who_am_i == ICM42688P_WHO_AM_I_VALUE)
		{
			printf("ICM42688P who_am_i = 0x%02x...OK\n", who_am_i);
			break;
		}

		HAL_Delay(50);
	}

	if(who_am_i != ICM42688P_WHO_AM_I_VALUE)
	{
		printf("ICM42688P Communication Failed! Read: 0x%02x, Expected: 0x%02x\n",
		       who_am_i, ICM42688P_WHO_AM_I_VALUE);
		printf("Check connections:\n");
		printf("- SPI wiring (MOSI, MISO, SCK, CS)\n");
		printf("- Power supply (3.3V)\n");
		printf("- Pull-up on CS if needed\n");
		return 1; //ERROR
	}

	// Device Reset
	ICM42688P_WriteByte(ICM42688P_DEVICE_CONFIG, 0x01); // Soft reset
	HAL_Delay(50);

	// Wait for reset to complete
	do {
		HAL_Delay(10);
		who_am_i = ICM42688P_ReadByte(ICM42688P_DEVICE_CONFIG);
	} while (who_am_i & 0x01);

	// Configure Power Management
	// Turn on gyro and accel in Low Noise mode
	ICM42688P_WriteByte(ICM42688P_PWR_MGMT0,
		ICM42688P_PWR_MGMT0_GYRO_MODE_LN | ICM42688P_PWR_MGMT0_ACCEL_MODE_LN);
	HAL_Delay(50);

	// Configure Gyroscope
	// Full scale: ±2000 dps, ODR: 1 kHz
	ICM42688P_WriteByte(ICM42688P_GYRO_CONFIG0,
		(ICM42688P_GYRO_FS_SEL_2000DPS << 5) | ICM42688P_ODR_1KHZ);
	HAL_Delay(10);

	// Configure Accelerometer
	// Full scale: ±16g, ODR: 1 kHz
	ICM42688P_WriteByte(ICM42688P_ACCEL_CONFIG0,
		(ICM42688P_ACCEL_FS_SEL_16G << 5) | ICM42688P_ODR_1KHZ);
	HAL_Delay(10);

	// Configure Gyro and Accel in Low Noise mode with filtering
//	ICM42688P_WriteByte(ICM42688P_GYRO_ACCEL_CONFIG0, 0x44); // Enable anti-alias filter
	ICM42688P_WriteByte(ICM42688P_GYRO_ACCEL_CONFIG0, 0x44);
	HAL_Delay(10);

	// Configure interrupts
	// Enable data ready interrupt on INT1
	ICM42688P_WriteByte(ICM42688P_INT_CONFIG, 0x12); // INT1 push-pull, active high
	HAL_Delay(10);

	ICM42688P_WriteByte(ICM42688P_INT_SOURCE0, 0x08); // UI data ready interrupt routed to INT1
	HAL_Delay(10);

	printf("ICM42688P initialized successfully!\n");

	return 0; //OK
}

void ICM42688P_Get6AxisRawData(int16_t* accel, int16_t* gyro)
{
	uint8_t data[12];

	// Read accelerometer and gyroscope data (starting from ACCEL_DATA_X1)
	ICM42688P_ReadBytes(ICM42688P_ACCEL_DATA_X1, 12, data);

	// Parse accelerometer data (big endian)
	accel[0] = (int16_t)((data[0] << 8) | data[1]);  // X-axis
	accel[1] = (int16_t)((data[2] << 8) | data[3]);  // Y-axis
	accel[2] = (int16_t)((data[4] << 8) | data[5]);  // Z-axis

	// Parse gyroscope data (big endian)
	gyro[1] = (int16_t)((data[6] << 8) | data[7]);   // X-axis
	gyro[0] = (int16_t)((data[8] << 8) | data[9]);   // Y-axis
	gyro[2] = (int16_t)((data[10] << 8) | data[11]); // Z-axis
}

void ICM42688P_Get3AxisGyroRawData(int16_t* gyro)
{
	uint8_t data[6];

	// Read gyroscope data (starting from GYRO_DATA_X1)
	ICM42688P_ReadBytes(ICM42688P_GYRO_DATA_X1, 6, data);

	// Parse gyroscope data (big endian)
	gyro[1] = (int16_t)((data[0] << 8) | data[1]);  // X-axis
	gyro[0] = (int16_t)((data[2] << 8) | data[3]);  // Y-axis
	gyro[2] = (int16_t)((data[4] << 8) | data[5]);  // Z-axis
}

void ICM42688P_Get3AxisAccRawData(int16_t* accel)
{
	uint8_t data[6];

	// Read accelerometer data (starting from ACCEL_DATA_X1)
	ICM42688P_ReadBytes(ICM42688P_ACCEL_DATA_X1, 6, data);

	// Parse accelerometer data (big endian)
	accel[0] = (int16_t)((data[0] << 8) | data[1]);  // X-axis
	accel[1] = (int16_t)((data[2] << 8) | data[3]);  // Y-axis
	accel[2] = (int16_t)((data[4] << 8) | data[5]);  // Z-axis
}

int ICM42688P_DataReady(void)
{
	// Check if INT1 pin is high (data ready)
	return LL_GPIO_IsInputPinSet(ICM42688P_INT1_PORT, ICM42688P_INT1_PIN);
}

// Helper function to convert raw gyro data to degrees per second
float ICM42688P_GyroRawToDPS(int16_t raw_data)
{
	// For ±2000 dps full scale: 2000 / 32768 = 0.061035 dps/LSB
	return (float)raw_data * 0.061035f;
}

// Helper function to convert raw accel data to g
float ICM42688P_AccelRawToG(int16_t raw_data)
{
	// For ±16g full scale: 16 / 32768 = 0.0004883 g/LSB
	return (float)raw_data * 0.0004883f;
}

// Helper function to convert raw temperature data to Celsius
float ICM42688P_TempRawToCelsius(int16_t raw_data)
{
	// Temperature formula: (TEMP_DATA / 132.48) + 25°C
	return ((float)raw_data / 132.48f) + 25.0f;
}

// Helper function to convert raw gyro data to radians per second
float ICM42688P_GyroRawToRadPerSec(int16_t raw_data)
{
    // For ±2000 dps full scale: 2000 / 32768 = 0.061035 dps/LSB
    // Convert dps to radians/sec: 0.061035 * (π/180) = 0.001065 rad/s/LSB
    return (float)raw_data * 0.001065f;
}

// Function to get temperature data
int16_t ICM42688P_GetTemperatureRaw(void)
{
	uint8_t data[2];

	// Read temperature data
	ICM42688P_ReadBytes(ICM42688P_TEMP_DATA1, 2, data);

	// Parse temperature data (big endian)
	return (int16_t)((data[0] << 8) | data[1]);
}

// Function to update all sensor data in the structure
void ICM42688P_UpdateAllData(void)
{
	int16_t accel_raw[3], gyro_raw[3];

	// Get raw data
	ICM42688P_Get6AxisRawData(accel_raw, gyro_raw);
	ICM42688P.temperature_raw = ICM42688P_GetTemperatureRaw();

	// Store raw data
	ICM42688P.acc_x_raw = accel_raw[0];
	ICM42688P.acc_y_raw = accel_raw[1];
	ICM42688P.acc_z_raw = accel_raw[2];

	ICM42688P.gyro_x_raw = gyro_raw[0];
	ICM42688P.gyro_y_raw = gyro_raw[1];
	ICM42688P.gyro_z_raw = gyro_raw[2];

	// Convert to physical units
	ICM42688P.acc_x = ICM42688P_AccelRawToG(accel_raw[0]);
	ICM42688P.acc_y = ICM42688P_AccelRawToG(accel_raw[1]);
	ICM42688P.acc_z = ICM42688P_AccelRawToG(accel_raw[2]);

	ICM42688P.gyro_x = ICM42688P_GyroRawToDPS(gyro_raw[0]);
	ICM42688P.gyro_y = ICM42688P_GyroRawToDPS(gyro_raw[1]);
	ICM42688P.gyro_z = ICM42688P_GyroRawToDPS(gyro_raw[2]);



	ICM42688P.temperature = ICM42688P_TempRawToCelsius(ICM42688P.temperature_raw);
}

void ICM42688P_UpdateAllData_Radians(void)
{
    int16_t accel_raw[3], gyro_raw[3];

    // Get raw data
    ICM42688P_Get6AxisRawData(accel_raw, gyro_raw);
    ICM42688P.temperature_raw = ICM42688P_GetTemperatureRaw();

    // Store raw data
    ICM42688P.acc_x_raw = accel_raw[0];
    ICM42688P.acc_y_raw = accel_raw[1];
    ICM42688P.acc_z_raw = accel_raw[2];

    ICM42688P.gyro_x_raw = gyro_raw[0];
    ICM42688P.gyro_y_raw = gyro_raw[1];
    ICM42688P.gyro_z_raw = gyro_raw[2];

    // Convert to physical units (accelerometer in g)
    ICM42688P.acc_x = ICM42688P_AccelRawToG(accel_raw[0]);
    ICM42688P.acc_y = ICM42688P_AccelRawToG(accel_raw[1]);
    ICM42688P.acc_z = ICM42688P_AccelRawToG(accel_raw[2]);

    // Convert to physical units (gyroscope in rad/s)
    ICM42688P.gyro_x_rad = ICM42688P_GyroRawToRadPerSec(gyro_raw[0]);
    ICM42688P.gyro_y_rad = ICM42688P_GyroRawToRadPerSec(gyro_raw[1]);
    ICM42688P.gyro_z_rad = ICM42688P_GyroRawToRadPerSec(gyro_raw[2]);

    ICM42688P.temperature = ICM42688P_TempRawToCelsius(ICM42688P.temperature_raw);
}
