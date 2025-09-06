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

	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8; //ICM-20602 MAX SPI CLK is 10MHz. But DIV2(42MHz) is available.
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 10;
	LL_SPI_Init(ICM42688P_SPI_CHANNEL, &SPI_InitStruct);
	LL_SPI_SetStandard(ICM42688P_SPI_CHANNEL, LL_SPI_PROTOCOL_MOTOROLA);

	/**ICM42688P GPIO Control Configuration
	 * PA15  ------> ICM42688P_SPI_CS_PIN (output)
	 * PC8   ------> ICM42688P_INT1_PIN (input)
	 */

	/* Chip Select Pin */
//	LL_GPIO_SetOutputPin(ICM42688P_SPI_CS_PORT, ICM42688P_SPI_CS_PIN); // Start with CS high

	LL_GPIO_ResetOutputPin(ICM42688P_SPI_CS_PORT, ICM42688P_SPI_CS_PIN); // Start with CS high

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

	LL_SPI_Enable(ICM42688P_SPI_CHANNEL);

	CHIP_DESELECT(ICM42688P);

	printf("SPI3 initialized for STM32H7\n");
}

// ====== BANK SELECT ======
void ICM42688P_SelectBank(uint8_t bank)
{
	printf("Selecting bank %d\n", bank);
	ICM42688P_WriteByte(ICM42688P_REG_BANK_SEL, bank);
	HAL_Delay(1); // Small delay after bank selection
}

uint8_t SPI3_SendByte(uint8_t data)
{
	while(LL_SPI_IsActiveFlag_TXP(ICM42688P_SPI_CHANNEL)==RESET);
	LL_SPI_TransmitData8(ICM42688P_SPI_CHANNEL, data);

	while(LL_SPI_IsActiveFlag_RXP(ICM42688P_SPI_CHANNEL)==RESET);
	return LL_SPI_ReceiveData8(ICM42688P_SPI_CHANNEL);
}

//////////////////////////////////////////////////////////////

//void ICM42688P_SelectBank(uint8_t bank)
//{
//	printf("Selecting bank %d\n", bank);
//	ICM42688P_WriteByte(ICM42688P_REG_BANK_SEL, bank);
//	HAL_Delay(1); // Small delay after bank selection
//}

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

void ICM42688P_ReadBytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;

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

void ICM20602_Writebytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;
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
	int16_t accel_raw_data[3] = {0};  // To remove offset
	int16_t gyro_raw_data[3] = {0};   // To remove offset

	ICM42688P_GPIO_SPI_Initialization();

	printf("Checking ICM42688P...\n");

	// check WHO_AM_I (0x75)
	who_am_i = ICM42688P_ReadByte(ICM42688P_WHO_AM_I);

	// who am i = 0x47
	if(who_am_i == 0x47)
	{
		printf("\nICM20602 who_am_i = 0x%02x...OK\n\n", who_am_i);
	}
	// recheck
	else if(who_am_i != 0x47)
	{
		who_am_i = ICM42688P_ReadByte(ICM42688P_WHO_AM_I); // check again WHO_AM_I (0x75)

		if (who_am_i != 0x47){
			printf( "ICM42688P Not OK: 0x%02x Should be 0x%02x\n", who_am_i, 0x47);
			return 1; //ERROR
		}
	}

	// Reset ICM42688P
	// DEVICE_CONFIG 0x11
	ICM42688P_WriteByte(ICM42688P_DEVICE_CONFIG, 0x01); // Software reset
	HAL_Delay(50);

	// Wait for reset to complete and switch to user bank 0
	ICM42688P_SelectBank(ICM42688P_BANK_SEL_0); // Select user bank 0
	HAL_Delay(10);

	// PWR_MGMT0 0x4E - Main power management
	// Enable Gyro and Accel in Low Noise mode, keep temperature sensor enabled
	ICM42688P_WriteByte(ICM42688P_PWR_MGMT0, ICM42688P_PWR_MGMT0_GYRO_MODE_LN | ICM42688P_PWR_MGMT0_ACCEL_MODE_LN);
	// 온도센서 끄면 자이로 값 이상하게 출력됨 (same as original comment)
	HAL_Delay(50);

	// GYRO_CONFIG0 0x4F - Gyro configuration
	// Set Gyro to ±2000dps and 1kHz ODR (equivalent to original 2000dps setting)
	ICM42688P_WriteByte(ICM42688P_GYRO_CONFIG0, (ICM42688P_GYRO_FS_SEL_2000DPS << 5) | ICM42688P_ODR_1KHZ);
	HAL_Delay(50);

	// ACCEL_CONFIG0 0x50 - Accelerometer configuration
	// Set Accel to ±16g and 1kHz ODR (equivalent to original 16g setting)
	ICM42688P_WriteByte(ICM42688P_ACCEL_CONFIG0, (ICM42688P_ACCEL_FS_SEL_16G << 5) | ICM42688P_ODR_1KHZ);
	HAL_Delay(50);

	// GYRO_CONFIG1 0x51 - Gyro filter configuration
	// Enable gyro DLPF with low-pass filter (equivalent to original 20Hz filter)
	ICM42688P_WriteByte(ICM42688P_GYRO_CONFIG1, 0x16); // DLPF enabled, ~53Hz bandwidth at 1kHz ODR
	HAL_Delay(50);

	// ACCEL_CONFIG1 0x53 - Accel filter configuration
	// Enable accel DLPF with low-pass filter (equivalent to original 44.8Hz filter)
	ICM42688P_WriteByte(ICM42688P_ACCEL_CONFIG1, 0x15); // DLPF enabled, ~53Hz bandwidth at 1kHz ODR
	HAL_Delay(50);

	// TMST_CONFIG 0x54 - Timestamp configuration (optional)
	ICM42688P_WriteByte(ICM42688P_TMST_CONFIG, 0x23); // Enable timestamp, 1kHz resolution
	HAL_Delay(50);

	// FIFO_CONFIG 0x16 - FIFO configuration (disable for this setup, equivalent to original)
	ICM42688P_WriteByte(ICM42688P_FIFO_CONFIG, 0x00); // FIFO bypass mode
	HAL_Delay(50);

	// INT_CONFIG 0x14 - Interrupt configuration
	ICM42688P_WriteByte(ICM42688P_INT_CONFIG, 0x12); // INT1 push-pull, active high, pulse mode
	HAL_Delay(50);

	// INT_CONFIG1 0x64 - Additional interrupt configuration
	ICM42688P_WriteByte(ICM42688P_INT_CONFIG1, 0x00); // Default settings
	HAL_Delay(50);

	// INT_SOURCE0 0x65 - Enable data ready interrupt (equivalent to original INT_ENABLE)
	ICM42688P_WriteByte(ICM42688P_INT_SOURCE0, 0x18); // Enable UI data ready interrupt for INT1
	HAL_Delay(50);

	printf("ICM42688P initialized successfully!\n");

	return 0; //OK
}

void ICM42688P_Get6AxisRawData(short* accel, short* gyro)
{
	unsigned char data[14];
	ICM42688P_ReadBytes(ICM42688P_ACCEL_DATA_X1, 14, data);

	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];

	gyro[0] = ((data[6] << 8) | data[7]);
	gyro[1] = ((data[8] << 8) | data[9]);
	gyro[2] = ((data[10] << 8) | data[11]);
}

void ICM42688P_Get3AxisGyroRawData(short* gyro)
{
	unsigned char data[6];
	ICM42688P_ReadBytes(ICM42688P_GYRO_DATA_X1, 6, data);

	gyro[0] = ((data[0] << 8) | data[1]);
	gyro[1] = ((data[2] << 8) | data[3]);
	gyro[2] = ((data[4] << 8) | data[5]);
}

void ICM42688P_Get3AxisAccRawData(short* gyro)
{
	unsigned char data[6];
	ICM42688P_ReadBytes(ICM42688P_ACCEL_DATA_X1, 6, data);

	gyro[0] = ((data[0] << 8) | data[1]);
	gyro[1] = ((data[2] << 8) | data[3]);
	gyro[2] = ((data[4] << 8) | data[5]);
}

int ICM42688P_DataReady(void)
{
	return LL_GPIO_IsInputPinSet(ICM42688P_INT1_PORT, ICM42688P_INT1_PIN);
}

//int ICM42688P_DataReady(void)
//{
//	// Check if INT1 pin is high (data ready)
//	return LL_GPIO_IsInputPinSet(ICM42688P_INT1_PORT, ICM42688P_INT1_PIN);
//}

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
