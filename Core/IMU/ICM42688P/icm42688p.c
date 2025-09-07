#include "icm42688p.h"
#include "stdio.h"
#include <math.h>

Struct_ICM42688P ICM42688P;

// DMA buffers
static uint8_t dma_tx_buffer[ICM42688P_DMA_BUFFER_SIZE];
static uint8_t dma_rx_buffer[ICM42688P_DMA_BUFFER_SIZE];
static volatile uint8_t dma_transfer_complete = 0;
static volatile uint8_t dma_transfer_error = 0;

// Clamp signed 12-bit value
int16_t clamp12(int32_t value) {
    if (value > 2047) return 2047;
    if (value < -2048) return -2048;
    return (int16_t)value;
}

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
	LL_SPI_SetBaudRatePrescaler(ICM42688P_SPI_CHANNEL, LL_SPI_BAUDRATEPRESCALER_DIV8);
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
	ICM42688P_WriteByte(ICM42688P_GYRO_CONFIG0, (ICM42688P_GYRO_FS_SEL_2000DPS << 5) | ICM42688P_ODR_4KHZ);
	HAL_Delay(50);

	// ACCEL_CONFIG0 0x50 - Accelerometer configuration
	// Set Accel to ±16g and 1kHz ODR (equivalent to original 16g setting)
	ICM42688P_WriteByte(ICM42688P_ACCEL_CONFIG0, (ICM42688P_ACCEL_FS_SEL_16G << 5) | ICM42688P_ODR_4KHZ);
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
//void ICM42688P_UpdateAllData(void)
//{
//	int16_t accel_raw[3], gyro_raw[3];
//
//	// Get raw data
//	ICM42688P_Get6AxisRawData(accel_raw, gyro_raw);
//	ICM42688P.temperature_raw = ICM42688P_GetTemperatureRaw();
//
//	// Store raw data
//	ICM42688P.acc_x_raw = accel_raw[0];
//	ICM42688P.acc_y_raw = accel_raw[1];
//	ICM42688P.acc_z_raw = accel_raw[2];
//
//	ICM42688P.gyro_x_raw = gyro_raw[0];
//	ICM42688P.gyro_y_raw = gyro_raw[1];
//	ICM42688P.gyro_z_raw = gyro_raw[2];
//
//	// Convert to physical units
//	ICM42688P.acc_x = ICM42688P_AccelRawToG(accel_raw[0]);
//	ICM42688P.acc_y = ICM42688P_AccelRawToG(accel_raw[1]);
//	ICM42688P.acc_z = ICM42688P_AccelRawToG(accel_raw[2]);
//
//	ICM42688P.gyro_x = ICM42688P_GyroRawToDPS(gyro_raw[0]);
//	ICM42688P.gyro_y = ICM42688P_GyroRawToDPS(gyro_raw[1]);
//	ICM42688P.gyro_z = ICM42688P_GyroRawToDPS(gyro_raw[2]);
//
//
//
//	ICM42688P.temperature = ICM42688P_TempRawToCelsius(ICM42688P.temperature_raw);
//}

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


void ICM42688P_CalibrateGyro(uint16_t samples)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t gyro[3];

    printf("Calibrating gyro... keep sensor still!\n");

    for(uint16_t i = 0; i < samples; i++)
    {
        ICM42688P_Get3AxisGyroRawData(gyro);
        sum_x += gyro[0];
        sum_y += gyro[1];
        sum_z += gyro[2];

        HAL_Delay(2); // small delay (depends on your ODR)
    }

    ICM42688P.gyro_offset_x = (float)sum_x / samples;
    ICM42688P.gyro_offset_y = (float)sum_y / samples;
    ICM42688P.gyro_offset_z = (float)sum_z / samples;

    printf("Gyro calibration done. Offsets: X=%.2f, Y=%.2f, Z=%.2f\n",
            ICM42688P.gyro_offset_x,
            ICM42688P.gyro_offset_y,
            ICM42688P.gyro_offset_z);
}

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

    // Apply gyro offset correction
    ICM42688P.gyro_x_raw = gyro_raw[0] - (int16_t)ICM42688P.gyro_offset_x;
    ICM42688P.gyro_y_raw = gyro_raw[1] - (int16_t)ICM42688P.gyro_offset_y;
    ICM42688P.gyro_z_raw = gyro_raw[2] - (int16_t)ICM42688P.gyro_offset_z;

    // Convert to physical units
    ICM42688P.acc_x = ICM42688P_AccelRawToG(accel_raw[0]);
    ICM42688P.acc_y = ICM42688P_AccelRawToG(accel_raw[1]);
    ICM42688P.acc_z = ICM42688P_AccelRawToG(accel_raw[2]);

    ICM42688P.gyro_x = ICM42688P_GyroRawToDPS(ICM42688P.gyro_x_raw);
    ICM42688P.gyro_y = ICM42688P_GyroRawToDPS(ICM42688P.gyro_y_raw);
    ICM42688P.gyro_z = ICM42688P_GyroRawToDPS(ICM42688P.gyro_z_raw);

    ICM42688P.temperature = ICM42688P_TempRawToCelsius(ICM42688P.temperature_raw);
}

void ICM42688P_CalibrateGyroToRegisters(uint16_t samples)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t gyro[3];

    printf("Calibrating gyro in hardware... keep sensor still!\n");

    for(uint16_t i = 0; i < samples; i++)
    {
        ICM42688P_Get3AxisGyroRawData(gyro);
        sum_x += gyro[0];
        sum_y += gyro[1];
        sum_z += gyro[2];
        HAL_Delay(2);
    }

    int16_t offset_x = -(sum_x / samples);
    int16_t offset_y = -(sum_y / samples);
    int16_t offset_z = -(sum_z / samples);

    // Write to registers
    ICM42688P_SelectBank(1); // Offset registers are in Bank 1

    ICM42688P_WriteByte(0x03, (offset_x >> 8) & 0xFF); // GYRO_X_OFFS_USRH
    ICM42688P_WriteByte(0x04, offset_x & 0xFF);        // GYRO_X_OFFS_USRL

    ICM42688P_WriteByte(0x05, (offset_y >> 8) & 0xFF); // GYRO_Y_OFFS_USRH
    ICM42688P_WriteByte(0x06, offset_y & 0xFF);        // GYRO_Y_OFFS_USRL

    ICM42688P_WriteByte(0x07, (offset_z >> 8) & 0xFF); // GYRO_Z_OFFS_USRH
    ICM42688P_WriteByte(0x08, offset_z & 0xFF);        // GYRO_Z_OFFS_USRL

    ICM42688P_SelectBank(0); // Back to user bank

    printf("Gyro HW offsets written: X=%d, Y=%d, Z=%d\n",
            offset_x, offset_y, offset_z);
}

// Writes gyro hardware offsets to ICM42688P registers in Bank 4
// Retries writes up to 3 times if verification fails
// Returns 0 on success, non-zero on failure
int ICM42688P_WriteHWOffsets(float gyro_bias_dps[3])
{
    // Validate input array
    if (!gyro_bias_dps) {
        printf("Error: Null pointer passed to ICM42688P_WriteHWOffsets\n");
        return -1;
    }

    // Convert gyro biases to LSBs (1 LSB = 1/32 dps for ±2000 dps)
    int16_t gx = clamp12((int32_t)roundf(-gyro_bias_dps[0] * 32.0f));
    int16_t gy = clamp12((int32_t)roundf(-gyro_bias_dps[1] * 32.0f));
    int16_t gz = clamp12((int32_t)roundf(-gyro_bias_dps[2] * 32.0f));

    int retries = 3;
    int verification_failed = 1;

    while (retries-- && verification_failed) {
        // Switch to USER BANK 4
        ICM42688P_SelectBank(4);
        HAL_Delay(1); // Ensure bank switch completes

        // Write gyro offsets
        ICM42688P_WriteByte(0x77, (uint8_t)(gx & 0xFF));                           // OFFSET_USER0: Gx[7:0]
        ICM42688P_WriteByte(0x78, (uint8_t)(((gx >> 8) & 0x0F) | ((gy >> 8) << 4))); // OFFSET_USER1: Gx[11:8] | Gy[11:8]
        ICM42688P_WriteByte(0x79, (uint8_t)(gy & 0xFF));                           // OFFSET_USER2: Gy[7:0]
        ICM42688P_WriteByte(0x7A, (uint8_t)(gz & 0xFF));                           // OFFSET_USER3: Gz[7:0]
        ICM42688P_WriteByte(0x7D, (uint8_t)((gz >> 8) & 0x0F));                    // OFFSET_USER4: Gz[11:8]

        // Verify written offsets
        uint8_t read_back[5];
        ICM42688P_ReadBytes(0x77, 5, read_back);
        verification_failed = 0;

        if (read_back[0] != (uint8_t)(gx & 0xFF)) {
            printf("Error: OFFSET_USER0 mismatch: wrote 0x%02X, read 0x%02X\n",
                   (uint8_t)(gx & 0xFF), read_back[0]);
            verification_failed = 1;
        }
        if (read_back[1] != (uint8_t)(((gx >> 8) & 0x0F) | ((gy >> 8) << 4))) {
            printf("Error: OFFSET_USER1 mismatch: wrote 0x%02X, read 0x%02X\n",
                   (uint8_t)(((gx >> 8) & 0x0F) | ((gy >> 8) << 4)), read_back[1]);
            verification_failed = 1;
        }
        if (read_back[2] != (uint8_t)(gy & 0xFF)) {
            printf("Error: OFFSET_USER2 mismatch: wrote 0x%02X, read 0x%02X\n",
                   (uint8_t)(gy & 0xFF), read_back[2]);
            verification_failed = 1;
        }
        if (read_back[3] != (uint8_t)(gz & 0xFF)) {
            printf("Error: OFFSET_USER3 mismatch: wrote 0x%02X, read 0x%02X\n",
                   (uint8_t)(gz & 0xFF), read_back[3]);
            verification_failed = 1;
        }
        if (read_back[4] != (uint8_t)((gz >> 8) & 0x0F)) {
            printf("Error: OFFSET_USER4 mismatch: wrote 0x%02X, read 0x%02X\n",
                   (uint8_t)((gz >> 8) & 0x0F), read_back[4]);
            verification_failed = 1;
        }

        if (verification_failed) {
            printf("Retrying offset write (%d attempts left)\n", retries);
            HAL_Delay(10); // Delay before retry
        }
    }

    // Switch back to USER BANK 0
    ICM42688P_SelectBank(0);
    HAL_Delay(1); // Ensure bank switch completes

    // Print written offsets
    printf("HW Gyro Offsets written: Gx=%d, Gy=%d, Gz=%d\n", gx, gy, gz);

    // Return status
    return verification_failed ? -2 : 0;
}

// Calibrates the ICM42688P gyro and writes offsets to hardware registers
// Assumes sensor is configured for ±2000 dps gyro
// Sensor must be still during calibration
// Returns 0 on success, non-zero on failure
int ICM42688P_CalibrateAndWriteOffsets(uint16_t sample_count)
{
    // Validate sample count
    if (sample_count == 0) {
        printf("Error: sample_count must be greater than 0\n");
        return -1;
    }

    // Verify gyro configuration
    ICM42688P_SelectBank(0);
    uint8_t gyro_config = ICM42688P_ReadByte(ICM42688P_GYRO_CONFIG0);
    if ((gyro_config & 0xE0) != (ICM42688P_GYRO_FS_SEL_2000DPS << 5)) {
        printf("Error: Gyro full-scale range mismatch: expected ±2000 dps (read 0x%02X)\n", gyro_config);
        return -2;
    }

    int32_t gyro_sum[3] = {0};
    int16_t gyro_raw[3];
    uint16_t valid_samples = 0;

    // Collect one sample before calibration to check raw Z-axis
    while (!ICM42688P_DataReady()) {
        HAL_Delay(1);
    }
    ICM42688P_Get3AxisGyroRawData(gyro_raw);
    printf("Pre-calibration raw gyro data: X=%d LSB (%.3f dps), Y=%d LSB (%.3f dps), Z=%d LSB (%.3f dps)\n",
           gyro_raw[0], ICM42688P_GyroRawToDPS(gyro_raw[0]),
           gyro_raw[1], ICM42688P_GyroRawToDPS(gyro_raw[1]),
           gyro_raw[2], ICM42688P_GyroRawToDPS(gyro_raw[2]));

    printf("Keep sensor still... collecting %d gyro samples\n", sample_count);

    // Collect samples, waiting for data ready interrupt
    for (uint16_t i = 0; i < sample_count; i++) {
        // Wait for data ready interrupt
        uint32_t timeout = 1000; // 1 second timeout
        while (!ICM42688P_DataReady() && timeout--) {
            HAL_Delay(1);
        }
        if (timeout == 0) {
            printf("Error: Data ready timeout during calibration\n");
            return -3;
        }

        // Read raw gyro data
        ICM42688P_Get3AxisGyroRawData(gyro_raw);

        // Validate data (avoid outliers)
        if (abs(gyro_raw[0]) < 32768 && abs(gyro_raw[1]) < 32768 && abs(gyro_raw[2]) < 32768) {
            gyro_sum[0] += gyro_raw[0];
            gyro_sum[1] += gyro_raw[1];
            gyro_sum[2] += gyro_raw[2];
            valid_samples++;
        } else {
            printf("Warning: Skipping outlier sample %d: X=%d, Y=%d, Z=%d\n",
                   i, gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        }

        // Delay based on ODR (4 kHz = 0.25 ms per sample, use 1 ms for safety)
        HAL_Delay(1);
    }

    // Check if enough valid samples were collected
    if (valid_samples < sample_count / 2) {
        printf("Error: Too few valid samples (%d/%d)\n", valid_samples, sample_count);
        return -4;
    }

    // Compute averages
    float gyro_bias[3];
    gyro_bias[0] = (float)gyro_sum[0] / valid_samples * 0.061035f; // raw -> dps
    gyro_bias[1] = (float)gyro_sum[1] / valid_samples * 0.061035f;
    gyro_bias[2] = (float)gyro_sum[2] / valid_samples * 0.061035f;

    // Print raw averages and biases
    printf("Raw averages (LSB, %d valid samples): X=%.2f, Y=%.2f, Z=%.2f\n",
           valid_samples, (float)gyro_sum[0] / valid_samples,
           (float)gyro_sum[1] / valid_samples, (float)gyro_sum[2] / valid_samples);
    printf("Calculated gyro biases (dps): X=%.3f, Y=%.3f, Z=%.3f\n",
           gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    // Write biases to hardware registers
    int result = ICM42688P_WriteHWOffsets(gyro_bias);
    if (result != 0) {
        printf("Error: Failed to write gyro offsets to hardware registers\n");
        return result;
    }

    // Verify Z-axis output after calibration
    while (!ICM42688P_DataReady()) {
        HAL_Delay(1);
    }
    ICM42688P_Get3AxisGyroRawData(gyro_raw);
    printf("Post-calibration raw gyro data: X=%d LSB (%.3f dps), Y=%d LSB (%.3f dps), Z=%d LSB (%.3f dps)\n",
           gyro_raw[0], ICM42688P_GyroRawToDPS(gyro_raw[0]),
           gyro_raw[1], ICM42688P_GyroRawToDPS(gyro_raw[1]),
           gyro_raw[2], ICM42688P_GyroRawToDPS(gyro_raw[2]));

    printf("Gyro biases successfully written to ICM42688P HW registers!\n");
    return 0;
}

// Applies hardcoded gyro offset values to ICM42688P registers in Bank 4
// offsets_lsb: Array of 3 int16_t values for Gx, Gy, Gz in LSBs (e.g., Gz = -4 for 0.131 dps bias)
// Returns 0 on success, non-zero on failure
int ICM42688P_ApplyHardcodedGyroOffsets(int16_t offsets_lsb[3])
{
    // Validate input array
    if (!offsets_lsb) {
        printf("Error: Null pointer passed to ICM42688P_ApplyHardcodedGyroOffsets\n");
        return -1;
    }

    // Clamp offsets to 12-bit signed range (-2048 to 2047)
    int16_t gx = clamp12(offsets_lsb[0]);
    int16_t gy = clamp12(offsets_lsb[1]);
    int16_t gz = clamp12(offsets_lsb[2]);

    // Print offsets
    printf("Applying hardcoded gyro offsets: Gx=%d LSB, Gy=%d LSB, Gz=%d LSB\n",
           gx, gy, gz);

    int retries = 3;
    int verification_failed = 1;

    while (retries-- && verification_failed) {
        // Switch to USER BANK 4
        ICM42688P_SelectBank(4);
        HAL_Delay(2); // Increased delay for SPI stability

        // Write gyro offsets with delays
        ICM42688P_WriteByte(0x77, (uint8_t)(gx & 0xFF)); HAL_Delay(1);           // OFFSET_USER0: Gx[7:0]
        ICM42688P_WriteByte(0x78, (uint8_t)(((gx >> 8) & 0x0F) | ((gy >> 8) << 4))); HAL_Delay(1); // OFFSET_USER1: Gx[11:8] | Gy[11:8]
        ICM42688P_WriteByte(0x79, (uint8_t)(gy & 0xFF)); HAL_Delay(1);           // OFFSET_USER2: Gy[7:0]
        ICM42688P_WriteByte(0x7A, (uint8_t)(gz & 0xFF)); HAL_Delay(1);           // OFFSET_USER3: Gz[7:0]
        ICM42688P_WriteByte(0x7D, (uint8_t)((gz >> 8) & 0x0F)); HAL_Delay(1);    // OFFSET_USER4: Gz[11:8]

        // Verify written offsets
        uint8_t read_back[5];
        ICM42688P_ReadBytes(0x77, 5, read_back);
        verification_failed = 0;

        if (read_back[0] != (uint8_t)(gx & 0xFF)) {
            printf("Error: OFFSET_USER0 mismatch: wrote 0x%02X, read 0x%02X\n",
                   (uint8_t)(gx & 0xFF), read_back[0]);
            verification_failed = 1;
        }
        if (read_back[1] != (uint8_t)(((gx >> 8) & 0x0F) | ((gy >> 8) << 4))) {
            printf("Error: OFFSET_USER1 mismatch: wrote 0x%02X, read 0x%02X\n",
                   (uint8_t)(((gx >> 8) & 0x0F) | ((gy >> 8) << 4)), read_back[1]);
            verification_failed = 1;
        }
        if (read_back[2] != (uint8_t)(gy & 0xFF)) {
            printf("Error: OFFSET_USER2 mismatch: wrote 0x%02X, read 0x%02X\n",
                   (uint8_t)(gy & 0xFF), read_back[2]);
            verification_failed = 1;
        }
        if (read_back[3] != (uint8_t)(gz & 0xFF)) {
            printf("Error: OFFSET_USER3 mismatch: wrote 0x%02X, read 0x%02X\n",
                   (uint8_t)(gz & 0xFF), read_back[3]);
            verification_failed = 1;
        }
        if (read_back[4] != (uint8_t)((gz >> 8) & 0x0F)) {
            printf("Error: OFFSET_USER4 mismatch: wrote 0x%02X, read 0x%02X\n",
                   (uint8_t)((gz >> 8) & 0x0F), read_back[4]);
            verification_failed = 1;
        }

        if (verification_failed) {
            printf("Retrying offset write (%d attempts left)\n", retries);
            HAL_Delay(10); // Delay before retry
        }
    }

    // Switch back to USER BANK 0
    ICM42688P_SelectBank(0);
    HAL_Delay(2); // Increased delay for stability

    // Check post-offset gyro output
    int16_t gyro[3];
    while (!ICM42688P_DataReady()) {
        HAL_Delay(1);
    }
    ICM42688P_Get3AxisGyroRawData(gyro);
    printf("Post-offset raw gyro data: X=%d LSB, Y=%d LSB, Z=%d LSB\n",
           gyro[0], gyro[1], gyro[2]);

    // Return status
    if (verification_failed) {
        printf("Error: Failed to apply hardcoded gyro offsets\n");
        return -2;
    }

    printf("Hardcoded gyro offsets successfully applied to ICM42688P HW registers!\n");
    return 0;
}

int ICM42688P_CalibrateGyroRawOffsets(uint16_t sample_count, int16_t offsets_lsb[3])
{
    // Validate inputs
    if (sample_count == 0) {
        printf("Error: sample_count must be greater than 0\n");
        return -1;
    }
    if (!offsets_lsb) {
        printf("Error: Null pointer for offsets_lsb\n");
        return -2;
    }

    // Verify gyro configuration (±2000 dps)
    ICM42688P_SelectBank(0);
    uint8_t gyro_config = ICM42688P_ReadByte(ICM42688P_GYRO_CONFIG0);
    if ((gyro_config & 0xE0) != (ICM42688P_GYRO_FS_SEL_2000DPS << 5)) {
        printf("Error: Gyro full-scale range mismatch: expected ±2000 dps (read 0x%02X)\n", gyro_config);
        return -3;
    }

    int32_t gyro_sum[3] = {0};
    int16_t gyro_raw[3];
    uint16_t valid_samples = 0;

    // Collect one sample before calibration to check raw Z-axis
    while (!ICM42688P_DataReady()) {
        HAL_Delay(1);
    }
    ICM42688P_Get3AxisGyroRawData(gyro_raw);
    printf("Pre-calibration raw gyro data: X=%d LSB, Y=%d LSB, Z=%d LSB\n",
           gyro_raw[0], gyro_raw[1], gyro_raw[2]);

    printf("Keep sensor still... collecting %d gyro samples\n", sample_count);

    // Collect samples
    for (uint16_t i = 0; i < sample_count; i++) {
        // Wait for data ready interrupt
        uint32_t timeout = 1000; // 1 second timeout
        while (!ICM42688P_DataReady() && timeout--) {
            HAL_Delay(1);
        }
        if (timeout == 0) {
            printf("Error: Data ready timeout during calibration\n");
            return -4;
        }

        // Read raw gyro data
        ICM42688P_Get3AxisGyroRawData(gyro_raw);

        // Validate data (avoid outliers)
        if (abs(gyro_raw[0]) < 32768 && abs(gyro_raw[1]) < 32768 && abs(gyro_raw[2]) < 32768) {
            gyro_sum[0] += gyro_raw[0];
            gyro_sum[1] += gyro_raw[1];
            gyro_sum[2] += gyro_raw[2];
            valid_samples++;
        } else {
            printf("Warning: Skipping outlier sample %d: X=%d, Y=%d, Z=%d\n",
                   i, gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        }

        // Delay based on ODR (4 kHz = 0.25 ms, use 1 ms for safety)
        HAL_Delay(1);
    }

    // Check if enough valid samples were collected
    if (valid_samples < sample_count / 2) {
        printf("Error: Too few valid samples (%d/%d)\n", valid_samples, sample_count);
        return -5;
    }

    // Compute raw offsets (negative of averages to cancel bias)
    offsets_lsb[0] = clamp12((int32_t)roundf(-(float)gyro_sum[0] / valid_samples));
    offsets_lsb[1] = clamp12((int32_t)roundf(-(float)gyro_sum[1] / valid_samples));
    offsets_lsb[2] = clamp12((int32_t)roundf(-(float)gyro_sum[2] / valid_samples));

    // Print results
    printf("Raw averages (LSB, %d valid samples): X=%.2f, Y=%.2f, Z=%.2f\n",
           valid_samples, (float)gyro_sum[0] / valid_samples,
           (float)gyro_sum[1] / valid_samples, (float)gyro_sum[2] / valid_samples);
    printf("Calculated gyro offsets (LSB): X=%d, Y=%d, Z=%d\n",
           offsets_lsb[0], offsets_lsb[1], offsets_lsb[2]);

    return 0;
}
