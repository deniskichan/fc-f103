#include "main.h"
#include "mpu6050.h"
#include <stdio.h>

//#define TIMEOUT

static HAL_StatusTypeDef MPU6050_ReadByte(MPU6050_Handle_t *hmpu, uint8_t reg_addr, uint8_t *rx_buff);

static HAL_StatusTypeDef MPU6050_BurstRead(MPU6050_Handle_t *hmpu, uint8_t reg_base_addr, uint8_t *rx_buff, uint8_t len);

static HAL_StatusTypeDef MPU6050_WriteByte(MPU6050_Handle_t *hmpu, uint8_t reg_addr, uint8_t *tx_buff);

static MPU6050_Status_t MPU6050_ReadRaw(MPU6050_Handle_t *hmpu, MPU6050_Raw_t *raw);

static int16_t make_int16(uint8_t high, uint8_t low);



MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hmpu, I2C_HandleTypeDef *hi2c, uint8_t I2C_device_addr)
{
	uint8_t rx_buff;
	uint8_t tx_buff;

	hmpu->hi2c = hi2c;
	hmpu->address = I2C_device_addr;

	//1. MPU reset
	tx_buff = 0x80;
    if (MPU6050_WriteByte(hmpu, MPU6050_REG_PWMGMT_1, &tx_buff) != HAL_OK)
    {
       return MPU6050_ERR;
    }

    //wait for reset
    HAL_Delay(20);

	//2. Check WHOAMI Reg
	if (MPU6050_ReadByte(hmpu, MPU6050_REG_WHOAMI, &rx_buff) != HAL_OK)
	{
		// Couldn't get response from device
		return MPU6050_ERR;
	} else if (rx_buff != 0x68 && rx_buff != 0x98)
	{
		// Invalid device found
		return MPU6050_ERR;
	}

	//3. Disable Sleep Mode + set PLL with X axis gyro reference
	tx_buff = 0x01;
	if (MPU6050_WriteByte(hmpu, MPU6050_REG_PWMGMT_1, &tx_buff) != HAL_OK)
	{
		return MPU6050_ERR;
	}
	return MPU6050_OK;
}

MPU6050_Status_t MPU6050_Read(MPU6050_Handle_t *hmpu, MPU6050_Data_t *data)
{
	MPU6050_Raw_t raw = {0};

	if (MPU6050_ReadRaw(hmpu, &raw) != MPU6050_OK)
	{
		return MPU6050_ERR;
	}

	data->acc_x_g = (raw.acc_x - hmpu->offset.acc_x) / MPU6050_ACCEL_SENS;
	data->acc_y_g = (raw.acc_y - hmpu->offset.acc_y) / MPU6050_ACCEL_SENS;
	data->acc_z_g = (raw.acc_z - hmpu->offset.acc_z) / MPU6050_ACCEL_SENS;

	data->gyro_x_dps = (raw.gyro_x - hmpu->offset.gyro_x) / MPU6050_GYRO_SENS;
	data->gyro_y_dps = (raw.gyro_y - hmpu->offset.gyro_y) / MPU6050_GYRO_SENS;
	data->gyro_z_dps = (raw.gyro_z - hmpu->offset.gyro_z) / MPU6050_GYRO_SENS;

	return MPU6050_OK;
}


static MPU6050_Status_t MPU6050_ReadRaw(MPU6050_Handle_t *hmpu, MPU6050_Raw_t *raw)
{
	uint8_t buff[14];
	HAL_StatusTypeDef status = MPU6050_BurstRead(hmpu, MPU6050_REG_ACCEL_START, buff, sizeof(buff));
	if (status != HAL_OK)
	{
		return MPU6050_ERR;
	}
	raw->acc_x = make_int16(buff[0], buff[1]);
	raw->acc_y =  make_int16(buff[2], buff[3]);
	raw->acc_z = make_int16(buff[4], buff[5]);

	raw->gyro_x =  make_int16(buff[8], buff[9]);
	raw->gyro_y =  make_int16(buff[10], buff[11]);
	raw->gyro_z =  make_int16(buff[12], buff[13]);

	return MPU6050_OK;
}

static int16_t make_int16(uint8_t high, uint8_t low)
{
	return (int16_t)(((uint16_t)high << 8) | low);
}


static HAL_StatusTypeDef MPU6050_ReadByte(MPU6050_Handle_t *hmpu, uint8_t reg_addr, uint8_t *rx_buff)
{
#ifdef TIMEOUT
//	HAL_StatusTypeDef status;
//	uint8_t retry = 0;
//	do {
//		status = HAL_I2C_Mem_Read(hi2c, I2C_dev_addr << 1, reg_addr, 1, rx_buff, 1, I2C_TIMEOUT);
//		if (status == HAL_OK)
//			return HAL_OK;
//		retry++;
//		HAL_Delay(2);
//	} while (status != HAL_OK && retry < I2C_MAX_RETRIES);
//
//	HAL_I2C_DeInit(hi2c);
//	HAL_I2C_Init(hi2c);
#endif

	return HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->address << 1, reg_addr, 1, rx_buff, 1, I2C_TIMEOUT);
}

static HAL_StatusTypeDef MPU6050_BurstRead(MPU6050_Handle_t *hmpu, uint8_t reg_base_addr, uint8_t *rx_buff, uint8_t len)
{
#ifdef TIMEOUT
//	HAL_StatusTypeDef status;
//	uint8_t retry = 0;
//	do {
//		status = HAL_I2C_Mem_Read(hi2c, I2C_dev_addr << 1, reg_base_addr, 1, rx_buff, len , I2C_TIMEOUT);
//		if (status == HAL_OK)
//			return HAL_OK;
//		retry++;
//		HAL_Delay(2);
//	} while (status != HAL_OK && retry < I2C_MAX_RETRIES);
//
//	HAL_I2C_DeInit(hi2c);
//	HAL_I2C_Init(hi2c);
#endif

	return HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->address << 1, reg_base_addr, 1, rx_buff, len , I2C_TIMEOUT);
}

static HAL_StatusTypeDef MPU6050_WriteByte(MPU6050_Handle_t *hmpu, uint8_t reg_addr, uint8_t *tx_buff)
{
#ifdef TIMEOUT
//	HAL_StatusTypeDef status;
//	uint8_t retry = 0;
//	do {
//		status = HAL_I2C_Mem_Write(hi2c, I2C_dev_addr << 1, reg_addr, 1, tx_buff, 1, I2C_TIMEOUT);
//		if (status == HAL_OK)
//			return HAL_OK;
//		retry++;
//		HAL_Delay(2);
//	} while (status != HAL_OK && retry < I2C_MAX_RETRIES);
//
//	HAL_I2C_DeInit(hi2c);
//	HAL_I2C_Init(hi2c);
#endif

	return HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->address << 1, reg_addr, 1, tx_buff, 1, I2C_TIMEOUT);
}

void MPU6050_AccelCalibration(MPU6050_Handle_t *hmpu, uint16_t samples)
{

	int32_t sum_x = 0, sum_y = 0, sum_z = 0;
	MPU6050_Raw_t raw_data;

	for (uint16_t i = 0; i < samples; i++)
	{
		MPU6050_ReadRaw(hmpu, &raw_data);
		sum_x += raw_data.acc_x;
		sum_y += raw_data.acc_y;
		sum_z += raw_data.acc_z;
		HAL_Delay(10);
	}
	hmpu->offset.acc_x = sum_x / samples;
	hmpu->offset.acc_y = sum_y / samples;
	hmpu->offset.acc_z = (sum_z / samples) - (uint16_t)MPU6050_ACCEL_SENS;
}

void MPU6050_GyroCalibration(MPU6050_Handle_t *hmpu, uint16_t samples)
{

	int32_t sum_x = 0, sum_y = 0, sum_z = 0;
	MPU6050_Raw_t raw_data;

	for (uint16_t i = 0; i < samples; i++)
	{
		MPU6050_ReadRaw(hmpu, &raw_data);
		sum_x += raw_data.gyro_x;
		sum_y += raw_data.gyro_y;
		sum_z += raw_data.gyro_z;
		HAL_Delay(10);
	}
	hmpu->offset.gyro_x = sum_x / samples;
	hmpu->offset.gyro_y = sum_y / samples;
	hmpu->offset.gyro_z = sum_z / samples;
}

MPU6050_Status_t MPU6050_ConfigureLowPassFilter(MPU6050_Handle_t *hmpu, MPU6050_DLPF_Config_t dlpf)
{
	uint8_t rx_buff = 0;

	if (MPU6050_ReadByte(hmpu, MPU6050_REG_CONFIG, &rx_buff) != HAL_OK)
	{
		return MPU6050_ERR;
	}

	rx_buff &= ~(0x7);
	rx_buff |= (uint8_t)dlpf;

	if (MPU6050_WriteByte(hmpu, MPU6050_REG_CONFIG, &rx_buff) != HAL_OK)
	{
		return MPU6050_ERR;
	}

	return MPU6050_OK;
}



MPU6050_Status_t MPU6050_InterruptConfig(MPU6050_Handle_t *hmpu, MPU6050_InterruptConfig_t level)
{
	uint8_t tx_buff, rx_buff;

	if (MPU6050_ReadByte(hmpu, MPU6050_REG_INT_PIN_CFG, &rx_buff) != HAL_OK)
	{
		return MPU6050_ERR;
	}

	tx_buff = (level == INT_LEVEL_ACTIVE_LOW) ? (rx_buff | (1 << 7)) : (rx_buff & ~(1 << 7));

	if (MPU6050_WriteByte(hmpu, MPU6050_REG_INT_PIN_CFG, &tx_buff) != HAL_OK)
	{
		return MPU6050_ERR;
	}
	return MPU6050_OK;
}

MPU6050_Status_t MPU6050_EnableInterrupt(MPU6050_Handle_t *hmpu, MPU6050_Interrupt_t interrupt)
{
	uint8_t tx_buff, rx_buff;

	if (MPU6050_ReadByte(hmpu, MPU6050_REG_INT_EN, &rx_buff) != HAL_OK)
	{
		return MPU6050_ERR;
	}

	tx_buff = rx_buff | interrupt;

	if (MPU6050_WriteByte(hmpu, MPU6050_REG_INT_EN, &tx_buff) != HAL_OK)
	{
		return MPU6050_ERR;
	}
	return MPU6050_OK;
}

MPU6050_Status_t MPU6050_DisableInterrupt(MPU6050_Handle_t *hmpu, MPU6050_Interrupt_t interrupt)
{
	uint8_t tx_buff, rx_buff;

	if (MPU6050_ReadByte(hmpu, MPU6050_REG_INT_EN, &rx_buff) != HAL_OK)
	{
		return MPU6050_ERR;
	}

	tx_buff = rx_buff & ~interrupt;

	if (MPU6050_WriteByte(hmpu, MPU6050_REG_INT_EN, &tx_buff) != HAL_OK)
	{
		return MPU6050_ERR;
	}
	return MPU6050_OK;
}


uint8_t MPU6050_GetAndClearInterruptStatus(MPU6050_Handle_t *hmpu)
{
	//Read INT status reg of the sensor to clear the interrupt
	uint8_t rx_buff = 0;

	MPU6050_ReadByte(hmpu, MPU6050_REG_INT_STATUS, &rx_buff);

	return rx_buff;
}

MPU6050_Status_t MPU6050_SetSampleRatePrescaler(MPU6050_Handle_t *hmpu, uint8_t prescaler)
{
	if (MPU6050_WriteByte(hmpu, MPU605_REG_SMPRT_DIV, &prescaler) != HAL_OK)
	{
		return MPU6050_OK;
	}
	return MPU6050_OK;
}
