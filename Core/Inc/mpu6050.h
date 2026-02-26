#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"


#define MPU6050_I2C_ADDRESS  		0x68
#define I2C_TIMEOUT  				500

#define MPU6050_ACCEL_SENS 			16384.0f
#define MPU6050_GYRO_SENS  			131.0f

//MPU6050 reg addresses
#define MPU6050_REG_WHOAMI  		117
#define MPU6050_REG_PWMGMT_1		107
#define MPU6050_REG_ACCEL_START 	59
#define MPU6050_REG_CONFIG			26
#define MPU6050_REG_INT_PIN_CFG		55
#define MPU6050_REG_INT_EN			56
#define MPU6050_REG_INT_STATUS		58
#define MPU605_REG_SMPRT_DIV		25


/*
 * Enums
 */
typedef enum
{
	MPU6050_OK,
	MPU6050_ERR
} MPU6050_Status_t;

typedef enum {
	DLPF_CFG_260HZ = 0,
	DLPF_CFG_184HZ = 1,
	DLPF_CFG_94HZ = 2,
	DLPF_CFG_44HZ = 3,
	DLPF_CFG_21HZ = 4,
	DLPF_CFG_10HZ = 5,
	DLPF_CFG_5HZ = 6,

}MPU6050_DLPF_Config_t;

typedef enum
{
	INT_LEVEL_ACTIVE_HIGH = 0x00,
	INT_LEVEL_ACTIVE_LOW = 0x01
}MPU6050_InterruptConfig_t;

typedef enum
{
	RAW_RDY_INT = 0x01,
	I2C_MST_INT = 0x08,
	FIFO_OFLOW_INT = 0x10,
	MOT_INT = 0x40,
	ALL_INT = 0xFF
}MPU6050_Interrupt_t;



/*
 * Structs
 */

typedef struct
{
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
}MPU6050_Raw_t;


typedef struct
{
	I2C_HandleTypeDef *hi2c;
	uint8_t address;
	MPU6050_Raw_t offset;
}MPU6050_Handle_t;

typedef struct
{
	float acc_x_g;
	float acc_y_g;
	float acc_z_g;
	float gyro_x_dps;
	float gyro_y_dps;
	float gyro_z_dps;
	float dt;
}MPU6050_Data_t;


MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hmpu, I2C_HandleTypeDef *hi2c, uint8_t I2C_dev_addr);

MPU6050_Status_t MPU6050_Read(MPU6050_Handle_t *hmpu, MPU6050_Data_t *data);

void MPU6050_AccelCalibration(MPU6050_Handle_t *hmpu, uint16_t samples);

void MPU6050_GyroCalibration(MPU6050_Handle_t *hmpu, uint16_t samples);

MPU6050_Status_t MPU6050_ConfigureLowPassFilter(MPU6050_Handle_t *hmpu, MPU6050_DLPF_Config_t dlpf);

MPU6050_Status_t MPU6050_InterruptConfig(MPU6050_Handle_t *hmpu, MPU6050_InterruptConfig_t level);

MPU6050_Status_t MPU6050_EnableInterrupt(MPU6050_Handle_t *hmpu, MPU6050_Interrupt_t interrupt);

MPU6050_Status_t MPU6050_DisableInterrupt(MPU6050_Handle_t *hmpu, MPU6050_Interrupt_t interrupt);

uint8_t MPU6050_GetAndClearInterruptStatus(MPU6050_Handle_t *hmpu);

MPU6050_Status_t MPU6050_SetSampleRatePrescaler(MPU6050_Handle_t *hmpu, uint8_t prescaler);

#endif /* INC_MPU6050_H_ */


