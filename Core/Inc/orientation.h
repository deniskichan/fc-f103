#ifndef INC_ORIENTATION_H_
#define INC_ORIENTATION_H_


#include "mpu6050.h"

#define RAD_TO_DEG 57.2957795f
#define TAU			0.3f	//how quickly accelerometer can fix gyro

typedef struct
{
	float roll;
	float pitch;
}Orientation_Handle_t;


void Orientation_Update(Orientation_Handle_t *o, MPU6050_Data_t *mpu_data);



#endif /* INC_ORIENTATION_H_ */
