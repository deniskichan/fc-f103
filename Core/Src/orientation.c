#include "orientation.h"
#include <math.h>



void Orientation_Update(Orientation_Handle_t *o, MPU6050_Data_t *mpu_data)
{
	float alpha = TAU / (TAU + mpu_data->dt);
	float acc_roll = atan2f(mpu_data->acc_y_g, mpu_data->acc_z_g) * RAD_TO_DEG;

	o->roll += mpu_data->gyro_x_dps * mpu_data->dt;
	o->roll = alpha * o->roll + (1.0f - alpha) * acc_roll;

	float acc_pitch = atan2f(
			-mpu_data->acc_x_g,
			sqrtf(mpu_data->acc_y_g * mpu_data->acc_y_g + mpu_data->acc_z_g * mpu_data->acc_z_g)
			) * RAD_TO_DEG;

	o->pitch += mpu_data->gyro_y_dps * mpu_data->dt;
	o->pitch = alpha * o->pitch + (1.0f - alpha) * acc_pitch;
}
