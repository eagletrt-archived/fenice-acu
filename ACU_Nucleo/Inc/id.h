#ifndef ID_H
#define ID_H

#include "main.h"

typedef struct{
	//IMU
	uint32_t imu_angular_rate;
	uint32_t imu_acceleration;
	//Infrared Temperature Sensor
	uint32_t ITS_1_0;
	uint32_t ITS_1_1;
	uint32_t ITS_1_2;
	uint32_t ITS_1_3;
	uint32_t ITS_2_0;
	uint32_t ITS_2_1;
	uint32_t ITS_2_2;
	uint32_t ITS_2_3;
	uint32_t ITS_3_0;
	uint32_t ITS_3_1;
	uint32_t ITS_3_2;
	uint32_t ITS_3_3;
	uint32_t ITS_4_0;
	uint32_t ITS_4_1;
	uint32_t ITS_4_2;
	uint32_t ITS_4_3;

}ID;

void ID_init(ID* _ID);


#endif
