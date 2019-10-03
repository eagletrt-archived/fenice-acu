#ifndef ID_H
#define ID_H

#include "main.h"

typedef struct{
	uint16_t imu_angular_rate;
	uint16_t imu_acceleration;
	uint16_t ITS_1; //Infrared Temperature Sensor
	uint16_t ITS_2;
	uint16_t ITS_3;

}ID;

void ID_intit(ID* _ID);


#endif
