#ifndef __PEDALS_H__
#define __PEDALS_H__

#include "stm32f7xx_hal.h"

typedef struct{
    int pot1_val;
    int pot2_val;
    int max_val;
    int min_val;
}PotStc;

uint8_t accel_implausibility_check();
uint8_t brake_implausibility_check();

#endif
