#ifndef __PEDALS_H__
#define __PEDALS_H__

#include "stm32f7xx_hal.h"

typedef struct{
    uint16_t pot1_val;
    uint8_t pot1_val_100;
    uint16_t pot2_val;
    uint8_t pot2_val_100;
    uint16_t pot1_max_val;
    uint16_t pot2_max_val;
    uint16_t pot1_range;
    uint16_t pot1_min_val;
    uint16_t pot2_min_val;
    uint16_t pot2_range;
}PotStc;

uint8_t accel_implausibility_check();
uint8_t brake_implausibility_check();

#endif
