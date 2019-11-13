#ifndef __PEDALS_H__
#define __PEDALS_H__

#include "stm32f7xx_hal.h"

typedef struct{
    int pot1_val;
    int pot2_val;
}PotStc;

uint8_t check_accel_pot();
uint8_t check_brake_pot();

#endif
