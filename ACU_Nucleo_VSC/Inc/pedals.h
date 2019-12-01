#ifndef __PEDALS_H__
#define __PEDALS_H__

#include "stm32f7xx_hal.h"

typedef struct{
    uint16_t pot1_val;      //potentiometer 1 values
    uint8_t pot1_val_100;   //potentiometer 1 values [0:100]
    uint16_t pot2_val;      //potentiometer 2 values
    uint8_t pot2_val_100;   //potentiometer 2 values [0:100]
    uint8_t pot_avr_100;    //averange of potentiometer 1 and 2 [0:100]
    uint16_t pot1_max_val;  //potentiometer 1 max values
    uint16_t pot2_max_val;  //potentiometer 2 max values
    uint16_t pot1_range;    //potentiometer 1 range (pot1_max_val - pot1_min_val)
    uint16_t pot1_min_val;  //potentiometer 1 min values
    uint16_t pot2_min_val;  //potentiometer 2 min values
    uint16_t pot2_range;    //potentiometer 2 range (pot2_max_val - pot2_min_val)
}PotStc;

uint8_t accel_implausibility_check();
uint8_t brake_implausibility_check();

#endif
