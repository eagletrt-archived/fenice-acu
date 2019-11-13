#include "pedals.h"
#include "stm32f7xx_hal.h"

PotStc accel, brake;

uint8_t check_accel_pot(){
    if(abs(accel.pot1_val - accel.pot2_val) > 90) return 0;
    else return 1;
}
uint8_t check_brake_pot(){
    if(abs(brake.pot1_val - brake.pot2_val) > 90) return 0;
    else return 1;
}