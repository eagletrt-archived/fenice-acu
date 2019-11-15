#include "pedals.h"
#include "stm32f7xx_hal.h"

PotStc accel, brake;
uint8_t accel_implausibility_check_count_flag = 0;
uint8_t brake_implausibility_check_count_flag = 0;
uint8_t accel_implausibility_check_count = 0;
uint8_t brake_implausibility_check_count = 0;

uint8_t accel_implausibility_check(){
    if(abs(accel.pot1_val_100 + accel.pot2_val_100 - 100) > 10){
        //error
        if(accel_implausibility_check_count_flag != 2){
            accel_implausibility_check_count_flag = 1;
            return 1;
        }
         //if aiccf = 2 -> just passed 50ms
        else return 0; //return error
    }else{
        accel_implausibility_check_count_flag = 0;
        accel_implausibility_check_count = 0;
        return 1; //retrun ok

    }
}
uint8_t brake_implausibility_check(){
    return 0;
}