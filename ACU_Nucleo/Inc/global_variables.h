//counters
#ifndef __GLOBAL_VARIABLES_H__
#define __GLOBAL_VARIABLES_H__

#include "can.h"
#include "id.h"
#include "stm32f7xx_hal.h"


extern canStruct can1,can3;
extern ID id;

extern uint16_t count_ms, count_dec, count_sec;
extern uint16_t count_inverter;
extern uint16_t count_accumulator;
extern uint16_t count_imu;


extern UART_HandleTypeDef huart3;

#endif

