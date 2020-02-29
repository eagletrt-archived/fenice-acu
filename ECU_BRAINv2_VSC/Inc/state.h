#ifndef __STATE_H__
#define __STATE_H__

#include "can.h"
#include "sd.h"
#include "global_variables.h"
#include "stdio.h"
#include "stm32f7xx_hal.h"
#include "string.h"
#include "stdbool.h"

enum state_t{STATE_INIT, STATE_IDLE, STATE_SETUP, STATE_RUN}current_state;
enum SCS{APPS, BSE, LV, MOT_TEMP_SX, MOT_TEMP_DX, INV_TEMP_SX, INV_TEMP_DX, INV_CUR_SX, INV_CUR_DX, NUM_SCS};

void init();
void idle();
void calib();
void setup();
void run();

void debug_operations();
void atc_pot_operations();
void imu_operations();

#endif
