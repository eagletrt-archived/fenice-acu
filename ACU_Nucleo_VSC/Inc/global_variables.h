//counters
#ifndef __GLOBAL_VARIABLES_H__
#define __GLOBAL_VARIABLES_H__

#include "can.h"
#include "stm32f7xx_hal.h"
#include "fatfs.h"

#define MAX_DEBUG_RX_L 20
#define MAX_DEBUG_TX_L 500

extern canStruct can1,can3;

extern uint16_t count_ms, count_dec, count_sec, count_min, count_hour;
extern uint16_t count_inverter;
extern uint16_t count_accumulator;
extern uint16_t count_imu;

extern const char code_version[];

extern char debug_rx[MAX_DEBUG_RX_L], debug_tx[MAX_DEBUG_TX_L];
extern uint8_t debug_rx_count, debug_msg_arrived;

extern UART_HandleTypeDef huart3;

extern uint8_t imu_connected, its0_connected, its1_connected, its2_connected, its3_connected;

extern FIL loggingFile;
extern FIL log_names_f;

extern TCHAR message[256];
extern char filename[256];
extern char filename_1[256];
extern char txt[1000];

extern int max_files;
extern int byteswritten;
extern int mount_ok;
extern int msg_counter;
extern int msg_index;

extern char buffer[256];
extern int bytes_read;

extern char *pointer;
extern char log_names[1000];

extern FRESULT res_open;
extern FRESULT res_mount;

extern int successfull_opening;

//-----------------------------------------//
//------------------ ID -------------------//
//-----------------------------------------//
//ACU ID
#define ID_ACU_1 0x100
#define ID_ACU_2 0x55
//IMU
#define ID_imu_angular_rate 0x4EC
#define ID_imu_acceleration 0x4ED
//--- Infrared Temperature Sensor ---//
#define ID_ITS_1_0 0x5B0
#define ID_ITS_1_1 0x5B1
#define ID_ITS_1_2 0x5B2
#define ID_ITS_1_3 0x5B3
#define ID_ITS_2_0 0x5B4
#define ID_ITS_2_1 0x5B5
#define ID_ITS_2_2 0x5B6
#define ID_ITS_2_3 0x5B7
#define ID_ITS_3_0 0x5B8
#define ID_ITS_3_1 0x5B9
#define ID_ITS_3_2 0x5BA
#define ID_ITS_3_3 0x5BB
#define ID_ITS_4_0 0x5BC
#define ID_ITS_4_1 0x5BD
#define ID_ITS_4_2 0x5BE
#define ID_ITS_4_3 0x5BF
//from inverter

#define ID_REQ_INV_SX 0x181
#define ID_REQ_INV_DX 0x182
#define ID_ASK_INV_SX 0x201
#define ID_ASK_INV_DX 0x202
#define ID_ASK_STATE 0x10

//from steer
#define ID_STEERING_WEEL_1 0xA0
#define ID_STEERING_WEEL_2 0xAF
//from BMS
#define ID_BMS_HV 0xAA
#define ID_BMS_LV 0xFF







#endif
