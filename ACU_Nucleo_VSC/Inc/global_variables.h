//counters
#ifndef __GLOBAL_VARIABLES_H__
#define __GLOBAL_VARIABLES_H__

#include "can.h"
#include "id.h"
#include "stm32f7xx_hal.h"
#include "fatfs.h"

#define MAX_DEBUG_RX_L 20
#define MAX_DEBUG_TX_L 500

extern canStruct can1,can3;
extern ID id;

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
#endif

