//counters
#ifndef __GLOBAL_VARIABLES_H__
#define __GLOBAL_VARIABLES_H__

#include "can.h"
#include "stm32f7xx_hal.h"
#include "fatfs.h"
#include "pedals.h"

#define MAX_DEBUG_RX_L 20
#define MAX_DEBUG_TX_L 2000

extern canStruct can1, can3;

extern uint16_t volatile count_ms, count_dec, count_sec, count_min, count_hour;
extern uint32_t volatile count_ms_abs;
extern uint16_t count_inverter;
extern uint16_t count_accumulator;
extern uint16_t count_imu;
extern uint16_t count_atc;

extern const char code_version[];

extern char debug_rx[MAX_DEBUG_RX_L], debug_tx[MAX_DEBUG_TX_L];
extern uint8_t debug_rx_count, debug_msg_arrived;

extern UART_HandleTypeDef huart3;

extern uint8_t imu_connected;
extern uint8_t its_connected;
extern uint8_t atc_connected;

//-----------------------------------------//
//--------------- FROM SD------------------//
//-----------------------------------------//

extern FIL loggingFile;
extern FIL pot_values_f;
extern FIL log_names_f;

extern TCHAR message[256];
extern char filename[256];
extern char filename_1[256];
extern char filename_pot[];
extern char txt[2000];
extern char filename_log[50];

extern int max_files;
extern UINT byteswritten;
extern int mount_ok;
extern int msg_counter;
extern int msg_index;

extern char buffer[256];
extern int bytes_read;

extern char *pointer;
extern char log_names[1100];

extern FRESULT res_open;
extern FRESULT res_mount;

extern int successfull_opening;

extern uint8_t pot_values_loaded;

//-----------------------------------------//
//------------- FROM PEDALS----------------//
//-----------------------------------------//
extern PotStc accel;
extern PotStc brake;
extern uint8_t accel_implausibility_check_count_flag;
extern uint8_t accel_implausibility_check_count;

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
//REQ is id that is sent FROM inverter
//ASK is id that is sent TO inverter
#define ID_REQ_INV_SX 0x181
#define ID_REQ_INV_DX 0x182
#define ID_ASK_INV_SX 0x201
#define ID_ASK_INV_DX 0x202
#define ID_ASK_STATE 0x10

//from steer
#define ID_STEERING_WEEL_1 0xA0
#define ID_STEERING_WEEL_2 0xAF
#define REQUEST_TS_ON 2
//from BMS
#define ID_BMS_HV 0xAA
#define ID_BMS_LV 0xFF

//from ATC (Analog To CAN)
#define ID_ATC_POT 0x34

#endif
