#include "global_variables.h"
#include "stm32f7xx_hal.h"
#include "fatfs.h"

//--- FOR TIMING COUNTER ---//
uint16_t volatile count_ms = 0, count_dec = 0, count_sec = 0, count_min = 0, count_hour = 0; 
uint32_t volatile count_ms_abs = 0;

//--- COUNTER FOR DEVICE PRESENCE ---//
uint16_t count_inverter = 0;
uint16_t count_accumulator = 0;
uint16_t count_imu = 0;
uint16_t count_atc = 0;

//--- CODE VERSION ---//
const char code_version[]="\r\nCode version = 1.0, updated 01/11/2019\r\n";

//--- FOR DEBUG ---//
char debug_rx[MAX_DEBUG_RX_L], debug_tx[MAX_DEBUG_TX_L];
uint8_t debug_rx_count = 0, debug_msg_arrived = 0;
uint8_t canSnifferMode = 0;

//--- DEVICE CONNECTED VARIABLES ---//
uint8_t imu_connected = 0;
uint8_t its_connected = 0;
uint8_t atc_connected = 0;



