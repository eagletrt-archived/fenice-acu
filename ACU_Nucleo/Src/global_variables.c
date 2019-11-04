#include "global_variables.h"
#include "stm32f7xx_hal.h"


uint16_t count_ms = 0, count_dec = 0, count_sec = 0, count_min = 0, count_hour = 0;
uint16_t count_inverter = 0;
uint16_t count_accumulator = 0;
uint16_t count_imu = 0;

const char code_version[]="\r\nCode version = 1.0, updated 01/11/2019\r\n";

char debug_rx[MAX_DEBUG_RX_L], debug_tx[MAX_DEBUG_TX_L];
uint8_t debug_rx_count = 0, debug_msg_arrived = 0;

uint8_t imu_connected = 0, its0_connected = 0, its1_connected = 0, its2_connected = 0, its3_connected = 0;

