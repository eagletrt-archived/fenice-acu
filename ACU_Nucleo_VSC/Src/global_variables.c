#include "global_variables.h"
#include "stm32f7xx_hal.h"
#include "fatfs.h"

//--- FOR TIMING COUNTER ---//
uint16_t count_ms = 0, count_dec = 0, count_sec = 0, count_min = 0, count_hour = 0;

//--- COUNTER FOR DEVICE PRESENCE ---//
uint16_t count_inverter = 0;
uint16_t count_accumulator = 0;
uint16_t count_imu = 0;

//--- CODE VERSION ---//
const char code_version[]="\r\nCode version = 1.0, updated 01/11/2019\r\n";

//--- FOR DEBUG ---//
char debug_rx[MAX_DEBUG_RX_L], debug_tx[MAX_DEBUG_TX_L];
uint8_t debug_rx_count = 0, debug_msg_arrived = 0;

//--- DEVICE CONNECTED VARIABLES ---//
uint8_t imu_connected = 0, its0_connected = 0, its1_connected = 0, its2_connected = 0, its3_connected = 0;


//--- RESULT VARIABLES FOR SD ---//
FRESULT res_open;
FRESULT res_mount;

//--- FILE OBJECT FOR SD ---//
FIL loggingFile;
FIL log_names_f;

TCHAR message[256];
char filename[256] = "abcabc.txt";
char filename_1[256]="log_names.txt";
char txt[1000];

int max_files = 100;
int byteswritten;
int mount_ok = 0;
int msg_counter = 0;
int msg_index = 0;

char buffer[256]="Starting Antenna Logging\r\n";
int bytes_read;

char *pointer;
char log_names[1000];



int successfull_opening = 0;

