#include "state.h"
#include "can.h"
#include "global_variables.h"
#include "stm32f7xx_hal.h"
#include "string.h"

// Default state
// Init variables
void init(){
	//HAL_UART_Transmit(&huart3, (uint8_t*)"porcodio\r\n", strlen("porcodio\r\n"), 10);
	if(fifoRxDataCAN_pop(&can1)){
		//HAL_UART_Transmit(&huart3, (uint8_t*)"ciao\r\n", strlen("ciao\r\n"), 10);
		if(can1.id == id.imu_acceleration){
			if(count_imu == 10 || count_imu == 11){
				HAL_UART_Transmit(&huart3, (uint8_t*)"IMU presente\r\n", strlen("IMU presente\r\n"), 10);
			}
			count_imu = 0;
		}else if(can1.id == id.imu_angular_rate){

		}
	}
	if(fifoRxDataCAN_pop(&can3)){

	}
}
void idle(){

}
void calib(){

}
void setup(){

}
void run(){

}
