#include "state.h"
#include "can.h"
#include "global_variables.h"
#include "stm32f7xx_hal.h"
#include "string.h"
#include "stdio.h"

// Default state
// Init variables
void init(){
	if(debug_msg_arrived == 1){
		debug_msg_arrived = 0; // reset flag
		if(strcmp(debug_rx,"help") == 0){
			sprintf(debug_tx,
					"\r\n***********ECU HELP***********\r\n"
					"Avaiable msg are:\r\n"
					"\t-- status -> print ECU status\r\n"
					"\t-- time -> print activity time\r\n"
					"\t-- codev  -> print code version\r\n");
			HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
		}else if(strcmp(debug_rx,"codev") == 0){
			HAL_UART_Transmit(&huart3,(uint8_t*)code_version, strlen(code_version), 100);
		}else if(strcmp(debug_rx,"time") == 0){
			sprintf(debug_tx,"\r\nTime: %d hours : %d min : %d sec : %d dec\r\n",count_hour,count_min, count_sec, count_dec );
			HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
		}else if(strcmp(debug_rx,"status") == 0){
			sprintf(debug_tx,
					"\r\n\ntype of status:\r\n"
					"\t 0 = OK\r\n"
					"\t 1 = ERROR\r\n"
					"\t 2 = BUSY\r\n"
					"\t 3 = TIMEOUT\r\n\n"
					"CAN1 status:\r\n"
					"\tCAN1 config status: %d \r\n"
					"\tCAN1 notification status: %d\r\n"
					"\tCAN1 start status: %d\r\n"
					,can1.configFilter_status,can1.activateNotif_status,can1.canStart_status);
			HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
			sprintf(debug_tx,
					"CAN3 status:\r\n"
					"\tCAN3 config status: %d \r\n"
					"\tCAN3 notification status: %d\r\n"
					"\tCAN3 start status: %d\r\n"
					,can3.configFilter_status,can3.activateNotif_status,can3.canStart_status);
			HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
		}else if(strcmp(debug_rx,"bug") == 0){
			sprintf(debug_tx,
					"\r\n"
					"          $\r\n"
					"        $   $\r\n"
					"       $     $\r\n"
					"       $$$$$$$\r\n"
					"       $$$$$$$\r\n"
					"       $$$$$$$\r\n"
					"       $$$$$$$\r\n"
					"       $$$$$$$\r\n"
					"       $$$$$$$\r\n"
					"       $$$$$$$\r\n"
					"  $$$$$$     $$$$$$\r\n"
					" $$$$$$$$   $$$$$$$$\r\n"
					"$$$$$$$$$$$$$$$$$$$$\r\n"
					" $$$$$$$$   $$$$$$$$\r\n"
					"  $$$$$$     $$$$$$\r\n");
			HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
		}else{
			sprintf(debug_tx,"\r\nERROR : msg %s doesn't exist\r\n",debug_rx);
			HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
		}
	}
	if(fifoRxDataCAN_pop(&can1)){
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
