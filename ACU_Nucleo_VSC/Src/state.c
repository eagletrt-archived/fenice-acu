#include "state.h"
#include "can.h"
#include "global_variables.h"
#include "stm32f7xx_hal.h"
#include "string.h"
#include "stdio.h"

// Default state
// Init variables
/*******************************************************************
 *                         START INIT STATE
 *******************************************************************/
void init(){
	if(debug_msg_arrived == 1){
		debug_msg_arrived = 0; // reset flag
		debug_operations();
	}
	if(fifoRxDataCAN_pop(&can1)){
		switch (can1.rx_id){
		case ID_imu_acceleration:
		case ID_imu_angular_rate:
			imu_operations();
			break;
		
		default:
			break;
		}
	}
	if(fifoRxDataCAN_pop(&can3)){

	}
	current_state = STATE_IDLE;
}
/*******************************************************************
 *                         END INIT STATE
 *******************************************************************/
/*******************************************************************
 *                        START IDLE STATE
 *******************************************************************/
void idle(){
	if(debug_msg_arrived == 1){
		debug_msg_arrived = 0; // reset flag
		debug_operations();
	}
	if(fifoRxDataCAN_pop(&can1)){
		switch(can1.rx_id){
			case ID_ASK_STATE:
				can1.dataTx[0] = (uint8_t)current_state;
				can1.dataTx[1] = 0;
				can1.dataTx[2] = 0;
				can1.dataTx[3] = 0;
				can1.dataTx[4] = 0;
				can1.dataTx[5] = 0;
				can1.dataTx[6] = 0;
				can1.dataTx[7] = 0;
				can1.tx_id = ID_ACU_1;
				CAN_Send(&can1, normalPriority);
				break;
			case ID_ASK_INV_DX:
				break;
			case ID_ASK_INV_SX:
				break;
			case ID_BMS_HV:
				break;
			case ID_BMS_LV:
				break;
			case ID_STEERING_WEEL_1:
				if(can1.dataRx[0] == 2){ //----- change the current state -----//
					current_state = can1.dataRx[1];
				}else if(can1.dataRx[0] == 3){ //----- change state to setup -----//
					current_state = STATE_SETUP;
				}
				break;
			case ID_ATC_POT:
				atc_pot_operations();
				break;
			default:
				break;
		}
	}

}
/*******************************************************************
 *                         END IDLE STATE
 *******************************************************************/
/*******************************************************************
 *                        START CALIB STATE
 *******************************************************************/
void calib(){

}
/*******************************************************************
 *                         END CALIB STATE
 *******************************************************************/
/*******************************************************************
 *                        START SETUP STATE
 *******************************************************************/
void setup(){
	if(fifoRxDataCAN_pop(&can1)){
		switch(can1.rx_id){
			case ID_STEERING_WEEL_1:
				if(can1.dataRx[0] == 4){ //----- change state to idle -----//
					current_state = STATE_IDLE;
				}else if(can1.dataRx[0] == 5){ //----- change state to run -----//
					current_state = STATE_RUN;
					can1.tx_id = ID_ACU_2;
					can1.dataRx[0] = 5;
					can1.tx_size = 1;
					CAN_Send(&can1, normalPriority);
				}
			case ID_ATC_POT:
				atc_pot_operations();
				break;
			default:
				break;
		}
	}
}
/*******************************************************************
 *                         END SETUP STATE
 *******************************************************************/
/*******************************************************************
 *                         START RUN STATE
 *******************************************************************/
void run(){
	if(fifoRxDataCAN_pop(&can1)){
		switch(can1.rx_id){
			case ID_STEERING_WEEL_1:
				if(can1.dataRx[0] == 6){ //----- change state to setup -----//
					current_state = STATE_SETUP;
				}
				break;
			case ID_ATC_POT:
				atc_pot_operations();
				break;
			default:
				break;
		}
	}
}
/*******************************************************************
 *                         END RUN STATE
 *******************************************************************/

void debug_operations(){
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
		if(current_state == STATE_INIT){
			HAL_UART_Transmit(&huart3,(uint8_t*)"\r\nCurrent state: STATE_INIT\r\n", strlen("\r\nCurrent state: STATE_INIT\r\n"), 100);
		}else if(current_state == STATE_IDLE){
			HAL_UART_Transmit(&huart3,(uint8_t*)"\r\nCurrent state: STATE_IDLE\r\n", strlen("\r\nCurrent state: STATE_IDLE\r\n"), 100);
		}else if(current_state == STATE_CALIB){
			HAL_UART_Transmit(&huart3,(uint8_t*)"\r\nCurrent state: STATE_CALIB\r\n", strlen("\r\nCurrent state: STATE_CALIB\r\n"), 100);
		}else if(current_state == STATE_SETUP){
			HAL_UART_Transmit(&huart3,(uint8_t*)"\r\nCurrent state: STATE_SETUP\r\n", strlen("\r\nCurrent state: STATE_SETUP\r\n"), 100);
		}else if(current_state == STATE_RUN){
			HAL_UART_Transmit(&huart3,(uint8_t*)"\r\nCurrent state: STATE_RUN\r\n", strlen("\r\nCurrent state: STATE_RUN\r\n"), 100);
		}
		sprintf(debug_tx,
				"\r\n"
				"Device connected : (0 = no, 1 = yes)\r\n"
				"\t IMU -> %d\r\n"
				"\t ITS0 -> %d\r\n"
				"\t ITS1 -> %d\r\n"
				"\t ITS2 -> %d\r\n"
				"\t ITS3 -> %d\r\n"
				,imu_connected, its0_connected, its1_connected, its2_connected, its3_connected);
		HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
	}else if(strcmp(debug_rx,"gay") == 0){
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

void imu_operations(){
	imu_connected = 1; //imu connected true
	if(count_imu == 10 || count_imu == 11){
		HAL_UART_Transmit(&huart3, (uint8_t*)"IMU presente\r\n", strlen("IMU presente\r\n"), 10);
	}
	count_imu = 0;
}
void atc_pot_operations(){
	atc_connected = 1;
	count_atc = 0;
	accel.pot1_val = can1.dataRx[0];
	accel.pot2_val = can1.dataRx[1];
	brake.pot1_val = can1.dataRx[2];
	brake.pot2_val = can1.dataRx[3];
	if(check_accel_pot()){
		accel.pot1_val = 0;
		accel.pot2_val = 0;
	}
	if(check_brake_pot()){
		brake.pot1_val = 0;
		brake.pot2_val = 0;
	}

}

