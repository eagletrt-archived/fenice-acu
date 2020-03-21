#include "state.h"

/*******************************************************************
 *                         USER FUNCTIONS
 *******************************************************************/ 
void set_bit_uint8(uint8_t* _var, uint8_t _nBit, uint8_t _bool){
	if(_bool == 0 || _bool == 1){
		(*_var) = ((*_var) & ( 0b11111111 ^ (0b00000001 << _nBit))) | (0b00000000 | (_bool << _nBit));
	}
}
void send_errors(){
	can1.tx_id = ID_ACU_ERRORS;
	can1.dataTx[0] = critical_errors[0];
	can1.dataTx[1] = critical_errors[1];
	can1.dataTx[2] = critical_errors[2];
	can1.dataTx[3] = critical_errors[3];
	can1.dataTx[4] = critical_errors[4];
	can1.dataTx[5] = critical_errors[5];
	can1.dataTx[6] = critical_errors[6];
	can1.dataTx[7] = critical_errors[7];
	can1.tx_size = 8;
	CAN_Send(&can1, highPriority);
}

uint8_t check_error_presence(){
	for(int i = 0; i < 8 ; i++){
		if(critical_errors[i] != 0 ) return 1;
	}
	return 0;
}

void shutdown(){
	/* Send inverter L disable */
	can3.tx_id = ID_ASK_INV_SX;
	can3.dataTx[0] = 0x51;
	can3.dataTx[1] = 0x04;
	can3.dataTx[2] = 0x00;
	can3.tx_size = 3;
	CAN_Send(&can1, highPriority);

	/* Send inverter R disable */
	can3.tx_id = ID_ASK_INV_DX;
	can3.dataTx[0] = 0x51;
	can3.dataTx[1] = 0x04;
	can3.dataTx[2] = 0x00;
	can3.tx_size = 3;
	CAN_Send(&can1, highPriority);
}
/*******************************************************************
 *                         STATE VARIABLES
 *******************************************************************/ 
/*** GLOBAL ***/
int setup_init = 0;
uint8_t critical_errors[8] = {0,0,0,0,0,0,0,0};

/*** FOR INIT STATE ***/
int init_step = 0;
uint32_t init_step_start_time = 0;
uint32_t init_precharge_start_time = 0;
uint32_t init_inv_resp = 0;

uint8_t inv_init_response = 0; // bit 0 = inv R -> 0 = no / 1 = YES ---- bit 1 = inv L -> 0 = no / 1 = yes
// Default state
// Init variables
/*******************************************************************
 *                         START INIT STATE
 *******************************************************************/
void init()
{
	if(init_step == 0){
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
		init_step = 1;
		/* Send inverter L disable */
		can3.tx_id = ID_ASK_INV_SX;
		can3.dataTx[0] = 0x51;
		can3.dataTx[1] = 0x04;
		can3.dataTx[2] = 0x00;
		can3.tx_size = 3;
		CAN_Send(&can1, normalPriority);

		/* Send inverter R disable */
		can3.tx_id = ID_ASK_INV_DX;
		can3.dataTx[0] = 0x51;
		can3.dataTx[1] = 0x04;
		can3.dataTx[2] = 0x00;
		can3.tx_size = 3;
		CAN_Send(&can1, normalPriority);

		/* Send req to inverter L presence */
		can3.tx_id = ID_ASK_INV_SX;
		can3.dataTx[0] = 0x3D;
		can3.dataTx[1] = 0xE2;
		can3.dataTx[2] = 0x00;
		can3.tx_size = 3;
		CAN_Send(&can1, normalPriority);

		/* Send req to inverter R presence */
		can3.tx_id = ID_ASK_INV_DX;
		can3.dataTx[0] = 0x3D;
		can3.dataTx[1] = 0xE2;
		can3.dataTx[2] = 0x00;
		can3.tx_size = 3;
		CAN_Send(&can1, normalPriority);

		init_step_start_time = count_ms_abs; // save the curret time

	}else if(init_step == 1){
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
		if (fifoRxDataCAN_pop(&can1)){
			switch (can1.rx_id)
			{
			case ID_ASK_INV_DX:
				if(can1.dataRx[0] == 0xE2 && can1.dataRx[1] == 0x01 && can1.dataRx[2] == 0x00 && can1.dataRx[3] == 0x00){
					set_bit_uint8(&inv_init_response, 0, 1); //set bit 0 to 1
				}
				break;
			case ID_ASK_INV_SX:
				if(can1.dataRx[0] == 0xE2 && can1.dataRx[1] == 0x01 && can1.dataRx[2] == 0x00 && can1.dataRx[3] == 0x00){
					set_bit_uint8(&inv_init_response, 1, 1); //set bit 1 to 1
				}
			case ID_ATC_POT:
				atc_pot_operations();
				break;
			default:
				break;
			}
		}
		if(fifoRxDataCAN_pop(&can3)){
			switch (can3.rx_id)
			{
				case ID_imu_acceleration:
					imu_operations();
					accel_x_h = can3.dataRx[0];
					accel_x_l = can3.dataRx[1];
					break;
				
				default:
					break;
			}
		}
		if(inv_init_response == 3){ // means that each inv has responded
			init_step = 2;
		}else{
			if(count_ms_abs - init_step_start_time > 1000 ){ //if is passed more than 1 second -> go ahead
				/* Send Error to steer */ 

				set_bit_uint8(&critical_errors[0],1,1);
				send_errors();
				init_step = 3;
			}
		}
	}else if(init_step == 2){
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
		/* Send periodical status inv L */
		can1.tx_id = ID_ASK_INV_SX;
		can1.dataTx[0] = 0x3D;
		can1.dataTx[1] = 0x40;
		can1.dataTx[2] = 0xFA; // each 250ms
		can1.tx_size = 3;
		CAN_Send(&can1, normalPriority);

		/* Send periodical status inv R */
		can1.tx_id = ID_ASK_INV_DX;
		can1.dataTx[0] = 0x3D;
		can1.dataTx[1] = 0x40;
		can1.dataTx[2] = 0xFA; // each 250ms
		can1.tx_size = 3;
		CAN_Send(&can1, normalPriority);

		init_step = 3;
	}else if(init_step == 3){
		current_state = STATE_IDLE; // Change state to STATE_IDLE
		/* Send msg to steer of changing state */
	}
}
/*******************************************************************
 *                         END INIT STATE
 *******************************************************************/
/*******************************************************************
 *                        START IDLE STATE
 *******************************************************************/
void idle()
{
	HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
	if (debug_msg_arrived == 1)
	{
		debug_msg_arrived = 0; // reset flag
		debug_operations();
	}
	if (fifoRxDataCAN_pop(&can1)) // Check if there are messages on CAN1 fifo
	{
		switch (can1.rx_id)
		{
		case ID_ASK_STATE:
			can1.dataTx[0] = (uint8_t)current_state;
			can1.tx_size = 1;
			can1.tx_id = ID_ACU_1;
			CAN_Send(&can1, normalPriority);
			break;
		case ID_ASK_INV_DX:
			break;
		case ID_ASK_INV_SX:
			break;
		case ID_BMS_HV:
			if (can1.dataRx[0] == 0x03)
			{
				// Turn ON tractive system
				if(check_error_presence() == 0){

				}
			}
			else if (can1.dataRx[0] == 0x04)
			{
				// Turn OFF tractive system
			}
			break;
		case ID_BMS_LV:
			break;
		case ID_STEERING_WEEL_1:
			switch (can1.dataRx[0])
			{
			case REQUEST_TS_ON:
				//If req Tractive System ON msg arrives -> go to setup state
				setup_init = 0;
				current_state = STATE_SETUP;
				break;
			default:
				break;
			}
			break;
		case ID_ATC_POT:
			atc_pot_operations();
			break;
		case ID_ATC_MAX_VAL:
			break;
		default:
			break;
		}
	}
	if(fifoRxDataCAN_pop(&can3)){
		switch (can3.rx_id)
		{
		case ID_imu_acceleration:
			imu_operations();
			accel_x_h = can3.dataRx[0];
			accel_x_l = can3.dataRx[1];
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
 *                        START SETUP STATE
 *******************************************************************/
void setup()
{
	if(setup_init == 0){
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
		setup_init = 1; //set that setup procedure is started
		if(check_error_presence() == 0){ //TODO: add brake check
			//if brake is pressed and there aren't critical erros -> 
				//-> send pre-charge request to HV
			can1.tx_id = ID_REQ_PRCH;
			can1.dataTx[0] = 1;
			can1.tx_size = 1;
			CAN_Send(&can1, normalPriority);
			init_precharge_start_time = count_ms_abs; //take the time when the pre-charge is sent
		}else{
			//Can't turn on TS caused by some errors
			current_state = STATE_IDLE; //return to idle state
			// TODO: report error to steer
			if(check_error_presence() != 0){
				send_errors();
			}
			//TODO: if brake isn't pressed -> send error
		}
	}else if(setup_init == 1){
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
		if(count_ms_abs - init_precharge_start_time > 5000){
			set_bit_uint8(&critical_errors[0],2,1); // set error
			send_errors(); // send errors
			set_bit_uint8(&critical_errors[0],2,0); // reset error
			current_state = STATE_IDLE; // return to STATE_IDLE
		}else if (fifoRxDataCAN_pop(&can1)){
			switch(can1.rx_id){
				case ID_BMS_HV:
					if(can1.dataRx[0] == 1){ //Pre-cherge ended sucessfully
						setup_init = 2;
					}else{ //Pre-charge failed
						set_bit_uint8(&critical_errors[0],3,1); // set error
						send_errors(); // send errors
						set_bit_uint8(&critical_errors[0],3,0); // reset error
						current_state = STATE_IDLE; // return to STATE_IDLE
					}
					break;
				default:
					break;
			}
		} else if(fifoRxDataCAN_pop(&can3)){
			switch (can3.rx_id)
			{
				case ID_imu_acceleration:
					imu_operations();
					accel_x_h = can3.dataRx[0];
					accel_x_l = can3.dataRx[1];
					break;
				
				default:
					break;
			}
		}
	}else if(setup_init == 2){
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
		//send command inverter enable
		can1.tx_id = ID_REQ_INV_DX;
		can1.dataTx[0] = 0x51;
		can1.dataTx[1] = 0x00;
		can1.dataTx[2] = 0x00;
		can1.tx_size = 3;
		CAN_Send(&can1, normalPriority);

		//send request inverter enable
		can1.tx_id = ID_ASK_INV_DX;
		can1.dataTx[0] = 0x3D;
		can1.dataTx[1] = 0xE8;
		can1.dataTx[2] = 0x00;
		can1.tx_size = 3;
		CAN_Send(&can1, normalPriority);

		setup_init = 3;

		init_inv_resp = count_ms_abs;
	}else if(setup_init == 3){
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
		if(count_ms_abs - init_inv_resp > 10000){
			// report error
			set_bit_uint8(&critical_errors[0],4,1);
			send_errors();
			set_bit_uint8(&critical_errors[0],4,0);
			//send pre-charge OFF req
			can1.tx_id = ID_REQ_PRCH;
			can1.dataTx[0] = 0x00;
			can1.tx_size = 1;
			CAN_Send(&can1, highPriority);
			current_state = STATE_IDLE;
		}else if (fifoRxDataCAN_pop(&can1)){
			switch(can1.rx_id){
				case ID_REQ_INV_DX:
					if(can1.dataRx[0] == 0xE0 && can1.dataRx[1] == 0x01 && can1.dataRx[2] == 0x00 && can1.dataRx[3] == 0x00){
						setup_init = 4;
					}
					break;
				default:
					break;
			}			
		} else if(fifoRxDataCAN_pop(&can3)){
			switch (can3.rx_id)
			{
				case ID_imu_acceleration:
					imu_operations();
					accel_x_h = can3.dataRx[0];
					accel_x_l = can3.dataRx[1];
					break;
				
				default:
					break;
			}
		}
	}else if(setup_init == 4){
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
		// In this state all is ready for run //
		// Waiting for run signal from steer //
		
	}
}
/*******************************************************************
 *                         END SETUP STATE
 *******************************************************************/
/*******************************************************************
 *                         START RUN STATE
 *******************************************************************/
void run()
{
	HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
	if(check_error_presence() != 0){
		send_errors();
		current_state = STATE_SETUP;
	}else{
		if (fifoRxDataCAN_pop(&can1))
		{
			switch (can1.rx_id)
			{
			case ID_STEERING_WEEL_1:
				if (can1.dataRx[0] == 6)
				{ //----- change state to setup -----//
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
		else if(fifoRxDataCAN_pop(&can3)){
			switch (can3.rx_id)
			{
				case ID_imu_acceleration:
					imu_operations();
					accel_x_h = can3.dataRx[0];
					accel_x_l = can3.dataRx[1];
					break;
				
				default:
					break;
			}
		}
	}
	
}
/*******************************************************************
 *                         END RUN STATE
 *******************************************************************/

void debug_operations()
{
	if (strcmp(debug_rx, "help") == 0)
	{
		sprintf(debug_tx,
				"\r\n***********ECU HELP***********\r\n"
				"Avaiable msg are:\r\n"
				"\t-- status -> print ECU status\r\n"
				"\t-- can sniffer -> enter in can sniffer mode (q for quit)\r\n"
				"\t-- send -> send a CAN msg (xxxx.xxx.xxx.xxx ... ecc)\r\n"
				"\t-- sd status -> print SD status and the name of file inside\r\n"
				"\t-- sd file -> print files inside the SD\r\n"
				"\t-- clean sd -> remove all log files on SD\r\n"
				"\t-- time -> print activity time\r\n"
				"\t-- codev  -> print code version\r\n"
				"\t-- best car? -> print the best car\r\n");
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}
	else if (strcmp(debug_rx, "codev") == 0)
	{
		HAL_UART_Transmit(&huart4, (uint8_t *)code_version, strlen(code_version),
						  100);
	}
	else if (strcmp(debug_rx, "time") == 0)
	{
		sprintf(debug_tx, "\r\nTime: %d hours : %d min : %d sec : %d dec\r\n",
				count_hour, count_min, count_sec, count_dec);
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}
	else if (strcmp(debug_rx, "status") == 0)
	{
		sprintf(debug_tx,
				"\r\n\ntype of status:\r\n"
				"\t 0 = OK\r\n"
				"\t 1 = ERROR\r\n"
				"\t 2 = BUSY\r\n"
				"\t 3 = TIMEOUT\r\n\n"
				"CAN1 status:\r\n"
				"\tCAN1 config status: %d \r\n"
				"\tCAN1 notification status: %d\r\n"
				"\tCAN1 start status: %d\r\n",
				can1.configFilter_status, can1.activateNotif_status,
				can1.canStart_status);
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 100);
		sprintf(debug_tx,
				"CAN3 status:\r\n"
				"\tCAN3 config status: %d \r\n"
				"\tCAN3 notification status: %d\r\n"
				"\tCAN3 start status: %d\r\n",
				can3.configFilter_status, can3.activateNotif_status,
				can3.canStart_status);
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 100);
		if (current_state == STATE_INIT)
		{
			HAL_UART_Transmit(&huart4,
							  (uint8_t *)"\r\nCurrent state: STATE_INIT\r\n",
							  strlen("\r\nCurrent state: STATE_INIT\r\n"), 100);
		}
		else if (current_state == STATE_IDLE)
		{
			HAL_UART_Transmit(&huart4,
							  (uint8_t *)"\r\nCurrent state: STATE_IDLE\r\n",
							  strlen("\r\nCurrent state: STATE_IDLE\r\n"), 100);
		}
		else if (current_state == STATE_SETUP)
		{
			HAL_UART_Transmit(
				&huart4, (uint8_t *)"\r\nCurrent state: STATE_SETUP\r\n",
				strlen("\r\nCurrent state: STATE_SETUP\r\n"), 100);
		}
		else if (current_state == STATE_RUN)
		{
			HAL_UART_Transmit(&huart4,
							  (uint8_t *)"\r\nCurrent state: STATE_RUN\r\n",
							  strlen("\r\nCurrent state: STATE_RUN\r\n"), 100);
		}
		sprintf(debug_tx,
				"\r\n"
				"Device connected : (0 = no, 1 = yes (for each bit))\r\n"
				"\t IMU -> %d\r\n"
				"\t ITS -> %d\r\n",
				imu_connected, its_connected);
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}else if(strcmp(debug_rx, "can sniffer") == 0){
		canSnifferMode = 1;
		sprintf(debug_tx,"\r\nEnter in can sniffer mode\r\n");
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}else if(strcmp(debug_rx, "q") == 0){
		canSnifferMode = 0;
		sprintf(debug_tx,"\r\nExit from can sniffer mode\r\n");
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}else if(strncmp(debug_rx, "send",4) == 0){
		uint8_t dataToSend[8];can1.dataTx[0] = dataToSend[0];
		uint32_t idToSend;
		uint8_t offset = (uint8_t)'0';
		idToSend = ((uint8_t)debug_rx[5]-offset)*1000 + ((uint8_t)debug_rx[6]-offset)*100 + ((uint8_t)debug_rx[7]-offset)*10 + ((uint8_t)debug_rx[8]-offset);
		dataToSend[0] = ((uint8_t)debug_rx[10]-offset)*100 + ((uint8_t)debug_rx[11]-offset)*10 + ((uint8_t)debug_rx[12]-offset);
		dataToSend[1] = ((uint8_t)debug_rx[14]-offset)*100 + ((uint8_t)debug_rx[15]-offset)*10 + ((uint8_t)debug_rx[16]-offset);
		dataToSend[2] = ((uint8_t)debug_rx[18]-offset)*100 + ((uint8_t)debug_rx[19]-offset)*10 + ((uint8_t)debug_rx[20]-offset);
		dataToSend[3] = ((uint8_t)debug_rx[22]-offset)*100 + ((uint8_t)debug_rx[23]-offset)*10 + ((uint8_t)debug_rx[24]-offset);
		dataToSend[4] = ((uint8_t)debug_rx[26]-offset)*100 + ((uint8_t)debug_rx[27]-offset)*10 + ((uint8_t)debug_rx[28]-offset);
		dataToSend[5] = ((uint8_t)debug_rx[30]-offset)*100 + ((uint8_t)debug_rx[31]-offset)*10 + ((uint8_t)debug_rx[32]-offset);
		dataToSend[6] = ((uint8_t)debug_rx[34]-offset)*100 + ((uint8_t)debug_rx[35]-offset)*10 + ((uint8_t)debug_rx[36]-offset);
		dataToSend[7] = ((uint8_t)debug_rx[38]-offset)*100 + ((uint8_t)debug_rx[39]-offset)*10 + ((uint8_t)debug_rx[40]-offset);
		can1.tx_id = idToSend;
		can1.dataTx[0] = dataToSend[0];
		can1.dataTx[1] = dataToSend[1];
		can1.dataTx[2] = dataToSend[2];
		can1.dataTx[3] = dataToSend[3];
		can1.dataTx[4] = dataToSend[4];
		can1.dataTx[5] = dataToSend[5];
		can1.dataTx[6] = dataToSend[6];
		can1.dataTx[7] = dataToSend[7];
		can1.tx_size = 8;
		CAN_Send(&can1, normalPriority);
		sprintf(debug_tx,"\r\nSent %ld %d %d %d %d %d %d %d %d\r\n", idToSend, dataToSend[0], dataToSend[1], dataToSend[2], dataToSend[3], dataToSend[4], dataToSend[5], dataToSend[6], dataToSend[7]);
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}else if (strcmp(debug_rx, "sd status") == 0){
		if(mount_ok == 1){
			HAL_UART_Transmit(&huart4, (uint8_t *)"\r\nSd mounted ", strlen("\r\nSd mounted "), 100);
			if(successfull_opening == 1){
				sprintf(debug_tx,"Sd successfully opened and created file: %s\r\n",filename_log);
				HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 100);
			}else{
				HAL_UART_Transmit(&huart4, (uint8_t *)"Sd open FAILED\r\n", strlen("Sd open FAILED\r\n"), 100);
			}
		}else{
			HAL_UART_Transmit(&huart4, (uint8_t *)"\r\nSd NOT mounted ", strlen("\r\nSd NOT mounted "), 100);
		}
	}
	else if(strcmp(debug_rx, "sd file") == 0){
		sprintf(debug_tx,"\r\nFiles inside sd are:\r\n");
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 1000);
		print_files_name();
	}
	else if (strcmp(debug_rx, "clean sd") == 0){
		sprintf(debug_tx,"\r\n\r\n");
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 1000);
		if(clean_sd() == true){
			sprintf(debug_tx,"\r\nClean done\r\n");
			HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 1000);
		}else{
			sprintf(debug_tx,"\r\nClean error\r\n");
			HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 1000);
		}
	}
	else if (strcmp(debug_rx, "best car?") == 0)
	{
		sprintf(debug_tx,
			"\r\n"
			"oooooooooooo oooooooooooo ooooo      ooo ooooo   .oooooo.   oooooooooooo\r\n" 
			"`888'     `8 `888'     `8 `888b.     `8' `888'  d8P'  `Y8b  `888'     `8\r\n"  
			" 888          888          8 `88b.    8   888  888           888\r\n"          
			" 888oooo8     888oooo8     8   `88b.  8   888  888           888oooo8\r\n"     
			" 888          888          8     `88b.8   888  888           888\r\n"     
			" 888          888       o  8       `888   888  `88b    ooo   888o\r\n"  
			"o888o        o888ooooood8 o8o        `8  o888o  `Y8bood8P'  o888ooooood8\r\n");
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}
	else
	{
		sprintf(debug_tx, "\r\nERROR : msg %s doesn't exist\r\n", debug_rx);
		HAL_UART_Transmit(&huart4, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}
}

void imu_operations()
{
	imu_connected = 1; // imu connected true
	if (count_imu == 10 || count_imu == 11)
	{
		HAL_UART_Transmit(&huart4, (uint8_t *)"IMU presente\r\n",
						  strlen("IMU presente\r\n"), 10);
	}
	count_imu = 0;
}
void atc_pot_operations()
{
	atc_connected = 1;
	set_bit_uint8(&critical_errors[0],0,0);
	count_atc = 0;
}