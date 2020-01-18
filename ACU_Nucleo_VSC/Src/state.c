#include "state.h"
#include "can.h"
#include "global_variables.h"
#include "stdio.h"
#include "stm32f7xx_hal.h"
#include "string.h"
/*******************************************************************
 *                         USER FUNCTIONS
 *******************************************************************/ 
void set_bit_uint8(uint8_t* _var, uint8_t _nBit, uint8_t _bool){
	if(_bool == 0 || _bool == 1){
		(*_var) = ((*_var) & ( 0b11111111 ^ (0b00000001 << _nBit))) | (0b00000000 | (_bool << _nBit));
	}
}
/*******************************************************************
 *                         STATE VARIABLES
 *******************************************************************/ 
/*** GLOBAL ***/
int setup_init = 0;
int critical_errors = 0;

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
			default:
				break;
			}
		}
		if(inv_init_response == 3){ // means that each inv has responded
			init_step = 2;
		}else{
			if(count_ms_abs - init_step_start_time > 1000 ){ //if is passed more than 1 second -> go ahead
				/* Send Error to steer */ 

				//TODO: send errors

				init_step = 3;
			}
		}
	}else if(init_step == 2){
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
	/*if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port,USER_BUTTON_Pin) == GPIO_PIN_SET){
		sprintf(txt,"%d\r\n%d\r\n%d\r\n%d\r\n",accel.pot1_val,accel.pot2_val,brake.pot1_val,brake.pot2_val);
		HAL_UART_Transmit(&huart3,(uint8_t*)txt,strlen(txt),10);
		res_open = f_open(&pot_values_f, (TCHAR const*)&filename_pot, FA_OPEN_ALWAYS | FA_WRITE );
		f_write(&pot_values_f,(TCHAR const*)&txt,strlen(txt), &byteswritten);
		f_close(&pot_values_f);
		HAL_Delay(1000);
	}*/
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
				if(critical_errors == 0){

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
				current_state = STATE_SETUP;
				break;
			default:
				break;
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
 *                        START SETUP STATE
 *******************************************************************/
void setup()
{
	if(setup_init == 0){
		setup_init = 1; //set that setup procedure is started
		if((atc_connected == 1) && (critical_errors = 0)){
			//If Analog to CAN device is connected, brake is pressed and there aren't critical erros -> 
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
		}
	}else if(setup_init == 1){
		if(count_ms_abs - init_precharge_start_time > 5000){
			// report error
			current_state = STATE_IDLE;
		}else if (fifoRxDataCAN_pop(&can1)){
			switch(can1.rx_id){
				case ID_BMS_HV:
					if(can1.dataRx[0] == 1){ //Pre-cherge ended sucessfully
						setup_init = 2;
					}else{ //Pre-charge failed
						current_state = STATE_IDLE;
						//TODO: send error to steer
					}
			}
		}
	}else if(setup_init == 2){
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
		if(count_ms_abs - init_inv_resp > 10000){
			// report error
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
			}			
		}
	}else if(setup_init == 4){
		// In this state all is ready for run //
		// Waiting for run signal from steer //
		
	}
	/*if (fifoRxDataCAN_pop(&can1))
	{
		switch (can1.rx_id)
		{
		case ID_STEERING_WEEL_1:
			switch (can1.dataRx[0])
			{
			//----- change state to run -----//
			case 5:
				// If inverter are ON and Brake is Pressed
				current_state = STATE_RUN;
				can1.tx_id = ID_ACU_2;
				can1.dataRx[0] = 5;
				can1.tx_size = 1;
				CAN_Send(&can1, normalPriority);
				break;
			// Turn On inverter Dx
			case 8:
				// If Inverter Temp < 80
				break;
			// Turn On inverter Sx
			case 9:
				// if Iverter Temp < 80
				break;
			default:
				break;
			}
			break;
		case ID_BMS_HV:
			switch (can1.dataRx[0])
			{
			case 4:
				// Shutdown Confirmed
				break;
			case 8:
				// Shutdown from Error
				break;
			}
			break;
		case ID_ATC_POT:
			atc_pot_operations();
			break;
		case ID_REQ_INV_DX:
			switch (can1.dataRx[0])
			{
			case 0x4A:
				// Update Inverter Dx Temp = (can1.RxData[2] * 256 + can1.RxData[1] - 15797) / 112.1182
				break;
			case 0xD8:
				if (can1.dataRx[2] == 0x0C  && request of shutdown == false)
				{
					can1.dataTx[0] = 0x09;
					can1.dataTx[1] = 0;
					can1.dataTx[2] = 0;
					can1.dataTx[3] = 0;
					can1.dataTx[4] = 0;
					can1.dataTx[5] = 0;
					can1.dataTx[6] = 0;
					can1.dataTx[7] = 0;
					can1.tx_id = ID_ACU_1;
					CAN_Send(&can1, normalPriority);
					// Inverter Dx true
				}
				else
				{
					can1.dataTx[0] = 0xD0;
					can1.dataTx[1] = 0;
					can1.dataTx[2] = 0;
					can1.dataTx[3] = 0;
					can1.dataTx[4] = 0;
					can1.dataTx[5] = 0;
					can1.dataTx[6] = 0;
					can1.dataTx[7] = 0;
					can1.tx_id = ID_ACU_1;
					CAN_Send(&can1, normalPriority);
					// Inverter Dx false
				}
				break;
			}
			break;
		case ID_REQ_INV_SX:
			switch (can1.dataRx[0])
			{
			case 0x4A:
				// Update Inverter Sx Temp = (can1.RxData[2] * 256 + can1.RxData[1] - 15797) / 112.1182
				break;
			case 0xD8:
				if (can1.dataRx[2] == 0x0C  && request of shutdown == false)
				{
					can1.dataTx[0] = 0x08;
					can1.dataTx[1] = 0;
					can1.dataTx[2] = 0;
					can1.dataTx[3] = 0;
					can1.dataTx[4] = 0;
					can1.dataTx[5] = 0;
					can1.dataTx[6] = 0;
					can1.dataTx[7] = 0;
					can1.tx_id = ID_ACU_1;
					CAN_Send(&can1, normalPriority);
					// Inverter Sx true
				}
				else
				{
					can1.dataTx[0] = 0x0C;
					can1.dataTx[1] = 0;
					can1.dataTx[2] = 0;
					can1.dataTx[3] = 0;
					can1.dataTx[4] = 0;
					can1.dataTx[5] = 0;
					can1.dataTx[6] = 0;
					can1.dataTx[7] = 0;
					can1.tx_id = ID_ACU_1;
					CAN_Send(&can1, normalPriority);
					// Inverter Sx false
				}
				break;
			}
			break;
		default:
			break;
		}
	}*/
}
/*******************************************************************
 *                         END SETUP STATE
 *******************************************************************/
/*******************************************************************
 *                         START RUN STATE
 *******************************************************************/
void run()
{
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
				"\t-- sd status -> print SD status and the name of file inside\r\n"
				"\t-- sd file -> print files inside the SD\r\n"
				"\t-- time -> print activity time\r\n"
				"\t-- codev  -> print code version\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}
	else if (strcmp(debug_rx, "codev") == 0)
	{
		HAL_UART_Transmit(&huart3, (uint8_t *)code_version, strlen(code_version),
						  100);
	}
	else if (strcmp(debug_rx, "time") == 0)
	{
		sprintf(debug_tx, "\r\nTime: %d hours : %d min : %d sec : %d dec\r\n",
				count_hour, count_min, count_sec, count_dec);
		HAL_UART_Transmit(&huart3, (uint8_t *)debug_tx, strlen(debug_tx), 100);
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
		HAL_UART_Transmit(&huart3, (uint8_t *)debug_tx, strlen(debug_tx), 100);
		sprintf(debug_tx,
				"CAN3 status:\r\n"
				"\tCAN3 config status: %d \r\n"
				"\tCAN3 notification status: %d\r\n"
				"\tCAN3 start status: %d\r\n",
				can3.configFilter_status, can3.activateNotif_status,
				can3.canStart_status);
		HAL_UART_Transmit(&huart3, (uint8_t *)debug_tx, strlen(debug_tx), 100);
		if (current_state == STATE_INIT)
		{
			HAL_UART_Transmit(&huart3,
							  (uint8_t *)"\r\nCurrent state: STATE_INIT\r\n",
							  strlen("\r\nCurrent state: STATE_INIT\r\n"), 100);
		}
		else if (current_state == STATE_IDLE)
		{
			HAL_UART_Transmit(&huart3,
							  (uint8_t *)"\r\nCurrent state: STATE_IDLE\r\n",
							  strlen("\r\nCurrent state: STATE_IDLE\r\n"), 100);
		}
		else if (current_state == STATE_SETUP)
		{
			HAL_UART_Transmit(
				&huart3, (uint8_t *)"\r\nCurrent state: STATE_SETUP\r\n",
				strlen("\r\nCurrent state: STATE_SETUP\r\n"), 100);
		}
		else if (current_state == STATE_RUN)
		{
			HAL_UART_Transmit(&huart3,
							  (uint8_t *)"\r\nCurrent state: STATE_RUN\r\n",
							  strlen("\r\nCurrent state: STATE_RUN\r\n"), 100);
		}
		sprintf(debug_tx,
				"\r\n"
				"Device connected : (0 = no, 1 = yes (for each bit))\r\n"
				"\t IMU -> %d\r\n"
				"\t ITS -> %d\r\n",
				imu_connected, its_connected);
		HAL_UART_Transmit(&huart3, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}
	else if (strcmp(debug_rx, "sd status") == 0){
		if(mount_ok == 1){
			HAL_UART_Transmit(&huart3, (uint8_t *)"\r\nSd mounted ", strlen("\r\nSd mounted "), 100);
			if(successfull_opening == 1){
				sprintf(debug_tx,"Sd successfully opened and created file: %s\r\n",filename_log);
				HAL_UART_Transmit(&huart3, (uint8_t *)debug_tx, strlen(debug_tx), 100);
			}else{
				HAL_UART_Transmit(&huart3, (uint8_t *)"Sd open FAILED\r\n", strlen("Sd open FAILED\r\n"), 100);
			}
		}else{
			HAL_UART_Transmit(&huart3, (uint8_t *)"\r\nSd NOT mounted ", strlen("\r\nSd NOT mounted "), 100);
		}
	}
	else if(strcmp(debug_rx, "sd file") == 0){
		sprintf(debug_tx,"\r\nFiles inside sd are:\r\n%s",log_names);
		HAL_UART_Transmit(&huart3, (uint8_t *)debug_tx, strlen(debug_tx), 1000);
	}
	else if (strcmp(debug_rx, "gay") == 0)
	{
		sprintf(debug_tx,
				"\r\n"
				"          $\r\n"
				"        $   $\r\n"
				"       $     $\r\n"
				"       $$$$$$$\r\n"
				"       $$$$$$$\r\n"
				"       $$$$$$$\r\n"
				"       $$$$$$$\r\n"
				"  $$$$$$     $$$$$$\r\n"
				" $$$$$$$$   $$$$$$$$\r\n"
				"$$$$$$$$$$$$$$$$$$$$\r\n"
				" $$$$$$$$   $$$$$$$$\r\n"
				"  $$$$$$     $$$$$$\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}
	else
	{
		sprintf(debug_tx, "\r\nERROR : msg %s doesn't exist\r\n", debug_rx);
		HAL_UART_Transmit(&huart3, (uint8_t *)debug_tx, strlen(debug_tx), 100);
	}
}

void imu_operations()
{
	imu_connected = 1; // imu connected true
	if (count_imu == 10 || count_imu == 11)
	{
		HAL_UART_Transmit(&huart3, (uint8_t *)"IMU presente\r\n",
						  strlen("IMU presente\r\n"), 10);
	}
	count_imu = 0;
}
void atc_pot_operations()
{
	atc_connected = 1;
	count_atc = 0;
}
