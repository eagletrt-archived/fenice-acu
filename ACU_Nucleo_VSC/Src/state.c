#include "state.h"
#include "can.h"
#include "global_variables.h"
#include "stdio.h"
#include "stm32f7xx_hal.h"
#include "string.h"

/*******************************************************************
 *                         STATE VARIABLES
 *******************************************************************/ 
int setup_init = 0;
int critical_errors = 0;
// Default state
// Init variables
/*******************************************************************
 *                         START INIT STATE
 *******************************************************************/
void init()
{
	if (debug_msg_arrived == 1)
	{
		debug_msg_arrived = 0; // reset flag
		debug_operations();
	}
	if (fifoRxDataCAN_pop(&can1))
	{
		switch (can1.rx_id)
		{
		case ID_imu_acceleration:
		case ID_imu_angular_rate:
			imu_operations();
			break;

		default:
			break;
		}
	}
	if (fifoRxDataCAN_pop(&can3))
	{
	}
	current_state = STATE_IDLE; // Change state to STATE_IDLE
}
/*******************************************************************
 *                         END INIT STATE
 *******************************************************************/
/*******************************************************************
 *                        START IDLE STATE
 *******************************************************************/
void idle()
{
	if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port,USER_BUTTON_Pin) == GPIO_PIN_SET){
		sprintf(txt,"%d\r\n%d\r\n%d\r\n%d\r\n",accel.pot1_val,accel.pot2_val,brake.pot1_val,brake.pot2_val);
		HAL_UART_Transmit(&huart3,(uint8_t*)txt,strlen(txt),10);
		res_open = f_open(&pot_values_f, (TCHAR const*)&filename_pot, FA_OPEN_ALWAYS | FA_WRITE );
		f_write(&pot_values_f,(TCHAR const*)&txt,strlen(txt), &byteswritten);
		f_close(&pot_values_f);
		HAL_Delay(1000);
	}
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
		if((atc_connected == 1) && (brake.pot_avr_100 > 50) && (critical_errors = 0)){
			//If Analog to CAN device is connected, brake is pressed and there aren't critical erros -> send request of TS ON to BMS_HV
			can1.dataTx[0] = 1;
			can1.tx_size = 1;
			can1.tx_id = ID_ACU_1;
			CAN_Send(&can1, normalPriority);
		}else{
			//Can't turn on TS caused by some errors
			current_state = STATE_IDLE; //return to idle state
			can1.dataTx[0] = 1;
			can1.tx_size = 1;
			can1.tx_id = ID_ACU_2;
			CAN_Send(&can1, highPriority);
		}
	}
	if (fifoRxDataCAN_pop(&can1))
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
				if (can1.dataRx[2] == 0x0C /* && request of shutdown == false*/)
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
				if (can1.dataRx[2] == 0x0C /* && request of shutdown == false*/)
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
	if (fifoRxDataCAN_pop(&can1))
	{
		switch (can1.rx_id)
		{
		case ID_STEERING_WEEL_1:
			if (can1.dataRx[0] ==
				6)
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
				"Device connected : (0 = no, 1 = yes)\r\n"
				"\t IMU -> %d\r\n"
				"\t ITS0 -> %d\r\n"
				"\t ITS1 -> %d\r\n"
				"\t ITS2 -> %d\r\n"
				"\t ITS3 -> %d\r\n",
				imu_connected, its0_connected, its1_connected, its2_connected,
				its3_connected);
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
	accel.pot1_val = can1.dataRx[0] * 256 + can1.dataRx[1];
	accel.pot2_val = can1.dataRx[2] * 256 + can1.dataRx[3];
	// brake.pot1_val = can1.dataRx[4] * 256 + can1.dataRx[5];
	// brake.pot2_val = can1.dataRx[6] * 256 + can1.dataRx[7];
	/*sprintf(txt, "CAN: %d %d %d %d\r\n ", can1.dataRx[0], can1.dataRx[1],
			can1.dataRx[2], can1.dataRx[3]);
	HAL_UART_Transmit(&huart3, (uint8_t*)txt, strlen(txt), 10);*/
	// val100 = (val - pot_min_val)/(pot_max_val - pot_min_val)*100

	if (accel.pot1_range != 0 && accel.pot2_range != 0)
	{
		accel.pot1_val_100 =
			((accel.pot1_val - accel.pot1_min_val) * 100) / (accel.pot1_range);
		accel.pot2_val_100 =
			((accel.pot2_val - accel.pot2_min_val) * 100) / (accel.pot2_range);
	}
	if (brake.pot1_range != 0 && brake.pot2_range != 0)
	{
		brake.pot1_val_100 =
			(brake.pot1_val - brake.pot1_min_val) / (brake.pot1_range) * 100;
		brake.pot2_val_100 =
			(brake.pot2_val - brake.pot2_min_val) / (brake.pot2_range) * 100;
	}
	/*sprintf(txt, "size: %d ", can1.rx_size);
	HAL_UART_Transmit(&huart3, (uint8_t*)txt, strlen(txt), 10);*/
	/*sprintf(txt, "%d %d\r\n", accel.pot1_val, accel.pot2_val);
	HAL_UART_Transmit(&huart3, (uint8_t*)txt, strlen(txt), 10);*/
	/*sprintf(txt, "%d %d\r\n", accel.pot1_val_100, accel.pot2_val_100);
	HAL_UART_Transmit(&huart3, (uint8_t*)txt, strlen(txt), 10);*/
	if (accel_implausibility_check() == 1)
	{
		accel.pot1_val = 0;
		accel.pot2_val = 0;
		accel.pot1_val_100 = 0;
		accel.pot2_val_100 = 0;
		// send error//
		can1.tx_id = 0x10;
		can1.dataTx[0] = 1;
		can1.tx_size = 1;
		// CAN_Send(&can1, highPriority);
		sprintf(txt, "POT FAIL %d\r\n", accel_implausibility_check_count_flag);
		HAL_UART_Transmit(&huart3, (uint8_t *)txt, strlen(txt), 10);
	}
	else if (accel_implausibility_check() == 0)
	{
		sprintf(txt, "POT WORK %d %d\r\n", accel.pot1_val_100,
				accel.pot2_val_100);
		HAL_UART_Transmit(&huart3, (uint8_t *)txt, strlen(txt), 10);
	}
	/*if(brake_implausibility_check()){
		brake.pot1_val = 0;
		brake.pot2_val = 0;
	}*/
}
