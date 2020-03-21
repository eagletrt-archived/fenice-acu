#include "can.h"

// extern CAN_HandleTypeDef hcan1;

extern UART_HandleTypeDef huart4;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan3;
canStruct can1, can3;
void can_init()
{
	if (CAN_initialization(&can1) == false){
		report_error_can1();
	}
	
	if(CAN_initialization(&can3) == false){
		report_error_can3();
	}
}

bool CAN_Send(canStruct *can, fifoPriority _fifoPriority)
{
	if (HAL_CAN_GetTxMailboxesFreeLevel(can->hcan) != 0)
	{
		if (CAN_Send_IT(can) == false)
		{
			return false; // error
		}
	}
	else
	{
		if (_fifoPriority == normalPriority)
		{
			if (can->hcan == &hcan1)
			{
				if (fifoTxDataCAN_normal_push(can) == false)
				{
					HAL_UART_Transmit(
						&huart4, (uint8_t *)("Error while normal push\r\n"),
						strlen("Error while normal push\r\n"), 10);
					return false;
				}
			}
		}
		else
		{ 
			if (can->hcan == &hcan1)
			{
				if (fifoTxDataCAN_high_push(can) == false)
				{
					// TODO: implementare errore
					return false;
				}
			}
		}
	}
	return true;
}

bool CAN_Send_IT(canStruct *can)
{
	uint32_t mailbox = 0;
	// CAN_TxMailBox_TypeDef mailbox;
	// mailbox.TIR = 0; //set to mailbox 0

	for (int i = 0; i < 8; i++)
	{
		can->dataTxBck[i] = can->dataTx[i];
	}
	can->idBck = can->tx_id;
	can->sizeBck = can->tx_size;

	bool flag = false; // error

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = can->tx_id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = can->tx_size;
	TxHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(can->hcan, &TxHeader, can->dataTx, &mailbox) ==
		HAL_OK)
	{
		flag = true; // ok
	}

	return flag;
}

bool CAN_Send_Bck(canStruct *can)
{
	bool flag = false; // error

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = can->idBck;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = can->sizeBck;
	TxHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(can->hcan, &TxHeader, can->dataTxBck,
							 (uint32_t *)CAN_TX_MAILBOX0) == HAL_OK)
	{
		flag = true; // ok
	}

	return flag;
}

bool CAN_initialization(canStruct *can)
{
	if(can->hcan == &hcan1){
		// CAN filter initialization
		can->canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
		can->canFilter.FilterIdLow = 0;
		can->canFilter.FilterIdHigh = 0;
		can->canFilter.FilterMaskIdHigh = 0;
		can->canFilter.FilterMaskIdLow = 0;
		can->canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		can->canFilter.FilterBank = 0;
		can->canFilter.FilterScale = CAN_FILTERSCALE_16BIT;
		can->canFilter.FilterActivation = ENABLE;
	}else if(can->hcan == &hcan3){
		// CAN filter initialization
		can->canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
		can->canFilter.FilterIdLow = ID_imu_acceleration;
		can->canFilter.FilterIdHigh = ID_imu_acceleration;
		can->canFilter.FilterMaskIdHigh = 0;
		can->canFilter.FilterMaskIdLow = 0;
		can->canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		can->canFilter.FilterBank = 0;
		can->canFilter.FilterScale = CAN_FILTERSCALE_16BIT;
		can->canFilter.FilterActivation = ENABLE;
	}

	// CAN filter configuration
	can->configFilter_status = HAL_CAN_ConfigFilter(can->hcan, &can->canFilter);

	can->activateNotif_status =
		HAL_CAN_ActivateNotification(can->hcan, can->rx0_interrupt);
	can->activateNotif_status =
		HAL_CAN_ActivateNotification(can->hcan, can->tx_interrupt);

	can->fifo.rxHead = 0;
	can->fifo.rxTail = 0;
	can->fifo.txHeadHigh = 0;
	can->fifo.txHeadNormal = 0;
	can->fifo.txTailHigh = 0;
	can->fifo.txTailNormal = 0;

	// CAN start
	can->canStart_status = HAL_CAN_Start(can->hcan);

	if (can->configFilter_status == HAL_OK &&
		can->activateNotif_status == HAL_OK && can->canStart_status == HAL_OK)
		return true; // no errors occurred
	else
		return false;
}

void report_error_can1()
{
	// HAL_GPIO_TogglePin(USER_LED_3_GPIO_Port, USER_LED_3_Pin);
}
void report_error_can3() {}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	if (hcan == &hcan1) {
		if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
			CAN_RxHeaderTypeDef header;
			HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, can1.dataRX_int);
			can1.rx_id_int = header.StdId;
			can1.rx_size_int = header.DLC;
			if(canSnifferMode == 1){
				sprintf(txt,"%ld %d %d %d %d %d %d %d %d\r\n", can1.rx_id_int, can1.dataRX_int[0], can1.dataRX_int[1], can1.dataRX_int[2], can1.dataRX_int[3], can1.dataRX_int[4], can1.dataRX_int[5], can1.dataRX_int[6], can1.dataRX_int[7] );
				HAL_UART_Transmit(&huart4, (uint8_t*)txt, strlen(txt), 50);
			}else{
					fifoRxDataCAN_push(&can1);
			}
		}
	}else if (hcan == &hcan3){
		if (HAL_CAN_GetRxFifoFillLevel(&hcan3, CAN_RX_FIFO0) != 0) {
			CAN_RxHeaderTypeDef header;
			HAL_CAN_GetRxMessage(&hcan3, CAN_RX_FIFO0, &header, can3.dataRX_int);
			can3.rx_id_int = header.StdId;
			can3.rx_size_int = header.DLC;
			fifoRxDataCAN_push(&can3);
		}	
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan == &hcan1) {
		HAL_UART_Transmit(&huart4, (uint8_t *)"rx on FIFO1\r\n", strlen("rx on FIFO1\r\n"), 10);
	}
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
	if (hcan == &hcan1) {
		HAL_UART_Transmit(&huart4, (uint8_t *)"CAN1 FIFO0 FULL\r\n", strlen("CAN1 FIFO0 FULL\r\n"), 10);
	}else if(hcan == &hcan3){
		HAL_UART_Transmit(&huart4, (uint8_t *)"CAN1 FIFO0 FULL\r\n", strlen("CAN3 FIFO0 FULL\r\n"), 10);
	}
}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {
	if (hcan == &hcan1) {
		HAL_UART_Transmit(&huart4, (uint8_t *)"CAN1 FIFO1 FULL\r\n", strlen("CAN1 FIFO1 FULL\r\n"), 10);
	}else if(hcan == &hcan3){
		HAL_UART_Transmit(&huart4, (uint8_t *)"CAN3 FIFO1 FULL\r\n", strlen("CAN3 FIFO1 FULL\r\n"), 10);
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	if (hcan == &hcan1) {
		if (fifoTxDataCAN_high_pop(&can1)) {
			if (CAN_Send_IT(&can1) == false) {
				// TODO: implementare errore
			} else {
				HAL_UART_Transmit(&huart4, (uint8_t *)("high\r\n"), strlen("high\r\n"), 10);
			}
		} else if (fifoTxDataCAN_normal_pop(&can1)) {
			if (CAN_Send_IT(&can1) == false) {
				// TODO: implementare errore
			}
		}
	}
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	sprintf(txt, "mb1: %d %d\r\n", can1.fifo.txTailNormal, can1.fifo.txHeadNormal);
	HAL_UART_Transmit(&huart4, (uint8_t *)(txt), strlen(txt), 10);
  	// HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	if (hcan == &hcan1) {
		if (fifoTxDataCAN_high_pop(&can1)) {
			if (CAN_Send_IT(&can1) == false) {
				// TODO: implementare errore
			} else {
				HAL_UART_Transmit(&huart4, (uint8_t *)("high\r\n"), strlen("high\r\n"), 10);
			}
		} else if (fifoTxDataCAN_normal_pop(&can1)) {
			if (CAN_Send_IT(&can1) == false) {
				// TODO: implementare errore
			}
		}
	}
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	sprintf(txt, "mb2: %d %d\r\n", can1.fifo.txTailNormal, can1.fifo.txHeadNormal);
	HAL_UART_Transmit(&huart4, (uint8_t *)(txt), strlen(txt), 10);
	// HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	if (hcan == &hcan1) {
		if (fifoTxDataCAN_high_pop(&can1)) {
			if (CAN_Send_IT(&can1) == 0) {
				// TODO: implementare errore
			} else {
				HAL_UART_Transmit(&huart4, (uint8_t *)("high\r\n"), strlen("high\r\n"), 10);
			}
		} else if (fifoTxDataCAN_normal_pop(&can1)) {
			if (CAN_Send_IT(&can1) == 0) {
				// TODO: implementare errore
			}
		}
	}
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	sprintf(txt, "--- Errore ---: %d\r\n", (int)hcan->ErrorCode);
	HAL_UART_Transmit(&huart4, (uint8_t *)(txt), strlen(txt), 10);
	if (hcan == &hcan1) {
		CAN_Send_Bck(&can1);
	}
}

bool fifoRxDataCAN_pop(canStruct *can)
{
	if (can->fifo.rxHead == can->fifo.rxTail)
	{
		return false;
	}
	else
	{
		can->rx_id = can->fifo.rx[can->fifo.rxTail].id;
		can->rx_size = can->fifo.rx[can->fifo.rxTail].size;
		for (uint8_t i = 0; i < can->rx_size; i++)
		{
			can->dataRx[i] = can->fifo.rx[can->fifo.rxTail].data[i];
		}
		can->fifo.rxTail = (can->fifo.rxTail + 1) % fifoLengthN;
		return true;
	}
}

bool fifoRxDataCAN_push(canStruct *can)
{
	if ((can->fifo.rxHead + 1) % fifoLengthN == can->fifo.rxTail)
	{
		return false;
	}
	else
	{
		can->fifo.rx[can->fifo.rxHead].id = can->rx_id_int;
		can->fifo.rx[can->fifo.rxHead].size = can->rx_size_int;
		for (uint8_t i = 0; i < can->rx_size_int; i++)
		{
			can->fifo.rx[can->fifo.rxHead].data[i] = can->dataRX_int[i];
		}
		can->fifo.rxHead = (can->fifo.rxHead + 1) % fifoLengthN;
		return true;
	}
}

bool fifoTxDataCAN_normal_pop(canStruct *can)
{
	if (can->fifo.txHeadNormal == can->fifo.txTailNormal)
	{
		return false;
	}
	else
	{
		can->tx_id = can->fifo.txNormal[can->fifo.txTailNormal].id;
		can->tx_size = can->fifo.txNormal[can->fifo.txTailNormal].size;
		for (uint8_t i = 0; i < can->tx_size; i++)
		{
			can->dataTx[i] = can->fifo.txNormal[can->fifo.txTailNormal].data[i];
		}
		can->fifo.txTailNormal = (can->fifo.txTailNormal + 1) % fifoLengthN;
		return true;
	}
}
bool fifoTxDataCAN_high_pop(canStruct *can)
{
	if (can->fifo.txHeadHigh == can->fifo.txTailHigh)
	{
		return false;
	}
	else
	{
		can->tx_id = can->fifo.txHigh[can->fifo.txTailHigh].id;
		can->tx_size = can->fifo.txHigh[can->fifo.txTailHigh].size;
		for (uint8_t i = 0; i < can->tx_size; i++)
		{
			can->dataTx[i] = can->fifo.txHigh[can->fifo.txTailHigh].data[i];
		}
		can->fifo.txTailHigh = (can->fifo.txTailHigh + 1) % fifoLengthH;
		return true;
	}
}
bool fifoTxDataCAN_normal_push(canStruct *can)
{
	if ((can->fifo.txHeadNormal + 1) % fifoLengthN == can->fifo.txTailNormal)
	{
		return false;
	}
	else
	{
		can->fifo.txNormal[can->fifo.txHeadNormal].id = can->tx_id;
		can->fifo.txNormal[can->fifo.txHeadNormal].size = can->tx_size;
		for (uint8_t i = 0; i < can->tx_size; i++)
		{
			can->fifo.txNormal[can->fifo.txHeadNormal].data[i] = can->dataTx[i];
		}
		can->fifo.txHeadNormal = (can->fifo.txHeadNormal + 1) % fifoLengthN;
		return true;
	}
}
bool fifoTxDataCAN_high_push(canStruct *can)
{
	if ((can->fifo.txHeadHigh + 1) % fifoLengthH == can->fifo.txTailHigh)
	{
		return false;
	}
	else
	{
		can->fifo.txHigh[can->fifo.txHeadHigh].id = can->tx_id;
		can->fifo.txHigh[can->fifo.txHeadHigh].size = can->tx_size;
		for (uint8_t i = 0; i < can->tx_size; i++)
		{
			can->fifo.txHigh[can->fifo.txHeadHigh].data[i] = can->dataTx[i];
		}
		can->fifo.txHeadHigh = (can->fifo.txHeadHigh + 1) % fifoLengthH;
		return true;
	}
}