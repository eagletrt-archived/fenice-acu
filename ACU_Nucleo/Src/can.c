#include "can.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stm32f7xx_hal_can.h"

//extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart3;
canStruct can1,can3;
char txt[100];
void can_init(){
	if(CAN_initialization(&can1)){
		report_error_can1();
	}
	/*
	if(CAN_initialization(&can3)){
		report_error_can3();
	}*/
}

uint8_t CAN_Send(canStruct* can, fifoPriority _fifoPriority){
	if (HAL_CAN_GetTxMailboxesFreeLevel(can->hcan) != 0){
		if(CAN_Send_IT(can) == 0){
			//HAL_UART_Transmit(&huart3,(uint8_t*)("Cagato fuori dal vaso\r\n"), strlen("Cagato fuori dal vaso\r\n"), 10);
			return 0;
		}
		else{
			//sprintf(txt, "id: %d\r\n", (int)can->id);
			//HAL_UART_Transmit(&huart3,(uint8_t*)(txt), strlen(txt), 10);
			//HAL_UART_Transmit(&huart3,(uint8_t*)("sent\r\n"), strlen("sent\r\n"), 10);
		}
	}else{
		//HAL_UART_Transmit(&huart3,(uint8_t*)("Metto in coda\r\n"), strlen("Metto in coda\r\n"), 10);
		if(_fifoPriority == normalPriority){
			if(can->hcan == &hcan1){
				if(fifoTxDataCAN_normal_push(can) == 0){
					HAL_UART_Transmit(&huart3,(uint8_t*)("Error while normal push\r\n"), strlen("Error while normal push\r\n"), 10);
					return 0;
				}
			}
			/*else{
				if(fifoTxDataCAN3_normal_push(&fifoCAN3, &fifodata) == 0){
					//TODO: implementare errore
					return 0;
				}*/
			//}
		}else{
			if(can->hcan == &hcan1){
				if(fifoTxDataCAN_high_push(can) == 0){
					//TODO: implementare errore
					return 0;
				}
			}
			/*else{
				if(fifoTxDataCAN3_high_push(&fifoCAN3, &fifodata) == 0){
					//TODO: implementare errore
					return 0;
				}*/
			//}
		}

	}
	return 1;
}


uint8_t CAN_Send_IT(canStruct* can){

	uint32_t mailbox = 0;
	//CAN_TxMailBox_TypeDef mailbox;
	//mailbox.TIR = 0; //set to mailbox 0

	for(int i = 0; i < 8; i++){
		can->dataTxBck[i] = can->dataTx[i];
	}
	can->idBck = can->id;

	uint8_t flag = 0; //error

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = can->id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = can->size;
	TxHeader.TransmitGlobalTime = DISABLE;

	if(HAL_CAN_AddTxMessage(can->hcan, &TxHeader, can->dataTx, &mailbox) == HAL_OK){
		flag = 1; //ok
	}

	return flag;
}

uint8_t CAN_Send_Bck(canStruct* can){

	uint8_t flag = 0; //error

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = can->idBck;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = can->size;
	TxHeader.TransmitGlobalTime = DISABLE;

	if(HAL_CAN_AddTxMessage(can->hcan, &TxHeader, can->dataTxBck,(uint32_t*)CAN_TX_MAILBOX0) == HAL_OK){
		flag = 1; //ok
	}

	return flag;
}

uint8_t CAN_initialization(canStruct *can){



	//CAN filter initialization
	can->canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	can->canFilter.FilterIdLow = 0;
	can->canFilter.FilterIdHigh = 0;
	can->canFilter.FilterMaskIdHigh = 0;
	can->canFilter.FilterMaskIdLow = 0;
	can->canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can->canFilter.FilterBank = 0;
	can->canFilter.FilterScale  = CAN_FILTERSCALE_16BIT;
	can->canFilter.FilterActivation = ENABLE;

	//CAN filter configuration
	can->configFilter_status = HAL_CAN_ConfigFilter(can->hcan, &can->canFilter);


	can->activateNotif_status = HAL_CAN_ActivateNotification(can->hcan, can->rx0_interrupt);
	can->activateNotif_status = HAL_CAN_ActivateNotification(can->hcan, can->tx_interrupt);

	can->fifo.rxHead = 0;
	can->fifo.rxTail = 0;
	can->fifo.txHeadHigh = 0;
	can->fifo.txHeadNormal = 0;
	can->fifo.txTailHigh = 0;
	can->fifo.txTailNormal = 0;

	//CAN start
	can->canStart_status = HAL_CAN_Start(can->hcan);



	if(can->configFilter_status == HAL_OK && can->activateNotif_status == HAL_OK && can->canStart_status == HAL_OK) return 0; // no errors occurred
	else return 1;

}



void report_error_can1(){
	//HAL_GPIO_TogglePin(USER_LED_3_GPIO_Port, USER_LED_3_Pin);
}
void report_error_can3(){

}

uint8_t fifoRxDataCAN_pop(canStruct * can){
	if(can->fifo.rxHead ==can->fifo.rxTail){
		return 0;
	}else{
		can->id = can->fifo.rx[can->fifo.rxTail].id;
		for(uint8_t i = 0; i < 8; i++){
			can->dataRx[i] = can->fifo.rx[can->fifo.rxTail].data[i];
		}
		can->fifo.rxTail = (can->fifo.rxTail + 1) % fifoLengthN;
		return 1;
	}
}

uint8_t fifoRxDataCAN_push(canStruct * can){
	if((can->fifo.rxHead + 1) % fifoLengthN == can->fifo.rxTail){
		return 0;
	}else{
		can->fifo.rx[can->fifo.rxHead].id = can->id;
		for(uint8_t i = 0; i < 8; i++){
			can->fifo.rx[can->fifo.rxHead].data[i] = can->dataTx[i];
		}
		can->fifo.rxHead = (can->fifo.rxHead + 1) % fifoLengthN;
		return 1;
	}
}

uint8_t fifoTxDataCAN_normal_pop(canStruct * can){
	if(can->fifo.txHeadNormal ==can->fifo.txTailNormal){
		return 0;
	}else{
		can->id = can->fifo.txNormal[can->fifo.txTailNormal].id;
		for(uint8_t i = 0; i < 8; i++){
			can->dataTx[i] = can->fifo.txNormal[can->fifo.txTailNormal].data[i];
		}
		can->fifo.txTailNormal = (can->fifo.txTailNormal + 1) % fifoLengthN;
		return 1;
	}
}
uint8_t fifoTxDataCAN_high_pop(canStruct * can){
	if(can->fifo.txHeadHigh ==can->fifo.txTailHigh){
		return 0;
	}else{
		can->id = can->fifo.txHigh[can->fifo.txTailHigh].id;
		for(uint8_t i = 0; i < 8; i++){
			can->dataTx[i] = can->fifo.txHigh[can->fifo.txTailHigh].data[i];
		}
		can->fifo.txTailHigh = (can->fifo.txTailHigh + 1) % fifoLengthH;
		return 1;
	}
}
uint8_t fifoTxDataCAN_normal_push(canStruct *can){
	if((can->fifo.txHeadNormal + 1) % fifoLengthN == can->fifo.txTailNormal){
		return 0;
	}else{
		can->fifo.txNormal[can->fifo.txHeadNormal].id = can->id;
		for(uint8_t i = 0; i < 8; i++){
			can->fifo.txNormal[can->fifo.txHeadNormal].data[i] = can->dataTx[i];
		}
		can->fifo.txHeadNormal = (can->fifo.txHeadNormal + 1) % fifoLengthN;
		return 1;
	}
}
uint8_t fifoTxDataCAN_high_push(canStruct * can){
	if((can->fifo.txHeadHigh + 1) % fifoLengthH == can->fifo.txTailHigh){
		return 0;
	}else{
		can->fifo.txHigh[can->fifo.txHeadHigh].id = can->id;
		for(uint8_t i = 0; i < 8; i++){
			can->fifo.txHigh[can->fifo.txHeadHigh].data[i] = can->dataTx[i];
		}
		can->fifo.txHeadHigh = (can->fifo.txHeadHigh + 1) % fifoLengthH;
		return 1;
	}
}

