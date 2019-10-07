#include "can.h"
#include "stm32f7xx_hal_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan3;
canStruct can1,can3;
fifoCanDataType fifoCAN1, fifoCAN3;

void can_init(){
	if(CAN1_initialization(&can1)){
		report_error_can1();
	}
	if(CAN3_initialization(&can3)){
		report_error_can3();
	}

	fifoCAN1.rxHead = 0;
	fifoCAN1.rxTail = 0;
	fifoCAN1.txHeadNormal = 0;
	fifoCAN1.txHeadHigh = 0;
	fifoCAN1.txTailNormal = 0;
	fifoCAN1.txTailHigh = 0;

	fifoCAN3.rxHead = 0;
	fifoCAN3.rxTail = 0;
	fifoCAN3.txHeadNormal = 0;
	fifoCAN3.txHeadHigh = 0;
	fifoCAN3.txTailNormal = 0;
	fifoCAN3.txTailHigh = 0;
}

uint8_t CAN_Send(canStruct* can, uint32_t id, fifoPriority _fifoPriority){
	if (HAL_CAN_IsTxMessagePending(can->hcan, CAN_TX_MAILBOX0) == 0){
		if(CAN_Send_IT(can, id) == 0){
			//TODO: implementare errore
			return 0;
		}
	}else{
		fifoDataType fifodata;
		for(int i = 0; i < 8; i++){
			fifodata.data[i] = can->dataTx[i];
		}
		fifodata.id = id;
		if(_fifoPriority == normalPriority){
			if(can->hcan == &hcan1){
				if(fifoTxDataCAN1_normal_push(&fifoCAN1, &fifodata) == 0){
					//TODO: implementare errore
					return 0;
				}
			}else{
				if(fifoTxDataCAN3_normal_push(&fifoCAN3, &fifodata) == 0){
					//TODO: implementare errore
					return 0;
				}
			}
		}else{
			if(can->hcan == &hcan1){
				if(fifoTxDataCAN1_high_push(&fifoCAN1, &fifodata) == 0){
					//TODO: implementare errore
					return 0;
				}
			}else{
				if(fifoTxDataCAN3_high_push(&fifoCAN3, &fifodata) == 0){
					//TODO: implementare errore
					return 0;
				}
			}
		}

	}
	return 1;
}


uint8_t CAN_Send_IT(canStruct* can, uint32_t id){

	uint32_t mailbox = 0;
	//CAN_TxMailBox_TypeDef mailbox;
	//mailbox.TIR = 0; //set to mailbox 0
	uint8_t flag = 0; //error

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = can->size;
	TxHeader.TransmitGlobalTime = DISABLE;

	if(HAL_CAN_AddTxMessage(can->hcan, &TxHeader, can->dataTx,&mailbox) == HAL_OK){
		flag = 1; //ok
	}

	return flag;
}

uint8_t CAN1_initialization(canStruct *can){

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

	//CAN interrupt activation
	can->activateNotif_status = HAL_CAN_ActivateNotification(can->hcan, CAN1_RX0_IRQn);

	//CAN start
	can->canStart_status = HAL_CAN_Start(can->hcan);

	if(can->configFilter_status == HAL_OK && can->activateNotif_status == HAL_OK && can->canStart_status == HAL_OK) return 0; // no errors occurred
	else return 1;

}

uint8_t CAN3_initialization(canStruct *can){

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

	//CAN interrupt activation
	can->activateNotif_status = HAL_CAN_ActivateNotification(can->hcan, CAN3_RX0_IRQn);

	//CAN start
	can->canStart_status = HAL_CAN_Start(can->hcan);

	if(can->configFilter_status == HAL_OK && can->activateNotif_status == HAL_OK && can->canStart_status == HAL_OK) return 0; // no errors occurred
	else return 1;

}

void report_error_can1(){

}
void report_error_can3(){

}

uint8_t fifoRxDataCAN1_pop(fifoCanDataType* _fifoCAN,fifoDataType* _rxData){
	if(_fifoCAN->rxHead ==_fifoCAN->rxTail){
		return 0;
	}else{
		_rxData->id = _fifoCAN->rx[_fifoCAN->rxTail].id;
		for(uint8_t i = 0; i < 8; i++){
			_rxData->data[i] = _fifoCAN->rx[_fifoCAN->rxTail].data[i];
		}
		_fifoCAN->rxTail = (_fifoCAN->rxTail + 1) % fifoLengthN;
		return 1;
	}
}

uint8_t fifoRxDataCAN1_push(fifoCanDataType* _fifoCAN, fifoDataType* _rxData){
	if((_fifoCAN->rxHead + 1) % fifoLengthN == _fifoCAN->rxTail){
		return 0;
	}else{
		_fifoCAN->rx[_fifoCAN->rxHead].id = _rxData->id;
		for(uint8_t i = 0; i < 8; i++){
			_fifoCAN->rx[_fifoCAN->rxHead].data[i] = _rxData->data[i];
		}
		_fifoCAN->rxHead = (_fifoCAN->rxHead + 1) % fifoLengthN;
		return 1;
	}
}

uint8_t fifoRxDataCAN3_pop(fifoCanDataType* _fifoCAN,fifoDataType* _rxData){
	if(_fifoCAN->rxHead ==_fifoCAN->rxTail){
		return 0;
	}else{
		_rxData->id = _fifoCAN->rx[_fifoCAN->rxTail].id;
		for(uint8_t i = 0; i < 8; i++){
			_rxData->data[i] = _fifoCAN->rx[_fifoCAN->rxTail].data[i];
		}
		_fifoCAN->rxTail = (_fifoCAN->rxTail + 1) % fifoLengthN;
		return 1;
	}
}
uint8_t fifoRxDataCAN3_push(fifoCanDataType* _fifoCAN,fifoDataType* _rxData){
	if((_fifoCAN->rxHead + 1) % fifoLengthN == _fifoCAN->rxTail){
		return 0;
	}else{
		_fifoCAN->rx[_fifoCAN->rxHead].id = _rxData->id;
		for(uint8_t i = 0; i < 8; i++){
			_fifoCAN->rx[_fifoCAN->rxHead].data[i] = _rxData->data[i];
		}
		_fifoCAN->rxHead = (_fifoCAN->rxHead + 1) % fifoLengthN;
		return 1;
	}
}

uint8_t fifoTxDataCAN1_normal_pop(fifoCanDataType* _fifoCAN,fifoDataType* _txData){
	if(_fifoCAN->txHeadNormal ==_fifoCAN->txTailNormal){
		return 0;
	}else{
		_txData->id = _fifoCAN->txNormal[_fifoCAN->txTailNormal].id;
		for(uint8_t i = 0; i < 8; i++){
			_txData->data[i] = _fifoCAN->txNormal[_fifoCAN->txTailNormal].data[i];
		}
		_fifoCAN->txTailNormal = (_fifoCAN->txTailNormal + 1) % fifoLengthN;
		return 1;
	}
}
uint8_t fifoTxDataCAN1_high_pop(fifoCanDataType* _fifoCAN,fifoDataType* _txData){
	if(_fifoCAN->txHeadHigh ==_fifoCAN->txTailHigh){
		return 0;
	}else{
		_txData->id = _fifoCAN->txHigh[_fifoCAN->txTailHigh].id;
		for(uint8_t i = 0; i < 8; i++){
			_txData->data[i] = _fifoCAN->txHigh[_fifoCAN->txTailHigh].data[i];
		}
		_fifoCAN->txTailHigh = (_fifoCAN->txTailHigh + 1) % fifoLengthH;
		return 1;
	}
}
uint8_t fifoTxDataCAN1_normal_push(fifoCanDataType* _fifoCAN,fifoDataType* _txData){
	if((_fifoCAN->txHeadNormal + 1) % fifoLengthN == _fifoCAN->txTailNormal){
		return 0;
	}else{
		_fifoCAN->txNormal[_fifoCAN->txHeadNormal].id = _txData->id;
		for(uint8_t i = 0; i < 8; i++){
			_fifoCAN->txNormal[_fifoCAN->txHeadNormal].data[i] = _txData->data[i];
		}
		_fifoCAN->txHeadNormal = (_fifoCAN->txHeadNormal + 1) % fifoLengthN;
		return 1;
	}
}
uint8_t fifoTxDataCAN1_high_push(fifoCanDataType* _fifoCAN,fifoDataType* _txData){
	if((_fifoCAN->txHeadHigh + 1) % fifoLengthH == _fifoCAN->txTailHigh){
		return 0;
	}else{
		_fifoCAN->txHigh[_fifoCAN->txHeadHigh].id = _txData->id;
		for(uint8_t i = 0; i < 8; i++){
			_fifoCAN->txHigh[_fifoCAN->txHeadHigh].data[i] = _txData->data[i];
		}
		_fifoCAN->txHeadHigh = (_fifoCAN->txHeadHigh + 1) % fifoLengthH;
		return 1;
	}
}

uint8_t fifoTxDataCAN3_normal_pop(fifoCanDataType* _fifoCAN,fifoDataType* _txData){
	if(_fifoCAN->txHeadNormal ==_fifoCAN->txTailNormal){
		return 0;
	}else{
		_txData->id = _fifoCAN->txNormal[_fifoCAN->txTailNormal].id;
		for(uint8_t i = 0; i < 8; i++){
			_txData->data[i] = _fifoCAN->txNormal[_fifoCAN->txTailNormal].data[i];
		}
		_fifoCAN->txTailNormal = (_fifoCAN->txTailNormal + 1) % fifoLengthN;
		return 1;
	}
}
uint8_t fifoTxDataCAN3_high_pop(fifoCanDataType* _fifoCAN,fifoDataType* _txData){
	if(_fifoCAN->txHeadHigh ==_fifoCAN->txTailHigh){
		return 0;
	}else{
		_txData->id = _fifoCAN->txHigh[_fifoCAN->txTailHigh].id;
		for(uint8_t i = 0; i < 8; i++){
			_txData->data[i] = _fifoCAN->txHigh[_fifoCAN->txTailHigh].data[i];
		}
		_fifoCAN->txTailHigh = (_fifoCAN->txTailHigh + 1) % fifoLengthH;
		return 1;
	}
}
uint8_t fifoTxDataCAN3_normal_push(fifoCanDataType* _fifoCAN,fifoDataType* _txData){
	if((_fifoCAN->txHeadNormal + 1) % fifoLengthN == _fifoCAN->txTailNormal){
		return 0;
	}else{
		_fifoCAN->txNormal[_fifoCAN->txHeadNormal].id = _txData->id;
		for(uint8_t i = 0; i < 8; i++){
			_fifoCAN->txNormal[_fifoCAN->txHeadNormal].data[i] = _txData->data[i];
		}
		_fifoCAN->txHeadNormal = (_fifoCAN->txHeadNormal + 1) % fifoLengthN;
		return 1;
	}
}
uint8_t fifoTxDataCAN3_high_push(fifoCanDataType* _fifoCAN,fifoDataType* _txData){
	if((_fifoCAN->txHeadHigh + 1) % fifoLengthH == _fifoCAN->txTailHigh){
		return 0;
	}else{
		_fifoCAN->txHigh[_fifoCAN->txHeadHigh].id = _txData->id;
		for(uint8_t i = 0; i < 8; i++){
			_fifoCAN->txHigh[_fifoCAN->txHeadHigh].data[i] = _txData->data[i];
		}
		_fifoCAN->txHeadHigh = (_fifoCAN->txHeadHigh + 1) % fifoLengthH;
		return 1;
	}
}

