#include "fenice.h"



#ifdef HAL_CAN_MODULE_ENABLED
#include "stm32f7xx_hal_can.h"

	uint8_t CAN_Send(canStruct* can, uint32_t id){

		uint32_t mailbox;
		uint8_t flag = 0; //error

		CAN_TxHeaderTypeDef TxHeader;
		TxHeader.StdId = id;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = can->size;
		TxHeader.TransmitGlobalTime = DISABLE;

		if (HAL_CAN_GetTxMailboxesFreeLevel(can->hcan) != 0 && HAL_CAN_IsTxMessagePending(can->hcan, CAN_TX_MAILBOX0) == 0){
			if(HAL_CAN_AddTxMessage(can->hcan, &TxHeader, can->dataTx, &mailbox) == HAL_OK){
				flag = 1; //ok
			}
		}

		return flag;
	}

	uint8_t CAN1_initialization(canStruct *can){

		// USB_HP_CAN_TX_IRQn interrupt configuration //
		HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
		// USB_LP_CAN_RX0_IRQn interrupt configuration //
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

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

		// USB_HP_CAN_TX_IRQn interrupt configuration //
		HAL_NVIC_SetPriority(CAN3_TX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);
		// USB_LP_CAN_RX0_IRQn interrupt configuration //
		HAL_NVIC_SetPriority(CAN3_RX0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);

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


#endif
