#ifndef _FENICE_H
#define _FENICE_H

#include "stm32f7xx_hal.h"

	void fenice_init();

	#ifdef HAL_CAN_MODULE_ENABLED

		typedef struct{

			int size; //size of data

			uint8_t dataTx[8];
			uint8_t dataRx[8];

			CAN_HandleTypeDef *hcan;
			CAN_FilterTypeDef canFilter;

			HAL_StatusTypeDef configFilter_status;
			HAL_StatusTypeDef activateNotif_status;
			HAL_StatusTypeDef canStart_status;

		}canStruct;

		uint8_t CAN1_initialization(canStruct *can);
		uint8_t CAN3_initialization(canStruct *can);
		void report_error_can1();
		void report_error_can3();
		uint8_t CAN_Send(canStruct* can, uint32_t id);

		typedef struct fifoRxDataType{
			uint16_t id;
			uint8_t data[8];
		}fifoRxDataType;
		typedef struct fifoTxDataType{
			uint16_t id;
			uint8_t data[8];
		}fifoTxDataType;

		uint8_t fifoRxDataCAN1_pop();
		uint8_t fifoRxDataCAN1_push();

		uint8_t fifoRxDataCAN3_pop();
		uint8_t fifoRxDataCAN3_push();

		uint8_t fifoTxDataCAN1_normal_pop();
		uint8_t fifoTxDataCAN1_high_pop();
		uint8_t fifoTxDataCAN1_normal_push();
		uint8_t fifoTxDataCAN1_high_push();

		uint8_t fifoTxDataCAN3_normal_pop();
		uint8_t fifoTxDataCAN3_high_pop();
		uint8_t fifoTxDataCAN3_normal_push();
		uint8_t fifoTxDataCAN3_high_push();


	#endif

#endif
