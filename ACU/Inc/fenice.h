#ifndef _FENICE_H
#define _FENICE_H

#include "stm32f7xx_hal.h"

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

	#endif

#endif
