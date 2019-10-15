#ifndef _FENICE_H
#define _FENICE_H


#define fifoLengthN 100
#define fifoLengthH 10


#include "stm32f7xx_hal.h"

	void fenice_init();

	#ifdef HAL_CAN_MODULE_ENABLED

		typedef enum fifoPriority_t{normalPriority, highPriority}fifoPriority;

		typedef struct{

			int size; //size of data

			uint8_t dataTx[8];
			uint8_t dataRx[8];
			uint8_t dataTxBck[8];

			uint32_t idBck;

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
		uint8_t CAN_Send(canStruct*, uint32_t, fifoPriority);
		uint8_t CAN_Send_IT(canStruct*, uint32_t);
		uint8_t CAN_Send_Bck(canStruct*);

		typedef struct fifoDataType{
			uint32_t id;
			uint8_t data[8];
		}fifoDataType;


		typedef struct fifoCanDataType{
			uint8_t rxHead;
			uint8_t rxTail;

			uint8_t txHeadNormal;
			uint8_t txTailNormal;

			uint8_t txHeadHigh;
			uint8_t txTailHigh;

			fifoDataType rx[fifoLengthN];
			fifoDataType txNormal[fifoLengthN];
			fifoDataType txHigh[fifoLengthH];

		}fifoCanDataType;



		uint8_t fifoRxDataCAN1_pop(fifoCanDataType*, fifoDataType*);
		uint8_t fifoRxDataCAN1_push(fifoCanDataType*, fifoDataType*);

		uint8_t fifoRxDataCAN3_pop(fifoCanDataType*, fifoDataType*);
		uint8_t fifoRxDataCAN3_push(fifoCanDataType*, fifoDataType*);

		uint8_t fifoTxDataCAN1_normal_pop(fifoCanDataType*, fifoDataType*);
		uint8_t fifoTxDataCAN1_high_pop(fifoCanDataType*, fifoDataType*);
		uint8_t fifoTxDataCAN1_normal_push(fifoCanDataType*, fifoDataType*);
		uint8_t fifoTxDataCAN1_high_push(fifoCanDataType*, fifoDataType*);

		uint8_t fifoTxDataCAN3_normal_pop(fifoCanDataType*, fifoDataType*);
		uint8_t fifoTxDataCAN3_high_pop(fifoCanDataType*, fifoDataType*);
		uint8_t fifoTxDataCAN3_normal_push(fifoCanDataType*, fifoDataType*);
		uint8_t fifoTxDataCAN3_high_push(fifoCanDataType*, fifoDataType*);

		void can_init();


	#endif


#endif
