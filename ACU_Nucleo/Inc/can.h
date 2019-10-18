#ifndef _FENICE_H
#define _FENICE_H


#define fifoLengthN 100
#define fifoLengthH 10


#include "stm32f7xx_hal.h"

	void fenice_init();

	#ifdef HAL_CAN_MODULE_ENABLED

		typedef enum fifoPriority_t{normalPriority, highPriority}fifoPriority;

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

		typedef struct{

			int size; //size of data

			uint8_t dataTx[8];
			uint8_t dataRx[8];
			uint8_t dataTxBck[8];


			uint32_t id;
			uint32_t idBck;


			CAN_HandleTypeDef *hcan;
			CAN_FilterTypeDef canFilter;

			HAL_StatusTypeDef configFilter_status;
			HAL_StatusTypeDef activateNotif_status;
			HAL_StatusTypeDef canStart_status;

			fifoCanDataType fifo;
			fifoCanDataType rxfifo;

			IRQn_Type rx0_interrupt;
			IRQn_Type tx_interrupt;



		}canStruct;

		uint8_t CAN_initialization(canStruct *can);
		void report_error_can1();
		void report_error_can3();
		uint8_t CAN_Send(canStruct*, fifoPriority);
		uint8_t CAN_Send_IT(canStruct*);
		uint8_t CAN_Send_Bck(canStruct*);

		uint8_t fifoRxDataCAN_pop(canStruct*);
		uint8_t fifoRxDataCAN_push(canStruct*);

		uint8_t fifoTxDataCAN_normal_pop(canStruct*);
		uint8_t fifoTxDataCAN_high_pop(canStruct*);
		uint8_t fifoTxDataCAN_normal_push(canStruct*);
		uint8_t fifoTxDataCAN_high_push(canStruct*);


		void can_init();


	#endif


#endif
