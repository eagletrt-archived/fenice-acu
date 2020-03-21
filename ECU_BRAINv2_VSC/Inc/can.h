#ifndef __CAN_H__
#define __CAN_H__

#define fifoLengthN 100
#define fifoLengthH 10

#include "stm32f7xx_hal.h"
#include "main.h"
#include "stdio.h"
#include "stm32f7xx_hal_can.h"
#include "string.h"
#include "global_variables.h"
#include "stdbool.h"

void fenice_init();

#ifdef HAL_CAN_MODULE_ENABLED

typedef enum fifoPriority_t { normalPriority, highPriority } fifoPriority;

typedef struct fifoDataType {
	uint32_t id;
	uint32_t size;
	uint8_t data[8];
} fifoDataType;

typedef struct fifoCanDataType {
	uint8_t rxHead;
	uint8_t rxTail;

	uint8_t txHeadNormal;
	uint8_t txTailNormal;

	uint8_t txHeadHigh;
	uint8_t txTailHigh;

	fifoDataType rx[fifoLengthN];
	fifoDataType txNormal[fifoLengthN];
	fifoDataType txHigh[fifoLengthH];

} fifoCanDataType;

typedef struct {

	int tx_size;  // size of data
	int rx_size;
	int rx_size_int;

	uint8_t dataTx[8];
	uint8_t dataRx[8];
	uint8_t dataRX_int[8];
	uint8_t dataTxBck[8];

	uint32_t tx_id;
	uint32_t rx_id;
	uint32_t rx_id_int;
	uint32_t idBck;
	uint32_t sizeBck;

	CAN_HandleTypeDef *hcan;
	CAN_FilterTypeDef canFilter;

	HAL_StatusTypeDef configFilter_status;
	HAL_StatusTypeDef activateNotif_status;
	HAL_StatusTypeDef canStart_status;

	fifoCanDataType fifo;

	IRQn_Type rx0_interrupt;
	IRQn_Type tx_interrupt;

} canStruct;

bool CAN_initialization(canStruct *can);
void report_error_can1();
void report_error_can3();
bool CAN_Send(canStruct *, fifoPriority);
bool CAN_Send_IT(canStruct *);
bool CAN_Send_Bck(canStruct *);

bool fifoRxDataCAN_pop(canStruct *);
bool fifoRxDataCAN_push(canStruct *);

bool fifoTxDataCAN_normal_pop(canStruct *);
bool fifoTxDataCAN_high_pop(canStruct *);
bool fifoTxDataCAN_normal_push(canStruct *);
bool fifoTxDataCAN_high_push(canStruct *);

void can_init();

extern canStruct can1, can3;

#endif // HAL_CAN_MODULE_ENABLE

#endif
