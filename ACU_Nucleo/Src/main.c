/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "id.h"
#include "state.h"
#include "can.h"
#include "global_variables.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

ID id;
uint8_t i_debug;
extern canStruct can1,can3;
extern fifoCanDataType fifoCAN1, fifoCAN3;
extern fifoPriority fifoPriority_t;

CAN_FilterTypeDef sFilter;

char txt[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  can1.rx0_interrupt = CAN1_RX0_IRQn;
  can1.hcan = &hcan1;

  can_init();

  sprintf(txt,"----------START---------\r\n");
  HAL_UART_Transmit(&huart3,(uint8_t*)txt, strlen(txt), 10);
  sprintf(txt,"Config Status: %d\r\n", can1.configFilter_status);
  HAL_UART_Transmit(&huart3,(uint8_t*)txt, strlen(txt), 10);
  sprintf(txt,"CAN Notification %d\r\n", can1.activateNotif_status);
  HAL_UART_Transmit(&huart3,(uint8_t*)txt, strlen(txt), 10);
  sprintf(txt,"CAN start status: %d\r\n", can1.canStart_status);
  HAL_UART_Transmit(&huart3,(uint8_t*)txt, strlen(txt), 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  can3.dataTx[0]=i_debug;
	  i_debug++;
	  can3.dataTx[1]=2;
	  can3.dataTx[2]=3;
	  can3.dataTx[3]=4;
	  can3.dataTx[4]=5;
	  can3.dataTx[5]=6;
	  can3.dataTx[6]=7;
	  can3.dataTx[7]=8;
	  can3.size = 8;

	  CAN_Send(&can3, 130, normalPriority);

	  HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* CAN1_SCE_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* CAN1_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* CAN1_TX_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB14 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	/*if(hcan == &hcan1){
		fifoDataType fifodata;
		if(fifoTxDataCAN1_high_pop(&fifoCAN1, &fifodata)){
			for(int i = 0; i < 8; i++){
				can1.dataTx[i] = fifodata.data[i];
			}
			if(CAN_Send_IT(&can1, fifodata.id) == 0){
				//TODO: implementare errore
			}
		}else if(fifoTxDataCAN1_normal_pop(&fifoCAN1, &fifodata)){
			for(int i = 0; i < 8; i++){
				can1.dataTx[i] = fifodata.data[i];
			}
			if(CAN_Send_IT(&can1, fifodata.id) == 0){
				//TODO: implementare errore
			}
		}else{
			//TODO: riattivare interrupt
		}
	}else{*/
	HAL_UART_Transmit(&huart3,(uint8_t*)("Messaggio trasmesso\r\n"), strlen("Messaggio trasmesso\r\n"), 10);
		//HAL_GPIO_TogglePin(USER_LED_4_GPIO_Port, USER_LED_4_Pin);
		fifoDataType fifodata;
		if(fifoTxDataCAN3_high_pop(&fifoCAN3, &fifodata)){
			for(int i = 0; i < 8; i++){
				can3.dataTx[i] = fifodata.data[i];
			}
			if(CAN_Send_IT(&can3, fifodata.id) == 0){
				//TODO: implementare errore
			}
		}else if(fifoTxDataCAN3_normal_pop(&fifoCAN3, &fifodata)){
			HAL_UART_Transmit(&huart3,(uint8_t*)("Prendo dalla fifo\r\n"), strlen("Prendo dalla fifo\r\n"), 10);
			for(int i = 0; i < 8; i++){
				can3.dataTx[i] = fifodata.data[i];
			}
			if(CAN_Send_IT(&can3, fifodata.id) == 0){
				//TODO: implementare errore
			}
		}else{
			//TODO: riattivare interrupt
			HAL_UART_Transmit(&huart3,(uint8_t*)("Fifo vuota\r\n"), strlen("Fifo vuota\r\n"), 10);
		}
	//}
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	HAL_UART_Transmit(&huart3,(uint8_t*)("Mailbox 1\r\n"), strlen("Mailbox 1\r\n"), 10);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	HAL_UART_Transmit(&huart3,(uint8_t*)("Mailbox 2\r\n"), strlen("Mailbox 2\r\n"), 10);
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	sprintf(txt,"--- Errore ---: %d\r\n",(int)hcan->ErrorCode);
	HAL_UART_Transmit(&huart3,(uint8_t*)(txt), strlen(txt), 10);
	if(hcan == &hcan1){
		CAN_Send_Bck(&can1);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

	HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
	HAL_GPIO_TogglePin(USER_LED_2_GPIO_Port, USER_LED_2_Pin);
	HAL_GPIO_TogglePin(USER_LED_3_GPIO_Port, USER_LED_3_Pin);

	HAL_Delay(100);

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
