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
CAN_HandleTypeDef hcan3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ID id;
uint8_t i_debug;
extern canStruct can1,can3;
extern fifoCanDataType fifoCAN1, fifoCAN3;
extern fifoPriority fifoPriority_t;

CAN_FilterTypeDef sFilter;
/*
extern fifoRxDataType fifoRxDataCAN1[fifoLengthN], fifoRxDataCAN3[fifoLengthN];
extern fifoTxDataType fifoTxDataCAN1_normal[fifoLengthN], fifoTxDataCAN1_high[fifoLengthR];
extern fifoTxDataType fifoTxDataCAN3_normal[fifoLengthN], fifoTxDataCAN3_high[fifoLengthR];*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN3_Init(void);
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
	ID_intit(&id);
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_CAN3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim3);
/*
  sFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilter.FilterIdLow = 0;
  sFilter.FilterIdHigh = 0;
  sFilter.FilterMaskIdHigh = 0;
  sFilter.FilterMaskIdLow = 0;
  sFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilter.FilterBank = 0;
  sFilter.FilterScale  = CAN_FILTERSCALE_16BIT;
  sFilter.FilterActivation = ENABLE;
  int err1 = HAL_CAN_ConfigFilter(&hcan3, &sFilter);
  int err2 = HAL_CAN_ActivateNotification(&hcan3, CAN3_RX0_IRQn);
  int err3 = HAL_CAN_Start(&hcan3);*/

  can_init();

  char txt[100];
  sprintf(txt,"----------START---------\r\n");
  HAL_UART_Transmit(&huart1,(uint8_t*)txt, strlen(txt), 10);
  sprintf(txt,"Config Status: %d\r\n", can3.configFilter_status);
  HAL_UART_Transmit(&huart1,(uint8_t*)txt, strlen(txt), 10);
  sprintf(txt,"CAN Notification %d\r\n", can3.activateNotif_status);
  HAL_UART_Transmit(&huart1,(uint8_t*)txt, strlen(txt), 10);
  sprintf(txt,"CAN start status: %d\r\n", can3.canStart_status);
  HAL_UART_Transmit(&huart1,(uint8_t*)txt, strlen(txt), 10);

  current_state = STATE_INIT;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(current_state == STATE_INIT){
		  init();
	  }else if(current_state == STATE_IDLE){
		  idle();
	  }else if(current_state == STATE_CALIB){
		  calib();
	  }else if(current_state == STATE_SETUP){
		  setup();
	  }else if(current_state == STATE_RUN){
		  run();
	  }

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

	  HAL_UART_Transmit(&huart1,(uint8_t*)txt, strlen(txt), 10);
	  sprintf(txt,"Config Status: %d\r\n", can3.configFilter_status);
	  HAL_UART_Transmit(&huart1,(uint8_t*)txt, strlen(txt), 10);
	  sprintf(txt,"CAN Notification %d\r\n", can3.activateNotif_status);
	  HAL_UART_Transmit(&huart1,(uint8_t*)txt, strlen(txt), 10);
	  sprintf(txt,"CAN start status: %d\r\n", can3.canStart_status);
	  HAL_UART_Transmit(&huart1,(uint8_t*)txt, strlen(txt), 10);

	  /*HAL_GPIO_TogglePin(USER_LED_3_GPIO_Port, USER_LED_2_Pin);
	  HAL_GPIO_TogglePin(USER_LED_3_GPIO_Port, USER_LED_3_Pin);
	  HAL_GPIO_TogglePin(USER_LED_4_GPIO_Port, USER_LED_4_Pin);
	  HAL_GPIO_TogglePin(USER_LED_5_GPIO_Port, USER_LED_5_Pin);*/
	  HAL_Delay(500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* CAN3_TX_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN3_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);
  /* CAN3_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN3_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);
  /* CAN3_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN3_RX1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN3_RX1_IRQn);
  /* CAN3_SCE_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN3_SCE_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN3_SCE_IRQn);
}

/**
  * @brief CAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN3_Init(void)
{

  /* USER CODE BEGIN CAN3_Init 0 */

  /* USER CODE END CAN3_Init 0 */

  /* USER CODE BEGIN CAN3_Init 1 */

  /* USER CODE END CAN3_Init 1 */
  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 3;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = DISABLE;
  hcan3.Init.AutoWakeUp = ENABLE;
  hcan3.Init.AutoRetransmission = DISABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN3_Init 2 */

  /* USER CODE END CAN3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 128;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 54;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, USER_LED_1_Pin|USER_LED_2_Pin|USER_LED_3_Pin|USER_LED_4_Pin 
                          |USER_LED_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USER_LED_1_Pin USER_LED_2_Pin USER_LED_3_Pin USER_LED_4_Pin 
                           USER_LED_5_Pin */
  GPIO_InitStruct.Pin = USER_LED_1_Pin|USER_LED_2_Pin|USER_LED_3_Pin|USER_LED_4_Pin 
                          |USER_LED_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim3){
		count_ms += 10;
		if(count_ms == 10){
			count_ms = 0;
			count_dec++;
			count_inverter++;
			if(count_inverter == 10){
				//TODO: implementare funzione
			}else if(count_inverter == 11){
				count_inverter = 10;
			}
			if(count_dec == 10){
				count_dec = 0;
				count_sec++;
				if(count_sec == 100){
					count_sec = 0;
				}
			}
		}
	}
}
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
		HAL_GPIO_TogglePin(USER_LED_4_GPIO_Port, USER_LED_4_Pin);
		fifoDataType fifodata;
		if(fifoTxDataCAN3_high_pop(&fifoCAN3, &fifodata)){
			for(int i = 0; i < 8; i++){
				can3.dataTx[i] = fifodata.data[i];
			}
			if(CAN_Send_IT(&can3, fifodata.id) == 0){
				//TODO: implementare errore
			}
		}else if(fifoTxDataCAN3_normal_pop(&fifoCAN3, &fifodata)){
			for(int i = 0; i < 8; i++){
				can3.dataTx[i] = fifodata.data[i];
			}
			if(CAN_Send_IT(&can3, fifodata.id) == 0){
				//TODO: implementare errore
			}
		}else{
			//TODO: riattivare interrupt
		}
	//}
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
	HAL_GPIO_TogglePin(USER_LED_4_GPIO_Port, USER_LED_4_Pin);
	HAL_GPIO_TogglePin(USER_LED_5_GPIO_Port, USER_LED_5_Pin);
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
