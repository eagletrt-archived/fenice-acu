/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "global_variables.h"
#include "sd.h"
#include "state.h"
#include "stdio.h"
#include "string.h"
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
CAN_HandleTypeDef hcan3;

SD_HandleTypeDef hsd2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

uint8_t i_debug;
extern canStruct can1, can3;
extern fifoPriority fifoPriority_t;

CAN_FilterTypeDef sFilter;

long int counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN3_Init(void);
static void MX_SDMMC2_SD_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
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
  MX_CAN3_Init();
  MX_SDMMC2_SD_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_FATFS_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
  for(int i = 0; i < 2; i++){
    HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
  }
  HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_5_GPIO_Port,LED_5_Pin, GPIO_PIN_RESET);
  HAL_TIM_Base_Start_IT(&htim2);

	can1.rx0_interrupt = CAN1_RX0_IRQn;
	can1.tx_interrupt = CAN1_TX_IRQn;
	can1.hcan = &hcan1;

	can_init();

	HAL_UART_Receive_IT(&huart4, (uint8_t *)&debug_rx[debug_rx_count],
						1);  // activate rx interrupt for debug

	current_state = STATE_INIT;
  
	init_sd();

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    can1.dataTx[0] = 0;
		can1.dataTx[1] = 0;
		can1.dataTx[2] = 0;
		can1.dataTx[3] = 0;
		can1.dataTx[4] = counter >> 24;
		can1.dataTx[5] = counter >> 16;
		can1.dataTx[6] = counter >> 8;
		can1.dataTx[7] = counter % 256;

		can1.tx_id = 0xA0;

		// CAN_Send(&can1, normalPriority);
		// HAL_Delay(500);

		// counter ++;

		if (current_state == STATE_INIT) {
			init();
		} else if (current_state == STATE_IDLE) {
			idle();
		} else if (current_state == STATE_SETUP) {
			setup();
		} else if (current_state == STATE_RUN) {
			run();
		}
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
  RCC_OscInitStruct.PLL.PLLQ = 9;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_SDMMC2
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_CLK48;
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
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* UART4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* CAN1_TX_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* CAN1_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* CAN1_SCE_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
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
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
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
  hcan3.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = DISABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = ENABLE;
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
  * @brief SDMMC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC2_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC2_Init 0 */

  /* USER CODE END SDMMC2_Init 0 */

  /* USER CODE BEGIN SDMMC2_Init 1 */

  /* USER CODE END SDMMC2_Init 1 */
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC2_Init 2 */

  /* USER CODE END SDMMC2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 108;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10800;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 10800;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 2000000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_1_Pin|LED_2_Pin|LED_3_Pin|LED_4_Pin 
                          |LED_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin LED_3_Pin LED_4_Pin 
                           LED_5_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|LED_3_Pin|LED_4_Pin 
                          |LED_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		count_ms += 1;
		count_ms_abs++; //absolute 32 bit counter -> up to 50 days 
		if (count_ms == 100) {
			count_ms = 0;
			count_dec++;
			//--- put your counter here (count each 0,1 sec) ---//
			count_inverter++;
			count_imu++;
			count_atc++;
			if (count_inverter == 10) {  //--- check if inverter is connected ---//
					   // TODO: to implement error functions
			} else if (count_inverter == 11) {
				count_inverter = 10;
			}
      /*******************************************************************
      *                           ATC COUNTER
      *******************************************************************/ 
			if (count_atc == 10) {  //--- check if Analog To Can is connected ---//
				set_bit_uint8(&critical_errors[0], 0, 1);
        send_errors();
				atc_connected = 0;
			} else if (count_atc == 11) {
				count_atc = 10;
			}
      /******************************************************************/ 
      /*******************************************************************
      *                           IMU COUNTER
      *******************************************************************/ 
			if (count_imu == 10) {  //--- check if imu is connected ---//
				// imu non presente //
				imu_connected = 0;  // imu not connected
				HAL_UART_Transmit(&huart4, (uint8_t *)"IMU non presente\r\n", strlen("IMU non presente\r\n"), 10);
			} else if (count_imu == 11) {
				count_imu = 10;
			}
      /*****************************************************************/ 
			if (count_dec == 10) {
				count_dec = 0;
				count_sec++;
				if (count_sec == 60) {
					count_sec = 0;
					count_min++;
					if (count_min == 60) {
						count_min = 0;
						count_hour++;
					}
				}
			}
		}
	}else if(htim == &htim4){
    cp = 0;
		polA_cont_up = 0;
		polA_cont_down = 0;
		polB_cont_up = 0;
		polB_cont_down = 0;
		HAL_TIM_Base_Start(&htim5);
		HAL_TIM_Base_Start_IT(&htim5);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }else if(htim == &htim5){

    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		HAL_TIM_Base_Stop(&htim5);
		// The next line it is not necessary but can be a good practice
		__HAL_TIM_SET_COUNTER(&htim5,0);

		char message[256] = "";
		char message2[256] = "";
		char mes[200] = "";
		int val = -1;
		int val2 = -1;

    //		sprintf(message, "\r\nCP = %u -- Encoder = %f", cp, enc_speed);
    //		sprintf(message2, "\r\nSpeed1 = %f -- Speed2 = %f", wheel_speed, wheel_speed2);
    //		print(&huart2, message);
    //		print(&huart2, message2);
    //		val = __HAL_TIM_GET_COUNTER(&htim3);
    //		val2 = __HAL_TIM_GET_COUNTER(&htim4);
    //		sprintf(mes,"\r\n TIM3 = %d -- TIM4 = %d",val,val2);
    //		print(&huart2, mes);

    // Resolution = 5um = 0.000005 m
    // cpr = 48'000
    // encoder diameter = 75.4 mm = 0.0754 m
    // wheel diameter = 0.395 m
    // encoder circumference = 3.1415926535 * 0.0754 = 0.236876086
    // wheel circumference = 3.1415926535 * 0.395 = 1.2409290981325
    // speed multiplier factor = 1.2409290981325 ?????? 0.236876086 = 5.238726792
    // second mult_factor = 1.2409/0.24 = 5.170416667
    // encoder speed = Resolution*cp/0.4s
    // wheel speed = encoder speed * sp_mult


    enc_speed = resolution*cp*-3.60/measurment_per;
    wheel_speed = enc_speed*mult_fact;
    wheel_speed2 = resolution*cp*mult_fact2*-3.6/measurment_per;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	/*sprintf(txt, "%d\r\n", HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0));
	HAL_UART_Transmit(&huart3, (uint8_t*)txt, strlen(txt), 10);*/

	HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	if (hcan == &hcan1) {
		// HAL_UART_Transmit(&huart3, (uint8_t*)"rx on FIFO0\r\n", strlen("rx on
		// FIFO0\r\n"), 10);
		if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
			CAN_RxHeaderTypeDef header;
			HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, can1.dataRX_int);
			can1.rx_id_int = header.StdId;
			can1.rx_size_int = header.DLC;
      if(canSnifferMode == 1){
        sprintf(txt,"%ld %d %d %d %d %d %d %d %d\r\n", can1.rx_id_int, can1.dataRX_int[0], can1.dataRX_int[1], can1.dataRX_int[2], can1.dataRX_int[3], can1.dataRX_int[4], can1.dataRX_int[5], can1.dataRX_int[6], can1.dataRX_int[7] );
        HAL_UART_Transmit(&huart4, (uint8_t*)txt, strlen(txt), 50);
      }else{
			  fifoRxDataCAN_push(&can1);
      }
			/*sprintf(txt, "DATA: %d %d %d %d %d %d %d %d\r\n", can1.dataRx[0],
					can1.dataRx[1], can1.dataRx[2], can1.dataRx[3],
					can1.dataRx[4], can1.dataRx[5], can1.dataRx[6],
					can1.dataRx[7]);*/
			// HAL_UART_Transmit(&huart3, (uint8_t *)txt, strlen(txt), 100);
			// HAL_UART_Transmit(&huart3, (uint8_t*)"ciao2\r\n",
			// strlen("ciao2\r\n"), 10);
		}
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan == &hcan1) {
		HAL_UART_Transmit(&huart4, (uint8_t *)"rx on FIFO1\r\n",
						  strlen("rx on FIFO1\r\n"), 10);
	}
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
	if (hcan == &hcan1) {
		HAL_UART_Transmit(&huart4, (uint8_t *)"FIFO0 FULL\r\n",
						  strlen("FIFO0 FULL\r\n"), 10);
	}
}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {
	if (hcan == &hcan1) {
		HAL_UART_Transmit(&huart4, (uint8_t *)"FIFO1 FULL\r\n",
						  strlen("FIFO1 FULL\r\n"), 10);
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	if (hcan == &hcan1) {
		if (fifoTxDataCAN_high_pop(&can1)) {
			if (CAN_Send_IT(&can1) == 0) {
				// TODO: implementare errore
			} else {
				HAL_UART_Transmit(&huart4, (uint8_t *)("high\r\n"),
								  strlen("high\r\n"), 10);
			}
		} else if (fifoTxDataCAN_normal_pop(&can1)) {
			// HAL_UART_Transmit(&huart3,(uint8_t*)("Prendo : 19:31:19.703:d: 19:31:19.703:alla fifo\r\n"),
			// strlen("Prendo dalla fifo\r\n"), 10);
			if (CAN_Send_IT(&can1) == 0) {
				// TODO: implementare errore
			}
		} else {
			// HAL_UART_Transmit(&huart3,(uint8_t*)("Fifo vuota\r\n"),
			// strlen("Fifo vuota\r\n"), 10);
		}
	} /*else{
	 HAL_UART_Transmit(&huart3,(uint8_t*)("Messaggio trasmesso\r\n"),
	 strlen("Messaggio trasmesso\r\n"), 10);
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
			 HAL_UART_Transmit(&huart3,(uint8_t*)("Prendo dalla fifo\r\n"),
	 strlen("Prendo dalla fifo\r\n"), 10); for(int i = 0; i < 8; i++){
				 can3.dataTx[i] = fifodata.data[i];
			 }
			 if(CAN_Send_IT(&can3, fifodata.id) == 0){
				 //TODO: implementare errore
			 }
		 }else{
			 //TODO: riattivare interrupt
			 HAL_UART_Transmit(&huart3,(uint8_t*)("Fifo vuota\r\n"),
	 strlen("Fifo vuota\r\n"), 10);
		 }*/
	  //}
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	sprintf(txt, "mb1: %d %d\r\n", can1.fifo.txTailNormal,
			can1.fifo.txHeadNormal);
	HAL_UART_Transmit(&huart4, (uint8_t *)(txt), strlen(txt), 10);
  HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	if (hcan == &hcan1) {
		if (fifoTxDataCAN_high_pop(&can1)) {
			if (CAN_Send_IT(&can1) == 0) {
				// TODO: implementare errore
			} else {
				HAL_UART_Transmit(&huart4, (uint8_t *)("high\r\n"),
								  strlen("high\r\n"), 10);
			}
		} else if (fifoTxDataCAN_normal_pop(&can1)) {
			// HAL_UART_Transmit(&huart3,(uint8_t*)("Prendo dalla fifo\r\n"),
			// strlen("Prendo dalla fifo\r\n"), 10);
			if (CAN_Send_IT(&can1) == 0) {
				// TODO: implementare errore
			}
		} else {
			// HAL_UART_Transmit(&huart3,(uint8_t*)("Fifo vuota\r\n"),
			// strlen("Fifo vuota\r\n"), 10);
		}
	}
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	sprintf(txt, "mb2: %d %d\r\n", can1.fifo.txTailNormal,
			can1.fifo.txHeadNormal);
	HAL_UART_Transmit(&huart4, (uint8_t *)(txt), strlen(txt), 10);
	HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	if (hcan == &hcan1) {
		if (fifoTxDataCAN_high_pop(&can1)) {
			if (CAN_Send_IT(&can1) == 0) {
				// TODO: implementare errore
			} else {
				HAL_UART_Transmit(&huart4, (uint8_t *)("high\r\n"),
								  strlen("high\r\n"), 10);
			}
		} else if (fifoTxDataCAN_normal_pop(&can1)) {
			// HAL_UART_Transmit(&huart3,(uint8_t*)("Prendo dalla fifo\r\n"),
			// strlen("Prendo dalla fifo\r\n"), 10);
			if (CAN_Send_IT(&can1) == 0) {
				// TODO: implementare errore
			}
		} else {
			// HAL_UART_Transmit(&huart3,(uint8_t*)("Fifo vuota\r\n"),
			// strlen("Fifo vuota\r\n"), 10);
		}
	}
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	sprintf(txt, "--- Errore ---: %d\r\n", (int)hcan->ErrorCode);
	HAL_UART_Transmit(&huart4, (uint8_t *)(txt), strlen(txt), 10);
	if (hcan == &hcan1) {
		CAN_Send_Bck(&can1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_GPIO_TogglePin(LED_2_GPIO_Port,LED_2_Pin);
	if (huart == &huart4) {
		if ((debug_rx[debug_rx_count] == '\r') |
			(debug_rx[debug_rx_count] == '\n')) {
			debug_msg_arrived = 1;		   // set flag
			debug_rx[debug_rx_count] = 0;  // set end of the string
			debug_rx_count = 0;			   // reset counter
		} else {
			if (debug_rx_count == MAX_DEBUG_RX_L) {
				// overflow
				debug_rx_count = 0;  // reset counter for overflow
			} else {
				HAL_UART_Transmit(&huart4, (uint8_t *)&debug_rx[debug_rx_count],1, 10);  // retransmit char
				debug_rx_count++;
			}
		}
		HAL_UART_Receive_IT(&huart4, (uint8_t *)&debug_rx[debug_rx_count], 1);  // activate rx interrupt for debug
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
  for(int i = 0; i < 4; i++){
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
  }
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
