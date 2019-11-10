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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

SD_HandleTypeDef hsd1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define MAX_DEBUG_RX_L 20
#define MAX_DEBUG_TX_L 500
char debug_rx[MAX_DEBUG_RX_L], debug_tx[MAX_DEBUG_TX_L];
uint8_t debug_rx_count = 0, debug_msg_arrived = 0;
uint8_t imu_connected = 0, its0_connected = 0, its1_connected = 0, its2_connected = 0, its3_connected = 0;
enum state_t{STATE_INIT, STATE_IDLE, STATE_CALIB, STATE_SETUP, STATE_RUN}current_state;
FIL loggingFile;
FIL log_names_f;

TCHAR message[256];
char filename[256] = "abcabc.txt";
char filename_1[256]="log_names.txt";
char txt[1000];

int max_files = 100;
int byteswritten;
int mount_ok = 0;
int msg_counter = 0;
int msg_index = 0;

char buffer[256]="Starting Antenna Logging\r\n";
int bytes_read;

char *pointer;
char log_names[356];

FRESULT res_open;
FRESULT res_mount;

int successfull_opening = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void init_sd();
void debug_operations();
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
  MX_SDMMC1_SD_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart3,(uint8_t*)"Program started\r\n",strlen("Program started\r\n"),10);
  res_mount = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
	HAL_Delay(10);

	while(successfull_opening != 1){
		init_sd();
	}
  HAL_UART_Receive_IT(&huart3,(uint8_t*) &debug_rx[debug_rx_count], 1); //activate rx interrupt for debug
  current_state = STATE_INIT;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //----------------------------------------------------------------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------------------------------------------------------------
  while (1)
  {
    if(debug_msg_arrived == 1){
      debug_msg_arrived = 0; // reset flag
      debug_operations();
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
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
  /* SDMMC1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SDMMC1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 1;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  huart3.Init.BaudRate = 2250000;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : SD_detect_Pin */
  GPIO_InitStruct.Pin = SD_detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_detect_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void init_sd(){
	if(res_mount != FR_OK){
		HAL_UART_Transmit(&huart3,(uint8_t*)"---mounting---\r\n",strlen("---mounting---\r\n"), 10);
		res_mount = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
	}
	if (res_mount == FR_OK) {
		sprintf(filename_1, "name.txt");
		res_open=f_open(&log_names_f, (TCHAR const*)&filename_1, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		f_read(&log_names_f, log_names, 256, (void*)&bytes_read);

		HAL_UART_Transmit(&huart3,(uint8_t*)"mounted, opened\r\n",strlen("mounted, opened\r\n"),10);

		sprintf(txt, "%s\r\n", log_names);
		HAL_UART_Transmit(&huart3,(uint8_t*)txt,strlen(txt),10);

		char name[256];

		f_close(&log_names_f);
		f_open(&log_names_f, (TCHAR const*)&filename_1, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

		for(int i = 0; i < max_files; i++){

			sprintf(name, "log_%d ", i);

			pointer = strstr(log_names, name);

			if(i == max_files){
				sprintf(filename,"default.txt");

				f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
				f_write(&loggingFile, buffer, strlen(buffer), (void*)&byteswritten);
				f_close(&loggingFile);
				f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

				HAL_UART_Transmit(&huart3,(uint8_t*)"created -> default.txt\r\n",strlen("created -> default.txt\r\n"),10);

				successfull_opening = 1;

				break;
			}

			if(i == 0 && pointer == NULL){

				sprintf(filename, "log_0 \t\r\n");

				f_write(&log_names_f, filename, strlen(filename), (void*)&byteswritten);
				f_close(&log_names_f);

				sprintf(filename, "Log_0.txt");

				f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
				f_write(&loggingFile, buffer, strlen(buffer), (void*)&byteswritten);
				f_close(&loggingFile);
				f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

				HAL_UART_Transmit(&huart3,(uint8_t*)"\r\ncreated -> Log_0\r\n",strlen("\r\ncreated -> Log_0\r\n"),10);

				successfull_opening = 1;

				break;
			}
			if(pointer == NULL){
				sprintf(filename, "log_%d \t\r\n", i);

				f_write(&log_names_f, filename, strlen(filename), (void*)&byteswritten);
				f_close(&log_names_f);

				sprintf(filename, "Log_%d.txt", i);

				f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
				f_write(&loggingFile, buffer, strlen(buffer), (void*)&byteswritten);
				f_close(&loggingFile);
				f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

				HAL_UART_Transmit(&huart3,(uint8_t*)"\r\ncreated -> ",strlen("\r\ncreated -> "),10);
				HAL_UART_Transmit(&huart3,(uint8_t*) filename, strlen(filename), 10);
				HAL_UART_Transmit(&huart3,(uint8_t*) "\r\n", strlen("\r\n"),10);

				successfull_opening = 1;

				break;
			}
		}

		mount_ok = 1;
		HAL_UART_Transmit(&huart3,(uint8_t*)"files closed\r\n",strlen("files closed\r\n"),10);
	}else {
		mount_ok = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart3){
		if((debug_rx[debug_rx_count] == '\r') | (debug_rx[debug_rx_count] == '\n')){
			debug_msg_arrived = 1; //set flag
			debug_rx[debug_rx_count] = 0; //set end of the string
			debug_rx_count = 0; //reset counter
		}else{
			if(debug_rx_count == MAX_DEBUG_RX_L){
				//overflow
				debug_rx_count = 0; //reset counter for overflow
			}else{
				HAL_UART_Transmit(&huart3, (uint8_t*)&debug_rx[debug_rx_count], 1, 10); //retransmit char
				debug_rx_count++;
			}
		}
		HAL_UART_Receive_IT(&huart3,(uint8_t*) &debug_rx[debug_rx_count], 1); //activate rx interrupt for debug
	}
}
void debug_operations(){
	if(strcmp(debug_rx,"help") == 0){
		sprintf(debug_tx,
				"\r\n***********ECU HELP***********\r\n"
				"Avaiable msg are:\r\n"
				"\t-- status -> print ECU status\r\n"
				"\t-- time -> print activity time\r\n"
				"\t-- codev  -> print code version\r\n");
		HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
	}/*else if(strcmp(debug_rx,"codev") == 0){
		HAL_UART_Transmit(&huart3,(uint8_t*)code_version, strlen(code_version), 100);
	}else if(strcmp(debug_rx,"time") == 0){
		sprintf(debug_tx,"\r\nTime: %d hours : %d min : %d sec : %d dec\r\n",count_hour,count_min, count_sec, count_dec );
		HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
	}else if(strcmp(debug_rx,"status") == 0){
		sprintf(debug_tx,
				"\r\n\ntype of status:\r\n"
				"\t 0 = OK\r\n"
				"\t 1 = ERROR\r\n"
				"\t 2 = BUSY\r\n"
				"\t 3 = TIMEOUT\r\n\n"
				"CAN1 status:\r\n"
				"\tCAN1 config status: %d \r\n"
				"\tCAN1 notification status: %d\r\n"
				"\tCAN1 start status: %d\r\n"
				,can1.configFilter_status,can1.activateNotif_status,can1.canStart_status);
		HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
		sprintf(debug_tx,
				"CAN3 status:\r\n"
				"\tCAN3 config status: %d \r\n"
				"\tCAN3 notification status: %d\r\n"
				"\tCAN3 start status: %d\r\n"
				,can3.configFilter_status,can3.activateNotif_status,can3.canStart_status);
		HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
		if(current_state == STATE_INIT){
			HAL_UART_Transmit(&huart3,(uint8_t*)"\r\nCurrent state: STATE_INIT\r\n", strlen("\r\nCurrent state: STATE_INIT\r\n"), 100);
		}else if(current_state == STATE_IDLE){
			HAL_UART_Transmit(&huart3,(uint8_t*)"\r\nCurrent state: STATE_IDLE\r\n", strlen("\r\nCurrent state: STATE_IDLE\r\n"), 100);
		}else if(current_state == STATE_CALIB){
			HAL_UART_Transmit(&huart3,(uint8_t*)"\r\nCurrent state: STATE_CALIB\r\n", strlen("\r\nCurrent state: STATE_CALIB\r\n"), 100);
		}else if(current_state == STATE_SETUP){
			HAL_UART_Transmit(&huart3,(uint8_t*)"\r\nCurrent state: STATE_SETUP\r\n", strlen("\r\nCurrent state: STATE_SETUP\r\n"), 100);
		}else if(current_state == STATE_RUN){
			HAL_UART_Transmit(&huart3,(uint8_t*)"\r\nCurrent state: STATE_RUN\r\n", strlen("\r\nCurrent state: STATE_RUN\r\n"), 100);
		}
		sprintf(debug_tx,
				"\r\n"
				"Device connected : (0 = no, 1 = yes)\r\n"
				"\t IMU -> %d\r\n"
				"\t ITS0 -> %d\r\n"
				"\t ITS1 -> %d\r\n"
				"\t ITS2 -> %d\r\n"
				"\t ITS3 -> %d\r\n"
				,imu_connected, its0_connected, its1_connected, its2_connected, its3_connected);
		HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
	}*/else if(strcmp(debug_rx,"gay") == 0){
		sprintf(debug_tx,
				"\r\n"
				"          $\r\n"
				"        $   $\r\n"
				"       $     $\r\n"
				"       $$$$$$$\r\n"
				"       $$$$$$$\r\n"
				"       $$$$$$$\r\n"
				"       $$$$$$$\r\n"
				"       $$$$$$$\r\n"
				"       $$$$$$$\r\n"
				"       $$$$$$$\r\n"
				"  $$$$$$     $$$$$$\r\n"
				" $$$$$$$$   $$$$$$$$\r\n"
				"$$$$$$$$$$$$$$$$$$$$\r\n"
				" $$$$$$$$   $$$$$$$$\r\n"
				"  $$$$$$     $$$$$$\r\n");
		HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
	}else{
		sprintf(debug_tx,"\r\nERROR : msg %s doesn't exist\r\n",debug_rx);
		HAL_UART_Transmit(&huart3,(uint8_t*)debug_tx, strlen(debug_tx), 100);
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
