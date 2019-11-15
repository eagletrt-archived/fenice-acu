#include "pedals.h"
#include "global_variables.h"
#include "stdio.h"
#include "stm32f7xx_hal.h"
#include "string.h"

PotStc accel, brake;
uint8_t accel_implausibility_check_count_flag = 0;
uint8_t brake_implausibility_check_count_flag = 0;
uint8_t accel_implausibility_check_count = 0;
uint8_t brake_implausibility_check_count = 0;

uint8_t accel_implausibility_check() {
	if (abs(accel.pot1_val_100 + accel.pot2_val_100 - 100) > 10) {
		// error
		/*sprintf(txt, "POT FAIL2** %d\r\n",
				accel_implausibility_check_count_flag);
		HAL_UART_Transmit(&huart3, (uint8_t*)txt, strlen(txt), 10);*/
		if (accel_implausibility_check_count_flag == 0) {
			accel_implausibility_check_count_flag = 1;
			/*sprintf(txt, "POT FAIL3***\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t*)txt, strlen(txt), 10);*/
			return 0;
		} else if (accel_implausibility_check_count_flag == 2) {
			accel_implausibility_check_count_flag = 3;
			sprintf(txt, "POT FAIL4****\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t*)txt, strlen(txt), 10);
			return 1;
		} else if (accel_implausibility_check_count_flag == 3) {
			return 2;
		} else {
			return 0;
		}
	} else {
		accel_implausibility_check_count_flag = 0;
		accel_implausibility_check_count = 0;
		/*sprintf(txt, "POT FAIL5*****\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)txt, strlen(txt), 10);*/
		return 0;  // retrun ok
	}
}
uint8_t brake_implausibility_check() { return 0; }