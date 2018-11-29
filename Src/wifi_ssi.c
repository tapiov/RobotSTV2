//
// File: wifi_ssi.c - Wifi SSI functions
//

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

#include "usart.h"

/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"

void ssi_update(float locx, float locy, float locz, float accelx,
						float accely, float accelz, float gyrox,
						float gyroy, float gyroz, float dist,
						float spd, float move, float temp, float pssr,
						float humd, char *sts) {

	char AT_Str[400];
	char Value_Str[400];
	char AT_Rpl[400];

	uint8_t len_at, len_value;

	memset(AT_Str,'\0',400);
	memset(Value_Str,'\0',400);
	memset(AT_Rpl,'\0',400);

	len_value = sprintf(Value_Str,
				"|%f|%f|%f|%f|%f|%f|%f|%f|%f|%f|%f|%f|%f|%f|%f|%s|\r",
				locx,locy,locz,accelx,accely,accelz,gyrox,gyroy,gyroz,
				dist,spd,move,temp,pssr,humd,sts);

	len_at = sprintf(AT_Str,"AT+S.INPUTSSI=%d\r",len_value);

	HAL_UART_Transmit_DMA(&huart6, (uint8_t *) AT_Str, len_at);
	HAL_Delay(1000);
	HAL_UART_Transmit_DMA(&huart6, (uint8_t *) Value_Str, len_value);

}
