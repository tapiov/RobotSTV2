
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "wifi_module.h"
#include "wifi_globals.h"
#include "wifi_interface.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void USART_PRINT_MSG_Configuration(UART_HandleTypeDef *UART_MsgHandle,
		uint32_t baud_rate);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

wifi_state_t wifi_state;
wifi_config config;
UART_HandleTypeDef UART_MsgHandle;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_StatusTypeDef autodetect = HAL_OK;

	char AT_Str[400];
	char Value_Str[400];
	char AT_Rpl[400];

#ifdef USART_PRINT_MSG
	// UART_Msg_Gpio_Init();
#if defined (USE_STM32L0XX_NUCLEO)
	USART_PRINT_MSG_Configuration(&UART_MsgHandle, 115200); //L0 max rate?
#else
	USART_PRINT_MSG_Configuration(&huart3, 115200);
#endif
	Set_UartMsgHandle(&huart3); //this is required for the console handler initialization
#endif

	printf("\r\n Robot console starting \r\n");

	printf("\r\n Please wait... \r\n");


#if defined (USE_STM32L0XX_NUCLEO)
	autodetect = WiFi_Module_UART_Configuration(115200); //115200 //460800 //921600
#else
	// autodetect = WiFi_Module_UART_Configuration(115200); //115200 //460800 //921600
#endif

	// WiFi_Module_UART_Configuration(115200); //115200 //460800 //921600
	USART2->CR3 |= 0x00000040;

	autodetect = HAL_OK;

//	if (autodetect == HAL_OK)
		UART_DMA_Init();
//	else {
//		printf("\rError in baud-rate auto-detection...\r\n");
//	}

	HAL_Delay(2000);
	printf("\r\n Console Ready... \r\n");

	//	AT commands to program Wifi module
	//	AT+S.SCFG=blink_led,0
	//	AT+S.SSIDTXT=<RobotSSID>
	//	AT+S.SCFG=wifi_auth_type,0
	//	AT+S.SCFG=wifi_priv_mode,2
	//	AT+S.SCFG=wifi_wpa_psk_text,<secretkey>
	//	AT+S.SCFG=ip_hostname,<hostname>
	//	AT+S.SCFG=ip_apdomainname,<robot.net>
	//	AT+S.SCFG=ip_apredirect,index.html
	//	AT+S.WCFG
	//
	//	## Will erase data, DO NOT USE IF NOT INTENDED
	//	## AT+S.FSUPDATE=i,192.168.0.51,\fs.img,,,,
	//
	//	## Write data to robot.fhtml table
	//	AT+S.INPUTSSI=<strlen><CR>
	//	|10|20|30.3|40|55|<CR>

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

  /* USER CODE END WHILE */

//		 memset(AT_Str, '\0', 400);
//		 memset(Value_Str, '\0', 400);
//
//		uint8_t len_value,len_at;
//		 len_value = sprintf(Value_Str,"|10|20|30.3|40|55|68|78|811|99|100|10|20|30.3|40|55|68|Status GOOD|\r");
//		 len_at = sprintf(AT_Str,"AT+S.INPUTSSI=%d\r",len_value);
//		 HAL_UART_Transmit_DMA(&huart2, (uint8_t *) AT_Str, len_at);
//		 HAL_Delay(5000);
//		 HAL_UART_Transmit_DMA(&huart2, (uint8_t *) Value_Str, len_value);
//
//		HAL_Delay(5000);
//
//		memset(AT_Str, '\0', 400);
//		memset(Value_Str, '\0', 400);
//
//		len_value = sprintf(Value_Str,"|10|20|30.3|40|55|68|78|811|99|100|10|20|30.3|40|55|68|Status BAD|\r");
//		len_at = sprintf(AT_Str,"AT+S.INPUTSSI=%d\r",len_value);
//		HAL_UART_Transmit_DMA(&huart2, (uint8_t *) AT_Str, len_at);
//		HAL_Delay(5000);
//		HAL_UART_Transmit_DMA(&huart2, (uint8_t *) Value_Str, len_value);
//
//		HAL_Delay(5000);


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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART6
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_UART7
                              |RCC_PERIPHCLK_UART8|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart8ClockSelection = RCC_UART8CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

#ifdef USART_PRINT_MSG
void USART_PRINT_MSG_Configuration(UART_HandleTypeDef *UART_MsgHandle,
		uint32_t baud_rate) {
	UART_MsgHandle->Instance = WIFI_UART_MSG;
	UART_MsgHandle->Init.BaudRate = baud_rate;
	UART_MsgHandle->Init.WordLength = UART_WORDLENGTH_8B;
	UART_MsgHandle->Init.StopBits = UART_STOPBITS_1;
	UART_MsgHandle->Init.Parity = UART_PARITY_NONE;
	UART_MsgHandle->Init.HwFlowCtl = UART_HWCONTROL_NONE; // USART_HardwareFlowControl_RTS_CTS;
	UART_MsgHandle->Init.Mode = UART_MODE_TX_RX;

	if (HAL_UART_DeInit(UART_MsgHandle) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UART_Init(UART_MsgHandle) != HAL_OK) {
		Error_Handler();
	}
#ifdef WIFI_USE_VCOM
	/*## -1- Enable USART2 DMAT & DMAR #################################################*/
	UART_MsgHandle->Instance->CR3 |= 0x00000040;
#endif //WIFI_USE_VCOM
}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
