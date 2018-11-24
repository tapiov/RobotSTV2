/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */

#define USART_PRINT_MSG 1
#define CONSOLE_UART_ENABLED 1
#define WIFI_USE_VCOM 1
#define SPWF04 1

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define UART2_WIFI_TX_Pin GPIO_PIN_5
#define UART2_WIFI_TX_GPIO_Port GPIOD
#define UART2_WIFI_RX_Pin GPIO_PIN_6
#define UART2_WIFI_RX_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define I2C1_X_SCL_Pin GPIO_PIN_8
#define I2C1_X_SCL_GPIO_Port GPIOB
#define I2C1_X_SDA_Pin GPIO_PIN_9
#define I2C1_X_SDA_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

typedef enum {
	wifi_state_reset = 0,
	wifi_state_ready,
	wifi_state_idle,
	wifi_state_connected,
	wifi_state_connecting,
	wifi_state_socket,
	wifi_state_socket_write,
	wifi_state_disconnected,
	wifi_state_activity,
	wifi_state_inter,
	wifi_state_print_data,
	wifi_state_input_buffer,
	wifi_state_error,
	wifi_undefine_state = 0xFF,
} wifi_state_t;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Private macro ------------------------------------------------------------ */
#ifdef USART_PRINT_MSG

#ifdef USE_STM32L0XX_NUCLEO

#define WIFI_UART_MSG                           USART2
#define USARTx_PRINT_CLK_ENABLE()              __USART2_CLK_ENABLE()
#define USARTx_PRINT_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_PRINT_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USARTx_PRINT_FORCE_RESET()             __USART2_FORCE_RESET()
#define USARTx_PRINT_RELEASE_RESET()           __USART2_RELEASE_RESET()

#define PRINTMSG_USARTx_TX_AF                       GPIO_AF4_USART2
#define PRINTMSG_USARTx_RX_AF                       GPIO_AF4_USART2

#endif //USE_STM32L0XX_NUCLEO

#if defined(USE_STM32F1xx_NUCLEO) || defined(USE_STM32F4XX_NUCLEO) || defined(USE_STM32L4XX_NUCLEO) || defined(USE_STM32F7XX_NUCLEO)

#define WIFI_UART_MSG                           USART3
#define UART_MsgHandle							huart3
#define USARTx_PRINT_CLK_ENABLE()              __HAL_RCC_USART3_CLK_ENABLE()
#define USARTx_PRINT_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()
#define USARTx_PRINT_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()

#define USARTx_PRINT_FORCE_RESET()             __HAL_RCC_USART3_FORCE_RESET()
#define USARTx_PRINT_RELEASE_RESET()           __HAL_RCC_USART3_RELEASE_RESET()

#define PRINTMSG_USARTx_TX_AF                       GPIO_AF7_USART3
#define PRINTMSG_USARTx_RX_AF                       GPIO_AF7_USART3

#endif //(USE_STM32F1xx_NUCLEO) || (USE_STM32F4XX_NUCLEO) || defined(USE_STM32F7XX_NUCLEO)

#define WiFi_USART_PRINT_TX_PIN                    STLK_TX_Pin
#define WiFi_USART_PRINT_TX_GPIO_PORT              GPIOD
#define WiFi_USART_PRINT_RX_PIN                    STLK_RX_Pin
#define WiFi_USART_PRINT_RX_GPIO_PORT              GPIOD

/* Definition for USARTx's NVIC */
#define USARTx_PRINT_IRQn                      USART3_IRQn
#define USARTx_PRINT_IRQHandler                USART3_IRQHandler

#endif //USART_PRINT_MSG

/* Definition for USARTx Pins */
#define WiFi_USART_TX_PIN                    UART2_WIFI_TX_Pin
#define WiFi_USART_TX_GPIO_PORT              GPIOD
#define WiFi_USART_RX_PIN                    UART2_WIFI_RX_Pin
#define WiFi_USART_RX_GPIO_PORT              GPIOD

#define WiFi_USART_RTS_PIN                    GPIO_PIN_12
#define WiFi_USART_RTS_GPIO_PORT              GPIOA
#define WiFi_USART_CTS_PIN                    GPIO_PIN_11
#define WiFi_USART_CTS_GPIO_PORT              GPIOA

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
