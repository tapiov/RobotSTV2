//
// File: wifi_ssi.c - Wifi SSI functions
//

//	char AT_Str[400];
//	char Value_Str[400];
//	char AT_Rpl[400];
//
//#ifdef USART_PRINT_MSG
//	// UART_Msg_Gpio_Init();
//#if defined (USE_STM32L0XX_NUCLEO)
//	USART_PRINT_MSG_Configuration(&UART_MsgHandle, 115200); //L0 max rate?
//#else
//	USART_PRINT_MSG_Configuration(&huart3, 115200);
//#endif
//	Set_UartMsgHandle(&huart3); //this is required for the console handler initialization
//#endif
//
//	printf("\r\n Robot console starting \r\n");
//
//	printf("\r\n Please wait... \r\n");
//
//
//#if defined (USE_STM32L0XX_NUCLEO)
//	autodetect = WiFi_Module_UART_Configuration(115200); //115200 //460800 //921600
//#else
//	// autodetect = WiFi_Module_UART_Configuration(115200); //115200 //460800 //921600
//#endif
//
//	// WiFi_Module_UART_Configuration(115200); //115200 //460800 //921600
//	USART2->CR3 |= 0x00000040;
//
//	autodetect = HAL_OK;
//
////	if (autodetect == HAL_OK)
////		UART_DMA_Init();
////	else {
////		printf("\rError in baud-rate auto-detection...\r\n");
////	}
//
//	HAL_Delay(2000);
//	printf("\r\n Console Ready... \r\n");
//
//	//	AT commands to program Wifi module
//	//	AT+S.SCFG=blink_led,0
//	//	AT+S.SSIDTXT=<RobotSSID>
//	//	AT+S.SCFG=wifi_auth_type,0
//	//	AT+S.SCFG=wifi_priv_mode,2
//	//	AT+S.SCFG=wifi_wpa_psk_text,<secretkey>
//	//	AT+S.SCFG=ip_hostname,<hostname>
//	//	AT+S.SCFG=ip_apdomainname,<robot.net>
//	//	AT+S.SCFG=ip_apredirect,index.html
//	//	AT+S.WCFG
//	//
//	//	## Will erase data, DO NOT USE IF NOT INTENDED
//	//	## AT+S.FSUPDATE=i,192.168.0.51,\fs.img,,,,
//	//
//	//	## Write data to robot.fhtml table
//	//	AT+S.INPUTSSI=<strlen><CR>
//	//	|10|20|30.3|40|55|<CR>
