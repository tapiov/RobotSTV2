/**
 ******************************************************************************
 * @file    wifi_const.h
 * @author  Central LAB
 * @version V2.0.1
 * @date    17-May-2016
 * @brief   Describes the constants and defines in X-CUBE-WIFI1
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/** @defgroup NUCLEO_WIFI_INTERFACE_Private_Defines
  * @{
  */

#define MAX_POP_SIZE                            200
/* AT Commands specifc to SPWF01 */
#ifdef SPWF01
#define AT_RESP                                 "\r\nOK\r\n"    
#define AT_RESP_LEN_OK                          6 //\r\nOK\r\n
#define AT_RESTORE_DEFAULT_SETTING              "AT&F\r"
#define AT_SAVE_CURRENT_SETTING                 "AT&W\r"
#define AT_WiFi_SCAN                            "AT+S.SCAN\r"
#define AT_SOCKET_OPEN                          "AT+S.SOCKON=%s,%d,%s,ind\r"
#define AT_SERVER_SOCKET_OPEN                   "AT+S.SOCKD=%d,%s,ind\r" //with indication option
#define AT_SERVER_SOCKET_CLOSE                  "AT+S.SOCKD=0\r"
#define AT_FWUPDATE                             "AT+S.FWUPDATE=%s,/%s,%d\r"
#define AT_HTTPGET_REQUEST                      "AT+S.HTTPGET=%s,%s,%d\r"
#define AT_HTTPPOST_REQUEST                     "AT+S.HTTPPOST=%s,%s,%d\r"
#define AT_PING                                 "AT+S.PING=%s\r"
#define AT_SETTIME                              "AT+S.SETTIME=%lu\r"
#define AT_DISPLAY_FILE_CONTENT                 "AT+S.FSP=/%s\r"
#define AT_SET_POWER_STATE                      "AT+CFUN=%d\r"
#define AT_DOWNLOAD_IMAGE_FILE                  "AT+S.HTTPDFSUPDATE=%s,/%s,%d\r"
#define AT_CLEAN_TLS_CERT                       "AT+S.TLSCERT2=clean,all\r"
#define AT_STORE_CA_CERT                        "AT+S.TLSCERT=f_ca,%d\r%s"
#define AT_STORE_CERT                           "AT+S.TLSCERT=f_cert,%d\r%s"
#define AT_STORE_KEY                            "AT+S.TLSCERT=f_key,%d\r%s"
#define AT_SET_CA_DOMAIN_NAME                   "AT+S.TLSDOMAIN=f_domain,%s\r"
#define AT_RESET                		"AT+CFUN=0\r"


/* AT Commands specifc to SPWF04 */
#elif defined SPWF04
#define AT_RESP                                 "AT-S.OK:"
#define AT_RESP_LEN_OK                          8 //AT-S.OK:
#define AT_RESTORE_DEFAULT_SETTING              "AT+S.FCFG\r"
#define AT_SAVE_CURRENT_SETTING                 "AT+S.WCFG\r"
#define AT_RESET                                "AT+S.RESET\r"
#define AT_WiFi_SCAN                            "AT+S.SCAN=s,\r" //d->no filter, s-> filter on ssid
#define AT_SOCKET_OPEN                          "AT+S.SOCKON=%s,%d,NULL,%s\r"
#define AT_WEB_SOCKET_OPEN                      "AT+S.WSOCKON=%s,%d,,0,,,,,\r" //no TLS now
#define AT_WEB_SOCKET_QUERY                     "AT+S.WSOCKQ=%d\r"
#define AT_WEB_SOCKET_READ                      "AT+S.WSOCKR=%d,%d\r" //2nd param:0->read the full buffer
#define AT_WEB_SOCKET_WRITE                     "AT+S.WSOCKW=%d,0,0,0,%d\r"//all default
#define AT_WEB_SOCKET_CLOSE                     "AT+S.WSOCKC=%d,0\r"//0->Normal Closure, 1-> Go away
#define AT_SERVER_SOCKET_OPEN                   "AT+S.SOCKDON=%d,%s\r"
#define AT_SERVER_SOCKET_QUERY                  "AT+S.SOCKDQ=%d,%d\r"
#define AT_SERVER_SOCKET_READ                   "AT+S.SOCKDR=%d,%d,%d\r"
#define AT_SERVER_SOCKET_WRITE                  "AT+S.SOCKDW=%d,%d,%d\r"
#define AT_SERVER_SOCKET_CLOSE                  "AT+S.SOCKDC=%d\r"//disconnect all clients
#define AT_SPECIFIC_CLIENT_ON_SERVER_CLOSE      "AT+S.SOCKDC=%d,%d\r"  //disconnect only a specific client socket
#define AT_FWUPDATE                             "AT+S.FWUPDATE=e,%s,/%s,%d,0,,\r"
#define AT_HTTPGET_REQUEST                      "AT+S.HTTPGET=%s,%s,%d,0,,,,\r"
#define AT_HTTPPOST_REQUEST                     "AT+S.HTTPPOST=%s,%s,%d,0,,,,\r"
#define AT_TFTPGET_REQUEST                      "AT+S.TFTPGET=%s,%d,%s,\r"
#define AT_TFTPPUT_REQUEST                      "AT+S.TFTPPUT=%s,%d,%s\r"    
#define AT_SETTIME                              "AT+S.TIME=%lu\r"
#define AT_DISPLAY_FILE_CONTENT                 "AT+S.FSP=/%s,,\r"
#define AT_RENAME_FILE                          "AT+S.FSR=/%s,%s\r"
#define AT_UNMOUNT_USER_MEMORY                  "AT+S.FSU=%d,%d\r"
#define AT_PING                                 "AT+S.PING=1,56,%s\r"
#define AT_SET_POWER_STATE                      "AT+S.PMS=%d\r"
#define AT_DOWNLOAD_IMAGE_FILE                  "AT+S.FSUPDATE=e,%s,/%s,%d,,,\r"
#define AT_CLEAN_TLS_CERT                       "AT+S.TLSCERT=content,2\r"
#define AT_LIST_TLS_CERT                        "AT+S.TLSCERT=content,1\r"
#define AT_STORE_CA_CERT                        "AT+S.TLSCERT=Ca,%d\r%s"
#define AT_STORE_CERT                           "AT+S.TLSCERT=cert,%d\r%s"
#define AT_STORE_KEY                            "AT+S.TLSCERT=key,%d\r%s"
#define AT_STORE_AUTH_KEY                       "AT+S.TLSCERT=Auth,%d\r%s"
#define AT_MQTT_CONN                            "AT+S.MQTTCONN=%s,%d,,,,,,,,,,\r"
#define AT_MQTT_SUB                             "AT+S.MQTTSUB=0,%s,\r"
#define AT_MQTT_PUB                             "AT+S.MQTTPUB=0,%s,,,%d\r"
#define AT_MQTT_DISC                            "AT+S.MQTTDISC=0\r"
#define AT_MQTT_UNSUB                           "AT+S.MQTTUNSUB=0,%s\r"
#define AT_SEND_MAIL                            "AT+S.SMTP=%s,%d,,,,,%s,%s,,,%s,,%d\r"
#define AT_LIST_OPEN_CLIENT_SOCKET              "AT+S.SOCKL\r"
#define AT_LIST_OPEN_WEB_CLIENT_SOCKET          "AT+S.WSOCKL\r"
#define AT_LIST_BOUND_CLIENT_SOCKET             "AT+S.SOCKDL\r"
#define AT_LIST_CLIENTS_ON_SPECIFIC_SERVER      "AT+S.SOCKDL=%d\r"    
#define AT_FILL_INTERNAL_BUFFER                 "AT+S.INPUTSSI=%d\r"
#endif

/* Common AT Commands */
#define EPOCH_TIME                              1453727657//Human time (GMT): Mon, 25 Jan 2016 13:14:17 GMT
#define EXTI_CONF_TIMER                         1900 //millisec
#define PROCESS_WIFI_TIMER                      1
#define SLEEP_RESUME_PREVENT                    2000
#define MIDDLEWARE_VERSION                      "3.1.1"
#if defined (USE_STM32L0XX_NUCLEO) && defined (WIFI_USE_VCOM)    
#define RINGBUF_SIZE                            4096
#else
#define RINGBUF_SIZE                            1024
#endif

#if defined(USE_STM32F4XX_NUCLEO) || defined(USE_STM32L4XX_NUCLEO)
#define DMA_BUFFER_SIZE                         2048//8192
#define DMA_BUFFER_END_INDEX                    2047//8191
#else
#define DMA_BUFFER_SIZE                         1
#define DMA_BUFFER_END_INDEX                    1
#endif
#define MAX_BUFFER_GLOBAL                       512
#define MAX_WIFI_SCAN_NETWORK                   256
#define RxBufferSize                            64
#define AT_RESP_LEN_GPIOR                       21//GPIO n = 0,in\r\n\r\nOK\r\n
#define AT_RESP_HELP_TEXT_LEN                   512
#define AT_ATTENTION_LEN                        5 //\r\nOK\r\n   
#define AT_ATTENTION                            "AT\r"
#define AT_GET_CONFIGURATION_VALUE              "AT+S.GCFG=%s\r"
#define AT_SET_CONFIGURATION_VALUE              "AT+S.SCFG=%s,%d\r"
#define AT_SET_CONFIGURATION_ADDRESS            "AT+S.SCFG=%s,%s\r"
#define AT_GET_STATUS_VALUE                     "AT+S.STS=%s\r"
#define AT_WIFI_ENABLE                          "AT+S.WIFI=%d\r"
//#define AT_TLSCERT                              "AT+S.TLSCERT=%s,%d\r%s"

//#define AT_GET_SSID                           "AT&F\r"
#define AT_SET_SSID                             "AT+S.SSIDTXT=%s\r"
#define AT_SET_SEC_KEY                          "AT+S.SCFG=wifi_wpa_psk_text,%s\r"
#define AT_HELP_TEXT                            "AT+S.HELP\r"
#define AT_RESET_MSG                            "\r\n+WIND:2:Reset\r\n"
#define UNDEFINE_LENGTH                         0xFFFF
#define AT_SOCKET_WRITE                         "AT+S.SOCKW=%d,%d\r"
#define AT_SOCKET_READ                          "AT+S.SOCKR=%d,%d\r"
#define AT_QUERY_PENDING_DATA                   "AT+S.SOCKQ=%d\r"
#define AT_SOCKET_CLOSE                         "AT+S.SOCKC=%d\r"
#define AT_DISPLAY_FILENAME_LIST                "AT+S.FSL\r"

#define AT_CREATE_FILE                          "AT+S.FSC=/%s,%d\r"
#define AT_APPEND_FILE                          "AT+S.FSA=/%s,%d\r"
#define AT_DELETE_FILE                          "AT+S.FSD=/%s\r"
#define AT_ERASE_FLASH_MEMORY                   "AT+S.HTTPDFSERASE\r"
#define AT_CMD_TO_DATA_MODE                     "AT+S.\r"
#define AT_DATA_TO_CMD_MODE                     "at+s." /* NOT \r */
#define AT_HTTPD                                "AT+S.HTTPD=%d\r"
#define	SPI_FIFO_RX_DEPTH	                			4 
    
/************Wi-Fi Config Variables**************/
    
/* Wi-Fi Config Variables specific to SPWF01 */
#ifdef SPWF01
#define LOCALECHO1                               "localecho1"
#define CONSOLE1_HWFC                            "console1_hwfc"
#define CONSOLE1_SPEED                           "console1_speed"
#define WIFI_IP_DNS                              "ip_dns"
#define IP_USE_DHCP_SERVER                       "ip_use_dhcp"

#elif defined SPWF04
#define CONSOLE_ENABLED                          "console_enabled"
#define LOCALECHO1                               "console_echo"
#define CONSOLE1_HWFC                            "console_hwfc"
#define CONSOLE1_SPEED                           "console_speed"
#define WIFI_IP_DNS1                             "ip_dns1"
#define WIFI_IP_DNS2                             "ip_dns2"
#define IP_USE_DHCP_SERVER                       "ip_use_dhcpc"
#endif

#define BLINK_LED                               "blink_led"
#define WIFI_PRIV_MODE                          "wifi_priv_mode"

#define IP_USE_HTTPD                            "ip_use_httpd"
#define WIFI_MODE                               "wifi_mode"
#define WIFI_WPA_SECURITY                       "wifi_wpa_psk_text"  
#define WIFI_CHANNEL_NUMBER                     "wifi_channelnum"  
#define WIFI_IP_ADDRESS                         "ip_ipaddr"
#define WIFI_IP_DEFAULT_GATEWAY                 "ip_gw"

#define WIFI_IP_NETMASK                         "ip_netmask"
#define WIFI_IP_HOSTNAME                        "ip_hostname"
#define WIFI_IP_APDOMAINNAME                    "ip_apdomainname"
#define WIFI_IP_APREDIRECT                      "ip_apredirect"
#define WIFI_IP_HTTP_TIMEOUT                    "ip_http_get_recv_timeout"
#define WIFI_IP_DHCP_TIMEOUT                    "ip_dhcp_timeout"
#define WIFI_REG_COUNTRY                        "wifi_reg_country"

#define WIFI_SLEEP_ENABLED                      "sleep_enabled"
#define WIFI_HT_MODE                            "wifi_ht_mode"
#define WIFI_OPR_RATE_MASK                      "wifi_opr_rate_mask"
#define WIFI_POWERSAVE                          "wifi_powersave"
#define WIFI_OPERATIONAL_MODE                   "wifi_operational_mode"
#define WIFI_LISTEN_INTERVAL                    "wifi_listen_interval"
#define WIFI_BEACON_WAKEUP                      "wifi_beacon_wakeup"
#define WIFI_STANDBY_ENABLED                    "standby_enabled"
#define WIFI_STANDBY_TIME                       "standby_time" 
#define WIFI_TX_POWER                           "wifi_tx_power"
#define WIFI_IBSS_IP_ADDR                       "192.168.2.100"
#define WIFI_IBSS_DEFAULT_GATEWAY               "192.168.2.1"
#define WIFI_IBSS_IP_DNS_ADDR                   "192.168.2.1"
#define WIFI_IBSS_IP_MASK                       "255.255.255.0"
#define WIFI_IP_USE_DHCP                        0

/**
  * @}
  */
