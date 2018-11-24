# RobotSTCOM
Robot-Wifi-UARTConsole

# AT commands to program Wifi module
AT+S.SCFG=blink_led,0

AT+S.SSIDTXT=<RobotSSID>

AT+S.SCFG=wifi_auth_type,0

AT+S.SCFG=wifi_priv_mode,2

AT+S.SCFG=wifi_wpa_psk_text,<secretkey>

AT+S.SCFG=ip_hostname,<hostname>

AT+S.SCFG=ip_apdomainname,<robot.net>

AT+S.SCFG=ip_apredirect,index.html

AT+S.WCFG

# Will erase data, DO NOT USE IF NOT INTENDED
AT+S.FSUPDATE=i,192.168.0.51,\fs.img,,,,

# Write data to robot.fhtml table
AT+S.INPUTSSI=<string lenght><CR>
|10|20|30.3|40|55|<CR>


