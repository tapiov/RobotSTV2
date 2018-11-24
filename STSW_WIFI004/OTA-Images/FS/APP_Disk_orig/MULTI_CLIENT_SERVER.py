# Script Name : MULTI_TCP_SE.py
# 
# Description : Instantiate in parallel two different TCP socket server
import usocket
import utime
from usocket import socket
from pyb import WIND
from network import WLAN

# Before to open socket we must wait connection UP event
while(WLAN().isconnected() == False):
 utime.sleep(1)

# Instantiate Server UDP
s1=socket(usocket.AF_INET, usocket.SOCK_STREAM)
s1.bind(2121) # Bind server 1 PORT
s1.listen(8)
s1.settimeout(1)

i=0

while True:
 try:
  data1=s1.recvfrom(100) # Get incoming message 1
  s1.send('Hi all')
 except:
  pass
