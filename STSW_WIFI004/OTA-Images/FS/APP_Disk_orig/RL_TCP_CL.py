# Script Name : LED_TCP_CL.py
# 
# Description : a basic script showing how to instantiate a connection with a remote TCP server handling
# a bidirectional communication.
# In particular, client once connected waits for incoming messages from server. 
# Remote commands handled by the client are:
# on - to switch on LED(1)
# off - to switch off LED(1)
from pyb import WIND
import usocket
from usocket import socket
import utime
from pyb import LED
from network import WLAN

#########################
# Class Name : RemoteLed
#
# Description : RemoteLed class is a basic class implementation which will instantiate the communication with remote server
# and handle incoming messages.
#
#########################
class RemoteLed:
 def __init__(self, myLed, start):
  print('START')
  self.newDataToProcess=0
  self.led=myLed
  self.s=socket(usocket.AF_INET, usocket.SOCK_STREAM)
  try:
   self.s.connect(('192.168.1.131', 2121))
   self.myStart=True
  except:
   self.myStart=False
 def processMsg(self):
  print('processing message...')
  data=self.s.recv(1024)
  data=data.rstrip()
  if b'on'==data:
   self.led.on()
  elif b'off'==data:
   self.led.off()
  else:
   print('unknown command received')
  self.newDataToProcess=0
  return
 def run(self):
  print("run" + str(self.newDataToProcess))
  self.s.send("\n\rRUN:")
  if(self.newDataToProcess==1):
   self.processMsg()
  return

#########################
# Function Name : cb
#
# Description : callback implementation to handle incoming event. All the incoming events on SPWF module are handled 
# as indication events. When incoming data are available WIND:55 is triggered. In order to handle this the user MUST 
# register a callback in order to get notified when new incoming data are available. In the example below this just 
# change the state of a binary variable (newDataToProcess). Main task loop will than read received data when 
# newDataToProcess is 1 otherwise waits for incoming data.
#########################
def cb():
 try:
  params=w.params()
  print(params)
  if int(params[0]) == 55:
   x.newDataToProcess=1
   print('new data')
  else:
   print('Indication '+str(w.code())+' occurred')
 except:
  print("error")
 return

#########################
# Init Main Program
#########################
l=LED(1)
l.on()
canStart=0

# Before to open socket we must wait connection UP event
while(WLAN().isconnected() == False):
 utime.sleep(1)

x=RemoteLed(l, canStart)
w=WIND()
w.callback(cb)
while x.myStart:
 x.run()
 utime.sleep(2)
print ('ERROR OCCURRED')
