# Script Name : RL_UDP_CL.py
# 
# Description : a basic script showing how to instantiate create UDP server and wait for incoming client connections
# In particular, connected clients can send commands to the server in order to switch on ond off LED(1) 
# Remote commands handled by the server are:
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
# Description : RemoteLed class is a basic class implementation which will instantiate server
# and handle incoming client connections and messages.
#
#########################
class RemoteLed:
 def __init__(self, myLed, start):
  print('START')
  self.newDataToProcess=0
  self.led=myLed
  self.s=socket(usocket.AF_INET, usocket.SOCK_DGRAM)
  try:
   self.s.bind(2121)
   self.s.listen(8) 
   self.myStart=True
   
  except:
   self.myStart=False
#METHODS
 def wait_client(self):
   try:
    self.s.settimeout(10000)
    self.m = self.s.accept()
    self.conn = self.m[0] 
    self.addr = self.m[1]
    print('Got new client', self.m)
   except:
      print('No Client')
      pass


 def processMsg(self):
  cmd=self.conn.recv(1024)
  print('processing message...' + str(cmd))
  if b'on'==cmd.rstrip():
   self.led.on()
  elif b'off'==cmd.rstrip():
   self.led.off()
  else:
   print('unknown command received')
  self.newDataToProcess=0
  return
  
 def run(self):
  try:
   self.conn.send("\n\rrun")
   pass
  except:
   pass
  if(self.newDataToProcess==1):
   print('process start...')
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
#
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
canContinue=0

# Before to open socket we must wait connection UP event
while(WLAN().isconnected() == False):
 utime.sleep(1)

x=RemoteLed(l, canContinue)
w=WIND()
#MAIN
x.wait_client()
w.callback(cb) 

while x.myStart:
 x.run()
#utime.sleep(2)
print ('ERROR OCCURRED')
