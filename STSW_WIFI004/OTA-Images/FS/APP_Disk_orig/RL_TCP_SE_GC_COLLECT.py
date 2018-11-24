# Script Name : RL_TCP_SRV_GC_COLLECT.py
# 
# Description : a basic script showing how to instantiate create TCP server and wait for incoming client connections
# In particular, connected clients can send commands to the server in order to switch on ond off LED(1) 
# Remote commands handled by the server are:
# on - to switch on LED(1)
# off - to switch off LED(1)
# Also the script runs garbage collection every time memory free available is lower than 25000 bytes
#
# NOTE: for more details on garbage colelction in MicroPython and how to use it properly into your script
# please, refer you to application note and on line documentation on micropython.org
from pyb import WIND
from usocket import socket
import utime, gc, machine
from pyb import LED
#########################
# Class Name : RemoteLed
#
# Description : RemoteLed class is a basic class implementation which will instantiate server
# and handle incoming client connections and messages.
#
#########################
class RemoteLed:
 def __init__(self, myLed, start):
  self.newDataToProcess=0
  self.led=myLed
  self.s=socket(socket.AF_INET, socket.SOCK_STREAM)
  try:
   self.s.bind(2121)
   self.myStart=True
  except:
   self.myStart=False
#METHODS
 def processMsg(self):
  tuple_data=self.s.recvfrom(1024)
  cmd=tuple_data[0].rstrip()
  if b'on'==cmd:
   self.led.on()
  elif b'off'==cmd:
   self.led.off()
  else:
   print ('unknown command received')
   print(cmd)
  self.newDataToProcess=0
  return
 def run(self):
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
#
#########################
def cb():
 try:
  print ('data rx...')
  if(w.code()==55):
   x.newDataToProcess=1
 except:
  print ('error on callback execution')
 return

#########################
# Init Main Program
#########################
l=LED(1)
l.on()
canContinue=0
x=RemoteLed(l, canContinue)
w=WIND()
w.callback(cb)
iter=0
while x.myStart:
 utime.sleep(0.2)
 iter=iter+1
 if(iter%2==0):
  x.run()
  i=gc.mem_free()
  print(i)
  print(str(iter))
  if(i<=25000):
   print ('trying to free memory...')
   gc.collect()
   print(gc.mem_free())
print ('ERROR OCCURRED')
