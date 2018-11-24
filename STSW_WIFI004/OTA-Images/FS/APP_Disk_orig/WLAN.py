import pyb
from pyb import LED
from network import WLAN
l=LED(1)
w=WLAN()
MySsid='AAAMyAp'
found=False
firstTimeConnected=True
firstTimeRouterLost=True
i=0
def flashingDelay():
 j=0
 while(j<50):
  pyb.delay(100)
  l.toggle()
  j=j+1
flashingDelay()
w.init(mode=WLAN.STA, ssid='IoT')
print('INIT')
flashingDelay()
nets=w.scan()
for net in nets:
 if net.ssid==MySsid:
  found=True
  print('<'+MySsid+'> found: trying to connect...')
  w.init(mode=WLAN.STA, ssid=net.ssid)
  flashingDelay()
  break
i=0
while(i<1000):
 if (w.isconnected()==False and i>10) or (found==False and firstTimeRouterLost==True):
  if firstTimeRouterLost==True:
   print(MySsid+' lost os not available')
   w.init(mode=WLAN.AP, ssid='ApLost')
   firstTimeRouterLost=False
 else:
  if w.isconnected()==True and firstTimeConnected==True and w.mode()==WLAN.STA:
   firstTimeConnected=False
  l.toggle()
 i=i+1
 pyb.delay(1000)
