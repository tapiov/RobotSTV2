# Script Name : WLAN.py
# 
# Description : 
import pyb
from pyb import LED
from network import WLAN


# Init Variables
MySsid='AAAMyAp'
myauth=(WLAN.WPA, 'routerpwd16')
found=False
firstTimeConnected=True
firstTimeRouterLost=True
i=0

# INstantiate Objects
l=LED(1) # Instantiate LED obj
w=WLAN() # INstantiate WLAN object

print('INIT')
nets=w.scan() # Get list of all available networks
for net in nets: # Loop on each single network
 if net.ssid==MySsid:  # Check if desired SSID is there
  found=True # Set the status flag as TRUE since network is there
  print('<'+MySsid+'> found: trying to connect...')
  w.init(mode=WLAN.STA, ssid=net.ssid, auth=myauth) # connect to the selected network
  break
i=0
while(i<1000):
 if (w.isconnected()==False and i>10) or (found==False and firstTimeRouterLost==True):
  if firstTimeRouterLost==True:
   print(MySsid+' not available')
   w.init(mode=WLAN.AP, ssid='ApLost', auth=None) # If network is not there the module is reconfigured as MiniAP creating an open network nammed 'ApLost'
   firstTimeRouterLost=False
 else:
  if w.isconnected()==True and firstTimeConnected==True and w.mode()==WLAN.STA:
   print('connected!')
   firstTimeConnected=False
  l.toggle()
 i=i+1
 pyb.delay(1000) # 1s delay

