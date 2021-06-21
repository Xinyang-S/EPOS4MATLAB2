from godirect import GoDirect
import time

godirect = GoDirect(use_ble=False, use_usb=True)
device = godirect.get_device()
#output = []
device.open(auto_start=True)
device.start(period=10)

sensors = device.get_enabled_sensors()
i = 1
while i<100:
    device.read()
    print(sensors[0].values[-1])
    #output.append(sensors[0].values[-1])
    '''f= open("Force.txt","w+")
    #if device.read():
        #print(sensors[0].values[-1])
    f.write(str(sensors[0].values[-1]))
        #sensors[0].clear()
    f.close()'''
    i+=1
    time.sleep(0.001)
device.stop()
device.close()
godirect.quit()
