from godirect import GoDirect
import time

godirect = GoDirect(use_ble=False, use_usb=True)
device = godirect.get_device()

device.open(auto_start=False)
device.start(period=100)

sensors = device.get_enabled_sensors()
i = 1
while i:
    f= open("Force.txt","w+")
    if device.read():
        print(sensors[0].values[-1])
        f.write(str(sensors[0].values[-1]))
        sensors[0].clear()
    f.close()
    time.sleep(0.07)
device.stop()
device.close()
godirect.quit()