from godirect import GoDirect
from datetime import datetime
import logging
import csv

logging.basicConfig()

# The first USB device found will be used. If no USB devices are found, then 
# the BLE device with the strongest signal over -100dB is used.
# Note that you can choose to enable USB, BLE, or both. By default both will be enabled.
godirect = GoDirect(use_ble=True, use_usb=True)
device = godirect.get_device(threshold=-100)
output = []
# Once a device is found or selected it must be opened. By default, only information will be 
# gathered on Open. To automatically enable the default sensors and start measurements send 
# auto_start=True and skip to get enabled sensors.

if device != None and device.open(auto_start=False):
    device.start(period=10)

    # You can select the specific sensors for data collection using device.enable_sensors().
    # Otherwise, the default sensors will be used when device.get_enabled_sensors() is called.
    # device.enable_sensors([2,3,4])
    sensors = device.get_enabled_sensors()
    j = 1
    startTime = datetime.now()
    while j<100:
        if device.read():

            for sensor in sensors:
                # The sensor.values call may read one sensor value, or multiple sensor values (if fast sampling)
                '''print(str(sensor.values))
                output.append(sensor.values)
                sensor.clear()'''
                for i in sensor.values:
                    #print(str(sensor.values[i]))
                    print(i)
                    output.append(i)
                    f = open("Force.txt", "w+")
                    f.write(str(i))
                    f.close()
            sensor.clear()
            j+=1
    EndTime = datetime.now()
    device.stop()
    device.close()
    print(len(output))
    print(EndTime - startTime)
else:
    print("Go Direct device not found/opened")

godirect.quit()

'''with open('Force_CSV.csv', mode='w') as Force_CSV:
    Force_CSV = csv.writer(Force_CSV, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    Force_CSV.writerow([i])'''