import struct

from godirect import GoDirect
from datetime import datetime
import logging
import csv
import socket
import sockets
logging.basicConfig()

# The first USB device found will be used. If no USB devices are found, then
# the BLE device with the strongest signal over -100dB is used.
# Note that you can choose to enable USB, BLE, or both. By default both will be enabled.
godirect = GoDirect(use_ble=True, use_usb=True)
device = godirect.get_device(threshold=-100)
output = []

godirect.quit()

my_socket= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
my_socket.connect(('127.0.0.1', 1249))
#MESSAGE= b'5.99785'



if device != None and device.open(auto_start=False):
    device.start(period=10)

    # You can select the specific sensors for data collection using device.enable_sensors().
    # Otherwise, the default sensors will be used when device.get_enabled_sensors() is called.
    # device.enable_sensors([2,3,4])
    sensors = device.get_enabled_sensors()
    j = 1
    startTime = datetime.now()
    while 1:
        if device.read():
            for sensor in sensors:
                #Time1 = datetime.now()
                # The sensor.values call may read one sensor value, or multiple sensor values (if fast sampling)
                for i in sensor.values:
                    #print(str(sensor.values[i]))
                    print(i)
                    MESSAGE = struct.pack('f', i)
                    #my_socket.sendto(MESSAGE, ('146.169.185.235', 1250))
                    my_socket.sendto(MESSAGE, ('127.0.0.1', 1250))
                    output.append(i)
                sensor.clear()
                #Time2 = datetime.now()
                #print(Time2 - Time1)
            j += 1
    my_socket.close
    EndTime = datetime.now()
    device.stop()
    device.close()
    print(len(output))
    print(EndTime - startTime)
else:
    print("Go Direct device not found/opened")


