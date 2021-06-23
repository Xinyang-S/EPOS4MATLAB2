import struct
from datetime import datetime
import csv
import socket
import sockets
import numpy as np
import cv2

# The first USB device found will be used. If no USB devices are found, then
# the BLE device with the strongest signal over -100dB is used.
# Note that you can choose to enable USB, BLE, or both. By default both will be enabled.
output = []

my_socket= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
my_socket.connect(('127.0.0.1', 1249))
#MESSAGE= b'5.99785'
force_single = [1.23112312, 123.123123212312, -2.1231231213, 23.232342452, 34.3243953456]
force_double = [1.2311231234965874948345, 123.1231232123123469048539240, -2.123123121334594850394, 23.232342452304958034, 34.32439520349850393456, 56.232323045984302944354]
force_test = np.arange(1.0, 50.0, 1).tolist()
# You can select the specific sensors for data collection using device.enable_sensors().
# Otherwise, the default sensors will be used when device.get_enabled_sensors() is called.
# device.enable_sensors([2,3,4])
j = 1
startTime = datetime.now()
looptime = datetime.now()
while 1:

    # The sensor.values call may read one sensor value, or multiple sensor values (if fast sampling)
    '''print(str(sensor.values))
    output.append(sensor.values)
    sensor.clear()'''
    looptime = datetime.now()
    for i in force_test:
        #print(str(sensor.values[i]))
        print(i)
        MESSAGE = struct.pack('f', i)
        #my_socket.sendto(MESSAGE, ('146.169.185.235', 1250))
        my_socket.sendto(MESSAGE, ('127.0.0.1', 1250))
        output.append(i)
        '''f = open("Force.txt", "w+")
        f.write(str(i))
        f.close()'''
        #cv2.waitKey(1)
    j += 1
my_socket.close
EndTime = datetime.now()
print(len(output))
print(EndTime - startTime)
print(j)