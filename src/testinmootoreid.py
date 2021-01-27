from Movement import *
from Serial import *
import serial.tools.list_ports
import time
ser = serial.Serial("/dev/ttyACM0", baudrate = 115200, timeout = 2, write_timeout = 2)

while(True):
    try:
        ser.write(pack("<hhhHH",0,0,0,0,0xAAAA))
        time.sleep(2)
        ser.write(pack("<hhhHH", 10, 0, 0, 0 ,0xAAAA))
        time.sleep(1)
        ser.write(pack("<hhhHH", 0, 10, 0, 0,0xAAAA))
        time.sleep(1)
        ser.write(pack("<hhhHH", 0, 0, 10, 10,0xAAAA))
        time.sleep(1)
    except KeyboardInterrupt:
        ser.close()
        break
