from Movement import *
from Serial import *
import serial.tools.list_ports

ser = SerialClass("").start()

ser.send_speed([10,10,10,0])

time.sleep(1)

ser.set_speed([0,0,0,0])


