
import serial
from struct import *
import serial.tools.list_ports
import threading
from TaskFile import *

lock = threading.Lock()

class SerialClass():

    def __init__(self, port):
        self.ser = serial.Serial(port, baudrate = 115200, timeout = 0.2, write_timeout=1)
        self.speeds = [0, 0, 0, 0]
        self.drive = False
        self.throw = False


    def send_speed(self):
        speeds = self.speeds
        package = pack('<hhhHH', speeds[0], speeds[1], speeds[2], speeds[3], 0xAAAA)
        self.ser.write(package)

    def open(self):
        self.ser.open()

    def close(self):
        self.ser.close()

    def get_speeds(self):
        d = self.speeds

    def set_speeds(self, speeds):
        self.speeds = speeds

    def read(self):
        self.ser.read(self.ser.inWaiting())


def serial_thread():
    global var_dict

    ser_dict = {}

    set_dict = get_vars(ser_dict)

    ser = SerialClass()

    speeds = []
    speeds.append(ser_dict["s1"])
    speeds.append(ser_dict["s2"])
    speeds.append(ser_dict["s3"])

    if(ser_dict["throw"]):
        speeds.append(ser_dict["t"])
    else:
        speeds.append(0)

    ser.set_speeds(speeds)

    ser.send_speed()
    ser.read()




