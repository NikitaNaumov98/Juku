
import serial
from struct import *
import serial.tools.list_ports
import threading
from TaskFile import *


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


    def serial_thread(self):
        global var_dict
        while(True):
            try:
                ser_dict = {"s1":0,"s2":0,"s3":0,"t":0,"throw":False}

                ser_dict = get_vars(ser_dict)

                speeds = []

                speeds.append(ser_dict["s1"])
                speeds.append(ser_dict["s2"])
                speeds.append(ser_dict["s3"])

                if(ser_dict["throw"]):
                    speeds.append(ser_dict["t"])
                else:
                    speeds.append(0)

                self.set_speeds(speeds)

                self.send_speed()
                self.read()
            except KeyboardInterrupt:
                self.close()


def uue_serial():

    print(serial.tools.list_ports.comports())

    ser = serial.Serial(port="/dev/ttyACM0", baudrate=115200)

    return  ser




def send_speed(speeds, ser):
    package = pack('<hhhHH', speeds[0], speeds[1], speeds[2], speeds[3], 0xAAAA)
    ser.write(package)
    ser.read(ser.inWaiting())
