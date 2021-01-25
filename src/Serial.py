
import serial
from struct import *
import serial.tools.list_ports
from threading import Thread
from TaskFile import *


class SerialClass:

    def __init__(self, port):
        self.ser = serial.Serial(port, baudrate = 115200, timeout = 0.2, write_timeout=1)
        self.speeds = [0, 0, 0, 0]
        self.drive = False
        self.throw = False

    def start(self):
        Thread(target=self.send_speed(),args=()).start()
        return self

    def send_speed(self):

        package = pack('<hhhHH', self.speeds[0], self.speeds[1], self.speeds[2], self.speeds[3], 0xAAAA)
        self.ser.write(package)
        self.ser.read(self.ser.inWaiting())

    def open(self):
        self.ser.open()

    def close(self):
        self.ser.close()

    def get_speeds(self):
        d = self.speeds

    def set_speeds(self, speeds):
        self.speeds = speeds






def uue_serial():

    print(serial.tools.list_ports.comports())

    ser = serial.Serial(port="/dev/ttyACM0", baudrate=115200)

    return  ser




def send_speeds(sisend):
    speeds = sisend[0]
    ser = sisend[1]
    package = pack('<hhhHH', speeds[0], speeds[1], speeds[2], speeds[3], 0xAAAA)
    ser.write(package)
    answer = ser.read(ser.inWaiting())
    return answer
