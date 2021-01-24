
import serial

class SerialClass():

    def __int__(self,port = "/dev/ttyACM0"):

        self.ser = serial.Serial(port,baudrate = 115200,timeout = 0)
        self.thrower_speed = 0
        self.speeds = [0,0,0]


    def send_speed(self):
        speed1, speed2, speed3 = self.speeds
        self.ser.write(str(speed1))
        self.ser.write(str(speed2))
        self.ser.write(str(speed3))
        self.ser.write(str(thrower_speed))
        self.ser.write(0xAAAA)
        
    
    
