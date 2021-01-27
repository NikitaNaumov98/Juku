from Movement import *
import config
import serial
import workers
from imageprocessing import *
import time
#from imageprocessing import Camera
from multiprocessing import Process, Lock, Array, shared_memory, Manager
import sys
import cv2
from struct import *
sys.path.append('..')


ser = serial.Serial("/dev/ttyACM0", baudrate = 115200, timeout = 2, write_timeout = 2)

lock = Lock()

def pid(x,integral,p,i,d,mid,old):

    error = mid - x
    integral += error
    return int(p*error+i*integral+d*(old - error)), error

def start_p(pros_val,speeds):
    run = workers.Value("i",1)
    robot_vision = workers.Process(name="Vision",target=workers.vision_worker,args=(run,pros_val,speeds,lock))
    image_processor = workers.Process(name="Processor",target=workers.processing_worker,args=(run,pros_val,speeds))
    return robot_vision, run, image_processor

integ = 0
integ1 = 0

stage = 0
if __name__ == "__main__":

    korv = basketDetect().start()

    speeds = [0,0,0,0]

    time.sleep(2)
    print(config.get("vision", "basket_color"))
    print(config.get('threshold', "thresh"))

    manager = Manager()
    speeds = manager.list([0,0,0,0])
    pros_val = manager.list([0,0,0,0,0,0,0])

    robot_vis, run ,robot_processor = start_p(pros_val,speeds)
    robot_vis.start()
    robot_processor.start()

    while(True):


        print(speeds)
        #cv2.imshow("aa",pros_val[4])
        ser.write(pack("<hhhHH",speeds[0],speeds[1],speeds[2],speeds[3],0xAAAA))
        print(pros_val[4])

        key = cv2.waitKey(1)
        if key & 0xFF == ord("q"):
            time.sleep(1)
            run = 0
            ser.write(pack("<hhhHH",0,0,0,0,0xAAAA))

            robot_vis.close()
            robot_vis.terminate()
            time.sleep(1)
            ser.close()
            cv2.destroyAllWindows()
            break



