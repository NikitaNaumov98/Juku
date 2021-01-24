from Movement import *
from Serial import *
from imageprocessing import *
import threading
from controllermovement import *
import sys
sys.path.append('..')




if __name__ == "__main__":

    pipeline, align, detector = init_image()
    ser = uue_serial()

    while(True):
        pall, korv = image_processing(pipeline, align, detector)
        direction = calculate_movement_direction(pall[0],pall[1],420)

        if(pall[1] > 700):
            input("kaugel on" )

        else:
            speeds = [0,0,0,0]
            speeds = omnimovement(speeds, 50, firstwheelangle, secondwheelangle, thirdwheelangel,direction)

        send_speed(speeds,ser)

