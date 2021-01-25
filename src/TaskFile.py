from Movement import *
from Serial import *
from imageprocessing import *
from imageprocessing import Camera
from multiprocessing import Process, Lock, Array, shared_memory
import sys
import cv2
sys.path.append('..')



if __name__ == "__main__":


    video = Camera().start()
    pilt = video.cam
    depth = video.depth
    pildiprotsess = ImageProcessing(pilt,depth).start()
    time.sleep(2)


    while(True):
        pilt = video.cam
        pildiprotsess.set_frame(pilt)
        thower = pildiprotsess.thrower_speed
        balls = pildiprotsess.ball_co
        basket = pildiprotsess.basket_co
        processed = pildiprotsess.balls
        cv2.imshow("test",processed)

        key = cv2.waitKey(1)

        if key & 0xFF == ord("q"):
            break


        

