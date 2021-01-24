import cv2
from vision import *
import config
import pyrealsense2 as rs
import numpy as np
import time
from plot import *

ball_co = []
basket_co = []
distance = []

def image_processing():

    global balls_co
    global basket_co
    global distance
        
    uusaeg = 0


    pipeline, align = init_rs()
    detector = init_detector()

    cv2.namedWindow("frame")
    cv2.namedWindow("depth")


    


    while(True):

       

        frames = pipeline.wait_for_frames()
        
        aligned_frames = align.process(frames)
        
        bgr = aligned_frames.get_color_frame()
        
        depth = aligned_frames.get_depth_frame()
        
        
        depth_image = np.asanyarray(depth.get_data())

        if(not depth):
            continue

       

        aeg = time.time()
        fps = str(1 // (aeg - uusaeg))
        uusaeg = time.time()

        bgr = np.asanyarray(bgr.get_data())
        bgr = cv2.rotate(bgr, cv2.ROTATE_180)

        bgr = blur(bgr)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

       

        balls = apply_ball_color_filter(hsv)
        basket = apply_basket_color_filter(hsv)

        ball_co, points = ball_detection(balls, detector)
        basket_co = basket_detection(basket)
        
        
        
        if(basket_co != None):
            middle = basket_co[0] + basket_co[2]/2
        else:
            middle = 0
            
        
        if(middle < 460 and middle > 420):
            basket_d = basket_distance(depth, basket_co)
        else:
            basket_d = 0

        bgr = draw_basket(basket_co, bgr, basket_d)
        bgr = draw_balls(ball_co, bgr, points)

        if(basket_d != 0):
            print(get_speed(basket_d))

        cv2.putText(bgr, fps, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("frame", bgr)
        cv2.imshow("depth", balls)
        
        key = cv2.waitKey(1)

        if key & 0xFF == ord("q"):
            break

    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    image_processing()
