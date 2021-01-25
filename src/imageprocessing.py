import cv2
from vision import *
import config
import pyrealsense2 as rs
import numpy as np
import time
from plot import *
from threading import Thread
from TaskFile import *


frame_buffer = []


class VideoGet:
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread.
    """

    def __init__(self, src=4):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.stream.set(cv2.CAP_PROP_EXPOSURE, 120.0)
        self.stream.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.stream.set(cv2.CAP_PROP_WB_TEMPERATURE, 4300)
        self.stream.set(cv2.CAP_PROP_SATURATION, 80)
        self.stream.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 100)
        self.stream.set(cv2.CAP_PROP_FPS, 60)
        self.stream.set(3, 1280)
        self.stream.set(4, 720)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()


    def stop(self):
        self.stopped = True

class Camera:

    def __init__(self):
        self.pipeline , self.align = init_rs()
        self.cam = None
        self.frame = self.pipeline.wait_for_frames()
        self.depth = None
        self.stream = True

    def start(self):
        Thread(target=self.get_color_frame,args=()).start()
        return self

    def get_color_frame(self):

        while(self.stream):
            self.frame = self.pipeline.wait_for_frames()
            vahe = self.frame.get_color_frame()
            bgr = np.asanyarray(vahe.get_data())
            self.cam = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            self.depth = self.frame.get_depth_frame()
        else:
            self.pipeline.stop()
            cv2.destroyAllWindows()
            quit()

    def stop(self):
        self.stream = False

    def get_frame(self):
        return  np.asanyarray(self.frame.get_data())



class ImageProcessing:

    def __init__(self,frame,dframe):
        self.processing = True
        self.image = frame
        self.depth_frame = dframe
        self.thrower_speed = None
        self.ball_co = None
        self.balls = None
        self.basket = None
        self.basket_co = None

    def start(self):
        Thread(target=self.process,args=(),daemon=True).start()
        return self

    def get_balls(self):
        return self.balls

    def process(self,):
        detector = init_detector()
        while(self.processing):

            bgr = self.image
            if(bgr != None):
                bgr = cv2.rotate(bgr, cv2.ROTATE_180)
                bgr = blur(bgr)

                hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                self.balls = apply_ball_color_filter(hsv)
                self.basket = apply_basket_color_filter(hsv)

                self.ball_co, points = ball_detection(balls, detector)
                self.basket_co = basket_detection(basket)

                if (basket_co != None):
                    middle = basket_co[0] + basket_co[2] / 2
                else:
                    middle = 0


                if (middle < 460 and middle > 420):
                    distance = basket_distance(self.depth_frame, basket_co)

                else:

                    distance = 0

                bgr = draw_basket(basket_co, bgr, distance)
                bgr = draw_balls(self.ball_co, bgr, points)

                if (distance != 0):
                    self.thrower_speed = get_speed(distance)



    def set_frame(self,frame):
        self.image = frame

    def get_frame(self):
        return self.image

    def get_speed(self):
        return self.thrower_speed

def take_sec(elem):
    return elem[1]

def choose_ball(balls):
    balls.sort(key = take_sec)
    return balls[0]

def init_image():
    pipeline, align = init_rs()
    detector = init_detector()
    return pipeline, align, detector


def image_processing(ball,hello):

    pipeline, align, detector = init_image()
    uusaeg = 0

    cv2.namedWindow("frame")
    cv2.namedWindow("depth")

    frames = pipeline.wait_for_frames()

    aligned_frames = align.process(frames)

    bgr = aligned_frames.get_color_frame()

    depth = aligned_frames.get_depth_frame()


    depth_image = np.asanyarray(depth.get_data())


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

    var = choose_ball(ball_co)
    ball[0] = var[0]
    ball[1] = var[1]


    if(middle < 460 and middle > 420):
       distance = basket_distance(depth_image, basket_co)

    else:

       distance = 0

    bgr = draw_basket(basket_co, bgr, distance)
    bgr = draw_balls(ball_co, bgr, points)

    if(distance != 0):
        thrower_speed = get_speed(distance)

    cv2.putText(bgr, fps, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("frame", bgr)
    cv2.imshow("balls", balls)
    cv2.imshow("basket", basket)

      

