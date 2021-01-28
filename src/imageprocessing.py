import cv2
from vision import *
import config
import pyrealsense2 as rs
import numpy as np
import time
from plot import *
from multiprocessing import Lock
from threading import Thread
from Movement import *
from TaskFile import *



class Camera:

    def __init__(self):
        self.pipeline = init_rs()
        self.cam = None
        self.frame = self.pipeline.wait_for_frames()
        self.depth = None
        self.stream = True

    def start(self):
        Thread(target=self.get_color_frame,args=(),daemon=True).start()
        return self

    def get_color_frame(self):

        while(self.stream):

            self.frame = self.pipeline.wait_for_frames()
            align_to = rs.stream.color
            align = rs.align(align_to)
            raamid = align.process(self.frame)
            rgb = raamid.get_color_frame()
            self.cam = np.asanyarray(rgb.get_data())
            #self.cam = cv2.rotate(rgb,cv2.ROTATE_180)
            #self.cam = cv2.cvtColor, cv2.COLOR_RGB2HSV)
            self.depth = raamid.get_depth_frame()

            #print(self.depth.get_height())
            #print(self.depth.get_width())
        else:
            self.pipeline.stop()
            cv2.destroyAllWindows()
            quit()

    def stop(self):
        self.stream = False

    def get_frame(self):
        return  self.cam

class basketDetect():

    def __init__(self):
        self.running = True
        self.frame = []
        self.korv = []
        self.depth = []
        self.coordinates = []
        self.distance = 0

    def start(self):
        Thread(target=self.get ,args=(),daemon=True).start()
        return self

    def stop(self):
        self.running = False

    def set(self,input,dinput):
        self.frame = input
        self.depth = dinput

    def get(self):
        while(self.running):
            if(type(self.frame) == np.ndarray):
                self.korv = apply_basket_color_filter(self.frame)

                self.coordinates = basket_detection(self.korv)
                self.distance = basket_distance(self.depth, self.coordinates)



class ImageProcessing:

    def __init__(self,frame,dframe):
        self.processing = True
        self.image = frame
        self.depth_frame = dframe
        self.thrower_speed = None
        self.ball_co = None
        self.balls = []
        self.basket = []
        self.basket_co = None
        self.points = []
        self.hsv = []
        self.detector = init_detector()

    def start(self):
        Thread(target=self.process,args=(),daemon=True).start()
        return self

    def get_balls(self):
        return self.balls

    def process(self,):

        while(self.processing):
            #start = time.time()
            sis = self.image.get_frame()
            #depth = self.depth_frame


            if(type(sis) != np.ndarray):
                continue

            sis = np.asanyarray(sis)
            sis = cv2.rotate(sis,cv2.ROTATE_180)
            sis = sis[0:650, 0:1280]
            self.hsv = cv2.cvtColor(sis, cv2.COLOR_RGB2HSV)
            #self.hsv = cv2.bilateralFilter(src=hsv, d=1, sigmaColor=75, sigmaSpace=75)
            self.balls = apply_ball_color_filter(self.hsv,sis)

            #self.basket = apply_basket_color_filter(hsv)

            self.ball_co, self.points = ball_detection(self.balls, self.detector)
            #self.basket_co = basket_detection(self.basket)
            #end = time.time()
            #print(start- end)


            #distance = basket_distance(depth, self.basket_co)
           # if (middle < 560 and middle > 420):



           #else:

               # distance = 0

            #sis = draw_basket(self.basket_co, sis, distance)
            #sis = draw_balls(self.ball_co, sis, points)

            #if (distance != 0):
            #    self.thrower_speed = get_speed(distance)

            #else:
             #   self.thrower_speed = 0


    def set_frame(self,frame):
        self.image = frame

    def set_dframe(self,frame):
        self.depth_frame = frame

    def get_frame(self):
        return self.image

    def get_speed(self):
        return self.thrower_speed

    def stop(self):
        self.processing = False

def take_sec(elem):
    return elem[1]

def choose_ball(balls):
    balls = sorted(balls,key=take_sec,reverse=True)
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
    while (True):
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

        bgr = bgr[0:440,0:1280]

        hsv = cv2.cvtColor(bgr, cv2.COLOR_RGB2HSV)

        hsv = cv2.bilateralFilter(src=hsv, d=1, sigmaColor=75, sigmaSpace=75)

        balls = apply_ball_color_filter(hsv,bgr)
        basket = apply_basket_color_filter(hsv)

        ball_co, points = ball_detection(balls, detector)
        basket_co = basket_detection(basket)


        if(basket_co != None):
            middle = basket_co[0] + basket_co[2]/2
        else:
            middle = 0

        distance = basket_distance(depth, basket_co)
        #if(middle < 520 and middle > 440):
         #   distance = basket_distance(depth, basket_co)
        #else:
         #  distance = 0

        bgr = draw_basket(basket_co, bgr, distance)
        bgr = draw_balls(ball_co, bgr, points)

        if(distance != 0):
            thrower_speed = get_speed(distance)

        cv2.putText(bgr, fps, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("frame", bgr)
        #cv2.imshow("balls", balls)
        #cv2.imshow("basket", basket)

        key = cv2.waitKey(1)

        if key & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break

def protsessipilti(process_var,speeds):
    detector = init_detector()
    integraalX = 0
    integraalY = 0
    oldX = 0
    oldY = 0
    pall = [0,0]
    seis = 0
    while(True):

        rgb = process_var[5]
        #depth = process_var[6]


        if(type(rgb) != np.ndarray):
            continue

        bgr = cv2.rotate(rgb, cv2.ROTATE_180)

        bgr = bgr[0:430, 0:640]



        balls = apply_ball_color_filter(bgr, bgr)
        basket = apply_basket_color_filter(bgr)

        ballc, points = ball_detection(balls, detector)

        if (ballc != []):
            # cv2.circle(rgb,(ballx,bally),(0,255,0),1)
            pall = choose_ball(ballc)
            #bgr = draw_balls(ballc, bgr, points)

        basket_co = basket_detection(basket)

        if (basket_co != None):
            pass
            #distance = basket_distance(depth, basket_co)
            #bgr = draw_basket(basket_co, bgr)
        else:
            distance = -1

        if(pall[0] > 320 and pall[0] < 370 and pall[1] > 390):


            integraalX = 0
            integraalY = 0
            speeds[0] = 0
            speeds[1] = 0
            speeds[2] = 0
            time.sleep(1)

        if(ballc != [] and seis != 1):
             valX , oldX = pid(pall[0],integraalX,0.02,0.00065,0.0012,320,oldX)
             valY, oldY = pid(pall[1],integraalY,0.02,0.00055,0.0003,240,oldY)
             direction = calculate_movement_direction(340 - pall[0], 480 - pall[1])
             speed = math.sqrt((340 - pall[0])**2+(480 - pall[1])**2)*0.13
             speeds[0] = int(calculate_wheel_speed(speed,direction,240))
             speeds[1] = int(calculate_wheel_speed(speed,direction,120))
             speeds[2] = - int(calculate_wheel_speed(speed,direction,0))
             process_var[4] = pall
        elif(ballc == [] and seis != 1):
             speeds[0] = 5
             speeds[1] = 5
             speeds[2] = 5

def uus_kaamerapilt(process_var,speeds):

    pipeline = init_rs()

    distance = -1


    to = rs.stream.color
    frames = pipeline.wait_for_frames()
    align = rs.align(to)
    aligned = align.process(frames)
    color = aligned.get_color_frame()
    depth = aligned.get_depth_frame()



    while(True):
        frames = pipeline.wait_for_frames()
        align = rs.align(to)
        aligned = align.process(frames)
        color = aligned.get_color_frame()
        depth = aligned.get_depth_frame()

        rgb = np.asanyarray(color.get_data())
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        hsv = cv2.bilateralFilter(src=rgb, d=1, sigmaColor=75, sigmaSpace=75)

        process_var[5] = hsv
        #process_var[6] = depth


