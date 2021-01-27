import cv2
import config
import numpy as np
import pyrealsense2 as rs
import scipy as sp
from scipy import signal

# TODO: display errors when configuration file is missing any of these values
# Get color ranges and noise removal kernels from config
ball_color_range = config.get("colours", config.get("vision", "ball_color"))
ball_noise_kernel = config.get("threshold", "thresh")
basket_color_range = config.get("colours", config.get("vision", "basket_color"))

def update_config():
    global ball_color_range
    global basket_color_range

    ball_color_range = config.get("colors", config.get("vision", "ball_color"))
    basket_color_range = config.get("colors", config.get("vision", "basket_color"))
    return ball_color_range, basket_color_range

#initalize realsense camera
def init_rs():

    pipeline = rs.pipeline()
    configu = rs.config()
    configu.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    configu.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)

    profile = pipeline.start(configu)
    sensor_dep = profile.get_device().query_sensors()[1]
    sensor_dep.set_option(rs.option.enable_auto_exposure, 0)
    sensor_dep.set_option(rs.option.enable_auto_white_balance, 0)
    sensor_dep.set_option(rs.option.white_balance, 4120)
    sensor_dep.set_option(rs.option.exposure, 130)
    sensor_dep.set_option(rs.option.brightness, 0)
    sensor_dep.set_option(rs.option.backlight_compensation, 0)
    sensor_dep.set_option(rs.option.contrast, 2)
    sensor_dep.set_option(rs.option.sharpness, 100)
    sensor_dep.set_option(rs.option.gain, 1)
    sensor_dep.set_option(rs.option.gamma, 420)
    sensor_dep.set_option(rs.option.hue, -67)
    sensor_dep.set_option(rs.option.saturation, 57)
    print(sensor_dep.get_option(rs.option.brightness))
    #align_to = rs.stream.color
    #algin = rs.align(align_to)
    
    return pipeline

# Get boolean image with ball color filter applied
def apply_ball_color_filter(hsv,bgr):
    # Apply ball color filter
    dilationval = ball_noise_kernel["d"]
    mask = cv2.inRange(hsv, ball_color_range["min"], ball_color_range["max"])

    #image = cv2.bitwise_and(bgr, bgr, mask=mask)

    mask = cv2.erode(mask, np.ones((dilationval + 3, dilationval + 3), np.uint8),2)
    mask = cv2.dilate(mask, np.ones((dilationval + 2, dilationval + 2), np.uint8), 1)

    return mask

def apply_basket_color_filter(hsv):

    dilationval = ball_noise_kernel["d"]

    if (dilationval % 2 == 0):
        dilationval += 1

    kernel = np.ones((dilationval, dilationval), np.uint8)

    mask = cv2.inRange(hsv, basket_color_range["min"], basket_color_range["max"])


    #image = cv2.bitwise_and(bgr, bgr, mask=mask)


    mask = cv2.erode(src=mask,kernel= np.ones((dilationval, dilationval), np.uint8),iterations=2)
    mask = cv2.dilate(src=mask,kernel = kernel, iterations=1)



    #mask = cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
 
    return mask

def init_detector():
    # blobdetection params
    blop = cv2.SimpleBlobDetector_Params()
    blop.filterByArea = True  # kõik erineva suurusega ringid on ilusad
    blop.filterByCircularity = False  # leiab nüüd teisi kujusi ka
    blop.minDistBetweenBlobs = 100# seltsis segasem
    blop.filterByInertia = False
    blop.filterByConvexity = False  # et ringis võiks ka olla sisemisi nurki??
    blop.minArea = 18
    blop.maxArea = 1000000
    detector = cv2.SimpleBlobDetector_create(blop)
    return detector

def ball_detection(image,detector):

    threshold = cv2.bitwise_not(image)
    #threshold = cv2.invert()
    keypoints = detector.detect(threshold)
    output = []
    #cont, hie = cv2.findContours(image,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #try:
        #maxc = max(cont, key=cv2.contourArea())
        #(x,y),  r = cv2.minEnclosingCircle(maxc)

        #if(r<3):
            #raise NotImplementedError

    #except Exception as e:
        #return None, None, None
    for point in range(len(keypoints)):
        x = int(keypoints[point].pt[0])
        y = int(keypoints[point].pt[1])
        output.append([x,y])

    return output, keypoints

def draw_balls(balls, image, points):

    if(balls != None):
        if(len(balls) != 0 ):
            for x in balls:
                cv2.putText(image, "x: " + str(x[0]) + " y: " + str(x[1]), (x[0],x[1]), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)


    return image

def draw_basket(corners, image, distance):

    if(corners != None):
        cv2.rectangle(image, (corners[0], corners[1]), (corners[0] + corners[2], corners[1] + corners[3]), (0, 255, 0), 2)
        cv2.putText(image, str(distance), (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)

    return image

def basket_detection(image):
    threshold = cv2.bitwise_not(image)
    contours, h = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if(len(contours) != 0):
        cnt = max(contours, key = cv2.contourArea)

        if(cv2.contourArea(cnt) > 400):

            x, y, w, h = cv2.boundingRect(cnt)
            corners = [x, y, w, h]
        else:
            return None
    else:
        return None

    return corners
def basket_distance(depth_array,corners):
    distances = []
    if(corners != None):
        X = corners[0]
        Y = corners[1]
        w = corners[2]
        h = corners[3]

        Xoffset = 30

        for y in range(480 - h, 480):
            a = range(X,X+w)

            for x in a:
                 distances.append(depth_array.get_distance(x,y))


        basket_distance = np.array(distances)
        mean = sp.signal.medfilt(basket_distance,5)
        #mean = np.median(mean)
        return mean

    return 0

def blur(bgr):

    blurval = ball_noise_kernel["b"]

    if (blurval % 2 == 0):
        blurval += 1

    bgr = cv2.blur(bgr, (blurval, blurval))

    return bgr
