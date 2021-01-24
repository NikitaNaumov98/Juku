import cv2
import config
import numpy as np
import pyrealsense2 as rs



pipeline = rs.pipeline()
configu = rs.config()
configu.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
configu.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

profile = pipeline.start(configu)
sensor_dep = profile.get_device().query_sensors()[1]
sensor_dep.set_option(rs.option.enable_auto_exposure, 0)
sensor_dep.set_option(rs.option.enable_auto_white_balance, 0)
sensor_dep.set_option(rs.option.exposure, 80)
sensor_dep.set_option(rs.option.brightness, 1)





# TODO: display errors when configuration file is missing any of these values
# Get color ranges and noise removal kernels from config
ball_color_range = config.get("colors", config.get("vision", "ball_color"))
ball_noise_kernel = config.get("threshold", "thresh")
basket_color_range = config.get("colors", config.get("vision", "basket_color"))

#initalize realsense camera
def init_rs():

    pipeline = rs.pipeline()
    configu = rs.config()
    configu.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
    configu.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    profile = pipeline.start(configu)
    sensor_dep = profile.get_device().query_sensors()[1]
    sensor_dep.set_option(rs.option.enable_auto_exposure, 0)
    sensor_dep.set_option(rs.option.enable_auto_white_balance, 0)
    sensor_dep.set_option(rs.option.exposure, 80)
    sensor_dep.set_option(rs.option.brightness, 1)
    return pipeline

# Get boolean image with ball color filter applied
def apply_ball_color_filter(hsv):
    # Apply ball color filter
    mask = cv2.inRange(hsv, ball_color_range["min"], ball_color_range["max"])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,ball_noise_kernel["d"])
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,ball_noise_kernel["d"])


    return mask

def apply_basket_color_filter(hsv):

    mask = cv2.inRange(hsv, basket_color_range["min"], basket_color_range["max"])
    return mask

def init_detector():
    # blobdetection params
    blop = cv2.SimpleBlobDetector_Params()
    blop.filterByArea = True  # kõik erineva suurusega ringid on ilusad
    blop.filterByCircularity = False  # leiab nüüd teisi kujusi ka
    blop.minDistBetweenBlobs = 200  # seltsis segasem
    blop.filterByInertia = False
    blop.filterByConvexity = False  # et ringis võiks ka olla sisemisi nurki??
    blop.minArea = 10
    blop.maxArea = 100000
    detector = cv2.SimpleBlobDetector_create(blop)
    return detector

def ball_detection(mask,rgb,detector):

    keypoints = detector.detect(mask)
    output = []
    for keypoint in keypoints:
        x = int(keypoint.pt[0])
        y = int(keypoint.pt[1])
        output.append([x,y])

    return output


def basket_detection(mask):

    im2, contours, h = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours[0]
    x, y, w, h = cv2.boundingRect(cnt)

    corners = [x, y, w, h]

    return corners

def depth_camera(pipeline):

    pipeline.get