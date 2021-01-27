import cv2
from functools import partial
import config
import pyrealsense2 as rs
import numpy as np
import time
from vision import *

pipeline = rs.pipeline()
configu = rs.config()
configu.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
configu.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

profile = pipeline.start()
sensor_dep = profile.get_device().query_sensors()[1]

sensor_dep.set_option(rs.option.enable_auto_exposure, 0)
sensor_dep.set_option(rs.option.enable_auto_white_balance, 0)
sensor_dep.set_option(rs.option.white_balance, 4120)
sensor_dep.set_option(rs.option.exposure, 130)
sensor_dep.set_option(rs.option.brightness, 0)
sensor_dep.set_option(rs.option.backlight_compensation, 0)
sensor_dep.set_option(rs.option.contrast,2)
sensor_dep.set_option(rs.option.sharpness, 100)
sensor_dep.set_option(rs.option.gain, 1)
sensor_dep.set_option(rs.option.gamma,420)
sensor_dep.set_option(rs.option.hue,-67)
sensor_dep.set_option(rs.option.saturation,57)
print(sensor_dep.get_option(rs.option.hue))

def start():
    # Ask for color name to threshold
    color_name = input("Enter color name: ")

    uusaeg = 0
    # Try to get saved range from config file, use whole color space as default if not saved
    # Color ranges are saved as { "min": (hmin, smin, vmin), "max": (hmax, smax, vmax) }
    color_range = config.get("colours", color_name, default={"min": (0, 0, 0), "max": (179, 255, 255), "blur": (0) , "dil": (0)})
    threshold_range = config.get("threshold","thresh")
    # Create trackbars (sliders) for HSV channels
    cv2.namedWindow("frame")
    cv2.resizeWindow("frame",600,900)

    def update_range(i, j, value):
        values = list(color_range[i])
        values[j] = value
        color_range[i] = tuple(values)

    def update_thresh(i, value):
        threshold_range[i] = value 
        print(threshold_range)

    a = -170
    b = 100
    def update_thing(val):
        print(sensor_dep.get_option(rs.option.saturation))
        sensor_dep.set_option(rs.option.saturation, val)

    def update_otherthing(val):

        val -= 170
        print(sensor_dep.get_option(rs.option.hue))
        sensor_dep.set_option(rs.option.hue, val)

    cv2.createTrackbar("h_min", "frame", color_range["min"][0], 179, partial(update_range, "min", 0))
    cv2.createTrackbar("s_min", "frame", color_range["min"][1], 255, partial(update_range, "min", 1))
    cv2.createTrackbar("v_min", "frame", color_range["min"][2], 255, partial(update_range, "min", 2))
    cv2.createTrackbar("h_max", "frame", color_range["max"][0], 179, partial(update_range, "max", 0))
    cv2.createTrackbar("s_max", "frame", color_range["max"][1], 255, partial(update_range, "max", 1))
    cv2.createTrackbar("v_max", "frame", color_range["max"][2], 255, partial(update_range, "max", 2))
    cv2.createTrackbar("dilation", "frame", threshold_range["d"], 20, partial(update_thresh, "d"))
    cv2.createTrackbar("blur", "frame", threshold_range["b"], 20 , partial(update_thresh, "b"))
    cv2.createTrackbar("hue", "frame", a, 340, update_otherthing)
    cv2.createTrackbar("sat", "frame", b, 100, update_thing)

    # Capture camera
    device = config.get("vision", "video_capture_device")
    #cap = cv2.VideoCapture(device)
    datector = init_detector()
    mat = np.zeros((430,840,3), np.uint8)
    while(True):
        # Read BGR frame
#        _, bgr = cap.read()
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame  or not color_frame:
            continue

        dilationval = threshold_range["d"]
        
        if(dilationval%2 == 0):
            dilationval += 1

        blurval = threshold_range["b"]

        if(blurval%2 == 0):
            blurval += 1

        kernel = np.ones((dilationval,dilationval),np.uint8)


        aeg = time.time()
        fps = str(1//(aeg-uusaeg))
        uusaeg = time.time()

        # Convert to HSV
        bgr = np.asanyarray(color_frame.get_data())
        bgr = cv2.rotate(bgr,cv2.ROTATE_180)
        bgr = bgr[0:430,0:1280]

        #bgr = cv2.bilateralFilter(bgr, 3, 75, 75)
        #bgr = cv2.GaussianBlur(bgr, (blurval,blurval),0)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_RGB2HSV)
        hsv = cv2.bilateralFilter(src=hsv, d=1, sigmaColor=75, sigmaSpace=75)
        # TODO: also apply all the filters you do when actually running the robot (eg noise removal)
        # Apply color mask to HSV image
        mask = cv2.inRange(hsv, color_range["min"], color_range["max"])
        
        # Display filtered imag
        image = cv2.bitwise_and(bgr,bgr, mask=mask)

        #thresholded = cv2.morphologyEx(image, cv2.MORPH_OPEN, np.ones((dilationval+2,dilationval+2),np.uint8))
        thresholded = cv2.erode(image, np.ones((dilationval+1, dilationval+1), np.uint8),2)
        thresholded = cv2.dilate(thresholded,np.ones((dilationval,dilationval),np.uint8))

        #thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, np.ones((dilationval,dilationval),np.uint8))
        #thresholded = cv2.bitwise_not(thresholded)


        #inv = cv2.bitwise_not(mask)
        #thresholded = cv2.bitwise_and(bgr,bgr,mask = inv)
        #balls, points = ball_detection(thresholded, datector)
        #image = draw_balls(balls, hsv, points)

        cv2.putText(image, fps, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("frame", bgr)
        cv2.imshow("ns",image)
        cv2.imshow("threshold",thresholded)

        # Handle keyboard input
        key = cv2.waitKey(1)

        if key & 0xFF == ord("q"):
            break

    # Overwrite color range
    config.set("colours", color_name, color_range)
    config.set("threshold","thresh",threshold_range)
    config.save()

    # Exit cleanly
    #cap.release()
    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    start()

