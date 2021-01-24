import cv2
from functools import partial
import config
import pyrealsense2 as rs
import numpy as np
import time


pipeline = rs.pipeline()
configu = rs.config()
configu.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
configu.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)

profile = pipeline.start(configu)
sensor_dep = profile.get_device().query_sensors()[1]
sensor_dep.set_option(rs.option.enable_auto_exposure, 0)
sensor_dep.set_option(rs.option.enable_auto_white_balance, 0)
sensor_dep.set_option(rs.option.exposure, 80)
sensor_dep.set_option(rs.option.brightness, 1)
print(sensor_dep.get_option(rs.option.brightness))


def start():
    # Ask for color name to threshold
    color_name = input("Enter color name: ")

    uusaeg = 0
    # Try to get saved range from config file, use whole color space as default if not saved
    # Color ranges are saved as { "min": (hmin, smin, vmin), "max": (hmax, smax, vmax) }
    color_range = config.get("colors", color_name, default={"min": (0, 0, 0), "max": (179, 255, 255), "blur": (0) , "dil": (0)})
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

    cv2.createTrackbar("h_min", "frame", color_range["min"][0], 179, partial(update_range, "min", 0))
    cv2.createTrackbar("s_min", "frame", color_range["min"][1], 255, partial(update_range, "min", 1))
    cv2.createTrackbar("v_min", "frame", color_range["min"][2], 255, partial(update_range, "min", 2))
    cv2.createTrackbar("h_max", "frame", color_range["max"][0], 179, partial(update_range, "max", 0))
    cv2.createTrackbar("s_max", "frame", color_range["max"][1], 255, partial(update_range, "max", 1))
    cv2.createTrackbar("v_max", "frame", color_range["max"][2], 255, partial(update_range, "max", 2))
    cv2.createTrackbar("dilation", "frame", threshold_range["d"], 20, partial(update_thresh, "d"))
    cv2.createTrackbar("blur", "frame", threshold_range["b"], 20 , partial(update_thresh, "b"))
    # Capture camera
    device = config.get("vision", "video_capture_device")
    #cap = cv2.VideoCapture(device)

    while(True):
        # Read BGR frame
#        _, bgr = cap.read()
        frames = pipeline.wait_for_frames()
        bgr = frames.get_color_frame()
        dilationval = threshold_range["d"]
        
        if(dilationval%2 == 0):
            dilationval += 1

        blurval = threshold_range["b"]

        if(blurval%2 == 0):
            blurval += 1

        kernel = np.ones((dilationval,dilationval),np.uint8)

        if not bgr:
            print("not")
            continue

        aeg = time.time()
        fps = str(1//(aeg-uusaeg))
        uusaeg = time.time()

        # Convert to HSV
        bgr = np.asanyarray(bgr.get_data())
        bgr = cv2.rotate(bgr,cv2.ROTATE_180)
        bgr = cv2.GaussianBlur(bgr, (blurval,blurval),1)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # TODO: also apply all the filters you do when actually running the robot (eg noise removal)
        # Apply color mask to HSV image
        mask = cv2.inRange(hsv, color_range["min"], color_range["max"])
        
        # Display filtered image

        image = cv2.bitwise_and(bgr,bgr, mask=mask)

        mask = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        thresholded = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        thresholded = cv2.bitwise_not(thresholded)
         

        cv2.putText(image, fps, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("frame", image)
        cv2.imshow("threshold",thresholded)

        # Handle keyboard input
        key = cv2.waitKey(1)

        if key & 0xFF == ord("q"):
            break

    # Overwrite color range
    config.set("colors", color_name, color_range)
    config.set("threshold","thresh",threshold_range)
    config.save()

    # Exit cleanly
    #cap.release()
    pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    start()
