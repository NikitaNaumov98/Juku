import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import serial
import queue
import time

lH = 0
lS = 0
lV = 0
hH = 255
hS = 255
hV = 255

def lhueupd(trackbarval):
    global lH
    lH = trackbarval
    return
  
def lsaturationupd(trackbarval):
    global lS
    lS = trackbarval
    return
    
def lvalueupd(trackbarval):
    global lV
    lV = trackbarval
    return

def hhueupd(trackbarval):
    global hH
    hH = trackbarval
    return
  
def hsaturationupd(trackbarval):
    global hS
    hS = trackbarval
    return
    
def hvalueupd(trackbarval):
    global hV
    hV = trackbarval
    return

def communication(thread_queue):
	global ser, speed
	while True:
		if ser.in_waiting > 0:
			inputbuff = ser.readline()
			inputbuff = inputbuff.decode('UTF-8').rstrip("\n")
			print(inputbuff)
		try:
			comm = thread_queue.get_nowait()
			if comm == "turn_right":
				speed = "sd:10:10:10\n"
				ser.write(speed.encode('UTF-8'))
				print(ser.readline().decode('UTF-8').rstrip("\n"))
			elif comm == "turn_left":
				speed = "sd:-10:-10:-10\n"
				ser.write(speed.encode('UTF-8'))
				print(ser.readline().decode('UTF-8').rstrip("\n"))
			elif comm == "stop":
				speed = "sd:0:0:0\n"
				ser.write(speed.encode('UTF-8'))
				print(ser.readline().decode('UTF-8').rstrip("\n"))
			elif comm == "exit":
				speed = "sd:0:0:0\n"
				ser.write(speed.encode('UTF-8'))
				print(ser.readline().decode('UTF-8').rstrip("\n"))
				ser.close()
				break
		except queue.Empty:
			pass
		if speed != "sd:0:0:0\n":
			ser.write(speed.encode('UTF-8'))
			ser.readline()

cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.createTrackbar('Hue lower value limit', 'RealSense', lH, 255, lhueupd)
cv2.createTrackbar('Saturation lower value limit', 'RealSense', lS, 255, lsaturationupd)
cv2.createTrackbar('Lower value limit', 'RealSense', lV, 255, lvalueupd)
cv2.createTrackbar('Hue upper value limit', 'RealSense', hH, 255, hhueupd)
cv2.createTrackbar('Saturation upper value limit', 'RealSense', hS, 255, hsaturationupd)
cv2.createTrackbar('Upper value limit', 'RealSense', hV, 255, hvalueupd)

blobparams = cv2.SimpleBlobDetector_Params()
blobparams.minArea = 1000
blobparams.maxArea = 240000
blobparams.minDistBetweenBlobs = 1000
blobparams.filterByColor = True
blobparams.filterByInertia = False
#change filterByConvexity to False if needed
blobparams.filterByConvexity = True
blobparams.filterByCircularity = False
detector = cv2.SimpleBlobDetector_create(blobparams)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

ser = serial.Serial("/dev/ttyACM0", timeout=2, write_timeout=2)
thread_queue = queue.Queue()
speed = "sd:0:0:0\n"
send_comm = "stop"

mainboard_thread = threading.Thread(target=communication, args=(thread_queue, ))
mainboard_thread.start()

# Start streaming
pipeline.start(config)

try:
	while True:

		# Wait for a coherent pair of frames: depth and color
		frames = pipeline.wait_for_frames()
		#depth_frame = frames.get_depth_frame()
		color_frame = frames.get_color_frame()
		#if not depth_frame or not color_frame:
		if not color_frame:
			continue

		# Convert images to numpy arrays
		#depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())
		framehsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

		# Apply colormap on depth image (image must be converted to 8-bit per pixel first)
		#depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
		
		lowerLimits = np.array([lH, lS, lV])
		upperLimits = np.array([hH, hS, hV])
		
		thresholded = cv2.inRange(framehsv, lowerLimits, upperLimits)
		res = cv2.bitwise_and(framehsv,framehsv, mask= thresholded)
		inverted = cv2.bitwise_not(res)
		keypoints = detector.detect(inverted)
		img_keypoints = cv2.drawKeypoints(color_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		#for i in range(len(keypoints)):  
			#coordinatesx = int(keypoints[i].pt[0])
			#coordinatesy = int(keypoints[i].pt[1])
			#cv2.putText(img_keypoints, str(coordinatesx), (coordinatesx, coordinatesy), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
			#cv2.putText(img_keypoints, str(coordinatesy), (coordinatesx + 150, coordinatesy), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)

		try:
			coordinatesx = int(keypoints[0].pt[0])
			coordinatesy = int(keypoints[0].pt[1])
			cv2.putText(img_keypoints, str(coordinatesx), (coordinatesx, coordinatesy), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
			cv2.putText(img_keypoints, str(coordinatesy), (coordinatesx + 150, coordinatesy), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
			if coordinatesx > 370 and coordinatesx <= 640 and send_comm != "turn_right":
				send_comm = "turn_right"
				thread_queue.put_nowait("turn_right")
			elif coordinatesx < 270 and coordinatesx >= 0 and send_comm != "turn_left":
				send_comm = "turn_left"
				thread_queue.put_nowait("turn_left")
			elif coordinatesx >= 270 and coordinatesx <= 370 and send_comm != "stop":
				send_comm = "stop"
				thread_queue.put_nowait("stop")
		except:
			if send_comm != "stop":
				send_comm = "stop"
				thread_queue.put_nowait("stop")
			pass

		images = np.hstack((img_keypoints, res))
		#cv2.imshow('RealSense', img_keypoints)
		#cv2.imshow('Processed', res)
		cv2.imshow('RealSense', images)
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

finally:

	# Stop streaming
	pipeline.stop()
	thread_queue.put_nowait("exit")
	time.sleep(1)
