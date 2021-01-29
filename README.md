# Juku

This is the repository of Team Juku in the course  PWIR 2020.

To run this code you must run the file TaskFile.py.
The serial port is hardcoded. That can be changed from TaskFile.py

To make new thresholds run Thresholding.py.
They are saved into config.ini.

Image processing and capturing functions are at the end of the imageprocessing.py file.

ball and basket filtering commands are in the vision.py file. And also realsense configuration is there.

The processing code uses Manager from the multiprocessing  lib to send image, ball information between processes. 
This code only follows the ball, there is no gamelogic implemented.
