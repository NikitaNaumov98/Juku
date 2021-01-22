
import math


firstwheelangle = 0
secondwheelangle = 120
thirdwheelangel = 240
turningspeed = 50

def calculate_wheel_speed(overall_speed, direction, wheel_angle):
    linear_velocity = overall_speed * math.cos(direction - wheel_angle)
    return linear_velocity

def calculate_movement_direction(Ball_X, Ball_Y, middleX):
    direction = int(math.atan2(math.radians(middleX - Ball_X), Ball_Y))
    return direction

def rotate_left(speeds):
    speeds[0] = turningspeed
    speeds[1] = turningspeed
    speeds[2] = turningspeed

def rotate_right(speeds):
    speeds[0] = - turningspeed
    speeds[1] = - turningspeed
    speeds[2] = - turningspeed

def moveYaxis(speeds):
    speeds[0] = 20
    speeds[1] = 0
    speeds[2] = 20

def stop_moving(speeds):
    speeds[0] = 0
    speeds[1] = 0
    speeds[2] = 0

def turn_around_ball(speeds, ):
    speeds[0] = 0
    speeds[1] = turningspeed
    speeds[2] = 0

def omnimovement(speeds, movingspeed, angle1, angle2, angle3, direction ):
    speeds[0] = calculate_wheel_speed(movingspeed, direction, angle1)
    speeds[1] = calculate_wheel_speed(movingspeed, direction, angle2)
    speeds[2] = calculate_wheel_speed(movingspeed, direction, angle3)

def play(speeds):
    pass