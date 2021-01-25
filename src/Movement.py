
import math


firstwheelangle = 0
secondwheelangle = 120
thirdwheelangel = 240
turningspeed = 50

#speeds are from 0 to 65535

#0-32767 is one way and
#65535-32767 is the other

#-100 ; 0 on 0;32767
#0 ; 100 on 32767;65535;

def calculate_wheel_speed(overall_speed, direction, wheel_angle):
    linear_velocity = overall_speed * math.cos(direction - wheel_angle)
    print(linear_velocity)
    mainboardunits = 32767 + 100 * linear_velocity

    return mainboardunits

def calculate_movement_direction(Ball_X, Ball_Y, middleX):
    direction = int(math.atan2(math.radians(middleX - Ball_X), Ball_Y))
    return direction

def rotate_left(speeds):
    speeds[0] = turningspeed
    speeds[1] = turningspeed
    speeds[2] = turningspeed

    return speeds

def rotate_right(speeds):
    speeds[0] = - turningspeed
    speeds[1] = - turningspeed
    speeds[2] = - turningspeed

    return speeds

def moveYaxis(speeds):
    speeds[0] = 20
    speeds[1] = 0
    speeds[2] = 20

    return speeds

def stop_moving(speeds):
    speeds[0] = 0
    speeds[1] = 0
    speeds[2] = 0

    return speeds

def turn_around_ball(speeds):
    speeds[0] = 0
    speeds[1] = turningspeed
    speeds[2] = 0

    return speeds

def omnimovement( movingspeed, angle1, angle2, angle3, direction ):
    speeds = [0,0,0,0]
    speeds[0] = int(calculate_wheel_speed(movingspeed, direction, angle1))
    speeds[1] = int(calculate_wheel_speed(movingspeed, direction, angle2))
    speeds[2] = int(calculate_wheel_speed(movingspeed, direction, angle3))


    return speeds


def movement(PallX,PallY,movingspeed):
    direction = calculate_movement_direction(PallX,PallY,415)
    omnimovement(movingspeed,0,120,240,direction)

