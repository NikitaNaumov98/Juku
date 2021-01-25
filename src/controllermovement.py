from Movement import *
from Serial import SerialClass
import signal
from xbox360controller import Xbox360Controller
x = 0
y = 0
r = 0
t = 1

ser = SerialClass("/dev/ttyACM0").start()


def axis_move(axis):
    global x, y
    x = axis.x
    y = axis.y
    direction = calculate_movement_direction(x,y,0)
    speeds = omnimovement(x/y,0,120,240,direction)
    print(speeds)
    ser.set_speeds(speeds)


def speed(axis):
    global r
    r = axis.y

def thrower(axis):
    global t
    t = t * -1

def ControllerMovement():
    try:

        with Xbox360Controller(0, axis_threshold=0.2) as controller:
            controller.axis_r.when_moved = axis_move
            controller.axis_l.when_moved = speed
            controller.button_a.when_pressed = thrower

            signal.pause()

    except  KeyboardInterrupt:
        pass


if __name__ == "__main__":

    ControllerMovement()
