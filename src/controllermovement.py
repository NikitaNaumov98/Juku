import Movement
from Serial import *
import signal

from xbox360controller import Xbox360Controller
x = 0
y = 0
r = 0

def axis_move(axis):
    global x, y
    x = axis.x
    y = axis.y
    print(x)
    print(y)

def speed(axis):
    global r
    r = axis.y

def ControllerMovement(ser):
    try:

        with Xbox360Controller(0, axis_threshold=0.2) as controller:
            controller.axis_r.when_moved = axis_move
            controller.axis_l.when_moved = speed
            signal.pause()

    except  KeyboardInterrupt:
        pass


if __name__ == "__main__":
    ControllerMovement(ser)



