import Movement
import Serial
import signal

from xbox360controller import Xbox360Controller


def ControllerMovement():
    while(True):
        with Xbox360Controller(0,axis_threshold=0.2) as controller:
            X = controller.axis_l.x
            Y = controller.axis_l.y

            if(controller.button_a.is_pressed):
                break


