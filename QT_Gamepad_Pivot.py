import numpy as np
import time
import pygame

from Q_Gait_Utilities import *

if __name__ == "__main__":
    pygame.init()
    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    axes = joystick.get_numaxes()
    a = 0
    b = 0
    c = 0
    while(True):
        pygame.event.pump()
        a = (joystick.get_axis(3) + 49*a)/50.0
        b = (joystick.get_axis(0) + 49*b)/50.0
        c = (joystick.get_axis(1) + 49*c)/50.0
        try:
            seek(np.array([ transX*c, \
                            transY*b, \
                            (heightHigh - heightLow) * a / 2 + (heightHigh + heightLow) / 2 \
                            ]), 0.01)
        except:
            pass
        time.sleep(0.001)