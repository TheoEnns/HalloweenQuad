import numpy as np
import time
import pygame

from Q_Gait_Utilities import *

x_axis, y_axis, z_axis = 0, 0, 0
pygame.init()
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
joystick = pygame.joystick.Joystick(0)
joystick.init()
axes = joystick.get_numaxes()

modes = ["Relax", "PivotMode", "Walk"]
mode = modes[0]
if __name__ == "__main__":
    while(True):
        try:
            pygame.event.pump()
            if joystick.get_button(2) == 1:
                mode = "Relax"
            elif joystick.get_button(1) == 1:
                mode = "PivotMode"
            elif joystick.get_button(0) == 1:
                mode = "Walk"

            if mode is "Relax":
                pass
            elif mode is "PivotMode":
                z_axis = (joystick.get_axis(3) + 49 * z_axis) / 50.0
                y_axis = (joystick.get_axis(0) + 49 * y_axis) / 50.0
                x_axis = (joystick.get_axis(1) + 49 * x_axis) / 50.0
                try:
                    seek(np.array([transX * x_axis, \
                                   transY * y_axis, \
                                   (heightHigh - heightLow) * z_axis / 2 + (heightHigh + heightLow) / 2 \
                                   ]), 0.01)
                except:
                    pass
                time.sleep(0.001)
            elif mode is "Walk":
                hat = joystick.get_hat(0)
                if hat[1] == -1:
                    walk_Backward(.2, g_backward)
                elif hat[1] == 1:
                    walk_Forward(.2, g_forward)
                elif hat[0] == -1:
                    walk_Left(.2, g_sideward)
                elif hat[0] == 1:
                    walk_Right(.2, g_sideward)
                else:
                    seek(np.array([0, 0, (heightHigh + heightLow) / 2 \
                                   ]), 0.01)
        except:
            pass