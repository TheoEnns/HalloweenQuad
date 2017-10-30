import math
import numpy as np
import time

import Q_Structure as qS
import Q_Geometry as qG
import Q_Motion as qM

from Q_Sound import *
from Q_Gait_Utilities import *

if __name__ == "__main__":
    _heightHigh = 25
    _heightLow = -59
    _transDelta = 27
    voice.say("DOWN")
    voice.runAndWait()
    trace(np.array([0,0,0]), np.array([0,0,_heightHigh]), 2)
    voice.say("BACKWARD")
    voice.runAndWait()
    trace(np.array([0,0,_heightHigh]), np.array([_transDelta,0,_heightHigh]), 1)
    voice.say("UP")
    voice.runAndWait()
    trace(np.array([_transDelta,0,_heightHigh]), np.array([_transDelta,0,_heightLow]), 2)
    voice.say("FOREWARD")
    voice.runAndWait()
    trace(np.array([_transDelta,0,_heightLow]), np.array([0,0,_heightLow]), 1)
    voice.say("RIGHT")
    voice.runAndWait()
    trace(np.array([0,0,_heightLow]), np.array([0,_transDelta,_heightLow]), 1)
    voice.say("DOWN")
    voice.runAndWait()
    trace(np.array([0,_transDelta,_heightLow]), np.array([0,_transDelta,_heightHigh]), 2)
    voice.say("LEFT")
    voice.runAndWait()
    trace(np.array([0,_transDelta,_heightHigh]), np.array([0,0,_heightHigh]), 1)
    voice.say("UP")
    voice.runAndWait()
    trace(np.array([0,0,_heightHigh]), np.array([0,0,_heightLow]), 2)
    voice.say("CENTER")
    voice.runAndWait()
    trace(np.array([0,0,_heightLow]), np.array([0,0,0]), 2)