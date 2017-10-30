import math
import numpy as np
import time
import pygame

import Q_Structure as qS
import Q_Geometry as qG
import Q_Motion as qM
from Q_Sound import *
from Q_Gait_Params import *

### INITIALIZE

##  Leg Setup
cb = qM.Cerebellum()
cb.setup(True)
ForeLeft  = qG.QuadLeg(np.array([ qS.HIP_FAR_X,  qS.HIP_Y, 0]), np.array([ qS.HIP_NEAR_X, qS.HIP_Y, 0]))
BackLeft  = qG.QuadLeg(np.array([-qS.HIP_NEAR_X,  qS.HIP_Y, 0]), np.array([-qS.HIP_FAR_X, qS.HIP_Y, 0]))
ForeRight = qG.QuadLeg(np.array([ qS.HIP_NEAR_X,-qS.HIP_Y, 0]), np.array([ qS.HIP_FAR_X, -qS.HIP_Y, 0]), True)
BackRight = qG.QuadLeg(np.array([-qS.HIP_FAR_X,-qS.HIP_Y, 0]), np.array([-qS.HIP_NEAR_X, -qS.HIP_Y, 0]), True)

ForeLeftCenter  = np.array([ centerX, centerY, centerZ])
BackLeftCenter  = np.array([-centerX, centerY, centerZ])
ForeRightCenter = np.array([ centerX,-centerY, centerZ])
BackRightCenter = np.array([-centerX,-centerY, centerZ])


### Leg Control Utilities
def set_Legs(targets):
    ForeLeftSolve = ForeLeft.getEffectorSolution(np.array([0, 0, 1]), ForeLeftCenter + targets["ForeLeft"])
    BackLeftSolve = BackLeft.getEffectorSolution(np.array([0, 0, 1]), BackLeftCenter + targets["BackLeft"])
    ForeRightSolve = ForeRight.getEffectorSolution(np.array([0, 0, 1]), ForeRightCenter + targets["ForeRight"])
    BackRightSolve = BackRight.getEffectorSolution(np.array([0, 0, 1]), BackRightCenter + targets["BackRight"])

    cb.multi_move_radians({cb.sIDMap["ForeLeftA"]: ForeLeftSolve["hipA_angle"], \
                           cb.sIDMap["ForeLeftB"]: ForeLeftSolve["hipB_angle"], \
                           cb.sIDMap["ForeLeftR"]: ForeLeftSolve["hipR_angle"], \
                           cb.sIDMap["BackLeftA"]: BackLeftSolve["hipA_angle"], \
                           cb.sIDMap["BackLeftB"]: BackLeftSolve["hipB_angle"], \
                           cb.sIDMap["BackLeftR"]: BackLeftSolve["hipR_angle"], \
                           cb.sIDMap["ForeRightA"]: ForeRightSolve["hipA_angle"], \
                           cb.sIDMap["ForeRightB"]: ForeRightSolve["hipB_angle"], \
                           cb.sIDMap["ForeRightR"]: ForeRightSolve["hipR_angle"], \
                           cb.sIDMap["BackRightA"]: BackRightSolve["hipA_angle"], \
                           cb.sIDMap["BackRightB"]: BackRightSolve["hipB_angle"], \
                           cb.sIDMap["BackRightR"]: BackRightSolve["hipR_angle"], \
                           }, 500, False)

def trace(start, stop, elapse):
    tick = 0.005
    endTime = (int)(elapse/tick)
    for incr in range(0, endTime):
        fraction = start*(1.0 - 1.0*incr/endTime) + stop*(1.0*incr/endTime)
        set_Legs({"ForeLeft":fraction,"BackLeft":fraction,"ForeRight":fraction,"BackRight":fraction})
        time.sleep(tick)

def gen_CreepGait(timer,offset,period,gait):
    trans,elevate = 0,0
    phase = math.floor(timer/period)
    fraction = (timer/period) - phase
    phase = (phase + offset)%4
    if(phase == 0):
        elevate = gait["heightHigh"] * (math.sin(fraction * math.pi)) + gait["heightLow"] * (1 - math.sin(fraction * math.pi))
        trans = -gait["trans"] * math.cos(fraction * math.pi)
    else:
        elevate = gait["heightLow"]
        trans = -gait["trans"] * (phase + fraction - 1) / 3 + gait["trans"] * (1 - (phase + fraction - 1) / 3)
    return elevate, trans

def walk_Forward(period,gait):
    timer = time.time()
    trans, elevate, lean = {}, {}, 0
    elevate["ForeLeft"],trans["ForeLeft"]   = gen_CreepGait(timer,3,period,gait)
    elevate["BackLeft"],trans["BackLeft"]   = gen_CreepGait(timer,2,period,gait)
    elevate["ForeRight"],trans["ForeRight"] = gen_CreepGait(timer,1,period,gait)
    elevate["BackRight"],trans["BackRight"] = gen_CreepGait(timer,0,period,gait)

    fraction = math.fmod(timer / period + 1,4)
    if(fraction > 1.8 and fraction < 2.2):
        fraction = (fraction - 1.8)/0.4
        lean = -gait["lean"] * (math.cos(fraction * math.pi))
    elif(fraction > 3.8 or fraction < 0.2):
        fraction = (math.fmod(timer/period+3,4) - 1.8)/0.4
        lean = gait["lean"] * (math.cos(fraction * math.pi))
    elif (fraction >= 0.2 and fraction <= 1.8):
        lean = -gait["lean"]
    elif (fraction >= 2.2 and fraction <= 3.8):
        lean = gait["lean"]

    set_Legs( \
        {"ForeLeft":  np.array([ trans["ForeLeft"]-15,  lean-20, elevate["ForeLeft"]]), \
         "BackLeft":  np.array([ trans["BackLeft"]-25,  lean+10, elevate["BackLeft"]]), \
         "ForeRight": np.array([ trans["ForeRight"]-15, lean+20, elevate["ForeRight"]]), \
         "BackRight": np.array([ trans["BackRight"]-25, lean-10, elevate["BackRight"]]) \
         })

def walk_Backward(period,gait):
    timer = -time.time()
    trans, elevate, lean = {}, {}, 0
    elevate["ForeLeft"],trans["ForeLeft"]   = gen_CreepGait(timer,3,period,gait)
    elevate["BackLeft"],trans["BackLeft"]   = gen_CreepGait(timer,2,period,gait)
    elevate["ForeRight"],trans["ForeRight"] = gen_CreepGait(timer,1,period,gait)
    elevate["BackRight"],trans["BackRight"] = gen_CreepGait(timer,0,period,gait)

    fraction = math.fmod(-timer / period + 1,4)
    if(fraction > 1.8 and fraction < 2.2):
        fraction = (fraction - 1.8)/0.4
        lean = -gait["lean"] * (math.cos(fraction * math.pi))
    elif(fraction > 3.8 or fraction < 0.2):
        fraction = (math.fmod(timer/period+3,4) - 1.8)/0.4
        lean = gait["lean"] * (math.cos(fraction * math.pi))
    elif (fraction >= 0.2 and fraction <= 1.8):
        lean = -gait["lean"]
    elif (fraction >= 2.2 and fraction <= 3.8):
        lean = gait["lean"]

    set_Legs( \
        {"ForeLeft":  np.array([ trans["ForeLeft"]-25,  lean-20, elevate["ForeLeft"]]), \
         "BackLeft":  np.array([ trans["BackLeft"]-15,  lean+10, elevate["BackLeft"]]), \
         "ForeRight": np.array([ trans["ForeRight"]-25, lean+20, elevate["ForeRight"]]), \
         "BackRight": np.array([ trans["BackRight"]-15, lean-10, elevate["BackRight"]]) \
         })

def walk_Right(period,gait):
    timer = time.time()
    trans, elevate, lean = {}, {}, 0
    elevate["ForeLeft"],trans["ForeLeft"]   = gen_CreepGait(timer,3,period,gait)
    elevate["BackLeft"],trans["BackLeft"]   = gen_CreepGait(timer,2,period,gait)
    elevate["ForeRight"],trans["ForeRight"] = gen_CreepGait(timer,1,period,gait)
    elevate["BackRight"],trans["BackRight"] = gen_CreepGait(timer,0,period,gait)

    fraction = math.fmod(timer / period + 1,4)
    if(fraction > 1.8 and fraction < 2.2):
        fraction = (fraction - 1.8)/0.4
        lean = -gait["lean"] * (math.cos(fraction * math.pi))
    elif(fraction > 3.8 or fraction < 0.2):
        fraction = (math.fmod(timer/period+3,4) - 1.8)/0.4
        lean = gait["lean"] * (math.cos(fraction * math.pi))
    elif (fraction >= 0.2 and fraction <= 1.8):
        lean = -gait["lean"]
    elif (fraction >= 2.2 and fraction <= 3.8):
        lean = gait["lean"]

    set_Legs( \
        {"ForeLeft":  np.array([ lean-20, -trans["ForeLeft"],  elevate["ForeLeft"]]), \
         "BackLeft":  np.array([ lean+10, trans["BackLeft"],  elevate["BackLeft"]]), \
         "ForeRight": np.array([ lean+20, -trans["ForeRight"], elevate["ForeRight"]]), \
         "BackRight": np.array([ lean-10, trans["BackRight"], elevate["BackRight"]]) \
         })


def walk_Left(period,gait):
    timer = time.time()
    trans, elevate, lean = {}, {}, 0
    elevate["ForeLeft"],trans["ForeLeft"]   = gen_CreepGait(timer,3,period,gait)
    elevate["BackLeft"],trans["BackLeft"]   = gen_CreepGait(timer,2,period,gait)
    elevate["ForeRight"],trans["ForeRight"] = gen_CreepGait(timer,1,period,gait)
    elevate["BackRight"],trans["BackRight"] = gen_CreepGait(timer,0,period,gait)

    fraction = math.fmod(timer / period + 1,4)
    if(fraction > 1.8 and fraction < 2.2):
        fraction = (fraction - 1.8)/0.4
        lean = -gait["lean"] * (math.cos(fraction * math.pi))
    elif(fraction > 3.8 or fraction < 0.2):
        fraction = (math.fmod(timer/period+3,4) - 1.8)/0.4
        lean = gait["lean"] * (math.cos(fraction * math.pi))
    elif (fraction >= 0.2 and fraction <= 1.8):
        lean = -gait["lean"]
    elif (fraction >= 2.2 and fraction <= 3.8):
        lean = gait["lean"]

    set_Legs( \
        {"ForeLeft":  np.array([ lean-20, trans["ForeLeft"],  elevate["ForeLeft"]]), \
         "BackLeft":  np.array([ lean+10, -trans["BackLeft"],  elevate["BackLeft"]]), \
         "ForeRight": np.array([ lean+20, trans["ForeRight"], elevate["ForeRight"]]), \
         "BackRight": np.array([ lean-10, -trans["BackRight"], elevate["BackRight"]]) \
         })

def seek(goal, elapse):
    set_Legs({"ForeLeft":goal,"BackLeft":goal,"ForeRight":goal,"BackRight":goal})
    time.sleep(elapse)

def relax():
    for actuator in cb.actuatorMap.values():
        actuator.torque_enable = False

def engage():
    for actuator in cb.actuatorMap.values():
        actuator.torque_enable = True
