import math
import numpy as np
from numpy.linalg import norm
# import matplotlib

import Q_Structure as qS

class QuadLeg:
    """ Robot leg """
    def __init__(self, hipVectorA, hipVectorB):
        self.hipA_angle = 0.0
        self.hipB_angle = 0.0
        self.hipR_angle = 0.0
        self.hipAPoint = hipVectorA
        self.hipBPoint = hipVectorB
        self.kneeAPoint = np.zeros(3)
        self.kneeBPoint = np.zeros(3)
        self.anklePoint = np.zeros(3)
        self.groundPoint = np.zeros(3)
        self.groundNormal = np.zeros(3)

    def getEffectorSolution(self, groundNormal, groundPoint):
        anklePoint = groundNormal*qS.FOOT_RADIUS + groundPoint
        tiltProjAnkle = self.hipAPoint - anklePoint
        tiltProjAnkle[1] = 0
        a = math.asin(qS.FEMUR_WIDTH/norm(tiltProjAnkle))
        b = math.atan2(math.fabs(tiltProjAnkle[2]),math.fabs(tiltProjAnkle[0]) )
        femur_tilt = math.asin(qS.FEMUR_WIDTH/norm(tiltProjAnkle)) + math.atan2(tiltProjAnkle[2],tiltProjAnkle[0] )
        #legTiltLength = math.sqrt(math.pow(norm(tiltProjAnkle),2) - qS.FEMUR_WIDTH*qS.FEMUR_WIDTH)
        femurVect = np.array([ -qS.FEMUR_WIDTH*math.sin(femur_tilt), 0, qS.FEMUR_WIDTH*math.cos(femur_tilt)])
        femurA = femurVect + self.hipAPoint
        femurB = femurVect + self.hipBPoint
        hipR_angle = femur_tilt - math.pi/2

        #Get Hip Orientation
        hipZ = np.cross( (femurB - anklePoint), (femurA - anklePoint) )
        hipZ = hipZ/norm(hipZ)
        hipY = np.array([0,-1,0]) if (femurA[1] > femurB[1]) else np.array([0,1,0])
        hipX = np.cross( hipY, hipZ)

        #Get HipA solution
        planarAnkle = np.array([np.dot((femurA - anklePoint),hipX), np.dot((femurA - anklePoint),hipY)])
        n_planarAnkle = norm(planarAnkle)
        if (qS.FEMUR_LENGTH + qS.TIBIA_LENGTH) < n_planarAnkle:
            raise Exception('Over Extension')
        if (qS.TIBIA_LENGTH - qS.FEMUR_LENGTH) > n_planarAnkle:
            raise Exception('Under Extension')
        if (qS.FEMUR_LENGTH + qS.TIBIA_LENGTH) == n_planarAnkle:
            self.hipA_angle = math.acos(np.dot(planarAnkle,hipY)/( n_planarAnkle )) #norm(hipY) is 1
        tibiaSq = math.pow(qS.TIBIA_LENGTH,2)
        femurSq = math.pow(qS.FEMUR_LENGTH,2)
        mid_lens = (tibiaSq - femurSq + math.pow(n_planarAnkle,2))/(2*n_planarAnkle)
        semichord = math.sqrt( tibiaSq - math.pow(mid_lens,2))
        centerMass = planarAnkle + mid_lens*(np.zeros(2) - planarAnkle)/n_planarAnkle
        kneeOpt1 = np.array([centerMass[0] + semichord*(planarAnkle[1])/n_planarAnkle,\
                             centerMass[1] - semichord*(planarAnkle[0])/n_planarAnkle])
        kneeOpt2 = np.array([centerMass[0] - semichord*(planarAnkle[1])/n_planarAnkle,\
                             centerMass[1] + semichord*(planarAnkle[0])/n_planarAnkle])
        spread = math.acos(np.dot(kneeOpt1, kneeOpt2) / (norm(kneeOpt1)*norm(kneeOpt2)))
        if spread>0:
            hipA_angle = math.atan2(kneeOpt1[1],kneeOpt1[0])
            kneeAPoint = - kneeOpt1[0]*hipX - kneeOpt1[1]*hipY
        else:
            hipA_angle = math.atan2(kneeOpt2[1],kneeOpt2[0])
            kneeAPoint = - kneeOpt2[0]*hipX - kneeOpt2[1]*hipY

        # Get HipB solution
        planarAnkle = np.array([np.dot((femurB - anklePoint), hipX), np.dot((femurB - anklePoint), hipY)])
        n_planarAnkle = norm(planarAnkle)
        if (qS.FEMUR_LENGTH + qS.TIBIA_LENGTH) < n_planarAnkle:
            raise Exception('Over Extension')
        if (qS.TIBIA_LENGTH - qS.FEMUR_LENGTH) > n_planarAnkle:
            raise Exception('Under Extension')
        if (qS.FEMUR_LENGTH + qS.TIBIA_LENGTH) == n_planarAnkle:
            self.hipB_angle = math.acos(np.dot(planarAnkle,hipY)/( n_planarAnkle )) #norm(hipY) is 1
        tibiaSq = math.pow(qS.TIBIA_LENGTH,2)
        femurSq = math.pow(qS.FEMUR_LENGTH,2)
        mid_lens = (tibiaSq - femurSq + math.pow(n_planarAnkle,2))/(2*n_planarAnkle)
        semichord = math.sqrt( tibiaSq - math.pow(mid_lens,2))
        centerMass = planarAnkle + mid_lens*(np.zeros(2) - planarAnkle)/n_planarAnkle
        kneeOpt1 = np.array([centerMass[0] + semichord*(planarAnkle[1])/n_planarAnkle,\
                             centerMass[1] - semichord*(planarAnkle[0])/n_planarAnkle])
        kneeOpt2 = np.array([centerMass[0] - semichord*(planarAnkle[1])/n_planarAnkle,\
                             centerMass[1] + semichord*(planarAnkle[0])/n_planarAnkle])
        spread = math.acos(np.dot(kneeOpt1, kneeOpt2) / (norm(kneeOpt1)*norm(kneeOpt2)))
        if spread < 0:
            hipB_angle = math.atan2(kneeOpt1[1],kneeOpt1[0])
            kneeBPoint = - kneeOpt1[0]*hipX - kneeOpt1[1]*hipY
        else:
            hipB_angle = math.atan2(kneeOpt2[1],kneeOpt2[0])
            kneeBPoint = - kneeOpt2[0]*hipX - kneeOpt2[1]*hipY

        hipA_angle = math.pi/2 - math.fmod(   hipA_angle, math.pi)
        hipB_angle = math.pi/2 - math.fmod( - hipB_angle, math.pi)
        return {"hipA_angle":hipA_angle, "hipB_angle":hipB_angle, "hipR_angle":hipR_angle, "anklePoint":anklePoint, "kneeAPoint":kneeAPoint, "kneeBPoint":kneeBPoint}


# if __name__ == "__main__":
#     leg = QuadLeg(np.array([qS.FEMUR_WIDTH,14,0]), np.array([qS.FEMUR_WIDTH,-14,0]))
#     print "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\thipA_angle (y=14), \t\t\thipB_angle (y=-14)"
#     print "Femurs at center test:\t\t\t\t\t\t\t\t\t\t\t\t\t ", leg.getEffectorSolution(np.array([0,0,1]), np.array([0,0,-117.47]))
#     print "Femurs at tilted slightly forward of center test:\t\t\t\t\t\t ", leg.getEffectorSolution(np.array([0,0,1]), np.array([0,10,-117.47]))
#     print "Femurs at tilted slightly backward of center test:\t\t\t\t\t\t ", leg.getEffectorSolution(np.array([0,0,1]), np.array([0,-10,-117.47]))
#     print "Femurs at slightly above center test:\t\t\t\t\t\t\t\t\t ", leg.getEffectorSolution( np.array([0,0,1]), np.array([0,0,-107.47]))
#     print "Femurs at slightly below center test:\t\t\t\t\t\t\t\t\t ", leg.getEffectorSolution( np.array([0,0,1]), np.array([0,0,-127.47]))
#
#     flip_leg = QuadLeg(np.array([qS.FEMUR_WIDTH, -14, 0]), np.array([qS.FEMUR_WIDTH, 14, 0]))
#     print "\n\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\thipA_angle (y=-14), \t\t\thipB_angle (y=14)"
#     print "Femurs at center test:\t\t\t\t\t\t\t\t\t\t\t\t\t ", flip_leg.getEffectorSolution(np.array([0,0,1]), np.array([0,0,-117.47]))
#     print "Femurs at tilted slightly forward of center test:\t\t\t\t\t\t ", flip_leg.getEffectorSolution(np.array([0,0,1]), np.array([0,10,-117.47]))
#     print "Femurs at tilted slightly backward of center test:\t\t\t\t\t\t ", flip_leg.getEffectorSolution(np.array([0,0,1]), np.array([0,-10,-117.47]))
#     print "Femurs at slightly above center test:\t\t\t\t\t\t\t\t\t ", flip_leg.getEffectorSolution( np.array([0,0,1]), np.array([0,0,-107.47]))
#     print "Femurs at slightly below center test:\t\t\t\t\t\t\t\t\t ", flip_leg.getEffectorSolution( np.array([0,0,1]), np.array([0,0,-127.47]))
#
#     print "\nFeet 10mm to the right \t\t\t\t\t\t\t\t\t\t\t\t\t\thipA_angle (y=14), \t\t\thipB_angle (y=-14)"
#     print "Femurs at center test:\t\t\t\t\t\t\t\t\t\t\t\t\t ", leg.getEffectorSolution(np.array([0,0,1]), np.array([-10,0,-117.47]))
#     print "Femurs at tilted slightly forward of center test:\t\t\t\t\t\t ", leg.getEffectorSolution(np.array([0,0,1]), np.array([-10,10,-117.47]))
#     print "Femurs at tilted slightly backward of center test:\t\t\t\t\t\t ", leg.getEffectorSolution(np.array([0,0,1]), np.array([-10,-10,-117.47]))
#     print "Femurs at slightly above center test:\t\t\t\t\t\t\t\t\t ", leg.getEffectorSolution( np.array([0,0,1]), np.array([-10,0,-107.47]))
#     print "Femurs at slightly below center test:\t\t\t\t\t\t\t\t\t ", leg.getEffectorSolution( np.array([0,0,1]), np.array([-10,0,-127.47]))
#
#     print "\nFeet 10mm to the left \t\t\t\t\t\t\t\t\t\t\t\t\t\thipA_angle (y=14), \t\t\thipB_angle (y=-14)"
#     print "Femurs at center test:\t\t\t\t\t\t\t\t\t\t\t\t\t ", leg.getEffectorSolution(np.array([0,0,1]), np.array([10,0,-117.47]))
#     print "Femurs at tilted slightly forward of center test:\t\t\t\t\t\t ", leg.getEffectorSolution(np.array([0,0,1]), np.array([10,10,-117.47]))
#     print "Femurs at tilted slightly backward of center test:\t\t\t\t\t\t ", leg.getEffectorSolution(np.array([0,0,1]), np.array([10,-10,-117.47]))
#     print "Femurs at slightly above center test:\t\t\t\t\t\t\t\t\t ", leg.getEffectorSolution( np.array([0,0,1]), np.array([10,0,-107.47]))
#     print "Femurs at slightly below center test:\t\t\t\t\t\t\t\t\t ", leg.getEffectorSolution( np.array([0,0,1]), np.array([10,0,-127.47]))
#     """
#                                                                                     hipA_angle (y=14), 			hipB_angle (y=-14)
#     Femurs at center test:													  {'kneeAPoint': array([ -0.00000000e+00,   5.99999999e+01,  -3.07450934e-03]), 'hipB_angle': -5.1241822328185194e-05, 'anklePoint': array([  0.  ,   0.  , -94.47]), 'hipA_angle': -5.1241822328185194e-05, 'kneeBPoint': array([  0.00000000e+00,  -5.99999999e+01,  -3.07450934e-03]), 'hipR_angle': 0.0}
#     Femurs at tilted slightly forward of center test:						  {'kneeAPoint': array([  0.        ,  59.55182966,   7.31980768]), 'hipB_angle': -0.13746135862052222, 'anklePoint': array([  0.  ,  10.  , -94.47]), 'hipA_angle': 0.12230145702215056, 'kneeBPoint': array([  0.        , -59.4340233 ,  -8.22173186]), 'hipR_angle': 0.0}
#     Femurs at tilted slightly backward of center test:						  {'kneeAPoint': array([ -0.        ,  59.4340233 ,  -8.22173186]), 'hipB_angle': 0.12230145702215056, 'anklePoint': array([  0.  , -10.  , -94.47]), 'hipA_angle': -0.13746135862052222, 'kneeBPoint': array([  0.        , -59.55182966,   7.31980768]), 'hipR_angle': 0.0}
#     Femurs at slightly above center test:									  {'kneeAPoint': array([  0.        ,  59.02923685,  10.74938125]), 'hipB_angle': 0.18012886418775298, 'anklePoint': array([  0.  ,   0.  , -84.47]), 'hipA_angle': 0.18012886418775298, 'kneeBPoint': array([  0.        , -59.02923685,  10.74938125]), 'hipR_angle': 0.0}
#     Femurs at slightly below center test:									  {'kneeAPoint': array([ -0.        ,  59.25522186,  -9.42436638]), 'hipB_angle': -0.15772593041503002, 'anklePoint': array([   0.  ,    0.  , -104.47]), 'hipA_angle': -0.15772593041503002, 'kneeBPoint': array([  0.        , -59.25522186,  -9.42436638]), 'hipR_angle': 0.0}
#
#                                                                                 hipA_angle (y=-14), 			hipB_angle (y=14)
#     Femurs at center test:													  {'kneeAPoint': array([ -0.00000000e+00,  -5.99999999e+01,  -3.07450934e-03]), 'hipB_angle': -5.1241822328185194e-05, 'anklePoint': array([  0.  ,   0.  , -94.47]), 'hipA_angle': -5.1241822328185194e-05, 'kneeBPoint': array([  0.00000000e+00,   5.99999999e+01,  -3.07450934e-03]), 'hipR_angle': 0.0}
#     Femurs at tilted slightly forward of center test:						  {'kneeAPoint': array([ -0.        , -59.4340233 ,  -8.22173186]), 'hipB_angle': 0.12230145702215056, 'anklePoint': array([  0.  ,  10.  , -94.47]), 'hipA_angle': -0.13746135862052222, 'kneeBPoint': array([  0.        ,  59.55182966,   7.31980768]), 'hipR_angle': 0.0}
#     Femurs at tilted slightly backward of center test:						  {'kneeAPoint': array([  0.        , -59.55182966,   7.31980768]), 'hipB_angle': -0.13746135862052222, 'anklePoint': array([  0.  , -10.  , -94.47]), 'hipA_angle': 0.12230145702215056, 'kneeBPoint': array([  0.        ,  59.4340233 ,  -8.22173186]), 'hipR_angle': 0.0}
#     Femurs at slightly above center test:									  {'kneeAPoint': array([  0.        , -59.02923685,  10.74938125]), 'hipB_angle': 0.18012886418775298, 'anklePoint': array([  0.  ,   0.  , -84.47]), 'hipA_angle': 0.18012886418775298, 'kneeBPoint': array([  0.        ,  59.02923685,  10.74938125]), 'hipR_angle': 0.0}
#     Femurs at slightly below center test:									  {'kneeAPoint': array([ -0.        , -59.25522186,  -9.42436638]), 'hipB_angle': -0.15772593041503002, 'anklePoint': array([   0.  ,    0.  , -104.47]), 'hipA_angle': -0.15772593041503002, 'kneeBPoint': array([  0.        ,  59.25522186,  -9.42436638]), 'hipR_angle': 0.0}
#
#     Feet 10mm to the right 														hipA_angle (y=14), 			hipB_angle (y=-14)
#     Femurs at center test:													  {'kneeAPoint': array([ -0.41607309,  59.86468526,  -4.00578855]), 'hipB_angle': -0.06717281988092871, 'anklePoint': array([-10.  ,   0.  , -94.47]), 'hipA_angle': -0.06717281988092871, 'kneeBPoint': array([ -0.41607309, -59.86468526,  -4.00578855]), 'hipR_angle': -0.10349683113004615}
#     Femurs at tilted slightly forward of center test:						  {'kneeAPoint': array([  0.3052218 ,  59.92722027,   2.9385558 ]), 'hipB_angle': -0.19752744354043728, 'anklePoint': array([-10.  ,  10.  , -94.47]), 'hipA_angle': 0.04925932978073155, 'kneeBPoint': array([ -1.21647253, -58.83328816, -11.71172058]), 'hipR_angle': -0.10349683113004615}
#     Femurs at tilted slightly backward of center test:						  {'kneeAPoint': array([ -1.21647253,  58.83328816, -11.71172058]), 'hipB_angle': 0.04925932978073155, 'anklePoint': array([-10.  , -10.  , -94.47]), 'hipA_angle': -0.19752744354043728, 'kneeBPoint': array([  0.3052218 , -59.92722027,   2.9385558 ]), 'hipR_angle': -0.10349683113004615}
#     Femurs at slightly above center test:									  {'kneeAPoint': array([  0.64406786,  59.73744085,   5.56985975]), 'hipB_angle': 0.09358612242477182, 'anklePoint': array([-10.  ,   0.  , -84.47]), 'hipA_angle': 0.09358612242477182, 'kneeBPoint': array([  0.64406786, -59.73744085,   5.56985975]), 'hipR_angle': -0.11512318490479467}
#     Femurs at slightly below center test:									  {'kneeAPoint': array([ -1.19182871,  58.63998026, -12.64643266]), 'hipB_angle': -0.21332205286340167, 'anklePoint': array([ -10.  ,    0.  , -104.47]), 'hipA_angle': -0.21332205286340167, 'kneeBPoint': array([ -1.19182871, -58.63998026, -12.64643266]), 'hipR_angle': -0.09396475438466267}
#
#     Feet 10mm to the left 														hipA_angle (y=14), 			hipB_angle (y=-14)
#     Femurs at center test:													  {'kneeAPoint': array([ -0.3526131 ,  59.91006495,   3.26493203]), 'hipB_angle': 0.05475932853984644, 'anklePoint': array([ 10.  ,   0.  , -94.47]), 'hipA_angle': 0.05475932853984644, 'kneeBPoint': array([ -0.3526131 , -59.91006495,   3.26493203]), 'hipR_angle': 0.10758314407918368}
#     Femurs at tilted slightly forward of center test:						  {'kneeAPoint': array([ -1.16771592,  59.00621493,  10.8121709 ]), 'hipB_angle': -0.08886402350300493, 'anklePoint': array([ 10.  ,  10.  , -94.47]), 'hipA_angle': 0.18225811491186428, 'kneeBPoint': array([  0.57175718, -59.76325142,  -5.29404134]), 'hipR_angle': 0.10758314407918368}
#     Femurs at tilted slightly backward of center test:						  {'kneeAPoint': array([  0.57175718,  59.76325142,  -5.29404134]), 'hipB_angle': 0.18225811491186428, 'anklePoint': array([ 10.  , -10.  , -94.47]), 'hipA_angle': -0.08886402350300493, 'kneeBPoint': array([ -1.16771592, -59.00621493,  10.8121709 ]), 'hipR_angle': 0.10758314407918368}
#     Femurs at slightly above center test:									  {'kneeAPoint': array([ -1.81879909,  58.07149123,  14.97978224]), 'hipB_angle': 0.2542262249792211, 'anklePoint': array([ 10.  ,   0.  , -84.47]), 'hipA_angle': 0.2542262249792211, 'kneeBPoint': array([ -1.81879909, -58.07149123,  14.97978224]), 'hipR_angle': 0.12082550154616056}
#     Femurs at slightly below center test:									  {'kneeAPoint': array([  0.6586993 ,  59.61320184,  -6.76995432]), 'hipB_angle': -0.11360963363852683, 'anklePoint': array([  10.  ,    0.  , -104.47]), 'hipA_angle': -0.11360963363852683, 'kneeBPoint': array([  0.6586993 , -59.61320184,  -6.76995432]), 'hipR_angle': 0.0969921521358521}
#     """