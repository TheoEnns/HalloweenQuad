import Q_Structure as qS

##  Leg Setup
heightHigh = 25
heightLow = -100
transX = 45
transY = 27

# safeF_heightHigh = 27
# safeF_heightLow = -0
# safeF_trans = 55
# safeF_lean = 15

# safeF_heightHigh = 20
# safeF_heightLow = -10
# safeF_trans = 40
# safeF_lean = 15

# # Good Params for low walk
# g_forward = {"heightHigh":20.0, "heightLow":-10.0, "trans":40.0, "lean":15.0}
# g_backward = {"heightHigh":20.0, "heightLow":-10.0, "trans":40.0, "lean":15.0}
# g_sideward = {"heightHigh":20.0, "heightLow":-10.0, "trans":25.0, "lean":-15.0}

# Good Params for high walk

g_forward = {"heightHigh":-20.0, "heightLow":-50.0, "trans":40.0, "lean":15.0}
g_backward = {"heightHigh":-20.0, "heightLow":-50.0, "trans":40.0, "lean":15.0}
g_sideward = {"heightHigh":-20.0, "heightLow":-50.0, "trans":30.0, "lean":15.0}

centerX = (qS.HIP_FAR_X + qS.HIP_NEAR_X) / 2
centerY = qS.HIP_Y + qS.FEMUR_WIDTH
centerZ = -117.47 #mm