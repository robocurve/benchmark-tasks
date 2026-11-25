"""Rack and table constants for the offline reach and torque checks (plan2.py, alone2.py); metres, kg.

World frame: z up, table top at TABLE_TOP; stations face each other along x; the rack is at +y. plan2.py holds the
server's own dimensions (iStarUSA D-118V2-ITX with TC-RAIL-20 chassis members); the server constants below are used
only by plan.py's older sweep. Values marked EST are estimates.
"""
TABLE_TOP = 0.7475            # yam_arena station table top
BASE_Z = 0.75                 # YAM base height
ARM_HALF_SPACING = 0.305      # station arm bases at local y +-0.305

# In Win IW-RF100: chassis only 430 x 43.5 x 365.9 mm (W x H x D); ballasted to 6.0 kg with its inner rails fitted
W, H, D = 0.430, 0.0435, 0.3659
M_SERVER = 6.0                # stated mass, inner rails included
M_SLIDING_RAIL = 0.0          # included in M_SERVER
EAR_HALF_WIDTH = 0.4826 / 2   # EIA-310 19-inch panel width (ears)
EAR_DEPTH = 0.006             # EST ear flange thickness along y

# EIA-310 rack opening and posts
RACK_OPENING = 0.450          # clear width between posts (EIA-310)
POST_X_IN, POST_X_OUT = 0.2255, 0.2855   # EST post profile, x extent from centre
POST_DEPTH = 0.045            # EST post profile depth along y
RACK_DEPTH = 0.56             # EST post-to-post depth set for the 20 in rail kit
RACK_HEIGHT = 0.560           # VEVOR 9U SPT-504-09 overall height
U = 0.04445
RACK_BASE = 0.052             # EST height of U1 bottom above the bench (rack without casters)
SERVER_U = 3                  # server sits in U3
Z_RACKED_BOTTOM = TABLE_TOP + RACK_BASE + (SERVER_U - 1) * U   # chassis bottom when racked

STAND_H = 0.060               # padded stand height (server bottom on the stand)
STAND_W = 0.300               # stand width along x: 67 mm of chassis overhang each side

# Grasp offsets on the chassis (server frame: x across, y from front face toward rear, z up)
FRONT_GRASP_FROM_FRONT = 0.05
REAR_GRASP_FROM_REAR = 0.13
EDGE_INSET = 0.015            # TCP inside the side edge
