"""Procedural RJ45-like mechanics, SI units. Dimensions require calibration.

This is an explicit approximate connector, not vendor CAD or a verified
standards-compliant RJ45 drawing. No enlarged latch or artificial handles.
"""
TOP=.7475
PANEL_X=.413
PLUG_X=.403
PITCH_Y=.01626
PITCH_Z=.01372 # provisional vertical layout; horizontal pitch is drawing-verified
SERVICE=0
TARGET=4
USED=tuple(i for i in range(8) if i!=TARGET)
CLIP_Z=.004
CLIP_TRAVEL=.0024
RELEASE_TRAVEL=.0018
CLIP_STIFFNESS=250.
LINKS=12
CABLE_RADIUS=.0025
LINK_LENGTH=.016
PANEL_WIDTH=.06591
PANEL_HEIGHT=.02730
PANEL_DEPTH=.03123

def port(i):
    row,col=divmod(i,4)
    return (PLUG_X,(col-1.5)*PITCH_Y,.95+row*PITCH_Z)

def cable_points(i):
    import math
    x,y,z=port(i)
    # Rest-shape approximation: chain is free to bend/collide after reset.
    return [(x-.015-LINK_LENGTH*j,y+.008*math.sin(j*.38+i),z-.004*j)
            for j in range(LINKS+1)]
