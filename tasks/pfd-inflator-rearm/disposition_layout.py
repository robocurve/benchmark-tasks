"""Metres in calibrated station coordinates, for the inert service training bench."""
import math

STOCK_A = (.340, -.190)
STOCK_B = (.420, -.267)
USED = (.203, .014)
HOLD = (.210, -.155)
HOLD_HALF = (.084,.081)
TABLE = .7475
SEAT_XY = (.310, -.070)
INDEX_DEG = 44.
INDEX_RADIUS = .0127
CAN_RADIUS = .0127
CAN_HEIGHT = .139
CUP_HALF_WIDTH = .023

def witness(angle):
    a = math.radians(angle)
    return (math.cos(a), math.sin(a))
