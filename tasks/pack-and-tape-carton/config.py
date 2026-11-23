"""Packing layout and finite outer tape layer, SI units."""
import math
from model import *
TOP=.7475
CENTER=(.32,0.,TOP+.006)
CUTTER_START=(.36,-.22,TOP+.020)
ROLL_START=(.14,-.15,TOP+.0025+TAPE_WIDTH/2)
CELL_LENGTH=.005
CELLS=72
# Explicitly prepared ordinary-tape start: last 60 mm is already unwound.
# No added grasp tab, no carton bond and no credit for this starting material.
PREUNWOUND_CELLS=12
PREUNWOUND_LENGTH=PREUNWOUND_CELLS*CELL_LENGTH
WOUND_CELLS=CELLS-PREUNWOUND_CELLS
ACTIVE_LENGTH=CELLS*CELL_LENGTH
ACTIVE_INNER_RADIUS=math.sqrt(OUTER_RADIUS**2-ACTIVE_LENGTH*TAPE_THICKNESS/math.pi)
BOARD=.003
# Products are preloaded, supported by two overlapping 3 mm bottom flaps.
PRELOADED=True
FLOOR_SURFACE_Z=CENTER[2]+1.5*BOARD
PRODUCTS=[('product_a',(.275,0.,FLOOR_SURFACE_Z+.015+.0001),(.080,.050,.030),.10,(.10,.37,.85)),('product_b',(.365,0.,FLOOR_SURFACE_Z+.015+.0001),(.060,.040,.030),.07,(.93,.28,.12))]


def wound_cell(i):
 # Reference Archimedean spiral, before the outer end is partially unwound.
 radius=math.sqrt(ACTIVE_INNER_RADIUS**2+(i+.5)*CELL_LENGTH*TAPE_THICKNESS/math.pi)
 theta=(i+.5)*CELL_LENGTH/OUTER_RADIUS
 p=(ROLL_START[0]+radius*math.cos(theta),ROLL_START[1]+radius*math.sin(theta),ROLL_START[2])
 return p,theta+math.pi/2


def tape_cell(i):
 if not 0 <= i < CELLS:raise IndexError('film cell outside finite ribbon')
 if i < WOUND_CELLS:return wound_cell(i)
 # A straight, ordinary ribbon continuation tangent to the last wound cell.
 # Cell centres remain exactly 5 mm apart along this free tail.
 point,yaw=wound_cell(WOUND_CELLS-1)
 distance=(i-WOUND_CELLS+1)*CELL_LENGTH
 return (point[0]+distance*math.cos(yaw),point[1]+distance*math.sin(yaw),point[2]),yaw
