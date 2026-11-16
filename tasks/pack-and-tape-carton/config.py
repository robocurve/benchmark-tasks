"""Packing layout and finite outer tape layer, SI units."""
import math
from model import *
TOP=.7475
CENTER=(.32,0.,TOP+.006)
CUTTER_START=(.36,-.22,TOP+.020)
ROLL_START=(.14,-.15,TOP+.0025+TAPE_WIDTH/2)
CELL_LENGTH=.005
CELLS=72
ACTIVE_LENGTH=CELLS*CELL_LENGTH
ACTIVE_INNER_RADIUS=math.sqrt(OUTER_RADIUS**2-ACTIVE_LENGTH*TAPE_THICKNESS/math.pi)
BOARD=.003
# Products are preloaded, supported by two overlapping 3 mm bottom flaps.
PRELOADED=True
FLOOR_SURFACE_Z=CENTER[2]+1.5*BOARD
PRODUCTS=[('product_a',(.275,0.,FLOOR_SURFACE_Z+.015+.0001),(.080,.050,.030),.10,(.10,.37,.85)),('product_b',(.365,0.,FLOOR_SURFACE_Z+.015+.0001),(.060,.040,.030),.07,(.93,.28,.12))]


def tape_cell(i):
 # Archimedean spiral; end is flush on the outer wrap, with no pull tab.
 radius=math.sqrt(ACTIVE_INNER_RADIUS**2+(i+.5)*CELL_LENGTH*TAPE_THICKNESS/math.pi)
 theta=(i+.5)*CELL_LENGTH/OUTER_RADIUS
 p=(ROLL_START[0]+radius*math.cos(theta),ROLL_START[1]+radius*math.sin(theta),ROLL_START[2])
 return p,theta+math.pi/2
