"""Every dimension of the Etch A Sketch and of the triangle, in metres, each marked by where it comes from.

Toy frame: origin on the table under the centre of the toy; X to the right as seen by a player sitting at the
knob side, Y away from that player (toward the top of the screen), Z up. NumPy only.

Sources (README has the links):
  [M] measured on a full-size Etch A Sketch and published in a Fab Academy 2018 machine-design page
      (body underside, heights, knob size and gaps, screen borders, stylus travel, knob turns over the travel)
  [B] backlash reported by two knob-driving projects (Cornell ECE 4760, 2009: the stylus "is reluctant to move
      with the knob for about the first 20 degrees" after a reversal; EtchABot: 100 to 140 of 2048 motor steps)
  [R] retailer listings of the Classic (about 454 g)
  [E] estimate
"""

import numpy as np

# --- the toy ---------------------------------------------------------------------------------------------
BODY = (.227, .186)              # [M] "Underside: 186 x 227mm": X (knob to knob direction) x Y
BODY_H = .02368                  # [M] "Height: 23.68mm" (frame face)
RIDGE_H = .02658                 # [M] "Height incl. Ridge: 26.58mm": the raised lip round the screen
CORNER_R = .0078                 # [M] "Fillet corner radius 7.8mm"
BORDER_SIDE = .0328              # [M] frame border beside the screen, left and right
BORDER_TOP = .0328               # [M] frame border above the screen
BORDER_BOTTOM = .0386            # [M] frame border below the screen (the knob side)
RIDGE_W = .004                   # [E] width of the lip round the screen
SCREEN = (BODY[0] - 2 * BORDER_SIDE, BODY[1] - BORDER_TOP - BORDER_BOTTOM)          # 161.4 x 114.6 mm
SCREEN_LL = (-BODY[0] / 2 + BORDER_SIDE, -BODY[1] / 2 + BORDER_BOTTOM)              # lower-left corner, toy frame
SCREEN_Z = .0215                 # [E] the grey screen sits 2 mm below the frame face
MASS = .454                      # [R] "about 454 g" (1 lb); listings range 454 to 635 g, the higher ones boxed

KNOB_D = .02815                  # [M] "Knob diameter: 28.15mm"
KNOB_H = .01232                  # [M] "Knob height: 12.32mm"
KNOB_TOP = .04042                # [M] "Base (incl. ridge) to top of knob: 40.42mm"
KNOB_BOTTOM = KNOB_TOP - KNOB_H  # 28.1 mm: a short stem joins the knob to the frame face
KNOB_GAP_SIDE = .00476           # [M] "Gap from knob to side body edge: ~4.76mm"
KNOB_GAP_BOTTOM = .00403         # [M] "Gap from knob to bottom body edge: ~4.03mm"
KNOB_R = KNOB_D / 2
KNOB_X = BODY[0] / 2 - KNOB_GAP_SIDE - KNOB_R            # 94.7 mm either side of the middle
KNOB_Y = -BODY[1] / 2 + KNOB_GAP_BOTTOM + KNOB_R         # 74.9 mm toward the player from the middle
STEM_R = .006                    # [E] the stem under the knob
KNOB_MASS = .008                 # [E]
KNOB_FRICTION = .03              # [E] N m to keep a knob turning (no published figure; see README)
KNOB_DAMPING = .1745             # N m s/rad, capped at KNOB_FRICTION: a friction-like drag (labyrinth recipe)

# --- the drawing mechanism ---------------------------------------------------------------------------------
TRAVEL = (.155, .108)            # [M] "X travel: 155 mm", "Y travel: 108 mm"
TRAVEL_LL = (SCREEN_LL[0] + (SCREEN[0] - TRAVEL[0]) / 2, SCREEN_LL[1] + (SCREEN[1] - TRAVEL[1]) / 2)
MM_PER_TURN = .0355              # [M] "360 deg = ~35.5 mm travel" (and "~10 deg turn per mm")
PER_RAD = MM_PER_TURN / (2 * np.pi)                      # stylus metres per radian of knob
BACKLASH = float(np.radians(20.))                        # [B] dead band after a reversal
LINE_WIDTH = .003                # drawn 3 mm wide so that the overhead camera sees it (the real line is < 1 mm)

# --- the triangle --------------------------------------------------------------------------------------------
START_ON_SCREEN = (.030, .020)   # the dot: 30 mm from the screen's left edge and 20 mm from its bottom edge
START = (SCREEN_LL[0] + START_ON_SCREEN[0], SCREEN_LL[1] + START_ON_SCREEN[1])     # toy frame
BASE_LEN = .080                  # first side, to the right
RISE = .060                      # second side, straight up
SLANT_LEN = float(np.hypot(BASE_LEN, RISE))             # 100 mm back to the dot
CORNERS = np.array(((0., 0.), (BASE_LEN, 0.), (BASE_LEN, RISE)))   # drawing frame: from the dot, X right, Y up


def screen_to_toy(point):
    """A point of the drawing (metres from the start dot, X right, Y up) in the toy frame (x, y)."""
    return np.asarray(START) + np.asarray(point, dtype=float)
