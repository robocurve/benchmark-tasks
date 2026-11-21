"""Every size, mass, friction and position of the golf-tee scene, with its source. The task file and the scorer both
read from here. Frame: x forward from the arm bases, y left, z up, metres; positions are from the midpoint between the
two arm bases.

Sources (opened 2026-11-20 and 2026-11-21):
- Ball: Rules of Golf, Equipment Rules: mass not greater than 45.93 g, diameter not less than 42.67 mm.
- Tee: Pride Professional Tee System (PTS) ProLength Plus wooden tee, 3-1/4 in (82.55 mm) long (maker's listing).
  Head and shank sizes are not published: 11 mm head and 4.8 mm shank are the usual wooden-tee sizes (estimate).
- Holder: Carl's Place Rubber Tee Holder (sold in 3s, USD 39.99, indoorgolfoutlet.com): "1.5 inches in height, 2
  inches wide at the base", for "your preferred tee type, whether wooden or plastic". It stands loose on the table
  (no mat); its photos show a tube on a flat disc with four rubber fins in a cross-shaped bore.
- Playable height: Golf Guidebook, tee height guide: with half the ball above a 460 cc driver's crown, "the ball is
  sitting about 1.5 to 1.75 inches above the ground" (38-44 mm). The score accepts 38 to 51 mm (up to 2 in) of tee
  above the holder's top. Pushed home (tip on the table) the 82.55 mm tee shows 82.55 - 38.1 = 44.45 mm.
Estimates (no page gives them): the holder's tube width (22 mm) and disc thickness (3 mm), that its bore runs through
to the table, its grip (4 sprung rubber pads, about 6 N to push a tee in, 3 N of squeeze per pad), tee mass,
frictions. Restitution is 0 (the simulator's default): no bounce.
"""

from __future__ import annotations

TOP = 0.7475  # the station table's top (yam_arena)

# -- holder (fixed in the simulation; the real one stands loose on its 2 in rubber disc, and a straight push does not
#    move it) -----------------------------------------------------------------------------------------------------
HOLDER_H = 0.0381  # 1.5 in
HOLDER_TOP = TOP + HOLDER_H
HOLE = (0.33, 0.0)  # the holder's bore axis (world x, y)
HOLE_HALF = 0.0095  # the square opening the pads move in (collision only; the holder is drawn round)
TUBE_HALF = 0.011  # the tube, 22 mm across (estimate)
DISC_R = 0.0254  # the base disc, 2 in across
DISC_T = 0.003  # estimate

# -- the holder's grip: 4 rubber pads on radial sliders, sprung inward (the rubber's squeeze) ----------------------
PAD_IN = 0.0020  # the pads' faces 2.0 mm from the axis at rest: a 4.8 mm shank pushes each 0.4 mm out
PAD_DEPTH, PAD_W, PAD_H = 0.004, 0.0036, 0.030  # radial, tangential, tall; from 2 to 32 mm below the top
PAD_TRAVEL = 0.003
PAD_PRELOAD = 0.002  # the spring's target 2 mm inside the stop
PAD_K = 1250.0  # N/m: 2.5 N against the stop, 3.0 N on a shank (estimate)
PAD_DAMP = 4.0
PAD_MASS = 0.002
PAD_MU = (0.5, 0.45)  # wood on rubber (estimate): 4 x 0.45 x 3 N, about 6 N to push a tee in

# -- tee (body origin at the tip, z along the tee toward the head) -----------------------------------------------
TEE_L = 0.08255  # 3-1/4 in
TEE_MASS = 0.0014  # estimate (birch, 1.4 g)
SHANK_R = 0.0024
HEAD_R = 0.0055
CUP_RING_R = 0.0046  # the cup's rim: the ball sits on a 9.2 mm circle
CUP_BEAD_R = 0.0006
TEE_MU = (0.5, 0.45)
TEE_START_TIP = (0.40, -0.155)  # lying on the table, tip toward -y (the right arm), head toward the holder

# -- ball -------------------------------------------------------------------------------------------------------------
BALL_R = 0.021335
BALL_MASS = 0.04593
BALL_MU = (0.5, 0.4)  # ionomer cover on wood and on the gripper pads (estimate)
BALL_START = (0.42, -0.03)  # on the table, beyond the holder (out of the arms' way)

# -- scoring (the scoring text, number for number) -----------------------------------------------------------------
UPRIGHT_DEG = 5.0
HEIGHT_RANGE = (0.038, 0.051)  # the tee's top above the holder's top
IN_HOLDER_M = 0.005  # the tee's tip within 5 mm of the holder's axis and below its top
BALL_ON_M = CUP_RING_R  # the ball rests on the cup: its centre inside the cup's ring (sideways)
BALL_REST = 0.02  # m/s
HANDS_CLEAR = 0.03  # every finger point at least 3 cm from the ball and the tee
PART_SCORE = 0.5
