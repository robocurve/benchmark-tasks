"""Geometry of the child-resistant medicine bottle scene on the yam_arena station table, shared by build_assets.py,
task.py and demo.py.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights are above the table
top (TOP) unless they say "world".

An amber pharmacy vial stands on the table in front of the arms, its push-and-turn cap on. The cap only unscrews
while it is pressed down: pressed past PRESSED, turning it anticlockwise (seen from above) unscrews it, and after
FREE_TURN it comes off. Turned without pressing, it spins freely and nothing unscrews. Pure Python, so the task file
can import it before Isaac Sim starts.
"""

from __future__ import annotations

import math

TOP = 0.7475                            # yam_arena.task_file.TABLE_TOP["station"]

# -- the vial (its origin is the centre of its base) ---------------------------------------------------------------------
BOTTLE_X, BOTTLE_Y = 0.34, 0.0
BODY_R, BODY_H = 0.023, 0.088           # a large pharmacy vial: 4.6 cm across
NECK_R, NECK_TOP = 0.0195, 0.093
BOTTLE_MASS = 0.08                      # with its pills
LABEL_Z = (0.018, 0.070)                # the pharmacy label's band
LABEL_SPAN = math.radians(250)          # around the front (facing the arms)

# -- the cap ---------------------------------------------------------------------------------------------------------------
CAP_R, CAP_H = 0.0255, 0.022            # 5.1 cm across, 2.2 cm tall
CAP_Z0 = 0.084                          # the cap's bottom edge at rest
CAP_C = CAP_Z0 + CAP_H / 2              # its centre at rest
CAP_MASS = 0.008

# -- the mechanism ----------------------------------------------------------------------------------------------------------
TRAVEL = 0.003                          # how far the cap presses down
PRESSED = 0.002                         # pressed past this, turning it unscrews it
SPRING = 800.0                          # N/m: 2.4 N at the bottom of the travel, 1.6 N at PRESSED
FREE_TURN = math.pi                     # half a turn anticlockwise while pressed, and the cap is free
SPIN_DAMPING = 0.0005                   # N m s/rad: the outer shell spinning on its own (the ratchet)
THREAD_DAMPING = 0.05                   # N m s/rad: the thread's drag while pressed, which pulls an unheld vial round
LIFT_MAX = 0.10                         # how far the freed cap can be lifted (it slides up its axis)
OFF = 0.015                             # lifted this far (its edge 6 mm above the neck), the cap is off
SEATED = 0.002                          # lifted less than this, the cap sits on the thread
TIPPED = math.radians(45)               # the vial tilted past this, at any time, is knocked over
