"""Protected-workspace transfer: a rigid component must never pass over a marked work-cell.

Robocurve / YAM Isaac Lab-Arena task-file interface (standalone task.py style):
  SCENE, OBJECTS, success(objects), score(objects), distance(objects), demo().

Safety scope is the CARRIED OBJECT's planar footprint vs a fixed vertical
keep-out column. It does NOT claim whole-arm self-collision or force safety.

All geometry is in metres, in the station's world frame (x forward from arms,
y to robot's left). The demonstration uses the LEFT arm only; the task is
compatible with the bimanual station, but is not a bimanual-coordination test.
"""
from __future__ import annotations

import math
import sys
import json
import os

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World-space task geometry, validated first in deterministic offline tests.
START_XY = (0.160, 0.150)
GOAL_XY = (0.265, -0.070)
CUBE_EDGE = 0.040
CUBE_MASS_KG = 0.050
PAD_XY = (0.090, 0.090)
PAD_THICKNESS = 0.001
ZONE_CENTER_XY = (0.225, 0.040)
ZONE_SIZE_XY = (0.060, 0.100)
ZONE_X = (ZONE_CENTER_XY[0] - ZONE_SIZE_XY[0]/2, ZONE_CENTER_XY[0] + ZONE_SIZE_XY[0]/2)
ZONE_Y = (ZONE_CENTER_XY[1] - ZONE_SIZE_XY[1]/2, ZONE_CENTER_XY[1] + ZONE_SIZE_XY[1]/2)
# A conservative circular XY bound contains the cube's footprint regardless of yaw.
CUBE_RADIUS = CUBE_EDGE / math.sqrt(2)
CLEARANCE_M = 0.004
PADDING = CUBE_RADIUS + CLEARANCE_M
GOAL_TOL = 0.025
SETTLE_TOL = 0.009
LIFT = 0.035
PROGRESS = 0.085

COMPONENT = Box(
    "component", pos=(*START_XY, TOP + PAD_THICKNESS + CUBE_EDGE/2),
    size=(CUBE_EDGE,)*3, mass=CUBE_MASS_KG, color=(0.12, 0.35, 0.82),
)
# Thin visible fixed markers, not obstacles to be driven around by collisions.
TARGET_PAD = Box(
    "target_pad", pos=(*GOAL_XY, TOP + PAD_THICKNESS/2),
    size=(*PAD_XY, PAD_THICKNESS), color=(0.12, 0.62, 0.23),
)
NO_GO_PAD = Box(
    "restricted_pad", pos=(*ZONE_CENTER_XY, TOP + PAD_THICKNESS/2),
    size=(*ZONE_SIZE_XY, PAD_THICKNESS), color=(0.88, 0.15, 0.12),
)
OBJECTS = [COMPONENT, TARGET_PAD, NO_GO_PAD]
SUCCESS_HOLD_S = 0.6
SCORE_OVER = "end"


def _sim_clock():
    """Find YAM's per-episode `_t` from the caller stack (optional, internal).

    This mirrors the approach used by accepted safety tasks supplied in the
    repository snapshot. If YAM changes, guard resets must be revalidated.
    """
    try:
        frame = sys._getframe(1)
    except ValueError:
        return None
    for _ in range(20):
        if frame is None:
            break
        owner = frame.f_locals.get("self")
        if owner is not None and hasattr(owner, "_t") and hasattr(owner, "task"):
            try:
                return id(owner), float(owner._t)
            except (TypeError, ValueError):
                return None
        frame = frame.f_back
    return None


def segment_hits_rect(a, b, pad=PADDING):
    """Liang-Barsky line segment / inflated rectangle intersection.

    The swept disc check catches crossing between sampled robot states.
    The padding includes object size plus a 4 mm additional clearance.
    """
    xmin, xmax = ZONE_X[0]-pad, ZONE_X[1]+pad
    ymin, ymax = ZONE_Y[0]-pad, ZONE_Y[1]+pad
    x0,y0 = a
    dx,dy = b[0]-x0,b[1]-y0
    lo,hi = 0.0,1.0
    for p,q in ((-dx,x0-xmin),(dx,xmax-x0),(-dy,y0-ymin),(dy,ymax-y0)):
        if abs(p) < 1.e-12:
            if q < 0: return False
        else:
            r=q/p
            if p < 0: lo=max(lo,r)
            else: hi=min(hi,r)
            if lo > hi: return False
    return True


class SafetyMonitor:
    """Latches exclusion-zone crossings over an entire trial, not just at end.

    update() accepts a simple (x,y,z) and low_z so most testing needs no Isaac.
    The live YAM hooks call observe(objects). Simulator clock rollbacks
    signal new trials. Without the simulator clock, explicit reset() is used.
    """
    def __init__(self):
        self.reset()
        self._clock = None
        self._last_objects = None

    def reset(self):
        self.last_xy = None
        self.breached = False
        self.lifted = False
        self.moved = False
        self.steps = 0
        self.trial_index = getattr(self, "trial_index", 0) + 1

    def update(self, xyz, low_z):
        x,y,z = tuple(map(float, xyz))
        if not all(map(math.isfinite, (x,y,z,float(low_z)))):
            raise ValueError("non-finite tracked object pose")
        xy=(x,y)
        if self.last_xy is not None:
            if segment_hits_rect(self.last_xy, xy):
                self.breached = True
        elif segment_hits_rect(xy,xy):
            self.breached = True
        self.last_xy = xy
        self.steps += 1
        self.lifted |= float(low_z) > TOP + PAD_THICKNESS + LIFT
        self.moved |= (self.lifted and math.dist(xy, START_XY) >= PROGRESS)

    def observe(self, objects):
        clock = _sim_clock()
        if clock is not None:
            if self._clock is not None and (clock[0] != self._clock[0] or clock[1] < self._clock[1] - 1.e-9):
                self.reset()
            if self._clock == clock:
                return
            self._clock = clock
        else:
            # Calling success() then score() in an offline test on the same mapping is one tick.
            if objects is self._last_objects:
                return
        self._last_objects = objects
        component = objects["component"]
        self.update(component.pos, component.low[2])
        trace_path = os.environ.get("KEEP_OUT_TRACE")
        if trace_path:
            rec = {"trial": self.trial_index, "t": clock[1] if clock else None,
                   "cube_xyz": [float(v) for v in component.pos],
                   "cube_low_z": float(component.low[2]),
                   "zone_breached": self.breached, "lifted": self.lifted,
                   "moved": self.moved, "current_score": self.grade(objects)}
            with open(trace_path, "a", encoding="utf8") as f:
                f.write(json.dumps(rec) + "\n")

    def grade(self, objects):
        state=objects["component"]
        pos=tuple(map(float,state.pos))
        on_target = (
            math.hypot(pos[0]-GOAL_XY[0], pos[1]-GOAL_XY[1]) <= GOAL_TOL
            and abs(float(state.low[2]) - (TOP + PAD_THICKNESS)) <= SETTLE_TOL
            and state.resting()
        )
        if self.breached:
            return 0.0
        if on_target and self.lifted:
            return 1.0
        if self.moved:
            return 0.5
        if self.lifted:
            return 0.25
        return 0.0

MONITOR = SafetyMonitor()


def success(objects):
    MONITOR.observe(objects)
    return MONITOR.grade(objects) == 1.0


def score(objects):
    MONITOR.observe(objects)
    return MONITOR.grade(objects)


def distance(objects):
    pos=objects["component"].pos
    return math.hypot(float(pos[0])-GOAL_XY[0],float(pos[1])-GOAL_XY[1])


# Scripted baseline: depart to the near side of the red rectangle, cross below
# its y_min, then approach the green pad. Waypoints are only desired TCP poses;
# actual object movement MUST be inspected in the sim before scoring claims.
GRASP_Z = TOP + PAD_THICKNESS + CUBE_EDGE/2 + 0.004
TRAVEL_Z = TOP + 0.140
VIA_1 = (0.115, 0.145)
VIA_2 = (0.115, -0.075)
PLACE_Z = TOP + PAD_THICKNESS + CUBE_EDGE/2 + 0.004


def _w(seconds, xy, z, finger, label):
    return Waypoint(seconds, (xy[0],xy[1],z), finger, label)


def demo():
    return {"left": [
        _w(1.2,START_XY,TRAVEL_Z,FINGER_OPEN,"approach start"),
        _w(1.2,START_XY,GRASP_Z,FINGER_OPEN,"descend"),
        _w(0.7,START_XY,GRASP_Z,FINGER_CLOSED,"grasp"),
        _w(1.4,START_XY,TRAVEL_Z,FINGER_CLOSED,"lift"),
        _w(1.3,VIA_1,TRAVEL_Z,FINGER_CLOSED,"detour 1"),
        _w(1.7,VIA_2,TRAVEL_Z,FINGER_CLOSED,"detour 2"),
        _w(1.5,GOAL_XY,TRAVEL_Z,FINGER_CLOSED,"carry to target"),
        _w(1.5,GOAL_XY,PLACE_Z,FINGER_CLOSED,"lower"),
        _w(0.9,GOAL_XY,PLACE_Z,FINGER_OPEN,"release"),
        _w(1.0,GOAL_XY,TRAVEL_Z,FINGER_OPEN,"retreat"),
        _w(1.0,GOAL_XY,TRAVEL_Z,FINGER_OPEN,"settle"),
    ]}
