"""Safety-gated reference trajectory planners.

Pure Python, deterministic and independently testable. The planners choose
robot TCP waypoints; they never write fixture joints or scorer state. Their
reported progress is based exclusively on the measured guard articulation.

The native simulator remains the authority for whether robot-guard contacts
actually occur. A commanded waypoint is never evidence of closure/latching.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import geometry as g


class ReferenceAbort(RuntimeError):
    """Safe abort of an unqualified reference, with all native evidence retained."""


def door_grip_command(aperture: float, support_n: float) -> float:
    """Use the weaker opposing pad's measured load to regulate the door pinch."""
    if not (math.isfinite(support_n) and support_n >= 0):
        raise ReferenceAbort('invalid_door_pad_force')
    # One aperture unit is a full finger stroke. Never open a loaded grasp.
    # The 1.2 N setpoint is for control only. The independent contact audit
    # still requires genuine opposing contacts on every working action.
    correction = min(.003, .0015 * max(0., 1.2 - support_n))
    return min(aperture, max(.14, aperture - correction))


@dataclass
class DoorClosurePlanner:
    """Monotonic rotation request with bounded virtual lead and stall watchdog.

    Repeatedly setting a goal to `measured + epsilon` cannot close a door that
    sticks: the goal itself stops moving. Here the planned goal progresses
    monotonically toward zero, while a lead bound and stall watchdog prevent
    indefinite displacement/pushing against an obstacle.
    """
    measured_at_start: float
    step_rad: float = math.radians(0.34)   # 6.8 deg/s commanded
    max_lead_rad: float = math.radians(9.0)
    min_progress_rad: float = math.radians(0.55)
    stall_limit_ticks: int = 44           # ~2.2s at 20 Hz
    timeout_ticks: int = 300              # 15s maximum close trial
    ticks: int = 0
    target_rad: float | None = None
    best_observed_rad: float | None = None
    last_progress_tick: int = 0
    complete: bool = False

    def __post_init__(self):
        if not math.isfinite(self.measured_at_start):
            raise ReferenceAbort('nonfinite_initial_hinge')
        if abs(self.measured_at_start-g.OPEN_RAD)>math.radians(8):
            raise ReferenceAbort('door_not_initially_open')
        self.target_rad=float(self.measured_at_start)
        self.best_observed_rad=float(self.measured_at_start)

    def step(self, hinge: float, bolt: float, switch: float) -> float:
        if not all(map(math.isfinite,(hinge,bolt,switch))):
            raise ReferenceAbort('nonfinite_closure_feedback')
        if switch >= g.SWITCH_ACTIVE and abs(hinge)>g.UNSAFE_SWITCH_OPEN_RAD:
            raise ReferenceAbort('premature_interlock_activation_while_open')
        # Any substantial latch extension before closure invalidates the
        # desired door-first mechanism; do not advance the latch prematurely.
        if bolt > 0.012 and abs(hinge)>g.CLOSED_TOL_RAD:
            raise ReferenceAbort('premature_latch_extension_while_open')
        if hinge < self.measured_at_start - math.radians(7):
            raise ReferenceAbort('door_moving_opposite_closure_direction')
        if abs(hinge)<=math.radians(2.0):
            self.complete=True
            self.target_rad=0.0
            return 0.0
        if self.complete:
            raise ReferenceAbort('guard_reopened_after_closure')
        self.ticks+=1
        if hinge > float(self.best_observed_rad)+self.min_progress_rad:
            self.best_observed_rad=hinge
            self.last_progress_tick=self.ticks
        if self.ticks>self.timeout_ticks:
            raise ReferenceAbort('door_closure_timeout')
        if self.ticks-self.last_progress_tick>self.stall_limit_ticks:
            raise ReferenceAbort('door_closure_stalled_no_measured_progress')
        # Monotone motion: clamp to a physically conservative maximum lead.
        next_nominal=min(0.0,float(self.target_rad)+self.step_rad)
        next_bounded=min(next_nominal,hinge+self.max_lead_rad)
        self.target_rad=max(float(self.target_rad),next_bounded)
        return float(self.target_rad)


@dataclass
class BoltInsertionPlanner:
    """Slide-bolt waypoint planner gated by physical door closure + switch."""
    measured_at_start: float
    step_m: float = 0.0008                 # 16mm/s at 20Hz
    max_lead_m: float = 0.008             # max 8mm beyond sensed slider
    progress_threshold_m: float = 0.0007
    stall_limit_ticks: int = 52
    timeout_ticks: int = 240
    ticks: int = 0
    target_m: float | None = None
    best_observed_m: float | None = None
    last_progress_tick: int = 0
    complete: bool = False

    def __post_init__(self):
        if not math.isfinite(self.measured_at_start):
            raise ReferenceAbort('nonfinite_initial_bolt')
        self.target_m=max(0.0,self.measured_at_start)
        self.best_observed_m=max(0.0,self.measured_at_start)

    def step(self, hinge: float, bolt: float, switch: float) -> float:
        if not all(map(math.isfinite,(hinge,bolt,switch))):
            raise ReferenceAbort('nonfinite_latch_feedback')
        if abs(hinge)>math.radians(2.5):
            raise ReferenceAbort('latch_blocked_door_not_closed')
        if switch < g.SWITCH_ACTIVE:
            raise ReferenceAbort('latch_blocked_interlock_not_engaged')
        if not can_insert(hinge,switch):
            raise ReferenceAbort('bolt_blocked_keeper_clearance')
        if bolt >= g.BOLT_SEATED + 0.006:
            self.complete=True
            self.target_m=min(g.BOLT_TRAVEL,bolt)
            return float(self.target_m)
        self.ticks+=1
        if bolt>float(self.best_observed_m)+self.progress_threshold_m:
            self.best_observed_m=bolt
            self.last_progress_tick=self.ticks
        if self.ticks>self.timeout_ticks:
            raise ReferenceAbort('bolt_insertion_timeout')
        if self.ticks-self.last_progress_tick>self.stall_limit_ticks:
            raise ReferenceAbort('bolt_insertion_stalled_no_measured_progress')
        proposed=min(g.BOLT_TRAVEL-0.001,float(self.target_m)+self.step_m)
        bounded=min(proposed,bolt+self.max_lead_m)
        self.target_m=max(float(self.target_m),bounded)
        return float(self.target_m)


def ready_for_latch(hinge: float, switch: float) -> bool:
    return abs(hinge)<=math.radians(2.5) and switch>=g.SWITCH_ACTIVE

# Shaft and slot dimensions use the hinge frame in the shipped USD. The
# projected circular shaft is wider by 1/cos(theta) in a keeper Y plane.
KEEPER_Y_FACES=(-.184,-.174)
SHAFT_X=-.014
SLOT_LEFT=SHAFT_X-.005
SLOT_RIGHT=SLOT_LEFT+g.KEEPER_WIDTH_M
INSERT_MARGIN=.00035
ALIGN_RAD=math.atan(((SLOT_LEFT+SLOT_RIGHT)/2-SHAFT_X)/(sum(KEEPER_Y_FACES)/2))

def keeper_clearance(angle):
    """Smallest signed metal-to-metal side gap anywhere through the eyelet."""
    if not math.isfinite(angle) or abs(angle)>=math.pi/2:
        raise ReferenceAbort('invalid_hinge_for_keeper_geometry')
    cosine=math.cos(angle)
    radius=g.LOCK_SHAFT_DIAMETER_M/(2*cosine)
    centers=[(SHAFT_X+y*math.sin(angle))/cosine for y in KEEPER_Y_FACES]
    return min(min(centers)-radius-SLOT_LEFT, SLOT_RIGHT-max(centers)-radius)

def can_insert(hinge,switch):
    return ready_for_latch(hinge,switch) and keeper_clearance(hinge)>=INSERT_MARGIN
