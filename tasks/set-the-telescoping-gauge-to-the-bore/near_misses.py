"""Near-successes and unintended strategies, scored offline by factory/check.py (lot A, 40.00 mm bore, unless a test
below picks another lot). Gauge states come from the part's own kinematics (GAUGE.state), so the tips and the
bounding box move with the plungers and any turn."""

import math

import lots
import task
from task import STAND_AXIS, GAUGE, HEAD0, SPAN_A, STAND_HEAD, START_UP, TOP


def qy(a):
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def qx(a):
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def gauge(span, pos=STAND_HEAD, quat=(0.0, 0.0, 0.0, 1.0), vel=None):
    q = (SPAN_A - span) / 2
    return {"gauge": GAUGE.state(joints={"top": q, "bottom": q}, pos=pos, quat_xyzw=quat, lin_vel=vel)}


D = 0.040


def full(g):
    """Every object at its start, then the gauge state `g` (for the direct checks below)."""
    return {o.name: o.state() for o in task.OBJECTS} | g

ON_TABLE = (0.22, 0.12, TOP + 0.006)  # lying on the table on its knob and head, plungers level (rolled 90 deg)

# Fail closed: before the simulator's grip check has run (unknown hands), even a perfect end state earns nothing.
task._grip["clear"] = False
assert task.score(full(gauge(D))) == 0.0 and not task.success(full(gauge(D)))
# Lots: in lot B (44.00 mm bore) the 40.00 mm span earns nothing and 44.00 mm earns full marks; likewise lot C.
task._grip["clear"] = True
for lot, d in (("B", 0.044), ("C", 0.048)):
    lots.use(lot)
    assert task.score(full(gauge(0.040))) == 0.0, lot
    assert task.score(full(gauge(d))) == 1.0 and task.success(full(gauge(d))), lot
    assert task.score(full(gauge(d + 0.00011))) == 0.0, lot
lots.use("A")
# The cases below: hands measured clear of the gauge.

CASES = [
    ("set to 40.00 mm, on the stand", gauge(D), 1.0, True),
    ("on the stand, 0.10 mm over (limit)", gauge(D + 0.00010), 1.0, True),
    ("on the stand, 0.11 mm over", gauge(D + 0.00011), 0.0, False),
    ("on the stand, 0.10 mm under (limit)", gauge(D - 0.00010), 1.0, True),
    ("on the stand, 0.11 mm under (an off-centre sweep's chord)", gauge(D - 0.00011), 0.0, False),
    ("on the stand at the start span (pulled straight out)", gauge(SPAN_A), 0.0, False),
    ("on the stand, 1 mm over (rocked part way)", gauge(D + 0.001), 0.0, False),
    ("on the stand, plungers level (rolled in the slots)", gauge(D, quat=qx(math.pi / 2)), 1.0, True),
    ("on the stand the other way round (head toward the robot)",
     gauge(D, pos=(STAND_HEAD[0] - 0.130, STAND_HEAD[1], STAND_HEAD[2]), quat=(0.0, 0.0, 1.0, 0.0)), 1.0, True),
    ("on the stand, still sliding", gauge(D, vel=(0.05, 0.0, 0.0)), 0.0, False),
    ("set right, lying on the table, tips clear", gauge(D, pos=ON_TABLE, quat=qx(math.pi / 2)), 0.5, False),
    ("set right, lying on the table on a tip", gauge(D, pos=(0.22, 0.12, TOP + 0.0200)), 0.0, False),
    ("set right, 0.2 mm over, on the table", gauge(D + 0.0002, pos=ON_TABLE, quat=qx(math.pi / 2)), 0.0, False),
    ("set right, left level in the bore", gauge(D - 0.00002, pos=HEAD0), 0.0, False),
    ("still in the bore as it started", gauge(SPAN_A, pos=HEAD0, quat=qy(START_UP)), 0.0, False),
    ("set right, handle over only the front post (one post 2 mm low)",
     gauge(D, pos=(STAND_HEAD[0], STAND_HEAD[1], STAND_HEAD[2]), quat=qy(math.radians(-1.5))), 0.5, False),
]
