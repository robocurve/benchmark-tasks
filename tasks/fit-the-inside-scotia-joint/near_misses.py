"""Near successes and unintended strategies, each with the score the task must give.

Each case replays a blade path through the task's own cut rule (offline: the saw's commanded poses, no arm dynamics),
then scores the end state. Run by factory/check.py; every case is scored in lot A.
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as t  # noqa: E402
import lots  # noqa: E402

lots.use("A")
SEAT = np.asarray(lots.current().pos("scotia"), float)  # upright on the block, where it is cut
UP = (0.0, 0.0, 0.0, 1.0)
CUT_POSE = (tuple(map(float, SEAT)), UP)


def frames(path=None, **kw):
    return t.saw_frames(t.cut_waypoints(SEAT, t.cut_path() if path is None else path, **kw))


GOOD = frames()


def pad(d, h=11.0):
    """A fingertip pad, 10 x 6 mm, in the scotia's (u, d, h) mm, 175-185 mm from the 45-degree end, at depth d."""
    u, z = np.meshgrid(np.linspace(175.0, 185.0, 6), np.linspace(h - 3.0, h + 3.0, 4))
    return np.column_stack([u.ravel(), np.full(u.size, float(d)), z.ravel()])


def cove_pad(gap, h=11.0):
    """A fingertip pad in the cove, following its curve `gap` mm off the wood, 175-185 mm from the 45-degree end."""
    u, z = np.meshgrid(np.linspace(175.0, 185.0, 6), np.linspace(h - 3.0, h + 3.0, 4))
    return np.column_stack([u.ravel(), np.asarray(t.front(z.ravel()), float) + gap, z.ravel()])


# The demo's hold: one pad on the back face, the other on the cove face at the same height (0.3 mm off each).
PINCH = [(pad(-0.3), cove_pad(0.3))]
assert t.pinched(PINCH)


def case(cut=GOOD, hands=True, saw=True, held=t.pinched(PINCH), saw_pos=None):
    st = t.SCOTIA.state(pos=tuple(map(float, SEAT)), quat_xyzw=UP)
    out = {"scotia": t.offline(st, cut, CUT_POSE, hands_clear=hands, saw_clear=saw, held=held)}
    if saw_pos is not None:
        out["coping_saw"] = (saw_pos, None, None)
    return out


def off(delta_mm):
    """The demo's path moved delta_mm toward the waste side (negative: into the wood that is kept)."""
    return frames(t.cut_path(off=t.KERF_SIDE + delta_mm))


def partial(n):
    """The first n points of the demo's line, the blade left in its kerf (no exit moves)."""
    return t.saw_frames(t.cut_waypoints(SEAT, t.cut_path()[:n])[:-3])


def straight_cut(u):
    """A plain square cut straight down at u (mm): no cope."""
    return [(u, t.PROFILE + 6.0)] + [(u, h) for h in np.arange(t.PROFILE, -2.01, -t.FEED_PER_HALF)]


def plunge():
    """Air strokes, then the blade dropped straight through the end in one 0.1 s move."""
    x0, z0 = t._world(SEAT, 10.0, t.PROFILE + 6.0)
    yc = SEAT[1] + t.DSIGN * t.PROFILE / 2000 - t.SAW_SIDE * (t.PIN_NEAR + t.PIN_FAR) / 2
    ws = [t._saw_wp(2.0, (x0, yc, z0), "over the line, frame raised")]
    side = 1
    for k in range(8):
        ws.append(t._saw_wp(t.HALF_STROKE_S, (x0, yc + side * t.STROKE, z0), f"stroke in the air {k}"))
        side = -side
    ws.append(t._saw_wp(0.1, (x0, yc + side * t.STROKE, t._world(SEAT, 10.0, -2.0)[1]), "saw plunge"))
    return t.saw_frames(ws)


def then_blade_at(p):
    """The demo cut, then the blade's middle taken to world point p (same saw orientation)."""
    sp, sq = GOOD[-1]
    r = t._rot(sq)
    mid = r @ np.array([0.0, (t.PIN_NEAR + t.PIN_FAR) / 2, 0.0])  # handle middle -> blade middle
    return GOOD + ((tuple(map(float, np.asarray(p) - mid)), sq),)


CASES = [
    # the full task and the end-state rules around it
    ("coped on the line, saw put down on the floor, scotia let go, hands 3 cm clear", case(), 1.0, True),
    ("coped on the line but the saw still in a hand", case(saw=False), 0.75, False),
    ("coped on the line but a gripper within 3 cm of the scotia", case(hands=False), 0.75, False),
    ("coped on the line, the saw put down on the block instead of the floor",
     case(saw_pos=(t.BLOCK_A[0], t.BLOCK_A[1], t.BLOCK_TOP + 0.016)), 0.75, False),
    # how closely the cut follows the line
    ("cut 0.5 mm into the wood that is kept", case(cut=off(-0.55)), 1.0, True),
    ("0.5 mm of wood left proud of the line (crushes when pushed home)", case(cut=off(0.5)), 1.0, True),
    ("cut 2 mm into the kept wood all along", case(cut=off(-2.0)), 0.75, False),
    ("1 mm of wood left proud of the line", case(cut=off(1.0)), 0.75, False),
    ("cut 3.5 mm into the kept wood", case(cut=off(-3.5)), 0.5, False),
    ("1.5 mm of wood left proud of the line", case(cut=off(1.5)), 0.5, False),
    ("cut 5 mm into the kept wood", case(cut=off(-5.0)), 0.25, False),
    ("cut 6.5 mm into the kept wood", case(cut=off(-6.5)), 0.0, False),
    ("cut 2 mm wide of the line on the waste side (2 mm of wood left proud)", case(cut=off(2.0)), 0.5, False),
    ("cut 3 mm wide of the line on the waste side: most of the offcut stays on", case(cut=off(3.0)), 0.0, False),
    # unintended strategies and faults
    ("not cut at all", case(cut=()), 0.0, False),
    ("a plain square cut 18.5 mm from the long point, no cope", case(cut=frames(straight_cut(18.45))), 0.0, False),
    ("a third of the line sawn, offcut still on", case(cut=partial(20)), 0.0, False),
    ("over half the line sawn within 1 mm of it, offcut still on", case(cut=partial(30)), 0.25, False),
    ("the line sawn without stroking the blade", case(cut=frames(stroke=0.0)), 0.0, False),
    ("the blade dropped through the end in one move", case(cut=plunge()), 0.0, False),
    ("sawn with the teeth facing up (frame lowered)", case(cut=frames(lean=-40.0)), 0.0, False),
    ("sawn while nothing holds the scotia", case(held=False), 0.0, False),
    ("sawn with the other gripper pressed on the scotia's back, both fingertips on that face (no pinch)",
     case(held=t.pinched([(pad(-0.3, 8.0), pad(-0.3, 15.0))])), 0.0, False),
    ("sawn with the other gripper's fingertips on the back and in the cove, 3 mm short of its face (no pinch)",
     case(held=t.pinched([(pad(-0.3), cove_pad(3.0))])), 0.0, False),
    ("coped, then the blade run into the installed scotia",
     case(cut=then_blade_at((t.X_SA - 0.009, 0.2, t.TOP + 0.008))), 0.0, False),
    ("coped, then the blade run into the side skirting",
     case(cut=then_blade_at((0.30, t.Y_SB + 0.009, t.TOP + 0.03))), 0.0, False),
    ("coped, then the blade pushed into the floor", case(cut=then_blade_at((0.30, 0.0, t.TOP - 0.002))), 0.0, False),
    ("coped, then the blade run into the scotia 30 mm back from its end",
     case(cut=then_blade_at((t._world(SEAT, 30.0, 9.0)[0], SEAT[1], t._world(SEAT, 30.0, 9.0)[1]))), 0.0, False),
]
