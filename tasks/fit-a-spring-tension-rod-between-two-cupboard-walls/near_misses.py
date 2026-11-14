"""Near-successes and unintended strategies, scored offline by factory/check.py. Rod states come from the part's own
kinematics (ROD.state), so the bounding box and the caps move with the slide and any tilt. Offline there is no
robot, so the release check passes; _run() below also checks the release rule with a gripper placed on the rod."""

import math
import os

import task
from task import BAND_Z0, BAND_Z1, CAP, FREE, GAP, ROD, TOP, WALL_CX, WALL_H, WALL_X0, WALL_X1

os.environ[task.OFFLINE_FLAG] = "1"  # the task credits these offline cases as let go

FIT_Q = FREE - GAP  # 20 mm squeezed when spanning the gap


def qx(deg):
    """Rolled about x: the +y end (tube cap) goes up for positive angles."""
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def _face_span(a):
    """Cap face to cap face (m) along a rod turned `a` rad (tilted or skewed) whose cap corners just touch the walls."""
    return (GAP - 2 * (CAP[0] / 2) * math.sin(a)) / math.cos(a)


def _turned(deg, z, x, quat, along, vel=None):
    """A fitted rod turned `deg` so that its cap corners touch both walls. `along(a)` gives the axis direction."""
    a = math.radians(deg)
    q = FREE - _face_span(a) if deg else FREE - GAP
    d = along(a)
    o = [-(q / 2) * c for c in d]  # the axis midpoint sits at (x, 0, z); the body origin is q/2 back along the axis
    return {"rod": ROD.state(joints={"slide": q}, pos=(x + o[0], o[1], TOP + z + o[2]),
                             quat_xyzw=quat(deg) if deg else None, lin_vel=vel)}


def fitted(z, x=WALL_CX, tilt=0.0, vel=None):
    """Spanning the gap, axis midpoint at height z, the +y end raised by `tilt` degrees."""
    return _turned(tilt, z, x, qx, lambda a: (0.0, math.cos(a), math.sin(a)), vel)


def skewed(deg, z=None):
    """Fitted level but turned `deg` about the vertical (the +y cap further back)."""
    return _turned(deg, MID if z is None else z, WALL_CX, qz, lambda a: (-math.sin(a), math.cos(a), 0.0))


def _solve(gap_between_centres):
    """Angle (deg) at which the two cap centres are `gap_between_centres` apart across the axis' turn."""
    lo, hi = 0.0, 0.5
    for _ in range(80):
        a = (lo + hi) / 2
        off = (_face_span(a) - CAP[1]) * math.sin(a)
        lo, hi = (a, hi) if off < gap_between_centres else (lo, a)
    return math.degrees((lo + hi) / 2)


MID = (BAND_Z0 + BAND_Z1) / 2


def short(gap):
    """Level in the band, tube cap on its wall, the bar's cap `gap` m short of the other wall."""
    return {"rod": ROD.state(joints={"slide": FIT_Q + gap}, pos=(WALL_CX, -FIT_Q / 2, TOP + MID))}


def tilt_for(dz):
    return _solve(dz)


def skew_for(dx):
    return _solve(dx)


CASES = [
    ("fitted level in the middle of the band", fitted(MID), 1.0, True),
    ("fitted at the bottom of the band", fitted(BAND_Z0), 1.0, True),
    ("fitted at the top of the band", fitted(BAND_Z1), 1.0, True),
    ("self-supporting, 1 mm below the band: partial", fitted(BAND_Z0 - 0.001), 0.5, False),
    ("self-supporting, 1 mm above the band: partial", fitted(BAND_Z1 + 0.001), 0.5, False),
    ("self-supporting but off-height, 40 mm up: partial", fitted(0.040), 0.5, False),
    ("self-supporting, top of the caps level with the wall tops (limit)", fitted(WALL_H - CAP[2] / 2), 0.5, False),
    ("cap tops 1 mm above the wall tops, centres on the walls: partial", fitted(WALL_H - CAP[2] / 2 + 0.001), 0.5,
     False),
    ("cap centres 1 mm above the wall tops: not bearing", fitted(WALL_H + 0.001), 0.0, False),
    ("fitted, ends 10 mm apart in height (limit)", fitted(MID, tilt=tilt_for(0.010)), 1.0, True),
    ("self-supporting but off-level, ends 11 mm apart: partial", fitted(MID, tilt=tilt_for(0.011)), 0.5, False),
    ("self-supporting but off-level, ends 31 mm apart: partial", fitted(MID, tilt=tilt_for(0.031)), 0.5, False),
    ("self-supporting low, rod 3 mm clear of the table (limit): partial",
     fitted(CAP[2] / 2 + 0.003), 0.5, False),
    ("squeezed between the walls but 2 mm off the table: not credited", fitted(CAP[2] / 2 + 0.002), 0.0, False),
    ("squeezed between the walls, resting on the table", fitted(CAP[2] / 2), 0.0, False),
    ("self-supporting, skewed 10 deg about the vertical: partial", skewed(10), 0.5, False),
    ("fitted, caps 9 mm apart front to back", skewed(skew_for(0.009)), 1.0, True),
    ("self-supporting, caps 11 mm apart front to back: partial", skewed(skew_for(0.011)), 0.5, False),
    ("fitted but still sliding down", fitted(MID, vel=(0.0, 0.0, -0.05)), 0.0, False),
    ("off-height and sliding down", fitted(0.040, vel=(0.0, 0.0, -0.05)), 0.0, False),
    ("caps centred on the walls' front edge, half on: partial", fitted(MID, x=WALL_X0), 0.5, False),
    ("cap centres 1 mm in front of the walls' front edge: not bearing", fitted(MID, x=WALL_X0 - 0.001), 0.0, False),
    ("cap centres 1 mm behind the walls' back edge: not bearing", fitted(MID, x=WALL_X1 + 0.001), 0.0, False),
    ("spanning the gap, caps flush with the front edge", fitted(MID, x=WALL_X0 + 0.015), 1.0, True),
    ("caps 1 mm over the front edge: partial", fitted(MID, x=WALL_X0 + 0.014), 0.5, False),
    ("spanning the gap at the back edge, caps flush", fitted(MID, x=WALL_X1 - 0.015), 1.0, True),
    ("one cap 0.3 mm short of its wall (contact limit)", short(0.0003), 1.0, True),
    ("one cap 0.5 mm short of its wall (not pressing)", short(0.0005), 0.0, False),
    ("one cap 1.5 mm short (a gap only a hand or the other cap's friction could hold)", short(0.0015), 0.0, False),
    ("one cap 3 mm short of its wall (not touching)", short(0.003), 0.0, False),
    ("lying on the table between the walls, unsqueezed",
     {"rod": ROD.state(pos=(WALL_CX, 0.0, TOP + 0.0125), quat_xyzw=qz(25))}, 0.0, False),
    ("lying across the wall tops",
     {"rod": ROD.state(pos=(WALL_CX, 0.0, TOP + WALL_H + 0.015))}, 0.0, False),
    ("still on its risers", {}, 0.0, False),
    ("squeezed on the risers", {"rod": ROD.state(joints={"slide": 0.03})}, 0.0, False),
]


def _release_cases(start) -> int:
    """The release rule, which check.py cannot reach offline: a gripper within 5 mm of the rod, or no measurement."""
    bad = 0
    checks = [
        ("held by a gripper (0 mm), fitted in the band", fitted(MID), 0.0, {"rod": 0.0}),
        ("held by a gripper (0 mm), off-height", fitted(0.040), 0.0, {"rod": 0.0}),
        ("gripper 4.9 mm from the rod, fitted in the band", fitted(MID), 0.0, {"rod": 0.0049}),
        ("gripper 5.0 mm from the rod, fitted in the band (limit)", fitted(MID), 1.0, {"rod": 0.005}),
        ("gripper 5.0 mm from the rod, off-level (limit): partial", fitted(MID, tilt=tilt_for(0.011)), 0.5,
         {"rod": 0.005}),
    ]
    for label, changes, want, gaps in checks:
        task._GAP.clear()
        task._GAP.update(gaps)
        st = {**start, **changes}
        got = 1.0 if task.success(st) else float(task.score(st))
        flag = "ok  " if got == want else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    task._GAP.clear()
    os.environ[task.OFFLINE_FLAG] = "0"  # in the simulator, before the first step has measured the grippers
    got = float(task.score({**start, **fitted(MID)}))
    os.environ[task.OFFLINE_FLAG] = "1"
    flag = "ok  " if got == 0.0 else "FAIL"
    bad += flag == "FAIL"
    print(f"{flag} {got:4.2f}  no gripper measurement yet: fails closed")
    return bad


def _run() -> int:
    start = {"rod": ROD.state()}  # score() reads only the rod
    bad = 0
    for label, changes, want, want_success in CASES:
        st = {**start, **changes}
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad + _release_cases(start)


if __name__ == "__main__":
    raise SystemExit(1 if _run() else 0)
