"""Strip the black conductor coming out of a wall box to a receptacle's strip gauge and push it into a push-in hole on the
receptacle's brass (hot) side, holding the receptacle beside the box as an electrician does when wiring a new receptacle.

On the station table, a short section of wall faces the robot: 1/2 in drywall on a 2x4 stud and bottom plate, with a blue
nail-on single-gang box (ABB/Carlon B118A pattern) nailed to the stud's side, flush with the drywall. A 14/2 NM-B cable
runs down the stud into the box's top; inside the box its sheath is stripped back, the white and bare conductors are
folded back into the box, and the black conductor (14 AWG solid copper, 2.69 mm over its jacket) comes out of the box's
front, bends down in a loop to the table and stands up with its last 7 cm vertical, about 12 cm in front of the wall.
In front of the right arm: a wire stripper (Klein 11055 pattern: 181 mm, blue and yellow handles, stripping holes for
10, 12, 14, 16 and 18 AWG solid, the larger holes nearer the pivot) lies flat on a grey stand 20 cm high, its jaws open
and overhanging the stand's front edge, pointing towards the wall. Its return spring holds the jaws open; squeezing the
handles shuts them. To the robot's right, a white 15 A duplex receptacle (Leviton T5320 pattern) lies face up on a
plywood tray: four push-in holes on its back, two on its brass-screw (hot) side and two on its silver-screw (neutral)
side. The receptacle's strip gauge is 5/8 in (16 mm).

To strip the wire, close the stripper's matching hole on it at the strip length from its end and pull the wire out of the
shut jaws: the cut jacket stays in the jaws. Hold the receptacle back up, push the stripped end straight into a push-in
hole until the jacket meets the receptacle's back; the hole's spring clamp then holds it.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402
import stripper as S  # noqa: E402
import wirekit as wk  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Strip the black wire coming from the wall box to the receptacle's 5/8 inch (16 mm) strip gauge with the wire stripper, "
               "then pick up the receptacle, push the wire fully into a push-in hole on the brass (hot) side of its back, and let go of the wire.")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- layout (world frame: x forward from the arms, y left, z up; m) ------------------------------------------------------
H = TOP + 0.200  # the stripper's plane: high enough that the left hand passes under it
PIVOT = np.array([0.30, -0.10, H])
CORE_L, WIRE_R = 0.070, 0.001346  # the conductor's free end, simulated as one straight piece (the strippable 50 mm and 20 mm below it), and its radius
WIRE_L = CORE_L
GAUGE = 0.016  # the receptacle's strip gauge: 5/8 in
BARE_FULL, BARE_PART = (0.014, 0.018), (0.012, 0.020)  # bare length for full and partial strip credit
EXPOSED_MAX = 0.0015  # copper showing between the jacket and the receptacle's back, at most
JACKET_IN_MAX = 0.0015  # the jacket this far inside a hole means the clamp is on insulation, not copper
SIDE = {"black": "brass"}
CLEAR = 0.008  # a fingertip within this of a wire's axis (outside the receptacle) counts as touching it

# the wall: 1/2 in drywall on a 2x4 stud, a nail-on single-gang box (ABB/Carlon B118A pattern: 3-3/4 x 2-1/4 x 2-7/8 in)
# flush with the drywall, its open front facing the robot
WALL_X = 0.520  # the drywall's front face
DRYWALL_T = 0.0127
BOX_Y, BOX_ZC = -0.100, TOP + 0.120  # the box's centre on the wall
BOX_W, BOX_HT, BOX_D, BOX_WALL = 0.057, 0.095, 0.073, 0.003
STUD_Y0 = BOX_Y + BOX_W / 2  # the box is nailed to the stud's side: the stud runs from here in +y
# the black conductor: anchored in the box (where the cable clamp holds the NM cable), folded out through the box's front
# in a loop and standing up at its end; its last CORE_L is the strippable end, vertical
CORE_C = np.array([0.400, BOX_Y, TOP + 0.100 + CORE_L / 2])  # the end's centre at the start
LEAD = (
    (WALL_X + 0.040, BOX_Y, BOX_ZC - 0.010),  # in the box, at the cable clamp
    (WALL_X - 0.003, BOX_Y, BOX_ZC - 0.012),  # out through the box's front
    (0.505, BOX_Y, TOP + 0.075),
    (0.492, BOX_Y, TOP + 0.045),
    (0.475, BOX_Y, TOP + 0.022),
    (0.452, BOX_Y, TOP + 0.012),
    (0.428, BOX_Y, TOP + 0.014),
    (0.408, BOX_Y, TOP + 0.030),
    (0.396, BOX_Y, TOP + 0.055),
    (0.398, BOX_Y, TOP + 0.080),
    (CORE_C[0], CORE_C[1], CORE_C[2] - CORE_L / 2),  # the end's lower end
)
# the receptacle: a loose part, lying face up (back down) on a tray until the right hand takes it; its frame's origin is the
# middle of its back face, its length along x, its width along y; push-in holes (dx along, dy across): brass (hot) on -y
REC_BODY = (0.072, 0.033, 0.022)  # length (x), width (y), depth (z) below the strap
REC_MASS = 0.045
CHANNEL, CH_DEPTH = 0.0026, 0.0175  # push-in channels: square (the 1.63 mm copper passes, the 2.69 mm jacket does not), and depth to the bottom
FUNNEL, FUNNEL_DEPTH = 0.005, 0.001  # each push-in hole's lead-in chamfer: square at the back face, and its depth
HOLES = {"brass_a": (0.010, -0.011), "brass_b": (-0.010, -0.011), "silver_a": (0.010, 0.011), "silver_b": (-0.010, 0.011)}
TRAY = np.array([0.250, -0.240])  # a plywood tray on a block, under the receptacle
TRAY_TOP = TOP + 0.060
REC_START = np.array([TRAY[0], TRAY[1], TRAY_TOP + 0.0005])
REC_HOLD = np.array([0.375, -0.130, TOP + 0.145])  # where the demo holds the receptacle (its back's middle) to connect it

LOOKS = {
    "rest": m.Look(color=m.rgb(0.35, 0.35, 0.38), roughness=0.7),
    "wood": m.Look(color=m.rgb(0.62, 0.46, 0.28), roughness=0.8),
    "spf": m.Look(color=m.rgb(0.88, 0.78, 0.58), roughness=0.8),
    "drywall": m.Look(color=m.rgb(0.92, 0.91, 0.88), roughness=0.9),
    "box": m.Look(color=m.rgb(0.16, 0.36, 0.78), roughness=0.5),
    "nm": m.Look(color=m.rgb(0.95, 0.95, 0.93), roughness=0.6),
    "white": m.Look(color=m.rgb(0.92, 0.92, 0.90), roughness=0.6),
    "copper": m.Look(color=m.rgb(0.80, 0.50, 0.32), roughness=0.35, metallic=0.9),
    "nylon": m.Look(color=m.rgb(0.93, 0.93, 0.90), roughness=0.5),
    "strap": m.Look(color=m.rgb(0.70, 0.71, 0.73), roughness=0.35, metallic=0.9),
    "brass": m.Look(color=m.rgb(0.80, 0.62, 0.25), roughness=0.3, metallic=0.9),
    "silver": m.Look(color=m.rgb(0.80, 0.81, 0.83), roughness=0.3, metallic=0.9),
    "dark": m.Look(color=m.rgb(0.08, 0.08, 0.08), roughness=0.6),
}


def hole_local(name: str) -> np.ndarray:
    dx, dy = HOLES[name]
    return np.array([dx, dy, 0.0])


def _box(parts, cols, name, size, c, look, collide=True):
    parts.append((name, m.box(size, c), look))
    if collide:
        cols.append(m.col_box(name + "_c", size, c))


def _between(parts, cols, name, x0, x1, y0, y1, z0, z1, look, collide=True):
    _box(parts, cols, name, (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), look, collide)


def _fixtures():
    parts, cols = [], []
    # the stripper's stand: under its neck (jaws overhang the front edge) and under its handles
    neck_top = H - S.JAW_T / 2 - 0.0005
    _box(parts, cols, "stand_neck", (0.024, 0.030, neck_top - TOP), (PIVOT[0] - 0.006, PIVOT[1], (neck_top + TOP) / 2), "rest")
    handle_top = H - S.HANDLE_T / 2 - 0.0005
    _box(parts, cols, "stand_handle", (0.012, 0.070, handle_top - TOP), (PIVOT[0] - 0.070, PIVOT[1] - 0.012, (handle_top + TOP) / 2), "rest")
    # the receptacle's tray: 1/2 in plywood on a block
    _between(parts, cols, "tray_block", TRAY[0] - 0.03, TRAY[0] + 0.03, TRAY[1] - 0.02, TRAY[1] + 0.02, TOP, TRAY_TOP - 0.0127, "rest")
    _between(parts, cols, "tray", TRAY[0] - 0.06, TRAY[0] + 0.06, TRAY[1] - 0.04, TRAY[1] + 0.04, TRAY_TOP - 0.0127, TRAY_TOP, "wood")
    # the wall: drywall with the box's opening, the stud beside the box, the box (open front), the cable into its top
    x0, x1 = WALL_X, WALL_X + DRYWALL_T
    by0, by1, bz0, bz1 = BOX_Y - BOX_W / 2, BOX_Y + BOX_W / 2, BOX_ZC - BOX_HT / 2, BOX_ZC + BOX_HT / 2
    wy0, wy1, wz1 = BOX_Y - 0.200, BOX_Y + 0.200, TOP + 0.340
    _between(parts, cols, "drywall_l", x0, x1, wy0, by0, TOP, wz1, "drywall")
    _between(parts, cols, "drywall_r", x0, x1, by1, wy1, TOP, wz1, "drywall")
    _between(parts, cols, "drywall_lo", x0, x1, by0, by1, TOP, bz0, "drywall")
    _between(parts, cols, "drywall_hi", x0, x1, by0, by1, bz1, wz1, "drywall")
    _between(parts, cols, "stud", x1, x1 + 0.089, STUD_Y0, STUD_Y0 + 0.038, TOP, wz1, "spf")
    _between(parts, cols, "plate", x1, x1 + 0.089, wy0, wy1, TOP, TOP + 0.038, "spf")
    bx1 = WALL_X + BOX_D
    _between(parts, cols, "box_back", bx1 - BOX_WALL, bx1, by0, by1, bz0, bz1, "box")
    _between(parts, cols, "box_top", WALL_X, bx1, by0, by1, bz1 - BOX_WALL, bz1, "box")
    _between(parts, cols, "box_bottom", WALL_X, bx1, by0, by1, bz0, bz0 + BOX_WALL, "box")
    _between(parts, cols, "box_side_l", WALL_X, bx1, by0, by0 + BOX_WALL, bz0, bz1, "box")
    _between(parts, cols, "box_side_r", WALL_X, bx1, by1 - BOX_WALL, by1, bz0, bz1, "box")
    _between(parts, cols, "box_flange", WALL_X - 0.0015, WALL_X, by0 - 0.002, by1 + 0.002, bz0 - 0.003, bz1 + 0.003, "box", collide=False)
    # the 14/2 NM-B cable: down the stud's face to the box's top knockout, its sheath stripped back inside the box; the white
    # and bare conductors folded back into the box (they are not part of this task)
    _between(parts, cols, "nm_cable", x1 + 0.002, x1 + 0.0065, BOX_Y - 0.0047, BOX_Y + 0.0047, bz1, wz1, "nm", collide=False)
    _between(parts, cols, "nm_in_box", WALL_X + 0.030, WALL_X + 0.0345, BOX_Y - 0.0047, BOX_Y + 0.0047, BOX_ZC, bz1, "nm", collide=False)
    parts.append(("white_cond", m.tube([(WALL_X + 0.032, BOX_Y + 0.004, BOX_ZC), (WALL_X + 0.012, BOX_Y + 0.012, BOX_ZC - 0.010),
                                         (WALL_X + 0.010, BOX_Y + 0.016, BOX_ZC - 0.035)], WIRE_R), "white"))
    parts.append(("bare_cond", m.tube([(WALL_X + 0.032, BOX_Y - 0.002, BOX_ZC), (WALL_X + 0.014, BOX_Y - 0.014, BOX_ZC + 0.010),
                                        (WALL_X + 0.012, BOX_Y - 0.018, BOX_ZC + 0.035)], 0.0008), "copper"))
    return parts, cols


def _receptacle():
    """The receptacle in its own frame: origin at the middle of its back, back facing -z, length along x, width along y."""
    parts, cols = [], []
    L, W, D = REC_BODY
    c = CHANNEL / 2
    fh, fd = FUNNEL / 2, FUNNEL_DEPTH
    hx = sorted({dx for dx, _ in HOLES.values()})
    hy = sorted({dy for _, dy in HOLES.values()})
    for z0, z1, half, tag in ((fd, CH_DEPTH, c, "ch"), (0.0, fd, fh, "fn")):  # above the chamfers: channels; at the mouth: chamfer squares
        xs = [-L / 2] + [v for x0 in hx for v in (x0 - half, x0 + half)] + [L / 2]
        ys = [-W / 2] + [v for y0 in hy for v in (y0 - half, y0 + half)] + [W / 2]
        for i in range(len(xs) - 1):
            for j in range(len(ys) - 1):
                if i % 2 == 1 and j % 2 == 1:
                    continue
                _between(parts, cols, f"rec_{tag}_{i}_{j}", xs[i], xs[i + 1], ys[j], ys[j + 1], z0, z1, "nylon")
    for name, (dx, dy) in HOLES.items():  # four sloped walls per chamfer, from FUNNEL square at the back to the channel
        for ax in (0, 1):
            for sgn in (-1.0, 1.0):
                pts = []
                for t in (-fh, fh):
                    for (r, z) in ((fh, 0.0), (c, fd), (fh, fd)):
                        p_ = [dx, dy, z]
                        p_[ax] += sgn * r
                        p_[1 - ax] += t
                        pts.append(p_)
                cols.append(m.col_hull(f"chamfer_{name}_{ax}_{int(sgn > 0)}", m.Mesh(np.array(pts), np.array([[0, 1, 2], [3, 4, 5]]))))
    _between(parts, cols, "rec_top", -L / 2, L / 2, -W / 2, W / 2, CH_DEPTH, D, "nylon")
    _between(parts, cols, "rec_strap", -0.053, 0.053, -0.011, 0.011, D, D + 0.0015, "strap")
    _between(parts, cols, "rec_face", -0.033, 0.033, -0.015, 0.015, D + 0.0015, D + 0.0055, "nylon")
    for k, dx in enumerate((-0.018, 0.018)):  # terminal screws: brass on -y (hot), silver on +y (neutral)
        parts.append((f"screw_b{k}", m.cylinder(0.004, 0.003, (dx, -W / 2 - 0.0015, 0.012), axis="y"), "brass"))
        parts.append((f"screw_s{k}", m.cylinder(0.004, 0.003, (dx, W / 2 + 0.0015, 0.012), axis="y"), "silver"))
    for name, (dx, dy) in HOLES.items():  # dark rims round the holes, and the release slots beside them
        parts.append((f"rim_{name}", m.box((CHANNEL + 0.001, CHANNEL + 0.001, 0.0004), (dx, dy, -0.0002)), "dark"))
        parts.append((f"slot_{name}", m.box((0.004, 0.0012, 0.0004), (dx, dy + (0.004 if dy > 0 else -0.004), -0.0002)), "dark"))
    return parts, cols


_fp, _fc = _fixtures()
_rp, _rc = _receptacle()
_HOLE_SPECS = tuple((name, tuple(hole_local(name)), (0.0, 0.0, 1.0)) for name in HOLES)
OBJECTS = [
    S.make("stripper", tuple(PIVOT)),
    m.fixed("fixtures", (0.0, 0.0, 0.0), _fp, _fc, LOOKS),
    m.free("receptacle", tuple(REC_START), _rp, _rc, LOOKS, mass=REC_MASS, friction=(0.8, 0.7), torsional_patch=0.01),
    wk.Pigtail("black", pos=tuple(CORE_C), size=(0.003, 0.003, CORE_L), length=CORE_L, jacket=(0.03, 0.03, 0.03), holes=_HOLE_SPECS,
               hole_body="receptacle", lead=LEAD, bend_k=float(__import__("os").environ.get("BEND_K", "1.0"))),
]
WIRES = ("black",)


# -- scoring ---------------------------------------------------------------------------------------------------------------


class Touch:
    """Whether any fingertip is on each wire now, from mechkit's fingertip points (updated every control step)."""

    def __init__(self):
        self.now = {w: False for w in WIRES}


TOUCH = Touch()


@m.on_step
def _touch(env):
    from yam_arena.scene import object_states

    tips = m.fingertips(env)
    states = object_states(env, WIRES)
    for w in WIRES:
        st = states[w]
        a, b = st.end - st.axis * WIRE_L, st.end - st.axis * max(0.0, st.depth)  # not the part inside the receptacle
        ab = b - a
        t = np.clip((tips - a) @ ab / (ab @ ab), 0.0, 1.0)
        d = np.linalg.norm(tips - (a + np.outer(t, ab)), axis=1)
        TOUCH.now[w] = bool(d.min() < CLEAR)


@m.on_reset
def _reset(env):
    TOUCH.now = {w: False for w in WIRES}


def wire_report(name: str, st) -> dict:
    """The wire's credit (out of 1) and why: 0.3 for the strip, 0.7 for the termination."""
    damaged = bool(st.nicked or st.crushed)
    bare = round(float(st.bare), 4)
    strip = 0.0
    if bare > 0 and st.slug_clear and not damaged:
        if BARE_FULL[0] <= bare <= BARE_FULL[1]:
            strip = 0.30
        elif BARE_PART[0] <= bare <= BARE_PART[1]:
            strip = 0.15
    side = st.latched.split("_")[0] if st.latched else None
    exposed = bare - st.depth if st.latched else None
    term = 0.0
    if (strip > 0 and side == SIDE[name] and st.depth >= 0.012 and -JACKET_IN_MAX <= exposed <= EXPOSED_MAX
            and not TOUCH.now[name]):
        term = 0.70
    return {"strip": strip, "terminated": term, "bare_mm": round(bare * 1000, 1), "damaged": damaged, "slug_clear": bool(st.slug_clear),
            "hole": st.latched, "side_ok": side == SIDE[name] if side else None,
            "exposed_mm": None if exposed is None else round(exposed * 1000, 1), "touched": TOUCH.now[name]}


def reversed_polarity(objects) -> bool:
    """A wire clamped on the other colour's side: the receptacle is wired with its polarity reversed."""
    return any(objects[w].latched and objects[w].latched.split("_")[0] != SIDE[w] for w in WIRES)


def score(objects) -> float:
    if reversed_polarity(objects):
        return 0.0
    return float(sum(r["strip"] + r["terminated"] for r in (wire_report(w, objects[w]) for w in WIRES)))


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


# -- the scripted demonstration (demokit) -----------------------------------------------------------------------------

DEBUG = bool(__import__("os").environ.get("DEMO_DEBUG"))
GRASP_X = -0.115  # tool x where the right hand pinches the handles
AIM_SHORT = 0.0  # where the jaws close, short of the strip length: the jacket is cut at the nearest 2 mm ring boundary
GRAB_FROM_END = 0.050  # the left hand holds the conductor this far below its end: close below the cut, so the jaws cannot tip it
PICK_OPEN = -0.012  # finger joint: 24 mm between the pads when coming to a wire
HOLE_FOR = {"black": "brass_a"}
REC_PINCH = np.array([0.010, 0.0, 0.016])  # the right hand's TCP on the receptacle (its frame): across its width in line with the brass_a
                                           # hole (so the push does not turn it in the grip), its fingertips 3 mm above the back


def demo_script(ctx):
    """Right hand: pinch the stripper's handles from above and squeeze or release them on cue; then let go of the stripper,
    take the receptacle from its tray by its sides and hold it back down beside the stripper. Left hand: take the wire's
    end from above, slide it into the open jaws at the 14 AWG hole with 16 mm above the jaws, wait for the jaws to shut,
    pull it down out of them, straighten it, lower it below the held receptacle, bring it under the brass hole, line it up
    with the hole and push it up until the jacket meets the back, let go. Positions are read from the scene as it is, so
    the moves follow where things are."""
    from demokit import down, level
    from yam_arena import ik

    l, r = ctx.left, ctx.right
    lo, up = S.handle_point(GRASP_X, "lower", S.OPEN_J), S.handle_point(GRASP_X, "upper", S.OPEN_J)
    half_up = S.HANDLE_W / 2 / math.cos(math.radians(S.OPEN_DEG))
    y_hi, y_lo = lo[1] + S.HANDLE_W / 2, up[1] - half_up
    span = y_hi - y_lo
    handles = PIVOT + np.array([GRASP_X, (y_hi + y_lo) / 2, 0.0])

    def wire(name):
        return ctx.state(name)

    def say(label, name):
        if not DEBUG:
            return
        w = wire(name)
        print(f"[demo] {label}: {name} end {np.round(w.end, 4)} tilt {math.degrees(math.acos(np.clip(w.axis[2], -1, 1))):.1f} | left TCP {np.round(l.tcp(), 4)} "
              f"grip {ctx.observed('left')[6]:.3f} | stripper {ctx.deg('stripper', 'upper'):.1f} at {np.round(ctx.pos('stripper'), 4)} | bare {w.bare * 1000:.1f} latched {w.latched} "
              f"depth {w.depth * 1000:.1f} | rec {np.round(ctx.pos('receptacle'), 4)}", flush=True)

    def at_jaws(name):  # the point on the wire where the jaws should close: the strip length from its end, aimed a little short
        w = wire(name)
        return w.end - w.axis * (strip_len - AIM_SHORT)

    def tip(name):
        return wire(name).end

    def settle_onto(goal, read, rounds=8, tol=0.0004):
        """Move by the measured error, let the arm settle, measure again: a continuous servo here only chases the arm's lag."""
        for _ in range(rounds):
            err = np.asarray(goal() if callable(goal) else goal) - read()
            if DEBUG:
                print(f"[demo]    settle: err {np.round(err * 1000, 2)} mm, TCP {np.round(l.tcp(), 4)}, cmd {np.round(l.at(), 4)}", flush=True)
            if np.linalg.norm(err) < tol:
                return
            yield from l.move(l.at() + 0.5 * err, t=0.5, label="l settle onto")
            yield from l.wait(0.5)

    def hole14():  # the chosen stripping hole's centre (14 AWG unless a cheat mode says otherwise), from where the stripper is now
        st = ctx.state("stripper")
        return S.hole_world(awg, st.pos, st.rotation())

    mode = ctx.mode  # cheat modes, to check that wrong ways score low: nick, long, short, silver, nostrip, none
    if mode in ("none", "still"):
        for _ in range(4):
            yield from ctx.until(lambda: False, timeout=0.5, label="do nothing")
            say("standing", "black")
        if DEBUG:
            w = wire("black")
            auth = [(np.asarray(a) + np.asarray(b)) / 2 for a, b in zip(LEAD, LEAD[1:])]
            for i, (pa, pb) in enumerate(zip(auth, w.lead_pos)):
                print(f"[demo]    lead_{i:02d}: authored {np.round(pa, 4)} now {np.round(pb, 4)}  off {np.linalg.norm(pb - pa) * 1000:.1f} mm", flush=True)
            print(f"[demo]    core: authored {np.round(CORE_C, 4)} now {np.round(w.pos, 4)}", flush=True)
        return
    def rec_hole(hname):
        """A push-in hole's mouth and its axis (into the receptacle) in the world, from where the receptacle is now."""
        st = ctx.state("receptacle")
        R = st.rotation()
        return st.pos + R @ hole_local(hname), R[:, 2]

    w0 = wire("black")
    grab0 = w0.end - w0.axis * GRAB_FROM_END
    yield from ctx.together(r.move(handles + [0, 0, 0.02], grip=down(0), t=2.5, label="r over handles"),
                            l.move(grab0 + [0, 0, 0.11], grip=level(-90), finger=PICK_OPEN, t=2.5, label="l over the conductor"))
    yield from r.move(handles + [0, 0, 0.004], t=1.2, label="r to handles")
    yield from r.close(span, force=6.0, t=0.5, label="r hold handles")
    awg = 16 if mode == "nick" else 14
    strip_len = {"long": 0.022, "short": 0.010}.get(mode, GAUGE)
    for name in WIRES:
        w0 = wire(name)
        grab = w0.end - w0.axis * GRAB_FROM_END
        yield from l.move(grab + [0, 0, 0.11], grip=level(-90), finger=PICK_OPEN, t=1.0, label=f"l over {name}")
        yield from l.move(grab, t=1.5, label=f"l down to {name}")
        yield from l.wait(0.5)
        say("at wire", name)
        yield from l.close(2 * WIRE_R, force=40.0, t=0.5, label=f"l grip {name}")
        yield from l.wait(0.3)
        if mode != "nostrip":
            # into the open jaws from beyond their tips, 1.6 mm off the line, the 16 mm mark at the jaws' plane
            h14 = hole14()
            target = np.array([h14[0] - 0.0005, h14[1] + 0.0016, h14[2]])  # half a millimetre towards the pivot: the jaws squeeze towards their tips
            grip_z = H + strip_len - AIM_SHORT - GRAB_FROM_END
            yield from l.move(np.array([PIVOT[0] + S.JAW_TIP + 0.025, target[1], grip_z]), t=1.5, label="l to jaw tips")
            yield from l.wait(0.3)
            say("at jaw tips", name)
            yield from l.move(np.array([target[0], target[1], grip_z]), t=1.5, label="l into jaws")
            yield from l.servo(target, read=lambda n=name: at_jaws(n), tol=0.0005, timeout=3.0, gain=0.12, label="l wire over the 14 AWG hole")
            yield from settle_onto(target, lambda n=name: at_jaws(n))
            seat = np.array([h14[0] - 0.0003, h14[1], h14[2]])  # onto the jaws' line, into the lower jaw's notch, which holds it while the other jaw closes
            yield from settle_onto(seat, lambda n=name: at_jaws(n))
            say("in jaws", name)
            yield from r.close(0.020, force=30.0, t=0.8, label="r squeeze")
            yield from r.wait(0.4)
            for _ in range(8):  # pull it out of the shut jaws, along the wire
                yield from l.move(l.at() - wire(name).axis * 0.005, t=0.35, label="l pull")
            say("pulled", name)
            yield from r.close(span, force=6.0, t=0.5, label="r release")
            yield from l.move(l.at() + [0, 0.003, 0], t=0.4, label="l off the lower jaw")
            yield from l.move(l.at() + [0.045, 0, 0], t=1.0, label="l out past the jaws")
            yield from l.wait(0.3)
            w = wire(name)  # straighten the wire in the hand: it turns about the fingers' closing axis (x) when pulled
            tilt_x = math.degrees(math.atan2(w.axis[1], w.axis[2]))
            if abs(tilt_x) > 1.0:
                yield from l.turn(tilt_x, axis=(1.0, 0.0, 0.0), t=max(0.5, abs(tilt_x) / 30.0), label="l straighten")
                yield from l.wait(0.3)
            say("straightened", name)
        # right hand: off the stripper, take the receptacle from its tray and hold it, back down, beside the box
        yield from r.open(t=0.4, label="r let go of the stripper")
        yield from r.wait(0.5)
        say("r open", name)
        yield from r.move(r.at() + [0, 0, 0.025], t=0.8, label="r up off the stripper")
        yield from r.wait(0.3)
        say("r up", name)
        rec = ctx.state("receptacle")
        pinch = rec.pos + rec.rotation() @ REC_PINCH
        yield from r.move(np.array([pinch[0], pinch[1], r.at()[2]]), grip=down(0), t=2.0, label="r across, clear of the open handle")  # level: the open handle splays towards -y
        if DEBUG:
            print(f"[demo]    r across: TCP {np.round(r.tcp(), 4)} cmd {np.round(r.at(), 4)} stripper at {np.round(ctx.pos('stripper'), 4)}", flush=True)
        yield from r.move(pinch + [0, 0, 0.03], t=1.2, label="r over the receptacle")
        yield from r.move(pinch, t=1.0, label="r down to the receptacle")
        yield from r.wait(0.3)
        yield from r.close(REC_BODY[1], force=40.0, t=0.5, label="r grip the receptacle")
        yield from r.wait(0.3)
        if DEBUG:
            print(f"[demo]    r gripped: TCP {np.round(r.tcp(), 4)} cmd {np.round(r.at(), 4)} rec at {np.round(ctx.pos('receptacle'), 4)} pinch {np.round(pinch, 4)}", flush=True)
        yield from r.move(r.at() + [0, 0, 0.06], t=1.0, label="r lift the receptacle")
        hold = REC_HOLD + REC_PINCH
        yield from r.move(hold + [0, 0, 0.015], t=2.0, label="r carry the receptacle")
        yield from r.move(hold, t=1.0, label="r hold the receptacle")
        yield from r.wait(0.8)
        say("receptacle held", name)
        # left hand: the conductor's end under the brass hole, then up into it until the jacket meets the back
        hname = "silver_a" if mode == "silver" else HOLE_FOR[name]
        mouth, _ = rec_hole(hname)
        reach = float(tip(name)[2] - l.tcp()[2])  # how far the tip stands above the hand now (the wire slips a little in the pull)
        yield from l.move(np.array([l.at()[0], l.at()[1], mouth[2] - 0.020 - reach]), t=1.2, label="l down, the end below the receptacle")
        yield from l.move(np.array([mouth[0], mouth[1], mouth[2] - 0.020 - reach]), t=1.5, label="l across, under it")
        say("across", name)
        below = mouth - [0, 0, 0.006]
        yield from l.move(np.array([mouth[0], mouth[1], below[2] - reach]), t=1.0, label="l under the hole")
        yield from l.wait(0.3)
        say("under the hole", name)
        def below_hole(d=0.006):  # a point on the hole's axis, d short of its mouth
            mo, ax = rec_hole(hname)
            return mo - ax * d

        def align():  # turn the hand about its TCP so the wire runs along the hole's axis: it leans in the grip as the lead pulls on it
            a, h = wire(name).axis, rec_hole(hname)[1]
            ang = math.degrees(math.acos(float(np.clip(a @ h, -1.0, 1.0))))
            if ang > 1.0:
                u = np.cross(a, h)
                yield from l.turn(ang, axis=tuple(u / np.linalg.norm(u)), t=max(0.4, ang / 30.0), label="l align with the hole")
                yield from l.wait(0.3)

        yield from align()
        yield from settle_onto(below_hole, lambda n=name: tip(n))
        yield from align()
        yield from settle_onto(below_hole, lambda n=name: tip(n))
        if DEBUG:
            off = tip(name) - below_hole()
            ang = math.degrees(math.acos(float(np.clip(wire(name).axis @ rec_hole(hname)[1], -1, 1))))
            print(f"[demo]    tip off the hole's axis by {np.round(off * 1000, 2)} mm, wire {ang:.1f} deg off the hole's axis", flush=True)
        for k in range(1, 15):  # push up in 2 mm steps, keeping the tip on the hole's axis so the hand stays over the wire
            goal = below_hole(0.006 - min(0.002 * k, 0.006 + strip_len + 0.001))
            err = goal - tip(name)
            yield from l.move(l.at() + err + rec_hole(hname)[1] * 0.001, t=0.3, label="l push in")
            yield from l.wait(0.2)
            if k % 4 == 0:
                yield from align()
            if DEBUG:
                print(f"[demo]    push {k}: tip err {np.round((goal - tip(name)) * 1000, 2)} mm, depth {wire(name).depth * 1000:.1f}, rec {np.round(ctx.pos('receptacle'), 4)}", flush=True)
        yield from l.wait(0.5)
        say("pushed", name)
        yield from l.move(l.at(), finger=PICK_OPEN, t=0.5, label="l let go")
        yield from l.move(l.at() + [0, 0.04, 0.0], t=1.0, label="l slide off past the fingertips")
        yield from l.move(l.at() + [0, 0.04, -0.03], t=1.0, label="l away")
    yield from ctx.until(lambda: False, timeout=1.5, label="hold")
    if DEBUG:
        from yam_arena.scene import object_states
        sts = object_states(ctx.emb._env, WIRES)
        print(f"[demo] report: {[wire_report(w, sts[w]) for w in WIRES]}, score {score(sts):.3f}", flush=True)
