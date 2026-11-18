"""Strip a 14 AWG solid pigtail to a receptacle's strip gauge and push it into a push-in hole on the receptacle's brass (hot) side.

On the station table, in front of the right arm: a wire stripper (Klein 11055 pattern: 181 mm, blue and yellow handles,
stripping holes for 10, 12, 14, 16 and 18 AWG solid, the larger holes nearer the pivot) lies flat on a grey stand 20 cm
high, its jaws open and overhanging the stand's front edge, pointing away from the robot. Its return spring holds the
jaws open; squeezing the handles shuts them.
In front of the left arm: a black 150 mm pigtail of 14 AWG solid copper THHN (2.69 mm over the jacket) stands end-up in
a wooden block. Beyond the stripper, towards the robot's right, a white 15 A duplex receptacle (Leviton T5320 pattern)
is held 18.5 cm up on a post, its back facing down: four push-in holes on its back, two on its brass-screw (hot) side,
which faces the robot, and two on its silver-screw (neutral) side, which faces away. The receptacle's strip gauge is
5/8 in (16 mm).

To strip the wire, close the stripper's matching hole on it at the strip length from its end and pull the wire out of the
shut jaws: the cut jacket stays in the jaws. Push the stripped end straight into a push-in hole until the jacket meets the
receptacle's back; the hole's spring clamp then holds it.
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
INSTRUCTION = ("Strip the black wire to the receptacle's 5/8 inch (16 mm) strip gauge with the wire stripper, then push it fully into a "
               "push-in hole on the brass (hot) side of the receptacle's back, and let go of it.")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- layout (world frame: x forward from the arms, y left, z up; m) ------------------------------------------------------
H = TOP + 0.200  # the stripper's plane: high enough that the left hand passes under it
PIVOT = np.array([0.30, -0.10, H])
WIRE_L, WIRE_R = 0.150, 0.001346
GAUGE = 0.016  # the receptacle's strip gauge: 5/8 in
BARE_FULL, BARE_PART = (0.014, 0.018), (0.012, 0.020)  # bare length for full and partial strip credit
EXPOSED_MAX = 0.0015  # copper showing between the jacket and the receptacle's back, at most
HOLDERS = {"black": np.array([0.40, 0.04])}  # the pigtail stands in its block, picked from above
REC = np.array([0.45, -0.05])  # the receptacle's centre (x, y), out to the side of the stripper; its length runs along y
ZB = TOP + 0.185  # its back face, facing down: high enough that a held wire hangs clear of the table
BODY = (0.033, 0.072, 0.022)  # body below the strap: width (x), length (y), depth (z)
CHANNEL, CH_DEPTH = 0.0026, 0.0175  # push-in channels: square (the 1.63 mm copper passes, the 2.69 mm jacket does not), and depth to the bottom
FUNNEL, FUNNEL_DEPTH = 0.005, 0.001  # each push-in hole's lead-in chamfer: square at the back face, and its depth
JACKET_IN_MAX = 0.0015  # the jacket this far inside a hole means the clamp is on insulation, not copper
# push-in holes (dx, dy from the centre): the brass (hot) side faces the robot (-x), the silver (neutral) side faces away
HOLES = {"brass_a": (-0.011, 0.010), "brass_b": (-0.011, -0.010), "silver_a": (0.011, 0.010), "silver_b": (0.011, -0.010)}
SIDE = {"black": "brass"}
CLEAR = 0.008  # a fingertip within this of a wire's axis counts as touching it

LOOKS = {
    "rest": m.Look(color=m.rgb(0.35, 0.35, 0.38), roughness=0.7),
    "wood": m.Look(color=m.rgb(0.62, 0.46, 0.28), roughness=0.8),
    "nylon": m.Look(color=m.rgb(0.93, 0.93, 0.90), roughness=0.5),
    "strap": m.Look(color=m.rgb(0.70, 0.71, 0.73), roughness=0.35, metallic=0.9),
    "brass": m.Look(color=m.rgb(0.80, 0.62, 0.25), roughness=0.3, metallic=0.9),
    "silver": m.Look(color=m.rgb(0.80, 0.81, 0.83), roughness=0.3, metallic=0.9),
    "dark": m.Look(color=m.rgb(0.08, 0.08, 0.08), roughness=0.6),
}


def hole_mouth(name: str) -> np.ndarray:
    dx, dy = HOLES[name]
    return np.array([REC[0] + dx, REC[1] + dy, ZB])


def _box(parts, cols, name, size, c, look, collide=True):
    parts.append((name, m.box(size, c), look))
    if collide:
        cols.append(m.col_box(name + "_c", size, c))


def _fixtures():
    parts, cols = [], []
    # the stripper's stand: under its neck (jaws overhang the front edge) and under its handles
    neck_top = H - S.JAW_T / 2 - 0.0005
    _box(parts, cols, "stand_neck", (0.024, 0.030, neck_top - TOP), (PIVOT[0] - 0.006, PIVOT[1], (neck_top + TOP) / 2), "rest")
    handle_top = H - S.HANDLE_T / 2 - 0.0005
    _box(parts, cols, "stand_handle", (0.012, 0.070, handle_top - TOP), (PIVOT[0] - 0.070, PIVOT[1] - 0.012, (handle_top + TOP) / 2), "rest")
    # a wooden block per pigtail: 20 x 20 x 30 mm with a 4 mm square hole down its middle
    hole, depth, side = 0.004, 0.030, 0.020
    wall = (side - hole) / 2
    for name, (x, y) in HOLDERS.items():
        for tag, size, c in (("px", (wall, side, depth), (x + (hole + wall) / 2, y)), ("nx", (wall, side, depth), (x - (hole + wall) / 2, y)),
                             ("py", (hole, wall, depth), (x, y + (hole + wall) / 2)), ("ny", (hole, wall, depth), (x, y - (hole + wall) / 2))):
            _box(parts, cols, f"block_{name}_{tag}", size, (c[0], c[1], TOP + depth / 2), "wood")
    # the receptacle, back down, its push-in channels open downward, each with a lead-in chamfer at the mouth
    W, L, D = BODY
    cx, cy = REC
    c = CHANNEL / 2
    fh, fd = FUNNEL / 2, FUNNEL_DEPTH
    hx = sorted({cx + dx for dx, _ in HOLES.values()})
    hy = sorted({cy + dy for _, dy in HOLES.values()})
    zc = ZB + fd + (CH_DEPTH - fd) / 2
    for z0, z1, half, tag in ((ZB + fd, ZB + CH_DEPTH, c, "ch"), (ZB, ZB + fd, fh, "fn")):  # above the chamfers: channels; at the mouth: chamfer squares
        xs = [cx - W / 2] + [v for x0 in hx for v in (x0 - half, x0 + half)] + [cx + W / 2]
        ys = [cy - L / 2] + [v for y0 in hy for v in (y0 - half, y0 + half)] + [cy + L / 2]
        for i in range(len(xs) - 1):
            for j in range(len(ys) - 1):
                if i % 2 == 1 and j % 2 == 1:
                    continue
                _box(parts, cols, f"rec_{tag}_{i}_{j}", (xs[i + 1] - xs[i], ys[j + 1] - ys[j], z1 - z0),
                     ((xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2, (z0 + z1) / 2), "nylon")
    for name, (dx, dy) in HOLES.items():  # four sloped walls per chamfer, from FUNNEL square at the back to the channel
        x0, y0 = cx + dx, cy + dy
        for ax in (0, 1):
            for sgn in (-1.0, 1.0):
                pts = []
                for t in (-fh, fh):
                    for (r, z) in ((fh, ZB), (c, ZB + fd), (fh, ZB + fd)):
                        p_ = [x0, y0, z]
                        p_[ax] += sgn * r
                        p_[1 - ax] += t
                        pts.append(p_)
                cols.append(m.col_hull(f"chamfer_{name}_{ax}_{int(sgn > 0)}", m.Mesh(np.array(pts), np.array([[0, 1, 2], [3, 4, 5]]))))
    _box(parts, cols, "rec_top", (W, L, D - CH_DEPTH), (cx, cy, ZB + CH_DEPTH + (D - CH_DEPTH) / 2), "nylon")
    zs = ZB + D
    _box(parts, cols, "rec_strap", (0.022, 0.106, 0.0015), (cx, cy, zs + 0.00075), "strap")
    _box(parts, cols, "rec_face", (0.030, 0.066, 0.004), (cx, cy, zs + 0.0015 + 0.002), "nylon")
    for k, dy in enumerate((-0.018, 0.018)):  # terminal screws: brass towards the robot (hot), silver away (neutral)
        parts.append((f"screw_b{k}", m.cylinder(0.004, 0.003, (cx - W / 2 - 0.0015, cy + dy, ZB + 0.012), axis="x"), "brass"))
        parts.append((f"screw_s{k}", m.cylinder(0.004, 0.003, (cx + W / 2 + 0.0015, cy + dy, ZB + 0.012), axis="x"), "silver"))
    for name, (dx, dy) in HOLES.items():  # dark rims round the holes, and the release slots beside them
        parts.append((f"rim_{name}", m.box((CHANNEL + 0.001, CHANNEL + 0.001, 0.0004), (cx + dx, cy + dy, ZB - 0.0002)), "dark"))
        parts.append((f"slot_{name}", m.box((0.0012, 0.004, 0.0004), (cx + dx + (0.004 if dx > 0 else -0.004), cy + dy, ZB - 0.0002)), "dark"))
    # a post under the strap's far (-y) end holds it, beyond every path the hands take
    _box(parts, cols, "post", (0.020, 0.010, zs - TOP), (cx, cy - 0.050, (zs + TOP) / 2), "rest")
    return parts, cols


_fp, _fc = _fixtures()
_HOLE_SPECS = tuple((name, tuple(hole_mouth(name)), (0.0, 0.0, 1.0)) for name in HOLES)
OBJECTS = [
    S.make("stripper", tuple(PIVOT)),
    m.fixed("fixtures", (0.0, 0.0, 0.0), _fp, _fc, LOOKS),
    wk.Pigtail("black", pos=(*HOLDERS["black"], TOP + 0.002 + WIRE_L / 2), size=(0.003, 0.003, WIRE_L), jacket=(0.03, 0.03, 0.03), holes=_HOLE_SPECS),
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
        a, b = st.end - st.axis * WIRE_L, st.end
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
GRAB_UP = 0.100  # the left hand holds a wire this far up from its bottom end: close below the cut, so the jaws cannot tip it
PICK_OPEN = -0.012  # finger joint: 24 mm between the pads when coming to a wire
HOLE_FOR = {"black": "brass_a"}


def demo_script(ctx):
    """Right hand: pinch the stripper's handles from above and squeeze or release them on cue. Left hand, for each wire in
    turn: lift it from the block, slide it into the open jaws at the 14 AWG hole with 16 mm above the jaws, wait for the
    jaws to shut, pull it down out of them, take it under the receptacle and push it up into its hole until the jacket
    meets the back, let go. Positions are read from the scene as it is, so the moves follow where things are."""
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
              f"depth {w.depth * 1000:.1f}", flush=True)

    def at_jaws(name):  # the point on the wire where the jaws should close: the strip length from its end, aimed a little short
        w = wire(name)
        return w.end - w.axis * (strip_len - AIM_SHORT)

    def tip(name):
        return wire(name).end

    def settle_onto(goal, read, rounds=8, tol=0.0004):
        """Move by the measured error, let the arm settle, measure again: a continuous servo here only chases the arm's lag."""
        for _ in range(rounds):
            err = np.asarray(goal) - read()
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
        return
    yield from ctx.together(r.move(handles + [0, 0, 0.02], grip=down(0), t=2.5, label="r over handles"),
                            l.move(np.r_[HOLDERS["black"], TOP + GRAB_UP] + [0, 0, 0.11], grip=level(-90), finger=PICK_OPEN, t=2.5, label="l over block"))
    yield from r.move(handles + [0, 0, 0.004], t=1.2, label="r to handles")
    yield from r.close(span, force=6.0, t=0.5, label="r hold handles")
    awg = 16 if mode == "nick" else 14
    strip_len = {"long": 0.022, "short": 0.010}.get(mode, GAUGE)
    for name in WIRES:
        grab = np.r_[HOLDERS[name], TOP + GRAB_UP]
        yield from l.move(grab + [0, 0, 0.11], grip=level(-90), finger=PICK_OPEN, t=1.8, label=f"l over {name}")
        yield from l.move(grab, t=1.5, label=f"l down to {name}")
        yield from l.wait(0.5)
        say("at wire", name)
        yield from l.close(2 * WIRE_R, force=40.0, t=0.5, label=f"l grip {name}")
        yield from l.wait(0.3)
        yield from l.move(l.at() + [0, 0, 0.11], t=1.5, label=f"l lift {name}")
        yield from l.wait(0.3)
        say("lifted", name)
        if mode != "nostrip":
            # into the open jaws from beyond their tips, 1.6 mm off the line, the 16 mm mark at the jaws' plane
            h14 = hole14()
            target = np.array([h14[0] - 0.0005, h14[1] + 0.0016, h14[2]])  # half a millimetre towards the pivot: the jaws squeeze towards their tips
            grip_z = H + strip_len - AIM_SHORT - (WIRE_L - GRAB_UP)
            yield from l.move(np.array([PIVOT[0] + S.JAW_TIP + 0.025, target[1], grip_z]), t=2.0, label="l to jaw tips")
            yield from l.wait(0.3)
            say("at jaw tips", name)
            yield from l.move(np.array([target[0], target[1], grip_z]), t=1.5, label="l into jaws")
            yield from l.servo(target, read=lambda n=name: at_jaws(n), tol=0.0005, timeout=3.0, gain=0.12, label="l wire over the 14 AWG hole")
            yield from settle_onto(target, lambda n=name: at_jaws(n))
            seat = np.array([h14[0] - 0.0003, h14[1], h14[2]])  # onto the jaws' line, into the lower jaw's notch, which holds it while the other jaw closes
            yield from settle_onto(seat, lambda n=name: at_jaws(n))
            say("in jaws", name)
            if DEBUG:
                st_, w_ = ctx.state("stripper"), wire(name)
                hit = wk.plane_hit(w_.pos, w_.axis, st_.pos, st_.rotation())
                print(f"[demo]    {name} crosses the stripper's plane at tool ({hit[0][0] * 1000:.2f}, {hit[0][1] * 1000:.2f}) mm (14 AWG hole at "
                      f"({S.HOLES[14][0] * 1000:.1f}, 0)), {(WIRE_L / 2 - hit[1]) * 1000:.1f} mm from the end; stripper yaw "
                      f"{math.degrees(math.atan2(st_.rotation()[1, 0], st_.rotation()[0, 0])):.2f} deg", flush=True)
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
        mouth = hole_mouth("silver_a" if mode == "silver" else HOLE_FOR[name])
        reach = float(tip(name)[2] - l.tcp()[2])  # how far the tip stands above the hand now (the wire slips a little in the pull)
        yield from l.move(np.array([l.at()[0], mouth[1], mouth[2] - 0.012 - reach]), t=1.5, label="l across")  # sideways first, clear of the post
        say("across", name)
        below = mouth - [0, 0, 0.006]
        yield from l.move(np.array([mouth[0], mouth[1], below[2] - reach]), t=2.0, label="l under the hole")
        yield from l.wait(0.3)
        say("under the hole", name)
        yield from settle_onto(below, lambda n=name: tip(n))
        if DEBUG:
            print(f"[demo]    tip off the hole's axis by {np.round((tip(name) - below)[:2] * 1000, 2)} mm, {(below[2] - tip(name)[2]) * 1000:.1f} mm below", flush=True)
        for k in range(1, 15):  # push up in 2 mm steps, keeping the tip on the hole's axis so the hand stays over the wire
            goal = below + [0, 0, min(0.002 * k, 0.006 + strip_len + 0.001)]
            err = goal - tip(name)
            yield from l.move(l.at() + err + [0, 0, 0.001], t=0.3, label="l push in")
            yield from l.wait(0.2)
        yield from l.wait(0.5)
        say("pushed", name)
        yield from l.move(l.at(), finger=PICK_OPEN, t=0.5, label="l let go")
        yield from l.move(l.at() + [0, 0.04, 0.0], t=1.0, label="l slide off past the fingertips")
        yield from l.move(np.array([0.44, l.at()[1], l.at()[2]]), t=1.2, label="l out")
        yield from l.move(np.array([0.44, l.at()[1], TOP + 0.20]), t=1.0, label="l up")
        yield from l.move(l.at(), grip=level(-90), t=0.6, label="l square up")
    yield from r.open(t=0.5, label="r let go")
    yield from ctx.together(r.move(r.at() + [0, 0, 0.08], t=1.0, label="r up"), l.move(l.at() + [0, 0.05, 0.0], t=1.0, label="l away"))
    yield from ctx.until(lambda: False, timeout=1.5, label="hold")
    if DEBUG:
        from yam_arena.scene import object_states
        sts = object_states(ctx.emb._env, WIRES)
        print(f"[demo] report: {[wire_report(w, sts[w]) for w in WIRES]}, score {score(sts):.3f}", flush=True)
