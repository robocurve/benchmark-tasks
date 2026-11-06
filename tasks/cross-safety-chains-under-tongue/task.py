"""Crossing a trailer's safety chains under its tongue (towing / trailer coupling). A fixed dark-grey trailer tongue
points at the robot along x; its coupler block at the robot end sits over a hitch ball with its underside 10 cm above
the table. Two dark steel chains (6 mm, 22 cm) hang from eye tabs under the tongue, one on each side (y +-5.6 cm,
8 cm up) and lie on the table, each ending in a yellow square end link. In front of the coupler a fixed tow plate
(top 5 cm up) carries two 8 mm pins 6 cm apart (y +-3 cm, tops 7.5 cm up). Each chain's end link goes over the pin on
the OPPOSITE side, so the chains cross under the coupler, and neither chain is left touching the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: CHAINS_CHEAT=none | hold | straight | short | one.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Hook the trailer's two safety chains onto the tow plate: each chain's yellow end link goes over the "
               "pin on the opposite side, so the chains cross under the coupler block. Leave neither chain touching "
               "the table. Then let go of both chains.")

DARK = (0.18, 0.19, 0.21)
STEEL = (0.45, 0.46, 0.49)
ZINC = (0.78, 0.79, 0.82)
CHAIN = (0.33, 0.34, 0.37)  # dark galvanised steel, visible on the light table
YELLOW = (0.95, 0.78, 0.08)

# -- trailer tongue and coupler (fixed) ---------------------------------------------------------------------------------
UNDER = 0.10  # coupler / tongue underside above the table
CPL_X0, CPL_X1, CPL_HW = 0.21, 0.29, 0.025  # coupler footprint: x 21-29 cm, y +-2.5 cm
COUPLER = Box("coupler", pos=((CPL_X0 + CPL_X1) / 2, 0.0, TOP + UNDER + 0.0175),
              size=(CPL_X1 - CPL_X0, 2 * CPL_HW, 0.035), color=DARK)
TONGUE = Box("tongue", pos=(CPL_X1 + 0.15, 0.0, TOP + UNDER + 0.015), size=(0.30, 0.030, 0.030), color=DARK)
EYE_X, EYE_Y, EYE_Z = 0.38, 0.056, TOP + 0.08  # chain eye (chain start) on each side
TABS = [Box(f"eye_tab_{s}", pos=(EYE_X, sg * 0.0325, TOP + 0.0925), size=(0.016, 0.035, 0.015), color=DARK)
        for s, sg in (("l", 1), ("r", -1))]

# -- tow plate, pins, ball mount (fixed) --------------------------------------------------------------------------------
PLATE_TOP = 0.05
PIN_X, PIN_Y, PIN_R, PIN_H = 0.18, 0.03, 0.004, 0.025
PIN_TOP = TOP + PLATE_TOP + PIN_H
PLATE = Box("tow_plate", pos=(0.18, 0.0, TOP + PLATE_TOP / 2), size=(0.08, 0.10, PLATE_TOP), color=STEEL)
PINS = {sg: Cylinder(f"pin_{s}", pos=(PIN_X, sg * PIN_Y, TOP + PLATE_TOP + PIN_H / 2), radius=PIN_R, height=PIN_H,
                     color=ZINC) for s, sg in (("l", 1), ("r", -1))}
# ball mount bar on the plate's midline; the hitch ball itself sits inside the coupler (hidden)
DRAWBAR = Box("ball_mount", pos=(0.1825, 0.0, TOP + PLATE_TOP + 0.01), size=(0.065, 0.016, 0.02), color=STEEL)

# -- chains (cables) with welded end links ------------------------------------------------------------------------------
R = 0.003
CH_LEN = 0.22
BAR = 0.008
LINK_W = 0.036
LINK_OFF = LINK_W / 2 - BAR / 2  # 14 mm: side-bar centre lines from the link centre
LINK_IN = LINK_W - 2 * BAR  # 20 mm opening
GAP = 0.001
CH_YAW = {1: 3 * math.pi / 4, -1: -3 * math.pi / 4}  # left chain lies to the front left, right chain front right


def _chain(s: str, sg: int):
    yaw = CH_YAW[sg]
    d = np.array([math.cos(yaw), math.sin(yaw)])
    start = np.array([EYE_X, sg * EYE_Y])
    c = cable(f"chain_{s}", start=(float(start[0]), float(start[1]), EYE_Z), yaw=yaw, length=CH_LEN, radius=R,
              anchor_start=True, cable_mass=0.05, bend_stiffness=0.002, bend_damping=0.001, color=CHAIN)
    throat = start + d * (CH_LEN + GAP + BAR / 2)
    link = Box(f"link_{s}", pos=(float(throat[0]), float(throat[1]), EYE_Z), size=(BAR, LINK_W, BAR), mass=0.008,
               color=YELLOW, yaw=yaw)
    return c, link


CHAIN_L, LINK_L = _chain("l", 1)
CHAIN_R, LINK_R = _chain("r", -1)
CHAINS = {1: (CHAIN_L, LINK_L), -1: (CHAIN_R, LINK_R)}
# A link's body is its throat bar (local x = along the chain, away from it); the other three bars are extra colliders.
LINK_PARTS = (("far", (2 * LINK_OFF, 0.0, 0.0), (BAR, LINK_W, BAR)),
              ("side_a", (LINK_OFF, LINK_OFF, 0.0), (LINK_W, BAR, BAR)),
              ("side_b", (LINK_OFF, -LINK_OFF, 0.0), (LINK_W, BAR, BAR)))

OBJECTS = [COUPLER, TONGUE, *TABS, PLATE, *PINS.values(), DRAWBAR, LINK_L, LINK_R, CHAIN_L, CHAIN_R]

TWIN_EXTRA = [
    {"name": "end_links", "kind": "body", "size_mm": [36, 36, 8], "colour": "yellow",
     "note": "square end link of 8 mm bars (20 mm opening), one on each chain end; real build: a printed "
             "36 mm square link"},
    {"name": "link_weld", "kind": "joint", "real": "chain end crimped/glued to the end link",
     "note": "one per chain"},
    {"name": "eye_swivel", "kind": "joint", "real": "chain's first link through the eye tab (free to swing)",
     "note": "ball joint at each eye, position fixed, rotation free"},
]

# -- spawn wrap: link colliders, link welds, eye swivels ----------------------------------------------------------------


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_chains_task", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        sg = {CHAIN_L.name: 1, CHAIN_R.name: -1}.get(name)
        if sg is None:
            return prim
        ch, link = CHAINS[sg]
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        root = stage.GetPrimAtPath(f"{parent}/{link.name}")
        mesh = None
        for p in stage.Traverse():
            if str(p.GetPath()).startswith(str(root.GetPath()) + "/") and p.HasAPI(UsdPhysics.CollisionAPI):
                mesh = p
                break
        rel_v = mesh.GetRelationship("material:binding") if mesh is not None else None
        rel_p = mesh.GetRelationship("material:binding:physics") if mesh is not None else None
        for pname, rel, size in LINK_PARTS:
            g = UsdGeom.Cube.Define(stage, f"{root.GetPath()}/{pname}")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, rel)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            for src in (rel_v, rel_p):
                if src is not None and src.GetTargets():
                    g.GetPrim().CreateRelationship(src.GetName()).SetTargets(src.GetTargets())
            if rel_v is not None and rel_v.GetTargets():
                UsdShade.MaterialBindingAPI.Apply(g.GetPrim())
        j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/link_weld")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_{ch.segments - 1}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/{link.name}")])
        j.CreateLocalPos0Attr(Gf.Vec3f(float(ch.seg_len / 2 + GAP + BAR / 2), 0.0, 0.0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        # eye: replace the kit's rigid world weld by a ball joint (position held, free to swing)
        stage.RemovePrim(f"{prim_path}/anchor")
        e = UsdPhysics.Joint.Define(stage, f"{prim_path}/eye_swivel")
        e.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        e.CreateLocalPos0Attr(Gf.Vec3f(*map(float, ch.start)))
        e.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        e.CreateLocalPos1Attr(Gf.Vec3f(-ch.seg_len / 2, 0.0, 0.0))
        e.CreateLocalRot1Attr(Gf.Quatf(math.cos(-ch.yaw / 2), 0.0, 0.0, math.sin(-ch.yaw / 2)))
        for dof in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(e.GetPrim(), dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        e.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._chains_task = True
    cablekit._spawn_cable = spawn


_install_spawn()

# -- env handle (gripper release check) --------------------------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_chains_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._chains_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World TCP positions of both grippers (yam_arena.runtime.tcp_world, env 0), [] if unknown."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry ----------------------------------------------------------------------------------------------------------


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def link_frame(objects, sg: int):
    """(centre, M) of a chain's end link; M columns = local x (throat -> far bar), local y, normal. The state's pos is
    the centre of the whole link's bounds (all four bars), i.e. the link centre."""
    s = objects[CHAINS[sg][1].name]
    return np.asarray(s.pos, float), _rot(s.quat_xyzw)


def _link_points(objects, sg: int) -> np.ndarray:
    c, m = link_frame(objects, sg)
    t = np.linspace(-LINK_OFF, LINK_OFF, 5)
    pts = []
    for a, b in ((0, 1), (1, 0)):
        for s in (-1, 1):
            pts += [c + m[:, a] * s * LINK_OFF + m[:, b] * ti for ti in t]
    return np.array(pts)


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out += [a + (b - a) * (i / k) for i in range(1, k + 1)]
    return np.array(out)


def _chain_poly(objects, sg: int) -> np.ndarray:
    return np.asarray(objects[CHAINS[sg][0].name].polyline(), float)


def link_on_pin(objects, sg: int, pin_sg: int) -> bool:
    """The pin's axis (from the plate top to the pin top) passes through the link's 20 mm opening."""
    c, m = link_frame(objects, sg)
    if not np.all(np.isfinite(c)):
        return False
    n = m[:, 2]
    if abs(float(n[2])) < 1e-3:
        return False
    p0 = np.array([PIN_X, pin_sg * PIN_Y, TOP + PLATE_TOP])
    s = float((c - p0) @ n) / float(n[2])
    if not -0.002 <= s <= PIN_H + 0.002:
        return False
    q = p0 + np.array([0.0, 0.0, s]) - c
    half = LINK_IN / 2
    return abs(float(q @ m[:, 0])) < half and abs(float(q @ m[:, 1])) < half


def _seg_cross(p, q, a, b):
    """2D intersection point of segments pq and ab (or None) and the params (t on pq, u on ab)."""
    r, s = q - p, b - a
    den = r[0] * s[1] - r[1] * s[0]
    if abs(den) < 1e-12:
        return None
    w = a - p
    t = (w[0] * s[1] - w[1] * s[0]) / den
    u = (w[0] * r[1] - w[1] * r[0]) / den
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return t, u
    return None


def crossings(objects) -> list[np.ndarray]:
    """Top-view crossing points of the two chains: [(x, y, z_left, z_right)]."""
    a, b = _chain_poly(objects, 1), _chain_poly(objects, -1)
    out = []
    for i in range(len(a) - 1):
        for j in range(len(b) - 1):
            r = _seg_cross(a[i, :2], a[i + 1, :2], b[j, :2], b[j + 1, :2])
            if r is None:
                continue
            t, u = r
            pa = a[i] + t * (a[i + 1] - a[i])
            pb = b[j] + u * (b[j + 1] - b[j])
            out.append(np.array([pa[0], pa[1], pa[2], pb[2]]))
    return out


def crossed_under_coupler(objects) -> bool:
    xs = crossings(objects)
    return bool(xs) and all(CPL_X0 <= x[0] <= CPL_X1 and abs(x[1]) <= CPL_HW and max(x[2], x[3]) < TOP + UNDER
                            for x in xs)


CLEAR = 0.010  # chain underside at least 1 cm above the table
RELEASE_DIST = 0.05  # every gripper TCP at least this far from both chains and end links


def chain_min_clear(objects) -> float:
    z = min(float(_chain_poly(objects, sg)[:, 2].min()) for sg in (1, -1))
    return z - R - TOP


def chain_max_z(objects) -> float:
    return max(float(_chain_poly(objects, sg)[:, 2].max()) for sg in (1, -1)) + R - TOP


def released(objects) -> bool:
    pts = np.vstack([_dense(_chain_poly(objects, sg)) for sg in (1, -1)] + [_link_points(objects, sg) for sg in (1, -1)])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(_chain_poly(objects, sg))) and np.all(np.isfinite(link_frame(objects, sg)[0]))
               for sg in (1, -1))


def score(objects) -> float:
    """Score 0 if: a state is invalid; a gripper TCP is within 5 cm of a chain or end link; an end link is on the pin
    on its own chain's side (not crossed); any part of a chain is higher than the coupler underside (10 cm), i.e. a
    chain was laid over the tongue. Otherwise: 0.3 if one end link is on the opposite pin (or both are but the crossing or
    clearance test fails); 1.0 if both are, the chains cross (top view) only inside the coupler footprint and below its underside, and every chain point
    is at least 1 cm above the table."""
    if not _finite(objects) or not released(objects):
        return 0.0
    if any(link_on_pin(objects, sg, sg) for sg in (1, -1)):
        return 0.0
    if chain_max_z(objects) > UNDER:
        return 0.0
    n = sum(link_on_pin(objects, sg, -sg) for sg in (1, -1))
    if n == 0:
        return 0.0
    if n == 1:
        return 0.3
    return 1.0 if crossed_under_coupler(objects) and chain_min_clear(objects) >= CLEAR else 0.3


def success(objects) -> bool:
    return (score(objects) >= 1.0 and all(objects[CHAINS[sg][0].name].resting(max_speed=0.02) for sg in (1, -1)))


def distance(objects) -> float:
    """Sum of each end link's horizontal distance to its opposite pin (m)."""
    d = 0.0
    for sg in (1, -1):
        c, _ = link_frame(objects, sg)
        d += float(np.hypot(c[0] - PIN_X, c[1] + sg * PIN_Y))
    return d


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    return {
        "left_on_opposite_pin": int(link_on_pin(objects, 1, -1)),
        "right_on_opposite_pin": int(link_on_pin(objects, -1, 1)),
        "no_own_side_pin": int(not any(link_on_pin(objects, sg, sg) for sg in (1, -1))),
        "under_tongue": int(chain_max_z(objects) <= UNDER),
        "crossed_under_coupler": int(crossed_under_coupler(objects)),
        "off_ground": int(chain_min_clear(objects) >= CLEAR),
        "released": int(released(objects)),
    }


VARIATION = {
    # chains hang from fixed eyes; only their lay on the table varies
    "cable:chain_l": {"dx": 0.0, "dy": 0.0, "yaw": 0.15},
    "cable:chain_r": {"dx": 0.0, "dy": 0.0, "yaw": 0.15},
}

# -- scripted demo -----------------------------------------------------------------------------------------------------

GRASP_OPEN = -0.010  # finger joint: pads ~20 mm apart round the 8 mm bar, so neither lands on another bar


def _yaw_of(q) -> float:
    m = _rot(q)
    return math.atan2(m[1, 0], m[0, 0])


def _wrap_half(a: float) -> float:
    return (a + math.pi / 2) % math.pi - math.pi / 2


def demo_script(ctx):
    """Left arm takes the left chain's end link by a side bar, carries it under the coupler and drops it over the right
    pin, then backs off; the right arm does the mirror image onto the left pin, so its chain lies over the first at the
    crossing. Closed loop on the live link poses."""
    from yam_arena.constants import FINGER_CLOSED

    cheat = os.environ.get("CHAINS_CHEAT", "")  # test only: none | hold | straight | short | one
    hi = TOP + 0.06
    carry = TOP + 0.115

    def log(tag):
        st = ctx.states()
        if not _finite(st):
            print(f"[demo t={ctx.t:5.1f}] {tag}: NON-FINITE STATE score={score(st):.2f}", flush=True)
            return
        print(f"[demo t={ctx.t:5.1f}] {tag}: score={score(st):.2f} {progress(st)} "
              f"clear={chain_min_clear(st):.3f} maxz={chain_max_z(st):.3f} "
              f"L={np.round(link_frame(st, 1)[0] - [0, 0, TOP], 3)} R={np.round(link_frame(st, -1)[0] - [0, 0, TOP], 3)} "
              f"tcp={[np.round(x - [0, 0, TOP], 3).tolist() for x in grippers()]}", flush=True)

    log("start")
    st0 = ctx.states()
    for sg in (1, -1):
        o = st0[CHAINS[sg][1].name]
        print(f"[demo] link {sg} pos={np.round(o.pos - [0, 0, TOP], 4)} ext={np.round(o.high - o.low, 4)}", flush=True)
    if cheat == "none":
        for _ in range(8):
            yield from ctx.left.wait(0.5)
        log("end")
        return

    def hook(arm, sg, pin_sg, drop_on_pin=True, release=True):
        """Arm on side sg takes its chain's end link and drops it over the pin on side pin_sg."""
        st = ctx.states()
        c, m = link_frame(st, sg)
        ly = math.atan2(m[1, 0], m[0, 0])
        gy0 = _wrap_half(ly)
        turn = -gy0  # the gripper ends at yaw 0 (wrist camera towards the coupler and pins)
        # pinch the side bar that, after the turn, lies on this arm's own side of the link (closer to the arm)
        best = None
        for s in (-1, 1):
            v = m[:2, 1] * s
            vr = np.array([math.cos(turn) * v[0] - math.sin(turn) * v[1],
                           math.sin(turn) * v[0] + math.cos(turn) * v[1]])
            if best is None or vr[1] * sg > best[1] * sg:
                best = (s, vr[1])
        bar = c + m[:, 1] * best[0] * LINK_OFF
        yield from arm.move((bar[0], bar[1], hi), yaw=gy0, finger=GRASP_OPEN, duration=1.5)
        # closed loop on the measured TCP (it lands ~6 mm off the command): centre over the bar while still above it,
        # then descend until the pads are 2.5 mm off the table, round the middle of the 8 mm bar
        k = 0 if sg == 1 else 1
        yield from arm.move((bar[0], bar[1], bar[2] + 0.030), yaw=gy0, duration=0.6)
        yield from arm.wait(0.3)
        cmd = arm.tcp() + np.r_[bar[:2] - grippers()[k][:2], 0.0]
        yield from arm.move((cmd[0], cmd[1], bar[2] + 0.016), yaw=gy0, duration=0.6)
        yield from arm.wait(0.3)
        z = arm.tcp()[2] + (TOP + 0.0155 - grippers()[k][2])
        yield from arm.move((cmd[0], cmd[1], z), yaw=gy0, duration=0.4)
        yield from arm.wait(0.3)
        g = grippers()
        print(f"[demo] pick bar={np.round(bar - [0, 0, TOP], 4)} gy0={gy0:.3f} cmd={np.round(arm.tcp() - [0, 0, TOP], 4)} "
              f"tcps={[np.round(x - [0, 0, TOP], 4).tolist() for x in g]}", flush=True)
        yield from arm.grip(FINGER_CLOSED, 0.6)
        yield from arm.move((bar[0], bar[1], carry), yaw=gy0, duration=0.8)
        log(f"{arm.name} link picked")
        # carry: first towards the robot and the midline, then over the target pin, turning to yaw 0
        tgt = np.array([PIN_X, pin_sg * PIN_Y + sg * LINK_OFF])
        if not drop_on_pin:  # aim 35 mm past the pin (towards the eyes): never over the pin
            tgt[0] += 0.035
        yield from arm.move((0.15, sg * 0.10, carry), yaw=gy0 / 2, duration=1.5)
        yield from arm.move((tgt[0], tgt[1], carry), yaw=0.0, duration=1.5)
        yield from arm.wait(0.4)
        aim = np.array([PIN_X, pin_sg * PIN_Y]) if drop_on_pin else tgt - np.array([0.0, sg * LINK_OFF])
        for _ in range(3):  # closed loop: link centre over the pin
            st = ctx.states()
            c, _ = link_frame(st, sg)
            e = aim - c[:2]
            if np.linalg.norm(e) < 0.002:
                break
            tgt = tgt + e
            yield from arm.move((tgt[0], tgt[1], carry), yaw=0.0, duration=0.6)
            yield from arm.wait(0.3)
        c, _ = link_frame(ctx.states(), sg)
        want = (PIN_TOP + BAR / 2 + 0.005) if drop_on_pin else (TOP + PLATE_TOP + BAR / 2 + 0.005)
        zt = arm.tcp()[2] - (c[2] - want)
        yield from arm.move((tgt[0], tgt[1], zt), yaw=0.0, duration=1.0)
        yield from arm.wait(0.3)
        log(f"{arm.name} link over pin")
        if not release:
            return
        yield from arm.grip(GRASP_OPEN, 0.5)
        yield from arm.wait(0.4)
        log(f"{arm.name} released")
        yield from arm.move((tgt[0], tgt[1], TOP + 0.15), yaw=0.0, duration=0.8)
        yield from arm.move((0.12, sg * 0.20, TOP + 0.15), yaw=0.0, duration=1.0)
        log(f"{arm.name} backed off")

    L, Rt = ctx.left, ctx.right
    if cheat == "straight":  # each chain straight onto its own-side pin
        yield from hook(L, 1, 1)
        yield from hook(Rt, -1, -1)
    elif cheat == "short":  # crossed, but laid on the table short of the opposite pin
        yield from hook(L, 1, -1, drop_on_pin=False)
        yield from hook(Rt, -1, 1, drop_on_pin=False)
    elif cheat == "one":  # only the left chain hooked crossed (partial credit check, expect 0.3)
        yield from hook(L, 1, -1)
    elif cheat == "hold":  # everything right, but the right gripper keeps hold of its link
        yield from hook(L, 1, -1)
        yield from hook(Rt, -1, 1, release=False)
    else:
        yield from hook(L, 1, -1)
        yield from hook(Rt, -1, 1)
    for _ in range(5):
        yield from L.wait(0.5)
    log("end")
