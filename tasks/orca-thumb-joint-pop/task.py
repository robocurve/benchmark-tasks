"""Pop an ORCA Hand's dislocated thumb tip back into its joint: both bearings into their arc grooves at once.

The thumb is ORCA v2's own CAD (orcahand_hardware, CC BY 4.0): T-PP (proximal phalanx, 18 x 19 x 49 mm) and T-DP (distal phalanx, 16 x 15 x 38 mm), from the STL files, thinned to
about 1,700 and 1,100 vertices (orca_thumb.npz). It is mounted on ORCA's own palm (orca_hand.npz, from the v2 URDF and its STLs, built by buildhand.py): T-PP sits on T-AP and T-TP at the URDF's
rest angles, and the three turn together on the thumb's CMC joint, at the URDF's place and axis on the palm. The hand is powered off, so the CMC turns against only its slack tendons (a soft spring,
0.3 N m s/rad damping). T-DP hangs off T-PP's end on its two slack tendons, dislocated 4 mm out of the joint, cocked 30 degrees down and 10 degrees about the thumb's axis. The tendons are a tether of six soft joints between T-PP and T-DP (three slides and three turns about the bearings' centre), all PhysX joint drives (estimates below).
T-DP's two bearings (MR84, 4 x 8 x 3 mm steel, per ORCA's assembly notes) are drawn on T-DP's own printed axle stubs (3.9 mm across, 3 mm long, each side of its base, read from T-DP.stl); T-DP's collider is the hull of T-DP and both bearings.
Code: the groove's rim and lip (gains switched on those joints), the joint that closes once both bearings are seated and opens again under a pull past the pop force, the tip's weight (a constant force), the flex test and the scorer.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,no_hold,cocked,poke,hover,slam,pull
Final:  python demokit.py task.py OUT
"""

from __future__ import annotations

import collections
import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the popped-out thumb tip of this robot hand back into its joint, then let go."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself
SCORE_OVER = "end"
TOP = d.TABLE_TOP
MM = 1e-3
HOLD_STEPS = 20  # 1 s at 20 Hz

LOOKS = {"pla": m.Look(m.rgb(0.66, 0.70, 0.77), metallic=0.0, roughness=0.8), "tip": m.Look(m.rgb(0.95, 0.96, 0.98), metallic=0.0, roughness=0.8),
         "palm": m.Look(m.rgb(0.86, 0.88, 0.91), metallic=0.0, roughness=0.8), "joint": m.Look(m.rgb(0.20, 0.21, 0.24), metallic=0.0, roughness=0.7),
         "cradle": m.Look(m.rgb(0.30, 0.55, 0.78), metallic=0.0, roughness=0.7),
         "steel": m.Look(m.rgb(0.55, 0.57, 0.60), metallic=0.9, roughness=0.3),
         "scale": m.Look(m.rgb(0.95, 0.95, 0.95), metallic=0.0, roughness=0.6), "mark": m.Look(m.rgb(0.05, 0.05, 0.05), metallic=0.0, roughness=0.6)}

_cad = np.load(HERE / "orca_thumb.npz")
PP_V, DP_V = (_cad[k].astype(float) for k in ("ppv", "dpv"))
_hand = np.load(HERE / "orca_hand.npz")  # the palm, fingers, T-TP and T-AP in T-PP's STL frame (mm), and the CMC joint
PP_F, DP_F = (_cad[k].astype(int) for k in ("ppf", "dpf"))  # STL frames, mm


def _map(p):
    """STL (x, y, z) in mm -> world offset in m, at the seated, powered-off pose: STL y (the bearings' axis) -> -x, STL z (the phalanx's length) -> -y (the tip points -y), STL x -> up."""
    p = np.asarray(p, float)
    return np.stack([-p[..., 1], -p[..., 2], p[..., 0]], axis=-1) * MM


# -- layout (metres, station frame): T-PP centred at (38, 1) cm, its bearings' axis along x; the hand hangs below it from a stand behind
ZC = TOP + float(os.environ.get("OR_ZC", "0.145"))  # the STL x = 0 plane of the phalanges: 14.5 cm up, so the hanging fingers clear the bench
PP_CENTRE = np.array([0.38, 0.01])  # the box centre of T-PP
O_PP = np.array([PP_CENTRE[0], PP_CENTRE[1] + 0.0155, ZC])  # T-PP's STL origin (its box centre is 15.5 mm along STL z)
PIVOT = O_PP + _map(_hand["cmc_at"])  # the thumb's CMC joint, from ORCA's URDF: 31 mm behind T-PP's origin and 20 mm below it
CMC_AX = _map(_hand["cmc_axis"]) / MM
CMC_AX = CMC_AX / np.linalg.norm(CMC_AX)
CMC_LO, CMC_HI = (math.degrees(float(v)) for v in _hand["cmc_range"])  # the URDF's range about the rest angle
K_CMC = 0.5  # N m/rad: the powered-off CMC's slack tendons (estimate)
# the printed stand (PLA): a post clamped to the back of the palm, on a foot plate on the bench (x0, x1, y0, y1, z0, z1)
STAND = {"foot": (O_PP[0] - 0.05, O_PP[0] + 0.08, O_PP[1] + 0.05, O_PP[1] + 0.115, TOP, TOP + 0.008),
         "post": (O_PP[0] - 0.005, O_PP[0] + 0.035, O_PP[1] + 0.06, O_PP[1] + 0.09, TOP + 0.008, ZC - 0.025)}
JC_STL = np.array([1.0, 0.0, 32.0])  # the joint's centre in T-PP's STL frame (mm), read from the fork's pocket
C_SEAT = O_PP + _map(JC_STL)  # where T-DP's STL origin (the bearings' centre) sits when seated
BEAR_B = 0.0075  # the bearings' offset from the axis, one each side, for the seating rule (T-DP's base is 15 mm wide)
BEAR_Y, BEAR_W, BEAR_OD, BEAR_ID = 6.0, 3.0, 8.0, 4.0  # mm: the drawn bearings on T-DP's axle stubs (STL y 4.5 to 7.5 each side, 3.9 mm across), centred 6 mm each side
OUT_MM = 4.0  # dislocated: the bearings stand this far out of the grooves
COCK_PITCH, COCK_ROLL = 30.0, 10.0  # degrees

# thresholds (the card's scoring)
SEAT_R = 0.8 * MM  # a bearing is in its groove when within 0.8 mm of the groove's centre
SEAT_DEPTH = 0.0036  # axial: the last 0.4 mm of the 4 mm travel is the pop
DETENT_N = float(os.environ.get("OR_POP", "8"))  # the pop-in force (estimate; also the pull that pops a seated joint out again)
RIM_S = 0.0015  # a bearing off its groove meets the rim 1.5 mm into the travel
PITCH_OK, ROLL_OK, YAW_OK = 12.0, 3.0, 3.0  # the groove's mouth admits the bearings within this much of line
PHYS_HZ = 240  # the harness's physics rate (12 physics steps a control step at 20 Hz); the grooves and the slam timing are read at every physics step
SLAM_FROM = SEAT_DEPTH - 0.003  # the slam's timed segment: the tip's last 3 mm of travel before the pop (the bearings from 3.4 to 0.4 mm out)
KNOCK_N = float(os.environ.get("OR_KNOCK", "3"))  # a sideways push on the seated joint harder than this (the arm pushes about 4 N sideways before it gives; the pop force is 8 N along the thumb) unseats it again (estimate)
SLAM_STEPS = 7  # that 3 mm covered in 7 physics steps or fewer (29 ms at 240 Hz: a mean of 0.1 m/s or more) is a slam, which can crack a printed lip (estimate)
HOLD_CAP_MM = 5.0
FLEX_DEG, FLEX_RET = 60.0, 5.0
CLEAR_M = 0.05

# the tendons' tether (powered off: slack tendons, a loose leash; estimates) and the groove's lip
K_LAT = 100.0  # N/m, sideways and up/down, about the tip's rest (a loose leash: the 20 g tip sags 2 mm at rest, well past the 0.8 mm seat limit, so the hands must line it up)
K_TURN = 0.05  # N m/rad, roll and yaw about the rest
K_PITCH = 0.03  # N m/rad, about the bearings' axis
K_LIP = DETENT_N / SEAT_DEPTH  # N/m along the thumb: the lip resists the bearings' entry, rising to 8 N at the pop
TIP_KG = 0.020
JIT = tuple(float(v) for v in os.environ.get("OR_JIT", "0,0,0,0,0").split(","))  # jittered starts (robustness runs): pitch, roll, yaw (deg) and sideways, up (mm) added to the hanging rest
CUTAWAY = os.environ.get("OR_CUTAWAY") == "1"  # stills only: T-PP drawn hidden, so the cutaway still shows T-DP's bearings (physics unchanged)
CHAIN = ("ty", "tx", "tz", "roll", "yaw", "dp")  # slide along the thumb, sideways, up; roll about the thumb, yaw, then the joint's own hinge (pitch)


def _R(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]), "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def _RH(rad):
    """T-PP's turn on the CMC joint, by `rad` from the rest pose."""
    return axis_angle_matrix(CMC_AX, float(rad))


def _x_to(a):
    """The quaternion (w, x, y, z) that turns +x onto the unit vector a."""
    k = np.cross([1.0, 0.0, 0.0], a)
    s, c = float(np.linalg.norm(k)), float(a[0])
    if s < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    h = math.atan2(s, c) / 2
    k = k / s * math.sin(h)
    return (math.cos(h), float(k[0]), float(k[1]), float(k[2]))


def _tip_torque():
    """The tip's weight about the CMC joint (N m), at the seated pose: the CMC's tendons sit at rest under it."""
    com = (_map(DP_V) + C_SEAT).mean(axis=0)
    return float(np.cross(com - PIVOT, [0.0, 0.0, -TIP_KG * 9.81]) @ CMC_AX)


def _hand_parts():
    """ORCA's palm, fingers, T-TP and T-AP (from orca_hand.npz), world offsets from PIVOT: visual meshes and ORCA's convex collision pieces, split into the fixed hand (h) and the pieces on the CMC (m)."""
    off = O_PP - PIVOT
    vis, col = {"h": [], "m": []}, {"h": [], "m": []}
    for k in sorted({k[:-2] for k in _hand.files if k.endswith("_v")}):
        g, kind, name = k[0], k[1], k.split("__", 1)[1]
        v = _map(_hand[k + "_v"].astype(float)) + off
        if kind == "v":
            vis[g].append(("h_" + name, m.Mesh(v, _hand[k + "_f"].astype(int)), "joint" if "Skin" in name else "palm"))
        else:
            col[g].append(m.col_hull("c_" + name, _hull(v)))
    return vis, col


def _hull(V):
    """The convex hull of points as a Mesh with outward, counter-clockwise faces."""
    from scipy.spatial import ConvexHull

    V = np.asarray(V, float)
    h = ConvexHull(V)
    c = V[h.vertices].mean(axis=0)
    f = h.simplices.copy()
    n = np.cross(V[f[:, 1]] - V[f[:, 0]], V[f[:, 2]] - V[f[:, 0]])
    flip = np.einsum("ij,ij->i", n, V[f[:, 0]] - c) < 0
    f[flip] = f[flip][:, [0, 2, 1]]
    return m.Mesh(V, f)


def _bearings():
    """The two 4 x 8 x 3 mm bearings in T-DP's STL frame (mm): rings about STL y, on the axle stubs."""
    h = BEAR_W / 2
    ring = m.lathe([(BEAR_ID / 2, -h), (BEAR_OD / 2, -h), (BEAR_OD / 2, h), (BEAR_ID / 2, h), (BEAR_ID / 2, -h)], 40)  # about z
    out = []
    for sy in (-1.0, 1.0):
        V = np.stack([ring.v[:, 0], ring.v[:, 2] + sy * BEAR_Y, ring.v[:, 1]], axis=-1)  # lathe z -> STL y: a swap of two axes, so the faces turn round
        F = ring.f[:, [0, 2, 1]]
        out.append((V, F))
    return out


def _bx(x0, x1, y0, y1, z0, z1):
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _cb(name, x0, x1, y0, y1, z0, z1):
    return m.col_box(name, (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _thumb() -> m.Part:
    off = O_PP - PIVOT
    vis = m.Mesh(_map(PP_V) + off, PP_F)
    shank = PP_V[:, 2] < 22.5
    hull = _hull(_map(PP_V[shank]) + off)
    hv, hc = _hand_parts()
    pp = m.hinge("pp", "X", (0.0, 0.0, 0.0), CMC_LO, CMC_HI, [("pp", vis, "pla", CUTAWAY)] + hv["m"], [m.col_hull("pp_c", hull)] + hc["m"], mass=0.03, spring=K_CMC, damping=0.3, friction=0.002, effort=20.0,
                 armature=0.0005, rot=_x_to(CMC_AX))
    # the tip, seated at joint zero, on its tether: three slides and three turns about the bearings' centre; the last turn is the joint's own hinge
    cs = tuple(float(v) for v in (C_SEAT - PIVOT))
    dv = _map(DP_V) + (C_SEAT - PIVOT)
    com = tuple(float(v) for v in dv.mean(axis=0))
    bears = [(_map(V) + (C_SEAT - PIVOT), F) for V, F in _bearings()]
    bv = [(f"bearing_{i}", m.Mesh(V, F), "steel") for i, (V, F) in enumerate(bears)]
    tip = m.hinge("dp", "X", cs, -40.0, 100.0, [("dp", m.Mesh(dv, DP_F), "tip")] + bv, [m.col_hull("dp_c", _hull(np.vstack([dv] + [V for V, _ in bears])))], mass=TIP_KG, damping=0.0004, friction=0.0, effort=5.0, armature=2e-6,
                  parent="yaw", com=com, inertia=(5e-6, 1.5e-6, 5e-6))
    ghost = dict(mass=0.004, friction=0.0, effort=200.0, inertia=(2e-7, 2e-7, 2e-7))
    links = [pp,
             m.slider("ty", "Y", -0.006, 0.0, (), (), at=cs, damping=1.5, armature=0.002, parent="pp", **ghost),
             m.slider("tx", "X", -0.006, 0.006, (), (), at=cs, damping=1.5, armature=0.002, parent="ty", **ghost),
             m.slider("tz", "Z", -0.008, 0.008, (), (), at=cs, damping=1.5, armature=0.002, parent="tx", **ghost),
             m.hinge("roll", "Y", cs, -30.0, 30.0, (), (), damping=0.0004, armature=2e-6, parent="tz", **ghost),
             m.hinge("yaw", "Z", cs, -30.0, 30.0, (), (), damping=0.0004, armature=2e-6, parent="roll", **ghost),
             tip]
    start = [("ty", -OUT_MM * MM), ("roll", math.radians(COCK_ROLL)), ("dp", math.radians(COCK_PITCH))]

    # the powered-off hand: ORCA's palm and four fingers (fixed), hanging below the thumb, clamped by the back of its palm to a printed stand on the bench. Coordinates relative to PIVOT.
    def fx(x0, x1, y0, y1, z0, z1):
        return (x0 - PIVOT[0], x1 - PIVOT[0], y0 - PIVOT[1], y1 - PIVOT[1], z0 - PIVOT[2], z1 - PIVOT[2])

    parts, cols = list(hv["h"]), list(hc["h"])
    for n, b in STAND.items():
        cols.append(_cb(f"c_{n}", *fx(*b)))
        parts.append((n, _bx(*fx(*b)), "cradle"))
    # a printed 50 mm scale on the stand's foot, beside the hand
    sc = (STAND["foot"][0] + 0.005, STAND["foot"][0] + 0.055, STAND["foot"][2] + 0.004, STAND["foot"][2] + 0.006, STAND["foot"][5], STAND["foot"][5] + 0.0005)
    parts.append(("scale", _bx(*fx(*sc)), "scale"))
    return m.mechanism("thumb", tuple(PIVOT), links, LOOKS, frame_parts=parts, frame_colliders=cols, start=start, solver_iters=48)


OBJECTS = [_thumb()]

# -- the live pieces: tether gains, the groove's rim and lip, the closing joint, the tip's weight, the flex test ----------------------------------------


def _gain(env, link, k, target, effort=None):
    asset, j = m._joint(env, "thumb", link)
    m._TOUCHED.add(("thumb", link))
    if effort is not None:
        asset.write_joint_effort_limit_to_sim(m._tensor(asset, effort), joint_ids=[j])
    asset.write_joint_stiffness_to_sim(m._tensor(asset, k), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])


def _tether(env):
    """The slack tendons hold the tip where it hangs; the lip pushes back along the thumb."""
    jp, jr, jy, jx, jz = JIT
    _gain(env, "pp", K_CMC, -_tip_torque() / K_CMC)  # the CMC's tendons at rest under the tip's weight
    _gain(env, "ty", K_LIP, -OUT_MM * MM, effort=200.0)
    _gain(env, "tx", K_LAT, jx * MM)
    _gain(env, "tz", K_LAT, jz * MM)
    _gain(env, "roll", K_TURN, math.radians(COCK_ROLL + jr))
    _gain(env, "yaw", K_TURN, math.radians(jy))
    _gain(env, "dp", K_PITCH, math.radians(COCK_PITCH + jp))
    m.push(env, "thumb", (0.0, 0.0, -TIP_KG * 9.81), body="dp")  # the tip's weight, at its centre of mass


def _pose(j):
    """From the tether's joints: the bearings' places relative to their grooves, in T-PP's frame (x along the bearings' axis, y along the thumb toward the hand, z up)."""
    t = np.array([j["tx"], j["ty"], j["tz"]])
    Rr = _R("y", math.degrees(j["roll"])) @ _R("z", math.degrees(j["yaw"]))
    return [t + Rr @ np.array([sx * BEAR_B, 0.0, 0.0]) - np.array([sx * BEAR_B, 0.0, 0.0]) for sx in (-1.0, 1.0)]


def _hook_physics(env) -> None:
    """Call TRIAL.physics after every physics step (240 Hz), not only every control step: Isaac Lab updates the scene's buffers once per physics step, inside each control step."""
    scene = env.unwrapped.scene
    if getattr(scene, "_orca_physics", False):
        return
    update = scene.update

    def update_hook(*a, **k):
        out = update(*a, **k)
        TRIAL.physics(env)
        return out

    scene.update = update_hook
    scene._orca_physics = True


class Trial:
    def reset(self, env=None) -> None:
        self.n, self.held, self.latched, self.wall = 0, 0, False, False
        self.pp_max, self.slam, self.flex_t, self.flex_max, self.flex_ret, self.flex_done = 0.0, False, -1, -90.0, 90.0, False
        self.score, self.full, self.last, self.tier = 0.0, False, {}, 0.0
        self.armed, self.pull_n, self.popped_out, self.closed, self.latch_n = True, 0, 0, False, 0
        self.live, self.k, self.k_in, self.pop_steps, self.pop_log = False, 0, None, None, ""
        self.ramp, self.ramp_t = None, 0.0  # slam8: the slide's target runs along the thumb this far each physics step
        self.hist = collections.deque(maxlen=SLAM_STEPS + 1)
        if env is not None:
            _tether(env)

    def physics(self, env) -> None:
        """Every physics step: the grooves' rim and lip, the pop, and the slam's timing, so a fast stroke can't pass the rim, or seat, between two control steps."""
        if not self.live or self.latched:
            return
        asset = env.unwrapped.scene["thumb"]
        q = asset.data.joint_pos[0].tolist()
        names = asset.joint_names
        J = {c: float(q[names.index(f"{c}_joint")]) for c in CHAIN}
        self.k += 1
        pitch, roll, yaw = (math.degrees(J[c]) for c in ("dp", "roll", "yaw"))
        b = _pose(J)
        lat = [float(math.hypot(p[0], p[2])) for p in b]
        dep = [OUT_MM * MM + float(p[1]) for p in b]  # each bearing's travel into its groove
        s = OUT_MM * MM + J["ty"]
        aligned = bool(max(lat) <= SEAT_R and abs(pitch) <= PITCH_OK and abs(roll) <= ROLL_OK and abs(yaw) <= YAW_OK)
        if not self.armed:
            # coming out: the bearings ride back over the lip; once out past the rim, the joint can be pressed in again
            if s <= RIM_S:
                self.armed = True
            return
        # the rim: a bearing off its groove stops on the rim (a stiff drive where it stands); in line, the lip's spring is all that resists
        if not aligned and s > RIM_S and not self.wall:
            _gain(env, "ty", 50000.0, J["ty"], effort=2000.0)
            self.wall = True
        elif aligned and self.wall:
            _gain(env, "ty", K_LIP, -OUT_MM * MM, effort=200.0)
            self.wall = False
        # the slam's clock: the physics step at which the tip last entered the final 3 mm before the pop
        if s < SLAM_FROM:
            self.k_in = None
        elif self.k_in is None:
            self.k_in = self.k
        if self.ramp is not None and not self.latched:
            self.ramp_t += self.ramp
            _gain(env, "ty", RAMPK, self.ramp_t, effort=2000.0)
        self.hist.append(s)  # the last SLAM_STEPS + 1 physics positions (the first is SLAM_STEPS steps back): a slam is the final 3 mm covered inside the last 7 steps, whatever came before (a pause earlier does not reset it)
        # both bearings in: the joint closes (the pop), and is a hinge again
        if aligned and min(dep) >= SEAT_DEPTH:
            self.pop_steps = self.k - self.k_in
            if len(self.hist) == self.hist.maxlen and self.hist[0] < SLAM_FROM:
                self.slam = True
            self.latched, self.closed, self.latch_n, self.ramp = True, False, 0, None
            _gain(env, "ty", 50000.0, 0.0, effort=2000.0)
            for c in ("tx", "tz"):
                _gain(env, c, 50000.0, 0.0, effort=2000.0)
            for c in ("roll", "yaw"):
                _gain(env, c, 200.0, 0.0, effort=200.0)
            _gain(env, "dp", K_PITCH, 0.0)  # the slack tendons, now about the closed joint
            self.pop_log = (f"[orca] POP at physics step {self.k} (control step {self.n}): lat {np.round(np.array(lat) * 1000, 2)} mm, dep {np.round(np.array(dep) * 1000, 2)} mm, "
                            f"pitch {pitch:.1f} roll {roll:.1f} yaw {yaw:.1f}; last 3 mm in {self.pop_steps} physics steps ({self.pop_steps * 1000 / PHYS_HZ:.0f} ms, "
                            f"mean {0.003 * PHYS_HZ / max(self.pop_steps, 1):.3f} m/s){': SLAM' if self.slam else ''}")

    def step(self, env) -> None:
        if self.n == 0:
            _tether(env)
            _hook_physics(env)
            self.live = True
        if self.pop_log:
            print(self.pop_log, flush=True)
            self.pop_log = ""
        th = m.state(env, "thumb")
        J = {k: th.joint(k) for k in ("pp",) + CHAIN}
        ang = J["pp"]
        Rp = _RH(ang)
        C = PIVOT + Rp @ (C_SEAT - PIVOT)
        pitch, roll, yaw = (math.degrees(J[k]) for k in ("dp", "roll", "yaw"))
        b = _pose(J)
        lat = [float(math.hypot(p[0], p[2])) for p in b]
        dep = [OUT_MM * MM + float(p[1]) for p in b]  # each bearing's travel into its groove
        s = OUT_MM * MM + J["ty"]
        if self.latched:
            # a seated joint pops out again when the tip is pulled out along the thumb harder than the pop force (the closed joint's stiff slide reads the pull), as the real joint is built to
            self.latch_n += 1
            self.closed = self.closed or -J["ty"] < 0.00005 or self.latch_n >= 10  # the closing drive has drawn the bearings home (they latch up to 0.4 mm short of it), or 0.5 s has passed
            side = abs(J["tx"]) * 50000.0  # a sideways knock: the stiff side slides read it as a force
            self.pull_n = self.pull_n + 1 if self.closed and (-J["ty"] * 50000.0 > DETENT_N or side > KNOCK_N) else 0
            if self.pull_n >= 2:
                self.latched, self.armed, self.wall, self.pull_n = False, False, False, 0
                self.popped_out += 1
                self.flex_t, self.flex_done, self.flex_max, self.flex_ret = -1, False, -90.0, 90.0
                _tether(env)
                print(f"[orca] POP OUT at step {self.n}: pulled {-J['ty'] * 50000.0:.1f} N, sideways {side:.1f} N", flush=True)
        # the hold cap: how far T-PP's middle has moved
        mid0 = np.array([PP_CENTRE[0], PP_CENTRE[1], ZC])
        mid = PIVOT + Rp @ (mid0 - PIVOT)
        self.pp_max = max(self.pp_max, float(np.linalg.norm(mid - mid0)) * 1000)
        # clear hands
        u = Rp @ np.array([0.0, -1.0, 0.0])
        tips = m.fingertips(env)
        seg = [PIVOT, C, C + u * 0.040]
        dmin = 9.0
        for t in tips:
            for a, bb in zip(seg[:-1], seg[1:]):
                ab = bb - a
                q = a + ab * float(np.clip(((t - a) @ ab) / (ab @ ab), 0, 1))
                dmin = min(dmin, float(np.linalg.norm(t - q)))
        clear = bool(dmin >= CLEAR_M)
        self.held = self.held + 1 if clear else 0
        # the flex test: once the hands have been clear for 1 s and the joint has popped, the powered hand's flexor turns the tip 70 degrees and back
        if self.latched and self.held >= HOLD_STEPS and self.flex_t < 0:
            self.flex_t = 0
        if self.flex_t >= 0 and not self.flex_done:
            T = self.flex_t / 20.0
            tgt = 70.0 * min(T / 1.5, 1.0) if T < 2.0 else 70.0 * max(0.0, 1.0 - (T - 2.0) / 1.5)
            _gain(env, "dp", 0.2, math.radians(tgt))
            self.flex_max = max(self.flex_max, pitch)
            if T >= 4.5:
                self.flex_done = True
                self.flex_ret = abs(pitch)
            self.flex_t += 1
        # tiers
        tier = 0.0
        if self.latched:
            tier = 0.5  # both bearings seated (the pop): the closed joint is a free hinge on slack tendons, so the released tip's droop doesn't count (the real trial reads this with the feeler gauge alone)
            if self.flex_done and self.flex_max >= FLEX_DEG and self.flex_ret <= FLEX_RET and clear:
                tier = 1.0
            if self.slam:
                tier = min(tier, 0.5)
        elif abs(pitch) <= 10.0 and abs(roll) <= 10.0 and s >= OUT_MM * MM - 0.0005 - 0.0035 and max(lat) <= 0.0015:
            tier = 0.25
        if self.pp_max > HOLD_CAP_MM:
            tier = min(tier, 0.25)
        self.tier = self.score = tier
        self.full = bool(tier == 1.0)
        self.n += 1
        self.last = dict(s_mm=round(s * 1000, 2), pitch=round(pitch, 1), roll=round(roll, 1), yaw=round(yaw, 1), tx=round(J["tx"] * 1000, 2), tz=round(J["tz"] * 1000, 2),
                         lat=[round(x * 1000, 2) for x in lat], dep=[round(x * 1000, 2) for x in dep], wall=self.wall, latched=self.latched, pp_mm=round(self.pp_max, 2),
                         clear_mm=round(dmin * 1000), held=self.held, flex=(round(self.flex_max, 1), round(self.flex_ret, 1), self.flex_done), slam=self.slam, pop_steps=self.pop_steps, k=self.k, k_in=self.k_in, out=self.popped_out, tier=tier)
        if self.n % 20 == 0 and os.environ.get("OR_TRACE"):
            print(f"[orca] step {self.n}: {self.last}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Both bearings seated (each within 0.8 mm of its groove's line and fully in), no slam, T-PP held (moved at most 5 mm), the hands 5 cm clear for 1 s, and the
    powered thumb's flex test passes (the tip turns at least 60 degrees about the joint and returns within 5 degrees)."""
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------

GRIP_N = float(os.environ.get("OR_GRIP", "30"))
PRESS_N = float(os.environ.get("OR_PRESS", "12"))
LG = tuple(float(v) for v in os.environ.get("OR_LG", "30,-90").split(","))  # the left hand: tilt from straight down, direction (jaws close along x)
PP_Z = float(os.environ.get("OR_PPZ", "15"))  # grip T-PP at this STL z: 13 mm wide there
GRIP_AT = float(os.environ.get("OR_GAT", "24")) * MM  # the right hand's TCP this far out along the tip from the bearings (STL z)
TCP_DZ = float(os.environ.get("OR_TDZ", "0")) * MM
UP = float(os.environ.get("OR_UP", "0.10"))
SQUARE = os.environ.get("OR_SQ", "1") == "1"
PRESS_S = float(os.environ.get("OR_PS", "4"))
PRESS_GAIN = float(os.environ.get("OR_PG", "0.2"))
PRESS_PAST = float(os.environ.get("OR_PP", "3")) * MM
PRESS_LEAD = float(os.environ.get("OR_PL", "30")) * MM
PHI = float(os.environ.get("OR_PHI", "90"))
TILT = float(os.environ.get("OR_TILT", "90"))  # the right hand's fingers 20 degrees down: the tip's top and bottom taper toward its end, so a pinch across them holds it about 20 degrees off the pads
ROLL = float(os.environ.get("OR_ROLL", "90"))  # the right hand's fingers point this way from +x
OPEN_HALF = float(os.environ.get("OR_OPEN", "40")) * MM  # the right jaws' half-opening on the way in
LIFT_FROM = float(os.environ.get("OR_LIFT", "40")) * MM  # the lower pad starts this far below the bearings' line, under the hanging end (22 mm down)
STEPPED = os.environ.get("OR_STEP", "1") == "1"  # press in small steps, squaring the tip between them
STEP_MM = float(os.environ.get("OR_SMM", "0.3"))  # each step's travel
STEP_S = float(os.environ.get("OR_SS", "14"))  # the stepped press's time limit
DRIFT = float(os.environ.get("OR_DRIFT", "1.0"))  # degrees of yaw or roll past which the press backs off 0.5 mm
ROT_G = float(os.environ.get("OR_RG", "0.3"))  # the share of the tip's angle the hand turns out each control step
ROT_MAX = float(os.environ.get("OR_RMAX", "1.0"))  # degrees a control step, at most
AX_G = float(os.environ.get("OR_AG", "0.3"))
LAT_G = float(os.environ.get("OR_LG2", "0.5"))
SLAM_LEAD = float(os.environ.get("OR_SLAM", "8")) * MM  # slam: the hand's target runs this far along the thumb each control step (0.16 m/s)
RAMPK = float(os.environ.get("OR_RAMPK", "12000"))  # slam8: the strike's spring on the slide, N/m
SLAMV8 = float(os.environ.get("OR_SLAMV8", "0.375")) * MM  # slam8: the strike's travel each physics step: 3 mm in 8 steps, not 7 (not a slam)
SLAMK = float(os.environ.get("OR_SLAMK", "50000"))  # slam_held: the strike, a spring on the tip's slide toward the seat (N/m, 3.6 mm out at 50000 is 180 N)
SLAM_STEER = os.environ.get("OR_SLAMV", "steer") == "steer"  # slam: the stroke keeps the stepped press's steering (pitch, roll, yaw and line), without its steps
SLAM_CAP = float(os.environ.get("OR_SLAMC", "60")) * MM  # slam: the hand's target leads the hand by up to this much along the thumb (about twice the press's push)
SLAM_SHORT = float(os.environ.get("OR_SLAMS", "0.3")) * MM  # slam: the stepped press stops this far short of the timed 3 mm


def vgrip(phi=90.0, tilt=90.0, roll=-90.0):
    """Fingers pointing along `phi` from +x, `tilt` from straight down (90: level), turned `roll` about their own axis so that the jaws close up and down."""
    return d._as_grip(d.tilted(tilt, phi).rot @ axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(roll)))


def demo_script(ctx):
    """Modes: full. none: the arms wait. no_hold: only the right hand, which grips the tip square and presses (T-PP unheld swings). cocked: full, but the tip is pressed as gripped, not squared. poke: the left hand holds, the right hand pushes the hanging tip's end with closed fingers, without gripping. hover: full up to the line-up, then the tip is held square at its seat's mouth and let go, with no press. slam: full, but the stepped press stops just short of the last 3 mm, squared, and the tip is struck home from there in one stroke (the hand's target runs along the thumb at 0.16 m/s, kept on the grooves' line). yawed: full, but the squared tip is turned 4.5 degrees in yaw and pressed as gripped. slam_held: the left hand holds T-PP; the right squares the tip to the last 3 mm, lets go, and a hard push strikes it home. knock: full, then the gripped tip is pushed 15 mm sideways and let go. yaw_hold: full, but the tip is held 2.8 degrees off in yaw and pressed to 3.45 mm in, then let go. regrip_seated: full, then after the pop the jaws open and close again before the release. release_raw: full without the ease-off before the jaws open. slam8: as slam_held with a slower strike, the last 3 mm in 8 physics steps (not a slam). nudge_up: full, then the gripped tip is pushed 15 mm up and let go. hover_hold: the tip is held square at its seat's mouth, T-PP held, with no press, to the end. slam_dwell: slam_held with a 1 s pause at the start of the last 3 mm. pull: full until the joint pops in, then the right hand pulls the gripped tip 15 mm back out along the thumb before letting go."""
    mode = ctx.mode
    if mode not in ("full", "none", "no_hold", "cocked", "poke", "hover", "slam", "pull", "yawed", "slam_held", "knock", "yaw_hold", "regrip_seated", "release_raw", "slam_dwell", "slam8", "nudge_up", "hover_hold"):
        raise ValueError(f"unknown mode {mode!r}")
    YAWD = 2.8 if mode == "yaw_hold" else 4.5  # yawed turns the tip this far in yaw, yaw_hold 2.8
    L, R = ctx.left, ctx.right
    if mode == "none":
        yield from L.wait(3.0)
        return
    gl = d.tilted(*LG)
    gr = vgrip(PHI, TILT, ROLL)
    pp_pt = O_PP + _map(np.array([3.4, 0.0, PP_Z]))
    if mode != "no_hold":
        yield from L.move(pp_pt + [0, 0, 0.06], grip=gl, finger=-0.020, label="left over T-PP")
        yield from L.servo(pp_pt + [0, 0, 0.06], timeout=1.0, label="left over")
        yield from L.move(pp_pt, t=1.2, label="left down")
        yield from L.servo(pp_pt, timeout=1.0, label="left down")
        yield from L.close(0.013, 30.0, label="hold T-PP")

    def seat():  # where the bearings' centre goes, from T-PP's own joint angle
        a = ctx.state("thumb").joint("pp")
        return PIVOT + _RH(a) @ (C_SEAT - PIVOT)

    rest = seat() + np.array([0.0, -OUT_MM * MM, 0.0])
    if mode == "poke":
        end = rest + np.array([0.0, -0.030 * math.cos(math.radians(COCK_PITCH)), -0.030 * math.sin(math.radians(COCK_PITCH))])
        yield from R.move(end + [0, -0.08, 0.0], grip=vgrip(90.0, 90.0), finger=0.0, label="right behind the tip's end")
        yield from R.move(end + [0, -0.03, 0.0], t=1.0, label="right to the tip's end")
        res = yield from R.press((0.0, 1.0, 0.0), PRESS_N, 0.030, label="poke")
        yield from R.move(R.at() + [0, -0.08, 0.02], t=1.0, label="right back")
    else:
        g = rest + np.array([0.0, -GRIP_AT, TCP_DZ])
        # the lower finger goes in under the hanging tip, then lifts it into line as the jaws close on its top and bottom: the lower finger rises as the upper comes down
        half0 = OPEN_HALF
        low = g + np.array([0.0, 0.0, half0 - LIFT_FROM])  # the lower pad LIFT_FROM below the bearings' line
        yield from R.move(low + [0, -0.03, 0.0], grip=gr, finger=-half0, label="right behind the tip")
        yield from R.servo(low + [0, -0.03, 0.0], timeout=1.0, label="right behind")
        yield from R.move(low, t=1.2, label="right in, under the tip")
        yield from R.servo(low, timeout=1.0, label="right in")
        f_close = float(np.clip(-(0.016 / 2 - GRIP_N / d.GRIPPER["stiffness"]), d.FINGER_OPEN, d.FINGER_CLOSED))
        yield from R.move(g, finger=f_close, t=2.0, label="lift into line and close")
        yield from R.servo(g, timeout=0.8, label="grip T-DP")
        yield from R.wait(0.3)
        print(f"[orca] gripped: {TRIAL.last}", flush=True)
        if os.environ.get("OR_TRACE"):
            ft = m.fingertips(ctx.emb._env)[12:] - seat()
            print(f"[orca] right fingers (mm from the seat): {np.round(ft * 1000, 1).tolist()}  tcp {np.round((R.tcp() - seat()) * 1000, 1)} cmd {np.round((g - seat()) * 1000, 1)}", flush=True)
        # square the tip by feedback: its pitch and roll from the tether's joints, turned out by swinging the hand about the bearings' centre
        for it in range(4 if SQUARE and mode != "cocked" else 0):
            j = ctx.state("thumb")
            Rp = _RH(j.joint("pp"))
            pc = seat() + Rp @ np.array([j.joint("tx"), j.joint("ty"), j.joint("tz")])
            p_deg, r_deg, y_deg = (math.degrees(j.joint(k)) for k in ("dp", "roll", "yaw"))
            if abs(p_deg) < 1.5 and abs(r_deg) < 1.0 and abs(y_deg) < 1.5:
                break
            if abs(p_deg) >= 1.5:
                yield from R.arc(pc, Rp @ np.array([1.0, 0.0, 0.0]), -p_deg, t=max(0.6, abs(p_deg) / 30.0), label="square pitch")
            if abs(r_deg) >= 1.0:
                yield from R.arc(pc, Rp @ np.array([0.0, 1.0, 0.0]), -r_deg, t=0.6, label="square roll")
            if abs(y_deg) >= 1.5:
                yield from R.arc(pc, Rp @ np.array([0.0, 0.0, 1.0]), -y_deg, t=max(0.6, abs(y_deg) / 30.0), label="square yaw")
            yield from R.wait(0.2)
            print(f"[orca] squared {it}: {TRIAL.last}", flush=True)
        # line the bearings up with the grooves: the tether's joints say where the tip's bearings are against the seat; shift the hand by the difference
        for it in range(3):
            j = ctx.state("thumb")
            off = np.array([j.joint("tx"), 0.0, j.joint("tz")])
            Rp = _RH(j.joint("pp"))
            if np.linalg.norm(off) < 0.0002:
                break
            yield from R.move(R.at() - Rp @ off, t=0.5, label="line up")
            yield from R.wait(0.2)
        print(f"[orca] lined up: {TRIAL.last}", flush=True)
        if mode in ("yawed", "yaw_hold"):  # turn the squared tip 4.5 (yaw_hold: 2.8) degrees about the vertical, then press as gripped (nothing turned out)
            j = ctx.state("thumb")
            Rp = _RH(j.joint("pp"))
            pc = seat() + Rp @ np.array([j.joint("tx"), j.joint("ty"), j.joint("tz")])
            yield from R.arc(pc, Rp @ np.array([0.0, 0.0, 1.0]), YAWD - math.degrees(j.joint("yaw")), t=0.8, label="yaw the tip")
            yield from R.wait(0.3)
            print(f"[orca] yawed: {TRIAL.last}", flush=True)
        # press the tip along the thumb until both bearings are in, steering by where its bearings are: the lip's 8 N is met by the arm's own stiffness as the command leads
        def bearings():
            j = ctx.state("thumb")
            return seat() + _RH(j.joint("pp")) @ np.array([j.joint("tx"), j.joint("ty"), j.joint("tz")])

        def stepped_press(until=None, fast=False, stop_s=None):
            """Press in small steps (STEP_MM of the bearings' travel), reading the tip's pitch, roll and yaw every control step and turning the hand about the bearings' centre to take them out;
            the bearings are also kept on their grooves' line by shifting the hand sideways. A step is taken only once the tip is within 1 degree; past DRIFT degrees the press backs off 0.5 mm."""
            c, rot = d.ik.tcp_pose(R.q, R.base)
            s_goal = None
            out = {"popped": False, "steps": 0, "backoffs": 0, "worst_yaw": 0.0, "worst_roll": 0.0}
            backing = 0
            rg = ROT_G if mode != "cocked" else 0.0
            yoff = YAWD if mode in ("yawed", "yaw_hold") else 0.0  # yawed: the tip is held 4.5 degrees off in yaw, and steered back to that if it drifts  # cocked: pressed as gripped, nothing turned out
            for k in range(round(STEP_S * 20)):
                if TRIAL.latched:
                    out["popped"] = True
                    break
                j = ctx.state("thumb")
                J = {q: j.joint(q) for q in ("pp",) + CHAIN}
                Rp = _RH(J["pp"])
                s = OUT_MM * MM + J["ty"]
                if stop_s is not None and s >= stop_s:
                    break  # yaw_hold: pressed to this far in, then held
                if until is not None and s >= until and max(abs(math.degrees(J[q])) for q in ("roll", "yaw")) <= 1.0 and abs(math.degrees(J["dp"])) <= 1.5 and math.hypot(J["tx"], J["tz"]) <= 0.0004:
                    break  # slam: squared at the start of the timed 3 mm, for the stroke
                if s_goal is None:
                    s_goal = OUT_MM * MM + PRESS_PAST if fast else s
                p_deg, r_deg, y_deg = (math.degrees(J[q]) for q in ("dp", "roll", "yaw"))
                if s > RIM_S - 0.0003:
                    out["worst_yaw"] = max(out["worst_yaw"], abs(y_deg))
                    out["worst_roll"] = max(out["worst_roll"], abs(r_deg))
                pc = seat() + Rp @ np.array([J["tx"], J["ty"], J["tz"]])
                # back off when the tip drifts
                if rg > 0 and backing == 0 and not fast and (abs(y_deg - yoff) > DRIFT or abs(r_deg) > DRIFT):
                    s_goal = max(0.0, s - 0.0005)
                    backing = 10
                    out["backoffs"] += 1
                # turn the hand about the bearings' centre: yaw about the vertical, roll about the thumb's axis, pitch about the bearings' axis
                cl = math.radians(ROT_MAX)
                Rc = (axis_angle_matrix(Rp @ np.array([0.0, 0.0, 1.0]), float(np.clip(-rg * math.radians(y_deg - yoff), -cl, cl)))
                      @ axis_angle_matrix(Rp @ np.array([0.0, 1.0, 0.0]), float(np.clip(-rg * math.radians(r_deg), -cl, cl)))
                      @ axis_angle_matrix(Rp @ np.array([1.0, 0.0, 0.0]), float(np.clip(-rg * math.radians(p_deg), -cl, cl))))
                t_now = R.tcp()
                c = c + (pc + Rc @ (t_now - pc) - t_now)
                rot = Rc @ rot
                # along the thumb to the step's goal; sideways and up onto the grooves' line
                u = Rp @ np.array([0.0, 1.0, 0.0])
                lat = -(Rp @ np.array([J["tx"], 0.0, J["tz"]]))
                c = c + (np.clip(SLAM_LEAD * u + LAT_G * lat, -SLAM_LEAD - 0.002, SLAM_LEAD + 0.002) if fast else np.clip(AX_G * (s_goal - s) * u + LAT_G * lat, -0.002, 0.002))
                lead = float((c - t_now) @ u)
                cap = SLAM_CAP if fast else PRESS_LEAD
                if lead > cap:
                    c = c - (lead - cap) * u
                square = rg == 0 or (abs(y_deg - yoff) <= 1.0 and abs(r_deg) <= 1.0 and abs(p_deg) <= 1.5)
                if fast:
                    pass
                elif backing > 0:
                    backing -= 1
                elif square and s >= s_goal - 0.00015:
                    s_goal = min(OUT_MM * MM + PRESS_PAST, s_goal + STEP_MM * MM)
                R._solve(c, rot)
                out["steps"] = k + 1
                if os.environ.get("OR_TRACE") and k % 10 == 0:
                    print(f"[orca] press {k}: s {s * 1000:.2f} goal {s_goal * 1000:.2f} p {p_deg:.2f} r {r_deg:.2f} y {y_deg:.2f} lat {np.round(np.array(TRIAL.last.get('lat', [0, 0])), 2)} "
                          f"wall {TRIAL.wall} lead {lead * 1000:.1f}", flush=True)
                yield
            R.grip = d._as_grip(rot)
            return out

        c = d.ik.tcp_pose(R.q, R.base)[0]
        res = {"popped": False, "steps": 0}
        stepped = STEPPED and mode not in ("hover", "hover_hold", "slam", "slam8", "slam_held", "slam_dwell")
        gain, lead_cap = PRESS_GAIN, PRESS_LEAD
        if mode in ("hover", "hover_hold"):
            if mode == "hover_hold":
                res = yield from stepped_press(until=0.0008)  # line up and square, to 0.8 mm into the mouth, and stop
            yield from R.wait(2.0)
            if mode == "hover_hold":  # the tip stays gripped square at the mouth, T-PP held, no press, to the end
                yield from R.wait(8.0)
                print(f"[orca] end {TRIAL.last}", flush=True)
                return
        elif mode == "yaw_hold":
            res = yield from stepped_press(stop_s=0.00345)
            yield from R.wait(1.0)
        elif stepped:
            res = yield from stepped_press()
        elif mode in ("slam_held", "slam_dwell", "slam8"):
            # full to the last 3 mm, squared; then a hard push on the tip along the thumb (a 50000 N/m spring on its slide, OR_SLAMK) strikes it home, the grip still closed and T-PP held
            res = yield from stepped_press(until=SLAM_FROM - SLAM_SHORT)
            print(f"[orca] {mode} from: {TRIAL.last}", flush=True)
            if mode == "slam_dwell":
                yield from R.wait(1.0)  # a pause at the start of the last 3 mm, then the strike
            if mode == "slam8":  # a strike that takes 8 physics steps for the last 3 mm: the slide's target runs home at 0.375 mm a physics step (0.09 m/s)
                TRIAL.ramp_t, TRIAL.ramp = float(ctx.state("thumb").joint("ty")), SLAMV8
                for c in ("tx", "tz"):  # the strike is a pure push along the thumb: the slow stroke keeps the tip on the grooves' line
                    _gain(ctx.emb._env, c, 20000.0, 0.0, effort=2000.0)
            else:
                _gain(ctx.emb._env, "ty", SLAMK, 0.0, effort=2000.0)  # a strong spring on the tip's slide toward the seat: a pure push along the thumb, with no torque on the tip
            for k in range(20):
                if TRIAL.latched:
                    res["popped"] = True
                    break
                yield
            if not TRIAL.latched:
                _gain(ctx.emb._env, "ty", K_LIP, -OUT_MM * MM, effort=200.0)
            print(f"[orca] struck: {TRIAL.last}", flush=True)
        elif mode == "slam":
            # pressed in steps as full, squared and in line, to just short of the last 3 mm; then struck home in one stroke: the hand's target runs SLAM_LEAD along the thumb each control step
            res = yield from stepped_press(until=SLAM_FROM - SLAM_SHORT)
            print(f"[orca] slam from: {TRIAL.last}", flush=True)
            if SLAM_STEER:
                res = yield from stepped_press(fast=True)
            c, rot = d.ik.tcp_pose(R.q, R.base)
            for k in range(0 if SLAM_STEER else round(1.0 * 20)):
                if TRIAL.latched:
                    res["popped"] = True
                    break
                j = ctx.state("thumb")
                Rp = _RH(j.joint("pp"))
                u = Rp @ np.array([0.0, 1.0, 0.0])
                lat = -(Rp @ np.array([j.joint("tx"), 0.0, j.joint("tz")]))
                c = c + SLAM_LEAD * u + np.clip(LAT_G * lat, -0.002, 0.002)  # straight along the thumb, SLAM_LEAD a control step, kept on the grooves' line
                lead = float((c - R.tcp()) @ u)
                if lead > 0.020:
                    c = c - (lead - 0.020) * u
                R._solve(c, rot)
                yield
            R._solve(R.tcp(), rot)
        for k in range(0 if (stepped or mode in ("hover", "slam", "slam8", "slam_held", "slam_dwell", "yaw_hold")) else round(PRESS_S * 20)):
            if TRIAL.latched:
                res["popped"] = True
                break
            Rp = _RH(ctx.state("thumb").joint("pp"))
            err = seat() + Rp @ np.array([0.0, PRESS_PAST, 0.0]) - bearings()  # aim a little past the seat, so the lead (the push) keeps growing until the joint pops
            c = c + np.clip(gain * err, -0.002, 0.002)
            u = Rp @ np.array([0.0, 1.0, 0.0])
            lead = float((c - R.tcp()) @ u)
            if lead > lead_cap:  # about 15 N: no harder
                c = c - (lead - lead_cap) * u
            R._solve(c, R.grip.rot)
            res["steps"] = k + 1
            yield
        print(f"[orca] pressed: {res} {TRIAL.last}", flush=True)
        yield from R.wait(0.3)
        if mode == "knock":  # seated, then pushed 15 mm sideways in the grip
            Rp = _RH(ctx.state("thumb").joint("pp"))
            yield from R.move(R.tcp() + 0.015 * (Rp @ np.array([1.0, 0.0, 0.0])), t=0.6, label="knock the tip sideways")
            yield from R.wait(0.3)
            print(f"[orca] knocked: {TRIAL.last}", flush=True)
        if mode == "nudge_up":  # seated, then pushed 15 mm up in the grip
            yield from R.move(R.tcp() + np.array([0.0, 0.0, 0.015]), t=0.6, label="nudge the tip up")
            yield from R.wait(0.3)
            print(f"[orca] nudged: {TRIAL.last}", flush=True)
        if mode == "pull":  # seated, then pulled straight back out in the grip
            u = _RH(ctx.state("thumb").joint("pp")) @ np.array([0.0, 1.0, 0.0])
            yield from R.move(R.tcp() - 0.015 * u, t=1.0, label="pull the tip back out")
            yield from R.wait(0.3)
            print(f"[orca] pulled: {TRIAL.last}", flush=True)
        if mode == "regrip_seated":  # seated, then the jaws open and close on the tip again before the release
            yield from R.move(R.tcp(), t=0.4, label="ease off")
            yield from R.open(label="open")
            yield from R.wait(0.3)
            yield from R.move(R.tcp(), finger=f_close, t=0.5, label="close again")
            yield from R.wait(0.3)
        if mode != "release_raw":  # release_raw skips the ease-off
            yield from R.move(R.tcp(), t=0.4, label="ease off")  # take back the lead the press built up, so the opened fingers don't lunge
        yield from R.open(label="release tip")
        yield from R.move(R.at() + [0, -0.10, 0.06], t=1.2, label="right back")
    if mode != "no_hold":
        yield from L.open(label="release T-PP")
        yield from L.move(L.at() + [0, 0, UP], t=1.0, label="left up")
    yield from R.wait(7.0)
    print(f"[orca] end {TRIAL.last}", flush=True)
