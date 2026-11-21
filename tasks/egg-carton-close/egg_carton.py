"""A pulp dozen-egg carton is closed after eggs were taken out. It sits on the counter open: the lid stands back past
upright, sprung open by the pulp hinge, and the front locking flap hangs outward. The lid only stays shut when the
flap is tucked up inside its front skirt: a bead along the flap's top then sits above a bead inside the skirt, and the
flap's own spring keeps it there. Shut with the flap left outside, the lid springs back open. Let go of a tucked flap
and it creeps back out within about a second and a half (a soft pulp hinge), so the flap has to be held in until the
lid is nearly down, then the lid brought the last few centimetres down over it. The eggs are wooden stand-ins, and
neither they nor the carton may be damaged: pressing the lid or the flap onto the eggs, pressing a latched carton
down onto them, touching an egg, dropping or upending the carton or splitting it at any time scores 0 for the whole
trial. Ordinary latch pressure and lifting or nudging the carton gently are allowed.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import gripkit  # noqa: E402
from hingekit import Hinge, hinged  # noqa: E402
from partkit import env, on_reset, on_step  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Close the egg carton."

PULP, EGG = (0.70, 0.66, 0.58), (0.86, 0.74, 0.56)
CX = float(os.environ.get("EGG_CX", 0.25))   # carton centre, m in front of the arm bases
HB = 0.045            # base (cup tray) height
BX = 0.05             # base half depth (x): 10 cm deep, 30 cm long (y)
BY = 0.15
EGG_R = 0.022         # wooden craft eggs, 4.4 cm across
EGG_TOP = HB + 0.02   # top of an egg above the table (the part showing above the cups)
HALF_MASS = float(os.environ.get("EGG_HALF_MASS", 0.25))   # kg: half the carton (40 g) plus six 35 g wooden eggs
OPEN = float(os.environ.get("EGG_OPEN", 100.0))      # lid drawn open this far (deg); 0 on its joint
REST = float(os.environ.get("EGG_REST", 60.0))       # flap drawn hanging this far outward (deg)
IN_MAX = 18.0         # flap stops against the front row of eggs this far inward of upright (deg)
LID_PAST = float(os.environ.get("EGG_LID_PAST", 12.0))  # lid stops on the egg tops this far past shut (deg)
CRUSH_LID = 6.0       # lid more than this past shut: pressed down onto the eggs
CRUSH_FLAP = 16.0     # flap more than this inside upright: pressed onto the front eggs
TILT, SPLIT = 20.0, 0.003   # carton tipped (deg: loose eggs roll out) or its two halves pulled apart (m: torn)
DROP = 1.0            # m/s downward: the carton dropped (a fall of about 5 cm); lifting it gently is allowed
TOUCH = 0.005         # a finger this close to an egg touches it
PRESS = 5.0           # N through the lid's and the flap's hinges together, once latched: pressed onto the eggs
PRESS_HOLD = 5        # control steps (0.25 s) the press must last: the bead snapping in is a brief bump, not a press
LID_RATE = float(os.environ.get("EGG_LID_RATE", 0.02))     # N m/rad: about 0.35 N at the lid's front when shut
FLAP_RATE = float(os.environ.get("EGG_FLAP_RATE", 0.0086))  # N m/rad: about 0.3 N at the flap's top when upright
FLAP_DAMP = float(os.environ.get("EGG_FLAP_DAMP", 0.10))    # N m s/rad: a tucked flap creeps out in ~1.5 s

LP = (BX, 0.0, HB)                 # lid hinge line (back top edge of the base), part frame
FP = (-BX - 0.001, 0.0, HB)        # flap hinge line (front top edge of the base)


def _rot(box, pivot, deg):
    """A box drawn in the closed position, turned deg about the y axis through pivot (USD/joint sense)."""
    (cx, cy, cz), size, color = box[:3]
    a0 = box[3] if len(box) > 3 else 0.0
    a = math.radians(deg)
    x, z = cx - pivot[0], cz - pivot[2]
    xr, zr = x * math.cos(a) + z * math.sin(a), -x * math.sin(a) + z * math.cos(a)
    t = math.radians(deg + a0) / 2
    return ((pivot[0] + xr, cy, pivot[2] + zr), size, color, (math.cos(t), 0.0, math.sin(t), 0.0))


def _egg_xy(xr):
    return [(xr, -0.125 + 0.05 * k) for k in range(6)]


def _eggs(xr):
    return [((x, y, HB), EGG_R, 0.04, EGG) for x, y in _egg_xy(xr)]


# lid, closed: domed top over the eggs, side walls outside the base's ends, front skirt with an inward bead
_LID = [((-0.00425, 0.0, 0.0815), (0.1085, 0.308, 0.003), PULP),          # top
        ((-0.057, 0.0, 0.0705), (0.003, 0.308, 0.025), PULP),             # front skirt, z 5.8-8.3 cm
        ((-0.054, 0.0, 0.062), (0.003, 0.300, 0.004), PULP),              # bead inside the skirt, z 6.0-6.4 cm
        ((-0.004, 0.1525, 0.0665), (0.109, 0.003, 0.033), PULP),          # end walls
        ((-0.004, -0.1525, 0.0665), (0.109, 0.003, 0.033), PULP),
        ((0.049, 0.0, 0.064), (0.003, 0.308, 0.038), PULP)]               # back wall at the hinge
# flap, upright: a 3 cm leaf with a bead along its top, flat below and sloped above (the skirt's bead cams it in)
_FLAP = [((-0.051, 0.0, 0.060), (0.002, 0.290, 0.030), PULP),
         ((-0.05325, 0.0, 0.0695), (0.0025, 0.290, 0.003), PULP),
         ((-0.0530, 0.0, 0.0722), (0.0035, 0.290, 0.0010), PULP, -45.0)]

BACK = hinged("carton_back", pos=(CX, 0.0, TOP), mass=HALF_MASS, friction=1.2, color=PULP,
              boxes=[((BX / 2 + 0.0005, 0.0, HB / 2), (BX - 0.001, 2 * BY, HB), PULP)],
              cylinders=_eggs(0.025),
              hinges=[Hinge("lid", pivot=LP, axis="Y", lower=-OPEN - LID_PAST, upper=0.5, rate=LID_RATE,
                            damping=0.002, mass=0.02, boxes=[_rot(b, LP, OPEN) for b in _LID])])
FRONT = hinged("carton_front", pos=(CX, 0.0, TOP), mass=HALF_MASS, friction=1.2, color=PULP,
               boxes=[((-BX / 2 - 0.0005, 0.0, HB / 2), (BX - 0.001, 2 * BY, HB), PULP)],
               cylinders=_eggs(-0.025),
               hinges=[Hinge("flap", pivot=FP, axis="Y", lower=-5.0, upper=REST + IN_MAX, rate=FLAP_RATE,
                             damping=FLAP_DAMP, mass=0.005, friction=0.5, boxes=[_rot(b, FP, -REST) for b in _FLAP])])
OBJECTS = [BACK, FRONT]

# The two halves are one moulded piece: weld them with a fixed joint (outside both articulations), so the carton
# tips and slides as a single 0.5 kg body while the lid (on the back half) and the flap (on the front half) can still
# touch each other.
import hingekit as _hk  # noqa: E402

_hk_spawn = _hk._spawn_part
WELD = os.environ.get("EGG_WELD", "1") == "1"


def _spawn_welded(prim_path, cfg, translation=None, orientation=None, **kwargs):
    prim = _hk_spawn(prim_path, cfg, translation=translation, orientation=orientation, **kwargs)
    if WELD and cfg.part_name == "carton_front":
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics
        stage = get_current_stage()
        j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld")
        j.CreateBody0Rel().SetTargets([Sdf.Path(prim_path.rsplit("/", 1)[0] + "/carton_back/body")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/body")])
        j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        print(f"[egg] welded carton halves at {prim_path}", flush=True)
    return prim


_hk._spawn_part = _spawn_welded
EGGS = {"carton_back": _egg_xy(0.025), "carton_front": _egg_xy(-0.025)}

# the closed fingers, as a 1 cm-radius rod along the gripper body's +y axis from 134 mm (the TCP) to 158 mm (the
# fingertips, 2.4 cm below the TCP). Calibrated from a body dump: in a down grip the gripper body sat 134 to 135 mm
# straight above the commanded TCP on both arms.
FINGER_BODY = "gripper"
FINGER_SEG = (0.134, 0.146, 0.158)
FINGER_R = 0.010

_S: dict = {}


@on_reset
def _reset(environment=None):
    _S.update(step=0, scored=None, last=0.0, recent=[], start=None, damage=None, fmax=0.0, fnow=0.0, held=0)


@on_step
def _count(environment=None):
    _S["step"] += 1


_reset()


def _qrot(q_wxyz, v):
    w, x, y, z = q_wxyz
    m = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return m @ np.asarray(v, float)


def _arm_bodies():
    """(arm name, body names, positions (env frame), quats wxyz) for both arms, or [] before the scene exists."""
    e = env()
    if e is None:
        return []
    scene = e.unwrapped.scene
    org = np.asarray(getattr(scene.env_origins, "torch", scene.env_origins)[0].detach().cpu(), float)
    out = []
    for name in ("left_robot", "right_robot"):
        arm = scene[name]
        pos = getattr(arm.data.body_pos_w, "torch", arm.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        quat = getattr(arm.data.body_quat_w, "torch", arm.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        out.append((name, list(arm.body_names), pos - org, quat))
    return out


def _tips():
    """Finger-tip bodies' origins (m, env frame), both arms; used for the 3 cm hands-off check."""
    out = []
    for _, names, pos, _q in _arm_bodies():
        out += [pos[names.index(n)] for n in ("tip_left", "tip_right")]
    return out


def _fingers():
    """Points along both arms' closed fingers, TCP to fingertips (m, env frame)."""
    out = []
    for _, names, pos, quat in _arm_bodies():
        i = names.index(FINGER_BODY)
        out += [pos[i] + _qrot(quat[i], (0.0, s, 0.0)) for s in FINGER_SEG]
    return out


def _clear() -> bool:
    """Every finger tip at least 3 cm outside the carton's closed outline (above, in front, behind or beyond)."""
    for t in _tips():
        dx = max(abs(t[0] - CX) - (BX + 0.009), 0.0)
        dy = max(abs(t[1]) - (BY + 0.005), 0.0)
        dz = max(t[2] - TOP - 0.085, 0.0)
        if math.sqrt(dx * dx + dy * dy + dz * dz) < 0.03:
            return False
    return True


def _egg_gap(objects, flap=-REST):
    """Smallest gap (m) between the fingers and any egg's exposed part (a 4.4 cm cylinder above the cups). The front
    row's outer sides do not count while the flap stands in front of them (within 30 degrees of upright or further in)
    and the finger is on the far side of it: the finger is then pressing on the flap, not on an egg."""
    best = 1.0
    fingers = _fingers()
    a = math.radians(flap)
    for half, xy in EGGS.items():
        p = np.asarray(objects[half].pos, float)
        for ex, ey in xy:
            for f in fingers:
                if half == "carton_front" and flap >= -30.0:
                    hx, hz = f[0] - p[0] - FP[0], f[2] - p[2] - FP[2]     # from the flap's hinge line
                    out = -hx * math.cos(a) + hz * math.sin(a)             # out from the flap's plane
                    up = hx * math.sin(a) + hz * math.cos(a)               # up along the flap
                    if out > -0.002 and up < 0.04:
                        continue
                dr = max(math.hypot(f[0] - p[0] - ex, f[1] - p[1] - ey) - EGG_R, 0.0)
                dz = max(f[2] - p[2] - EGG_TOP, 0.0)
                best = min(best, math.hypot(dr, dz) - FINGER_R)
    return best


def _up(q) -> np.ndarray:
    """The body's z axis in the world frame, from its quaternion (x, y, z, w)."""
    x, y, z, w = (float(v) for v in q)
    return np.array((2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)))


def _tilt(q, q0) -> float:
    """Tip (deg): the angle between the carton's vertical axis now and at the start. Turning the carton about the
    vertical on the tabletop does not change it (review TK-3JWKR5); only tipping it off flat does."""
    return math.degrees(math.acos(min(1.0, max(-1.0, float(np.dot(_up(q), _up(q0)))))))


def _state(objects):
    lid = objects["carton_back"].sliders["lid"] + OPEN          # 0 = shut, OPEN = as found, negative = pressed past
    flap = objects["carton_front"].sliders["flap"] - REST       # 0 = upright, + = tucked in, -REST = as found
    return lid, flap


def _check_damage(objects, lid, flap):
    """The first damage seen this trial, or None. Kept until the scene is reset."""
    if _S["damage"]:
        return _S["damage"]
    why = None
    b, f = objects["carton_back"], objects["carton_front"]
    pb, pf = np.asarray(b.pos, float), np.asarray(f.pos, float)
    s = _S["start"]
    if lid < -CRUSH_LID:
        why = f"lid pressed onto the eggs ({-lid:.1f} deg past shut)"
    elif flap > CRUSH_FLAP:
        why = f"flap pressed onto the eggs ({flap:.1f} deg in)"
    elif _S["step"] > 5 and min(b.lin_vel[2], f.lin_vel[2]) < -DROP:
        why = f"carton dropped ({-min(b.lin_vel[2], f.lin_vel[2]):.2f} m/s)"
    elif max(_tilt(b.quat_xyzw, s["qb"]), _tilt(f.quat_xyzw, s["qf"])) > TILT:
        why = "carton tipped"
    elif np.linalg.norm((pb - pf) - (s["pb"] - s["pf"])) > SPLIT:
        why = "carton halves pulled apart"
    elif _S["step"] > 5 and _S["held"] >= PRESS_HOLD:
        why = f"pressed {_S['fnow']:.1f} N onto the eggs through the latched lid and flap"
    else:
        g = _egg_gap(objects, flap)
        if g < TOUCH:
            why = f"finger on an egg ({g * 1000:.1f} mm)"
    if why:
        _S["damage"] = f"step {_S['step']}: {why}"
    return _S["damage"]


def _leaf_forces():
    """Force (N) carried by the lid's and the flap's hinge joints, from the solver's joint reaction; None if the
    simulator does not report it."""
    e = env()
    if e is None:
        return None
    out = {}
    for half, leaf in (("carton_back", "lid"), ("carton_front", "flap")):
        asset = e.unwrapped.scene[half]
        view = getattr(asset, "root_view", None) or getattr(asset, "root_physx_view", None)
        try:
            w = view.get_link_incoming_joint_force()
            names = list(view.shared_metatype.link_names)
        except Exception as ex:  # noqa: BLE001
            if not _S.get("wrench_err"):
                _S["wrench_err"] = True
                print(f"[egg] no joint force: {ex!r}", flush=True)
            return None
        t = getattr(w, "torch", w)
        t = t.detach().cpu().numpy() if hasattr(t, "detach") else (t.numpy() if hasattr(t, "numpy") else np.asarray(t))
        w = np.asarray(t, float).reshape(-1, len(names), 6)[0]
        out[leaf] = float(np.linalg.norm(w[names.index(leaf)][:3]))
    return out


def _dump_bodies():
    for name, names, pos, quat in _arm_bodies():
        for n, p, q in zip(names, pos, quat):
            print(f"[egg-body] step {_S['step']} {name} {n} pos {np.round((p - (CX, 0, TOP)) * 1000, 1).tolist()} "
                  f"quat {np.round(q, 4).tolist()}", flush=True)


def score(objects) -> float:
    """1.0: at the end the lid is within 5 degrees of shut with the flap within 6 degrees of upright (inside the
    skirt, where its bead holds the lid), no finger tip within 3 cm of the carton, and the lid still (moved under
    0.5 degrees) for the last second, wherever the carton has been slid or turned on the table (review TK-3JWKR5).
    0 otherwise, and 0 for the whole trial if at any time the lid was pressed more than
    6 degrees past shut or the flap more than 16 degrees in (onto the eggs), more than 5 N was pressed through the lid
    and flap for 0.25 s once the carton was latched (onto the eggs; the press that latches it comes before), a finger
    came within 5 mm of an egg, or the carton fell at over 1 m/s, tipped 20 degrees off flat (its vertical axis; a
    turn on the tabletop is not a tip) or its halves pulled 3 mm apart.
    Lifting or nudging the carton gently is not damage."""
    if _S["scored"] == _S["step"]:
        return _S["last"]
    _S["scored"] = _S["step"]
    lid, flap = _state(objects)
    b, f = objects["carton_back"], objects["carton_front"]
    pb, pf = np.asarray(b.pos, float), np.asarray(f.pos, float)
    if _S["start"] is None:
        _S["start"] = dict(pb=pb.copy(), pf=pf.copy(), qb=np.asarray(b.quat_xyzw, float).copy(),
                           qf=np.asarray(f.quat_xyzw, float).copy())
    s0 = _S["start"]
    moved = max(np.linalg.norm((pb - s0["pb"])[:2]), np.linalg.norm((pf - s0["pf"])[:2]))
    locked = 0.0 - CRUSH_LID <= lid <= 5.0 and flap >= -6.0
    lf = _leaf_forces()
    _S["fnow"] = 0.0 if lf is None or not locked else lf["lid"] + lf["flap"]
    _S["held"] = _S["held"] + 1 if _S["fnow"] > PRESS else 0
    if _S["step"] > 5:
        _S["fmax"] = max(_S["fmax"], _S["fnow"])
    damage = _check_damage(objects, lid, flap)
    r = _S["recent"]
    r.append((lid, _clear()))
    del r[:-20]
    still = len(r) == 20 and all(abs(a - lid) < 0.5 and c for a, c in r)
    s = 0.0 if damage else (1.0 if (locked and still) else 0.0)  # moved is logged only
    _S["last"] = s
    dump = os.environ.get("EGG_BODIES", "")
    if dump and _S["step"] in [int(v) for v in dump.split(",")]:
        _dump_bodies()
    if os.environ.get("EGG_DEBUG") and _S["step"] % 10 == 0:
        fs = " ".join(f"{np.round((p - (CX, 0, TOP)) * 1000, 0).tolist()}" for p in _fingers())
        lf = "none" if lf is None else f"{lf['lid']:.2f}/{lf['flap']:.2f}N max {_S['fmax']:.2f}"
        print(f"[egg] step {_S['step']} lid {lid:.1f} flap {flap:.1f} moved {moved * 1000:.1f}mm hingeF {lf} "
              f"lift {max(pb[2] - s0['pb'][2], pf[2] - s0['pf'][2]) * 1000:.1f}mm "
              f"tilt {_tilt(b.quat_xyzw, s0['qb']):.1f}/{_tilt(f.quat_xyzw, s0['qf']):.1f} "
              f"split {np.linalg.norm((pb - pf) - (s0['pb'] - s0['pf'])) * 1000:.1f}mm "
              f"egg_gap {_egg_gap(objects, flap) * 1000:.0f}mm "
              f"clear {r[-1][1]} damage {damage} score {s:.2f} | fingers {fs}", flush=True)
    return s


# -- scripted demo: the left arm lifts the flap from under its outer face and tucks it in, short of the eggs; the right
# arm comes down behind the lid, pushes it over and down to 45 degrees open and waits; the left arm lets the flap go
# and backs out; the right arm brings the lid down over the flap at once, just past shut, pauses, and lifts away. -----
FZ = float(os.environ.get("EGG_FZ", 0.012))   # TCP height above the point of the finger that pushes
FT = float(os.environ.get("EGG_FT", 0.011))   # finger half thickness: TCP this far off the pushed face
LT = float(os.environ.get("EGG_LT", 0.006))   # lid: TCP this far off the lid face
LZ = float(os.environ.get("EGG_LZ", 0.028))   # the finger reaches this far below the TCP: lift lid targets by it
YL, YR = 0.08, -0.08


def _lid_pt(c, d, h):
    """World point on the lid's top surface, d from its hinge, h off it, with the lid c degrees open."""
    v = (-d, 0.083 - HB + h)
    a = math.radians(c)
    return (CX + LP[0] + v[0] * math.cos(a) + v[1] * math.sin(a), TOP + LP[2] - v[0] * math.sin(a) + v[1] * math.cos(a))


def _flap_pt(u, r, h):
    """World point on the flap's outer face, r up from its hinge, h out from it, the flap u degrees in of upright."""
    a = math.radians(u)
    return (CX + FP[0] - h * math.cos(a) + r * math.sin(a), TOP + FP[2] + h * math.sin(a) + r * math.cos(a))


def demo():
    v = os.environ.get("EGG_VARIANT", "")
    R = gripkit.pitched(0.0)
    C = FINGER_CLOSED
    hold_c = float(os.environ.get("EGG_HOLD", 45.0))
    tuck = float(os.environ.get("EGG_TUCK", 30.0))
    if v == "flap_press":        # wrong: fold the flap hard in against the eggs
        tuck = 55.0
    rf, d = float(os.environ.get("EGG_RF", 0.025)), float(os.environ.get("EGG_D", 0.095))
    shut_to = float(os.environ.get("EGG_SHUT", -16.0))
    if v == "crush":             # wrong: lock it, then lean the lid down onto the eggs
        shut_to = -30.0

    left = [Waypoint(2.0, (CX - 0.13, YL, TOP + 0.15), C, "above, in front of the flap", R)]
    x, z = _flap_pt(-REST, rf, FT + 0.012)
    left += [Waypoint(1.5, (x - 0.01, YL, z - 0.006 + FZ), C, "outside the flap, low", R)]
    for u in np.linspace(-REST, tuck, 9):
        x, z = _flap_pt(u, rf, FT)
        left.append(Waypoint(0.3, (x, YL, z + FZ), C, f"tuck {u:.0f}", R))

    right = [Waypoint(2.0, (CX + 0.12, YR, TOP + 0.17), C, "above, behind the lid", R)]
    if v == "egg_touch":         # wrong: rest the fingertips on a front-row egg first, then close it properly
        ex, ey = -0.025, -0.075
        right = [Waypoint(1.5, (CX + ex, ey, TOP + 0.15), C, "above a front egg", R),
                 Waypoint(1.0, (CX + ex, ey, TOP + EGG_TOP + 0.024 + 0.004), C, "fingertips 4 mm above the egg", R),
                 Waypoint(0.8, (CX + ex, ey, TOP + 0.17), C, "up again", R),
                 Waypoint(1.2, (CX + 0.12, YR, TOP + 0.17), C, "above, behind the lid", R)]
    x, z = _lid_pt(OPEN, d, LT + 0.015)
    right += [Waypoint(1.5, (x, YR, z), C, "behind the lid", R)]
    for c in np.linspace(OPEN, hold_c, 9):
        x, z = _lid_pt(c, d, LT)
        right.append(Waypoint(0.35, (x, YR, z + LZ * (1 - c / OPEN)), C, f"lid {c:.0f}", R))

    t_l = sum(w.duration for w in left)
    t_r = sum(w.duration for w in right)
    t_go = max(t_l, t_r) + 0.8
    lp, rp = left[-1].pos, right[-1].pos
    left.append(Waypoint(t_go - t_l, lp, C, "hold the flap in", R))
    right.append(Waypoint(t_go - t_r, rp, C, "hold the lid", R))
    if v == "lid_only":          # wrong: shut the lid over the flap left outside
        left = left[:1] + [Waypoint(t_go - left[0].duration, left[0].pos, C, "wait", R)]
    if v == "slow":              # wrong: tuck the flap, let go, then close the lid 2.5 s later
        right[-1] = Waypoint(t_go - t_r + 2.5, rp, C, "hold the lid", R)
    left += [Waypoint(0.4, lp, C, "keep it in while the lid comes down to 20 degrees", R),
             Waypoint(0.3, (lp[0] - 0.04, YL, lp[2] + 0.005), C, "let go of the flap, back out", R),
             Waypoint(1.5, (CX - 0.15, YL, TOP + 0.18), C, "clear", R),
             Waypoint(1.5, (0.18, 0.30, TOP + 0.22), C, "park aside, carton in view", R)]
    shut = []
    mid = float(os.environ.get("EGG_MID", 20.0))
    x, z = _lid_pt(mid, d, LT)
    shut.append(Waypoint(0.4, (x, YR, z + LZ * (1 - mid / OPEN)), C, f"lid down to {mid:.0f}", R))
    for c in np.linspace(mid, shut_to, 4)[1:]:
        x, z = _lid_pt(c, d, LT)
        shut.append(Waypoint(0.15, (x, YR, z + LZ * (1 - c / OPEN)), C, f"shut {c:.0f}", R))
    right += shut + [Waypoint(0.8, shut[-1].pos, C, "press shut", R)]
    x, z = shut[-1].pos[0], shut[-1].pos[2]
    right += [Waypoint(1.2, (x, YR, z + 0.10), C, "lift away", R),
              Waypoint(1.5, (CX, YR - 0.08, TOP + 0.20), C, "clear", R),
              Waypoint(1.5, (0.18, -0.30, TOP + 0.22), C, "park aside, carton in view", R)]
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    t_end = max(t_l, t_r) + 2.0
    left.append(Waypoint(t_end - t_l, left[-1].pos, C, "wait", R))
    right.append(Waypoint(t_end - t_r, right[-1].pos, C, "wait", R))
    if v == "nothing":
        return {"left": [Waypoint(3.0, (0.18, 0.30, TOP + 0.22), FINGER_OPEN, "park")],
                "right": [Waypoint(3.0, (0.18, -0.30, TOP + 0.22), FINGER_OPEN, "park")]}
    return {"left": left, "right": right}
