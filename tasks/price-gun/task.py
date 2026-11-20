"""On the station table: three small spice bottles standing in a row, and a hand labeller (a "price gun") lying on its
right side to the right of them. Each bottle is 46 mm across and 111 mm tall with a flat red cap, and weighs 90 g, so
it tips over at a touch. The price gun has a body that holds the label roll, a nose with a small roller under its
front, a handle, and a sprung trigger in front of the handle. Squeezing the trigger fully and letting it go feeds one
label, which hangs under the roller; wiping the roller across a surface sticks the label down.

The task: put one price label flat on the cap of each bottle, then put the gun down. One hand holds the bottle while
the other squeezes and wipes.

Simulation notes
- The trigger is a hinge with a wound spring: a step hook aims its drive past the rest stop, so it rests on the stop
  with a preload. One gripper holds the gun by the handle (behind) and the trigger (in front): closing fully is a
  full stroke, opening to FINGER_HOLD lets the trigger out and still holds the gun.
- Labels are state, not bodies (class Labels): a full stroke and release presents a label under the roller; where the
  roller then touches a cap (or the table) the label is laid down behind it as it is wiped; each of its four cells
  sticks where the roller passes over it. Thin display-only shapes show the labels.
- The arm's joints are soft and carry the gun's weight, so the scripted demo aims each pose of the loaded arm high by
  the sag a static model predicts (function _Carry).
- A step hook prints lines starting "PG|" (state every 5 steps) and "PG!" (label events) for tuning.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, READY, SHOULDER, TCP_OFFSET, WRIST  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BODY_GREY = (0.20, 0.21, 0.23)
TRIGGER_RED = (0.80, 0.10, 0.10)
RUBBER = (0.08, 0.08, 0.08)
LABEL_WHITE = (0.97, 0.97, 0.95)
SPICE = (0.45, 0.26, 0.12)
CAP_RED = (0.72, 0.08, 0.08)
PAPER = (0.93, 0.90, 0.80)

# -- the label ---------------------------------------------------------------------------------------------------------
LABEL_W = 0.020  # across the gun (along the roller)
LABEL_L = 0.011  # along the feed direction
LABEL_T = 0.0006  # drawn thickness
CELL_ROWS = (0.00275, 0.00825)  # cell centres behind the touch-down line, along the wipe
CELL_COLS = (-0.005, 0.005)  # cell centres either side of the roller's middle

# -- the bottles -------------------------------------------------------------------------------------------------------
# A 2.37 oz retail spice bottle: 46 mm across, 111 mm tall with its flip-top cap, about 90 g full. The cap's flat top
# is 44 mm across.
JAR_R = 0.023
JAR_H = 0.111
CAP_R = 0.022
CAP_H = 0.020
CAP_FLAT_R = 0.021  # a label must lie inside this radius: the cap's top, less its rounded edge
JAR_MASS = 0.09
JAR_X = 0.36
JAR_Y = {"left": 0.13, "middle": 0.00, "right": -0.13}
JAR_ORDER = ("left", "middle", "right")
LID_Z = TOP + JAR_H


def _label_shape():
    return ((0.0, 0.0, -1.0), (LABEL_L, LABEL_W, LABEL_T), LABEL_WHITE)  # parked out of sight; a hook places and shows it


def _jar(name: str, y: float):
    labkit.DISPLAY[f"jar_{name}"] = {"body": [_label_shape(), _label_shape()]}
    return part(
        f"jar_{name}", pos=(JAR_X, y, TOP + JAR_H / 2), mass=JAR_MASS, friction=0.4,
        cylinders=[
            ((0.0, 0.0, -CAP_H / 2), JAR_R, JAR_H - CAP_H, SPICE),  # the bottle
            ((0.0, 0.0, (JAR_H - CAP_H) / 2), CAP_R, CAP_H, CAP_RED),  # its cap
            ((0.0, 0.0, -CAP_H / 2 - 0.005), JAR_R + 0.0003, 0.045, PAPER),  # its paper label
        ],
    )


JARS = [_jar(name, JAR_Y[name]) for name in JAR_ORDER]
labkit.DISPLAY["jar_middle"]["root"] = [_label_shape() for _ in range(4)]  # labels stuck to the table are drawn from here
TABLE_SHOW_FROM = np.array([JAR_X, JAR_Y["middle"], TOP + JAR_H / 2])

# -- the price gun -----------------------------------------------------------------------------------------------------
# Gun frame (g): x forward to the nose, y to the gun's left, z up when it is held like a pistol; origin on the gun's
# mid-plane. The gun starts lying on its right side with its nose pointing left (+y) and its handle towards the
# robots, so its shapes are written in a part frame (p) with
#     p = (g.x, -g.z, g.y)         and the part is turned 90 degrees about the vertical,
# which puts gun x along world +y, gun z along world +x and gun y (its left side) up.
GUN_X, GUN_Y = 0.25, -0.30  # world position of the gun frame's origin
GUN_HALF_W = 0.025  # half the body's width
GUN_REST_Z = GUN_HALF_W  # height of the gun's mid-plane above the table when it lies on its side
GUN_YAW = math.pi / 2
GUN_MASS = 0.41  # body 0.38, trigger 0.02, roller 0.01


def _p(g):
    """A point in the gun frame, in the part frame."""
    return (g[0], -g[2], g[1])


def _g(p):
    """A point in the part frame, in the gun frame."""
    return np.array([p[0], p[2], -p[1]])


def _pbox(centre, size):
    """A box given in the gun frame (centre, size along gun x, y, z), in the part frame."""
    return _p(centre), (size[0], size[2], size[1])


def gun_start_world(g):
    """Where a point of the gun frame is in the world at the start."""
    return np.array([GUN_X + g[2], GUN_Y + g[0], TOP + GUN_REST_Z + g[1]])


BODY_G = ((0.010, 0.0, 0.060), (0.150, 0.050, 0.070))  # holds the label roll
NOSE_G = ((0.100, 0.0, 0.040), (0.040, 0.050, 0.040))
HANDLE_G = ((-0.040, 0.0, -0.030), (0.032, 0.040, 0.110))  # back face at x -0.056, front face at x -0.024
HANDLE_CORE_G = ((-0.040, 0.0, -0.030), (0.030, 0.038, 0.108))  # inside the handle: counts its volume twice, so it weighs double
WINDOW_G = ((0.010, 0.0, 0.096), (0.060, 0.030, 0.002))  # white label window on top of the body
HANDLE_BACK = HANDLE_G[0][0] - HANDLE_G[1][0] / 2

# The trigger hangs from a pivot under the body, 23 mm in front of the handle, with a finger rest on its front face.
# Squeezing turns it about the gun's y axis (the part's Z axis), from 0 up to TRIGGER_TRAVEL, when its tip meets the
# handle.
TRIGGER_PIVOT_G = (0.005, 0.0, 0.020)
TRIGGER_G = ((0.005, 0.0, -0.025), (0.012, 0.034, 0.090))  # at rest: front face at x +0.011, tip at z -0.070
REST_PAD_G = ((0.015, 0.0, -0.022), (0.008, 0.034, 0.012))  # the finger rest: 8 mm proud, 36 to 48 mm below the pivot
TRIGGER_TRAVEL = 0.26  # rad (23 mm at the trigger's tip)
TRIGGER_K = 1.73  # N m per rad
TRIGGER_WIND = 0.4335  # rad the spring is wound at rest: 0.75 N m on the rest stop, 1.2 N m when squeezed
TRIGGER = Joint(
    "trigger", kind="hinge", axis="Z", anchor=_p(TRIGGER_PIVOT_G),
    lower=0.0, upper=TRIGGER_TRAVEL, initial=0.0, stiffness=TRIGGER_K, damping=0.03, target=0.0, mass=0.02,
    boxes=[(*_pbox(*TRIGGER_G), TRIGGER_RED), (*_pbox(*REST_PAD_G), TRIGGER_RED)],
)
STROKE_FULL = 0.90  # fraction of the trigger's travel that counts as a full squeeze
STROKE_OUT = 0.25  # ... and below which a full squeeze has been let go: one label is fed

ROLLER_R = 0.008
ROLLER_LEN = 0.036
ROLLER_G = (0.118, 0.0, 0.014)  # under the front of the nose
ROLLER = Joint(
    "roller", kind="hinge", axis="Z", anchor=_p(ROLLER_G),
    # A weak spring brings it back once lifted. Its damping stands for the label's pull as it is wiped on:
    # 0.4 N at the rim at 12 mm/s.
    lower=-6.0, upper=6.0, initial=0.0, stiffness=0.0005, damping=0.0021, target=0.0, mass=0.01,
    cylinders=[(_p(ROLLER_G), ROLLER_R, ROLLER_LEN, RUBBER, "Z")],
)

# A fed label waiting under the roller, drawn sticking out in front of it.
labkit.DISPLAY["gun"] = {"body": [(_p((ROLLER_G[0] + 0.004, 0.0, ROLLER_G[2] - ROLLER_R - 0.001)), (LABEL_L, LABEL_T, LABEL_W), LABEL_WHITE)]}
GUN = part(
    "gun", pos=(GUN_X, GUN_Y, TOP + GUN_REST_Z + 0.0005), mass=0.38, friction=0.8, yaw=GUN_YAW,
    boxes=[(*_pbox(*BODY_G), BODY_GREY), (*_pbox(*NOSE_G), BODY_GREY), (*_pbox(*HANDLE_G), BODY_GREY), (*_pbox(*HANDLE_CORE_G), BODY_GREY),
           (*_pbox(*WINDOW_G), LABEL_WHITE)],
    joints=[TRIGGER, ROLLER],
)

OBJECTS = [*JARS, GUN]


def _gun_com():
    """The gun's centre of mass in the gun frame: the body's shapes at one density, plus the trigger and the roller."""
    shapes = [BODY_G, NOSE_G, HANDLE_G, HANDLE_CORE_G, WINDOW_G]
    vol = np.array([s[0] * s[1] * s[2] for _, s in shapes])
    body = (vol[:, None] * np.array([c for c, _ in shapes])).sum(axis=0) / vol.sum()
    return (0.38 * body + 0.02 * np.array(TRIGGER_G[0]) + 0.01 * np.array(ROLLER_G)) / GUN_MASS


GUN_COM_G = _gun_com()

# For the offline plan check: world-frame boxes the arms must not hit.
PLANCHECK_BOXES = [
    ("table", (0.35, 0.0, TOP - 0.02), (0.8, 1.2, 0.04)),
    *[(f"jar_{name}", (JAR_X, y, TOP + JAR_H / 2), (2 * JAR_R, 2 * JAR_R, JAR_H)) for name, y in JAR_Y.items()],
]

# -- how one gripper holds the gun ---------------------------------------------------------------------------------------
# The fingers straddle the handle (behind) and the trigger's finger rest (in front), 42 mm below the trigger's pivot.
# In the gun frame the gripper's closing axis is gun -x, its approach axis gun -y, and its third ("across") axis gun -z.
GRIP_Y, GRIP_Z = -0.009, REST_PAD_G[0][2]  # pad tips 3 mm off the table at the pick-up; pads centred on the finger rest
PAD_FRONT = REST_PAD_G[0][0] + REST_PAD_G[1][0] / 2  # the finger rest's front face at rest (gun x)
PAD_LEVER = TRIGGER_PIVOT_G[2] - (REST_PAD_G[0][2] + REST_PAD_G[1][2] / 2)  # its top edge below the pivot: where a flat finger bears
_SQ_X = TRIGGER_PIVOT_G[0] + (PAD_FRONT - TRIGGER_PIVOT_G[0]) * math.cos(TRIGGER_TRAVEL) - PAD_LEVER * math.sin(TRIGGER_TRAVEL)
REST_GAP = PAD_FRONT - HANDLE_BACK  # between the pads when they touch the handle and the finger rest: 75 mm
SQUEEZED_GAP = _SQ_X - HANDLE_BACK  # ... and with the trigger fully squeezed: 65 mm
GRIP_HOLD = np.array([(HANDLE_BACK + PAD_FRONT) / 2, GRIP_Y, GRIP_Z])  # the point of the gun at the TCP, trigger out
GRIP_SQ = np.array([(HANDLE_BACK + _SQ_X) / 2, GRIP_Y, GRIP_Z])  # ... and squeezed
FINGER_N_PER_M = 2500.0  # grip force per metre the pads are commanded inside the object (two 5000 N/m fingers)
PRELOAD_N = TRIGGER_K * TRIGGER_WIND / PAD_LEVER  # the push on the finger rest that starts the trigger moving: 21 N
FULL_N = TRIGGER_K * (TRIGGER_WIND + TRIGGER_TRAVEL) / PAD_LEVER  # ... and that holds it fully squeezed: 33 N
HOLD_N = 14.0  # held with less than the preload, the trigger stays out
SQUEEZE_N = 44.0  # more than FULL_N and under the gripper's 50 N limit: both fingers stay in position control
FINGER_HOLD = -(REST_GAP - HOLD_N / FINGER_N_PER_M) / 2
FINGER_SQUEEZE = -(SQUEEZED_GAP - SQUEEZE_N / FINGER_N_PER_M) / 2
FINGER_CARRY = FINGER_CLOSED  # both fingers at their 50 N limit: the firmest grip, for carrying the gun on its side


# -- labels: fed by the trigger, stuck by the roller ---------------------------------------------------------------------
TOUCH_TOL = 0.001  # m: the roller presses where it is within 1 mm of the surface
LIFT_STEPS = 3  # control steps off the surface before a label counts as left behind


class Labels:
    """The labels' state. step() takes measured poses only."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.armed = False  # the trigger has been fully squeezed and not yet let go
        self.fed = 0
        self.hanging = 0  # labels waiting under the roller (2 or more: fed on top of each other)
        self.applied = []  # dicts: surface, o, w, l (2-D, in the surface's frame), layers, stuck[4], open, off
        self.events = []

    def _say(self, text):
        self.events.append(text)

    def step(self, stroke, roller, axis, fwd, jars):
        """stroke: the trigger's travel, 0 (out) to 1 (squeezed). roller: the roller's centre; axis: its axle (the gun's
        left); fwd: the gun's forward axis. jars: name -> (pos, 3x3 rotation). All in the world frame."""
        if stroke >= STROKE_FULL:
            if not self.armed:
                self._say("full stroke")
            self.armed = True
        elif self.armed and stroke <= STROKE_OUT:
            self.armed = False
            self.fed += 1
            self.hanging += 1
            self._say(f"label {self.fed} fed" + ("" if self.hanging == 1 else f" on top of a waiting label ({self.hanging} waiting)"))

        # which surface is the roller on, and how far above it is the roller at each column of cells?
        touching = None
        for name, (pos, rot) in jars.items():
            up, top = rot[:, 2], pos + rot[:, 2] * (JAR_H / 2)
            local = rot.T @ (roller - pos)
            if math.hypot(local[0], local[1]) > CAP_R:
                continue
            gaps = [float((roller + axis * c - top) @ up) - ROLLER_R for c in CELL_COLS]
            if min(gaps) <= TOUCH_TOL:
                touching = (name, local[:2], (rot.T @ -fwd)[:2], (rot.T @ axis)[:2], gaps)
                break
        if touching is None:
            gaps = [float((roller + axis * c)[2]) - TOP - ROLLER_R for c in CELL_COLS]
            if min(gaps) <= TOUCH_TOL:
                touching = ("table", roller[:2].copy(), -fwd[:2], axis[:2], gaps)

        current = self.applied[-1] if self.applied and self.applied[-1]["open"] else None
        if touching is not None and self.hanging and (current is None or current["surface"] != touching[0]):
            name, at, w, lat, gaps = touching
            w, lat = w / (np.linalg.norm(w) or 1.0), lat / (np.linalg.norm(lat) or 1.0)
            if current is not None:
                current["open"] = False
            current = dict(surface=name, o=np.array(at, float), w=w, l=lat, layers=self.hanging, stuck=[False] * 4, open=True, off=0)
            self.applied.append(current)
            self.hanging = 0
            self._say(f"label touched down on {name} at {1000 * at[0]:+.1f} {1000 * at[1]:+.1f} mm" + (" (doubled)" if current["layers"] > 1 else ""))
        if current is not None:
            if touching is not None and touching[0] == current["surface"]:
                current["off"] = 0
                rel = touching[1] - current["o"]
                u, v = float(rel @ current["w"]), float(rel @ current["l"])
                for k in range(4):
                    row, col = divmod(k, 2)
                    over = abs(u - CELL_ROWS[row]) <= 0.0015 and abs(CELL_COLS[col] - v) <= ROLLER_LEN / 2
                    if over and not current["stuck"][k] and touching[4][col] <= TOUCH_TOL:
                        current["stuck"][k] = True
                        self._say(f"cell {k} stuck on {current['surface']} (wiped {1000 * u:.1f} mm)")
            else:
                current["off"] += 1
                if current["off"] >= LIFT_STEPS:
                    current["open"] = False
                    self._say(f"roller left {current['surface']}: {sum(current['stuck'])} of 4 cells stuck")

    def corners(self, label):
        return [label["o"] + label["w"] * u + label["l"] * v for u in (0.0, LABEL_L) for v in (-LABEL_W / 2, LABEL_W / 2)]

    def on(self, surface):
        return [a for a in self.applied if a["surface"] == surface]

    def good(self, name):
        """Exactly one single label on this bottle's cap, all of it on the flat top and all four cells stuck."""
        mine = self.on(name)
        if len(mine) != 1 or mine[0]["layers"] != 1 or not all(mine[0]["stuck"]):
            return False
        return all(float(np.linalg.norm(c)) <= CAP_FLAT_R for c in self.corners(mine[0]))

    def summary(self):
        return f"fed {self.fed} waiting {self.hanging} " + " ".join(
            f"{a['surface'].replace('jar_', '')}:{''.join('x' if s else '.' for s in a['stuck'])}" for a in self.applied
        )


_labels = Labels()
_state = {"step": 0, "hist": [], "still": False, "gun_free": False, "jars_free": False, "show": None, "show_failed": False}
LOG_EVERY = 5  # control steps
STILL_STEPS, STILL_TOL = 20, 0.001  # at rest: nothing moved 1 mm over the last 1 s
RELEASE_DIST = 0.030  # m: every fingertip this far from the gun and from every bottle


def _np(x):
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _wind_spring(env):
    """Aim the trigger's spring past its rest stop, so the trigger rests on the stop with a preload."""
    import torch

    asset = env.unwrapped.scene["gun"]
    ids = [list(asset.joint_names).index(TRIGGER.joint_name)]
    asset.set_joint_position_target(torch.tensor([[-TRIGGER_WIND]], device=asset.device), joint_ids=ids)


def gun_axes(rot):
    """The gun's forward, left and up axes in the world, from the part's rotation."""
    return rot[:, 0], rot[:, 2], -rot[:, 1]


def _box_distance(p, centre, size):
    d = np.abs(np.asarray(p) - np.asarray(centre)) - np.asarray(size) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)) + min(float(d.max()), 0.0))


def _read(env):
    """Everything the hooks need, from the simulator: poses in the table frame, joints, fingertips."""
    from yam_arena import runtime as _rt

    b = env.unwrapped
    origin = _np(b.scene.env_origins[0])
    out = {"gun": labkit._part_state(env, labkit._SPECS["gun"]), "jars": {}, "tips": [], "arms": {}}
    for name in JAR_ORDER:
        out["jars"][f"jar_{name}"] = labkit._part_state(env, labkit._SPECS[f"jar_{name}"])
    for arm in ("left", "right"):
        robot = b.scene[f"{arm}_robot"]
        g = _rt.link_poses(robot)["gripper"]
        tcp = g[:3, :3] @ np.array(TCP_OFFSET) + g[:3, 3] - origin
        ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        fingers = _np(robot.data.joint_pos)[0][list(ids)]
        mid = tcp + g[:3, 2] * PAD_TIP_PAST_TCP
        out["tips"] += [mid + abs(fingers[0]) * g[:3, 0], mid - abs(fingers[1]) * g[:3, 0]]
        out["arms"][arm] = (tcp, g[:3, :3], fingers)
    return out


def _update(read):
    """One control step of the label model and of the at-rest and let-go checks."""
    gun = read["gun"]
    rot = gun.rotation()
    fwd, left, _ = gun_axes(rot)
    roller = gun.to_world(_p(ROLLER_G))
    stroke = gun.joints["trigger"] / TRIGGER_TRAVEL
    jars = {n: (np.asarray(s.pos, float), s.rotation()) for n, s in read["jars"].items()}
    _labels.step(stroke, roller, left, fwd, jars)

    gun_free, jars_free = True, True
    for tip in read["tips"]:
        tg = _g(rot.T @ (np.asarray(tip) - np.asarray(gun.pos)))
        gun_free = gun_free and min(_box_distance(tg, *b) for b in (BODY_G, NOSE_G, HANDLE_G)) >= RELEASE_DIST
        for pos, r in jars.values():
            local = r.T @ (np.asarray(tip) - pos)
            radial = math.hypot(local[0], local[1]) - JAR_R
            axial = abs(local[2]) - JAR_H / 2
            jars_free = jars_free and math.hypot(max(radial, 0.0), max(axial, 0.0)) >= RELEASE_DIST
    _state["gun_free"], _state["jars_free"] = gun_free, jars_free
    h = _state["hist"]
    h.append(np.concatenate([np.asarray(gun.pos, float)] + [p for p, _ in jars.values()]))
    del h[:-STILL_STEPS]
    _state["still"] = len(h) == STILL_STEPS and max(float(np.abs(p - h[-1]).max()) for p in h) < STILL_TOL
    return stroke, roller, fwd, left, jars


def _find_shows():
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    want = {"gun": "/gun/body/show_0"}
    for name in JAR_ORDER:
        for i in range(2):
            want[f"jar_{name}:{i}"] = f"/jar_{name}/body/show_{i}"
    for i in range(4):
        want[f"table:{i}"] = f"/jar_middle/show_{i}"
    found = {}
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        for key, tail in want.items():
            if path.endswith(tail):
                found.setdefault(key, []).append(prim)
    missing = sorted(set(want) - set(found))
    if missing:
        raise RuntimeError(f"label shapes not found on the stage: {missing}")
    return found


def _place(prims, pos, yaw_deg):
    from pxr import Gf, UsdGeom

    for prim in prims:
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            attr = op.GetAttr()
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                attr.Set(Gf.Vec3f(*map(float, pos)) if attr.GetTypeName() == "float3" else Gf.Vec3d(*map(float, pos)))
            elif op.GetOpType() == UsdGeom.XformOp.TypeRotateZ:
                attr.Set(float(yaw_deg))
        UsdGeom.Imageable(prim).MakeVisible()


def _draw():
    """Show the labels as they stand. Drawing never stops a run: a failure is reported once and the task carries on."""
    if _state["show_failed"]:
        return
    try:
        from pxr import UsdGeom

        if _state["show"] is None:
            _state["show"] = _find_shows()
        shows = _state["show"]
        used = set()
        if _labels.hanging:
            for prim in shows["gun"]:
                UsdGeom.Imageable(prim).MakeVisible()
            used.add("gun")
        count = {}
        for a in _labels.applied:
            where = "table" if a["surface"] == "table" else a["surface"]
            i = count.get(where, 0)
            count[where] = i + 1
            key = f"{where}:{i}"
            if key not in shows:
                continue
            centre = a["o"] + a["w"] * (LABEL_L / 2)
            lifted = 0.0 if all(a["stuck"]) or a["open"] else 0.002  # a label left partly unstuck is drawn standing proud
            if where == "table":
                pos = (centre[0] - TABLE_SHOW_FROM[0], centre[1] - TABLE_SHOW_FROM[1], TOP - TABLE_SHOW_FROM[2] + LABEL_T / 2 + lifted)
            else:
                pos = (centre[0], centre[1], JAR_H / 2 + LABEL_T / 2 + lifted)
            _place(shows[key], pos, math.degrees(math.atan2(a["w"][1], a["w"][0])))
            used.add(key)
        for key, prims in shows.items():
            if key not in used:
                for prim in prims:
                    UsdGeom.Imageable(prim).MakeInvisible()
    except Exception as e:  # noqa: BLE001
        _state["show_failed"] = True
        print(f"PG! drawing labels failed, carrying on without: {type(e).__name__}: {e}", flush=True)


@on_reset
def _reset(env):
    _labels.reset()
    _state.update(step=0, hist=[], still=False, gun_free=False, jars_free=False)
    _wind_spring(env)
    _draw()
    print(
        f"PG! reset: rest gap {1000 * REST_GAP:.1f} mm, squeezed gap {1000 * SQUEEZED_GAP:.1f} mm, hold {HOLD_N:.1f} N at finger "
        f"{1000 * FINGER_HOLD:.1f} mm, squeeze {SQUEEZE_N:.0f} N at finger {1000 * FINGER_SQUEEZE:.1f} mm, trigger moves at {PRELOAD_N:.1f} N, "
        f"full at {FULL_N:.1f} N, centre of mass {np.round(GUN_COM_G, 4).tolist()}",
        flush=True,
    )


@on_step
def _watch(env):
    _wind_spring(env)
    _state["step"] += 1
    read = _read(env)
    seen = len(_labels.events)
    stroke, roller, fwd, left, jars = _update(read)
    for text in _labels.events[seen:]:
        print(f"PG! step {_state['step']:4d} {text}", flush=True)
    if len(_labels.events) > seen:
        _draw()
    if _state["step"] % LOG_EVERY:
        return
    gun = read["gun"]
    tcp, grot, fingers = read["arms"]["right"]
    off = grot.T @ (gun.to_world(_p(GRIP_HOLD)) - tcp)  # the gun's grip point seen from the gripper: closing, across, approach
    gl = grot.T @ left  # the gun's left axis seen from the gripper: (0, 0, -1) when it sits square
    gf = grot.T @ fwd  # ... and its forward axis: (-1, 0, 0)
    askew = (math.degrees(math.atan2(gl[1], -gl[2])), math.degrees(math.atan2(gf[1], -gf[0])), math.degrees(math.atan2(gf[2], -gf[0])))
    name = min(jars, key=lambda n: float(np.linalg.norm(jars[n][0][:2] - roller[:2])))
    pos, rot = jars[name]
    local = rot.T @ (roller - pos)
    moved = " ".join(
        f"{n.replace('jar_', '')}:{1000 * np.linalg.norm(p[:2] - np.array([JAR_X, JAR_Y[n.replace('jar_', '')]])):.0f}/{read['jars'][n].tilt_deg():.0f}"
        for n, (p, _) in jars.items()
    )
    lf = read["arms"]["left"][2]
    print(
        f"PG| step {_state['step']:4d} stroke {stroke:+.2f} roller {roller[0]:+.3f} {roller[1]:+.3f} {roller[2] - TOP:+.3f} "
        f"over {name.replace('jar_', '')} {1000 * local[0]:+.0f} {1000 * local[1]:+.0f} gap {1000 * (local[2] - JAR_H / 2 - ROLLER_R):+.1f} "
        f"nose_down {math.degrees(math.asin(max(-1.0, min(1.0, -fwd[2])))):+5.1f} left_up {math.degrees(math.asin(max(-1.0, min(1.0, left[2])))):+5.1f} "
        f"R {1000 * fingers[0]:+.1f} {1000 * fingers[1]:+.1f} in_grip {1000 * off[0]:+.1f} {1000 * off[1]:+.1f} {1000 * off[2]:+.1f} "
        f"askew roll {askew[0]:+.1f} pitch {askew[1]:+.1f} yaw {askew[2]:+.1f} "
        f"L {1000 * lf[0]:+.1f} {1000 * lf[1]:+.1f} jars_mm/deg {moved} | {_labels.summary()} | score {score(read['jars']):.2f}",
        flush=True,
    )


# -- scoring -----------------------------------------------------------------------------------------------------------
def _standing(jar) -> bool:
    return jar.tilt_deg() < 10.0 and abs(float(jar.pos[2]) - (TOP + JAR_H / 2)) < 0.005


def priced(objects):
    """The bottles that stand upright on the table with exactly one label, whole and flat, on the cap's top."""
    return [n for n in JAR_ORDER if f"jar_{n}" in objects and _standing(objects[f"jar_{n}"]) and _labels.good(f"jar_{n}")]


def score(objects):
    """0.25 for each priced bottle (0.75 for all three). 1.0: all three priced, and also no stray label (none on the
    table, no second label on a cap, at most one unused label waiting at the gun's nose), the gun resting on the
    table and let go, every fingertip 3 cm or more from the gun and from every bottle, and nothing moving for 1 s."""
    n = len(priced(objects))
    if n < 3:
        return 0.25 * n
    tidy = not _labels.on("table") and _labels.hanging <= 1 and len(_labels.applied) == 3
    gun_down = "gun" not in objects or float(objects["gun"].pos[2]) - TOP < 0.06
    if tidy and gun_down and _state["gun_free"] and _state["jars_free"] and _state["still"]:
        return 1.0
    return 0.75


def success(objects):
    return score(objects) == 1.0


# -- scripted demo -----------------------------------------------------------------------------------------------------
class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


_down_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)  # yam_arena's own: straight down, turned by yaw
if not hasattr(_demo.grip_rotation, "_plain"):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)  # any waypoint yaw that carries a rotation, whichever module made it
        return _down_grip(yaw) if rot is None else rot

    _grip_rotation._plain = _down_grip
    _demo.grip_rotation = _grip_rotation


def _turn(axis, deg):
    return axis_angle_matrix(np.array(axis, float), math.radians(deg))


DOWN = _down_grip(0.0)  # straight down, fingers closing along world y: one behind the handle, one in front of the trigger
LEVEL = _turn((0, 1, 0), -90.0) @ DOWN  # level, pointing forward (+x): the gun upright, nose to the left (+y)
TILT = 45.0  # degrees the nose points down while labelling: then only the roller is below the caps' tops


def level(nose_down_deg: float):
    """The level grip with the wrist rolled so that the gun's nose points down by this many degrees."""
    return _turn((1, 0, 0), -nose_down_deg) @ LEVEL


def tcp_for_roller(roller, rot, grip):
    """Where the TCP must be for the roller's centre to be at ``roller``, with the gun held at ``grip`` in this rotation."""
    fwd, left, up = -rot[:, 0], -rot[:, 2], -rot[:, 1]
    rel = np.array(ROLLER_G) - np.asarray(grip)
    return np.asarray(roller, float) - (fwd * rel[0] + left * rel[1] + up * rel[2])


# The arm's drives are soft (80 N m/rad at joints 1-3, 10 N m/rad at joints 4-6) and compensate only the arm's own
# weight, so the gun's 4 N pulls a loaded pose down by up to 4 cm and 6 degrees. The demo commands each loaded pose
# offset by the deflection a static model predicts.
BASE_R, BASE_L = np.array(station.BASES["right"], float), np.array(station.BASES["left"], float)
ARM_K = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3, float)
GUN_WEIGHT = 9.81 * GUN_MASS


def _tcp_offset(g, grip):
    """A point of the gun frame as an offset from the TCP, in the TCP frame, when the gun is held at ``grip``."""
    v = np.asarray(g, float) - np.asarray(grip, float)
    return np.array([-v[0], -v[2], -v[1]])


def _point_jacobian(q, off):
    jac = np.zeros((3, 6))
    for i in range(6):
        d = np.zeros(6)
        d[i] = 1e-6
        pa, ra = ik.tcp_pose(q + d, BASE_R)
        pb, rb = ik.tcp_pose(q - d, BASE_R)
        jac[:, i] = ((pa + ra @ off) - (pb + rb @ off)) / 2e-6
    return jac


def _sag(q, grip, push=None):
    """Joint deflection (rad) at arm pose ``q`` under the gun's weight and, if given, the push of a surface on the roller
    (a world-frame force, N)."""
    torque = _point_jacobian(q, _tcp_offset(GUN_COM_G, grip)).T @ np.array([0.0, 0.0, -GUN_WEIGHT])
    if push is not None:
        torque = torque + _point_jacobian(q, _tcp_offset(ROLLER_G, grip)).T @ np.asarray(push, float)
    return torque / ARM_K


class _Carry:
    """Turns a wanted TCP pose of the loaded right arm into the pose to command."""

    def __init__(self, seed):
        self.q = np.array(seed, float)
        self.trace = []  # (wanted joints, commanded joints), for the offline check

    def __call__(self, pos, rot, grip, push=None):
        q, _, _ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), self.q, base_pos=BASE_R, iters=300)
        self.q = q
        cmd = np.clip(q - _sag(q, grip, push), ik.LOWER, ik.UPPER)
        p, r = ik.tcp_pose(cmd, BASE_R)
        self.trace.append((q, cmd))
        return p, Orient(r)


LEVEL_SEED = (0.5, 0.5, 0.29, 0.21, 0.5, 0.0)  # the arm's posture in the level grip
CLEAR = 0.030  # m: the roller's underside this far above a cap while feeding a label and moving to the next bottle
HIGH = 0.070  # m: ... and while crossing the row to the first bottle
PRESS_N = 1.3  # N: the push of the cap on the roller that the touch-down and wipe poses are worked out for. Pressing
#   bends the soft arm (15 mm and 2 to 3 degrees per newton at the roller), so those poses allow for it and the roller sits flat.
PUSH = np.array([0.0, 0.0, PRESS_N])
WIPE = 0.020  # m: dragged back towards the handle; the soft arm lags a few mm, and the label is 11 mm long
GRIP_START = gun_start_world(GRIP_HOLD)
ABOVE = GRIP_START + np.array([0.0, 0.0, 0.12])
LIFTED = GRIP_START + np.array([0.0, 0.0, 0.13])
SET_DOWN = GRIP_START + np.array([0.0, 0.0, 0.004])
HOVER = np.array([0.27, -0.25, TOP + 0.25])  # where the gun is turned upright (TCP): in front of the right arm, clear of the bottles

# The left arm holds each bottle from the robots' side: 30 degrees round to the left and pointing 15 degrees below
# level, so its wrist stays clear of the right gripper and its palm stays below the caps.
LEFT_GRIP = _turn((0, 0, 1), -30.0) @ _turn((0, 1, 0), -75.0) @ DOWN
LEFT_REACH = LEFT_GRIP[:, 2]  # the direction the left gripper points
HOLD_Z = TOP + 0.055
FINGER_JAR = -(2 * JAR_R - 0.003) / 2  # pads commanded 3 mm inside the bottle: about 7.5 N
PARK_L = (np.array([0.25, 0.30, TOP + 0.20]), Orient(LEFT_GRIP))


def _left_hold(name):
    at = np.array([JAR_X, JAR_Y[name], HOLD_Z])
    at[:2] -= 0.003 * LEFT_REACH[:2] / np.linalg.norm(LEFT_REACH[:2])  # the pads' flats reach the bottle's widest line
    return at


def demo():
    """Right arm: pick the gun up by handle and trigger (stroke 1), turn it upright in front of the arm, carry it over
    the first bottle; at each bottle let the trigger out (a label is fed), touch the roller down on the cap, wipe
    20 mm back, lift, squeeze again; lay the gun back on its mark. Left arm: hold each bottle by its sides while it is
    labelled."""
    right, left = [], []
    carry = _Carry(READY)
    cur = {"r": None, "l": None}

    def step(t, label, r=None, l=None):
        if r is not None:
            cur["r"] = r
        if l is not None:
            cur["l"] = l
        for arm, plan in (("r", right), ("l", left)):
            pos, finger, rot = cur[arm]
            plan.append(Waypoint(float(t), tuple(float(v) for v in pos), float(finger), label, rot))

    def held_tcp(tcp, tilt, grip, finger, push=None):  # the loaded arm in the level grip, by where the TCP should be
        pos, orient = carry(tcp, level(tilt), grip, push)
        return pos, finger, orient

    def held(roller, tilt, grip, finger, push=None):  # ... by where the roller's centre should be
        return held_tcp(tcp_for_roller(roller, level(tilt), grip), tilt, grip, finger, push)

    def lying(pos, finger):  # the loaded arm in the straight-down grip
        p, orient = carry(pos, DOWN, GRIP_SQ)
        return p, finger, orient

    back = -0.06 * LEFT_REACH
    over = np.array([0.0, 0.0, 0.125])  # the open fingers cross from bottle to bottle above the caps
    first = _left_hold(JAR_ORDER[0])
    lg = Orient(LEFT_GRIP)
    step(2.5, "over the gun's handle", r=(ABOVE, FINGER_OPEN, 0.0), l=(PARK_L[0], FINGER_OPEN, lg))
    step(1.5, "fingers round the handle and the trigger", r=(GRIP_START, FINGER_OPEN, 0.0), l=(first + back, FINGER_OPEN, lg))
    step(1.0, "close on the handle and the trigger", r=(GRIP_START, FINGER_HOLD, 0.0), l=(first, FINGER_OPEN, lg))
    step(1.0, "squeeze: stroke 1", r=(GRIP_START, FINGER_CARRY, 0.0), l=(first, FINGER_JAR, lg))
    step(2.0, "lift the gun", r=lying(LIFTED, FINGER_CARRY))
    carry.q = np.array(LEVEL_SEED)
    step(3.5, "turn the gun upright", r=held_tcp(HOVER, 0.0, GRIP_SQ, FINGER_CARRY))
    step(1.5, "tilt its nose down", r=held_tcp(HOVER, TILT, GRIP_SQ, FINGER_CARRY))
    for i, name in enumerate(JAR_ORDER):
        touch_xy = np.array([JAR_X, JAR_Y[name] + LABEL_L / 2])  # so the label ends centred on the cap
        up = np.array([*touch_xy, LID_Z + ROLLER_R + CLEAR])
        down = np.array([*touch_xy, LID_Z + ROLLER_R])  # resting on the cap
        wiped = np.array([0.0, WIPE, 0.0])
        if i == 0:
            step(2.5, "carry the gun over the first bottle", r=held(up + np.array([0.0, 0.0, HIGH - CLEAR]), TILT, GRIP_SQ, FINGER_CARRY))
            step(1.2, "down to just above the cap", r=held(up, TILT, GRIP_SQ, FINGER_CARRY))
            step(0.8, "steady", r=held(up, TILT, GRIP_SQ, FINGER_CARRY))
        step(1.0, f"let the trigger out: label {i + 1} is fed", r=held(up, TILT, GRIP_HOLD, FINGER_HOLD))
        step(0.8, "steady", r=held(up, TILT, GRIP_HOLD, FINGER_HOLD))
        step(1.2, "touch the roller down on the cap", r=held(down, TILT, GRIP_HOLD, FINGER_HOLD, PUSH))
        step(1.6, "wipe the label on", r=held(down - wiped, TILT, GRIP_HOLD, FINGER_HOLD, PUSH))
        step(0.8, "lift off", r=held(up - wiped, TILT, GRIP_HOLD, FINGER_HOLD))
        step(0.8, f"squeeze: stroke {i + 2}", r=held(up - wiped, TILT, GRIP_SQ, FINGER_SQUEEZE))
        here = _left_hold(name)
        step(0.8, "let the bottle go", l=(here, FINGER_OPEN, lg))
        step(0.9, "draw the left hand back", l=(here + back, FINGER_OPEN, lg))
        if i + 1 < len(JAR_ORDER):
            nxt = JAR_ORDER[i + 1]
            there = _left_hold(nxt)
            nxt_up = np.array([JAR_X, JAR_Y[nxt] + LABEL_L / 2, LID_Z + ROLLER_R + CLEAR])
            step(0.9, "lift the left hand over the bottles", l=(here + back + over, FINGER_OPEN, lg))
            step(1.8, "on to the next bottle", r=held(nxt_up, TILT, GRIP_SQ, FINGER_SQUEEZE), l=(there + back + over, FINGER_OPEN, lg))
            step(0.9, "left hand down beside the bottle", l=(there + back, FINGER_OPEN, lg))
            step(0.9, "left hand round the bottle", l=(there, FINGER_OPEN, lg))
            step(0.8, "hold the bottle", l=(there, FINGER_JAR, lg))
    away = np.array([0.30, -0.26, LID_Z + ROLLER_R + 0.06])
    step(1.2, "lift the left hand clear", r=held(away, TILT, GRIP_SQ, FINGER_SQUEEZE), l=(cur["l"][0] + over, FINGER_OPEN, lg))
    step(1.5, "left hand back to its side; grip the gun tight", r=held(away, TILT, GRIP_SQ, FINGER_CARRY), l=(PARK_L[0], FINGER_OPEN, lg))
    carry.q = np.array(READY)
    step(3.5, "turn the gun onto its side, over its mark", r=lying(LIFTED, FINGER_CARRY))
    step(2.0, "lower it", r=lying(SET_DOWN, FINGER_CARRY))
    step(0.8, "let go: an unused label is fed and stays at the nose", r=(SET_DOWN, FINGER_OPEN, 0.0))
    step(1.5, "clear", r=(ABOVE, FINGER_OPEN, 0.0))
    demo.trace = carry.trace
    return {"left": left, "right": right}
