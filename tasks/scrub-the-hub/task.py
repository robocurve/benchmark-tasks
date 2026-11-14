"""An IV flush on a training arm. The patient's forearm (skin-colored, about 7 cm thick and 32 cm long, tapering to the
wrist) lies on a dark padded armrest, pointing at the robot. A peripheral IV catheter is taped in the forearm under a
clear film dressing. A short clear extension line runs from it around the arm's right side and across the armrest to a
needleless connector (a clear valve with a blue cap ring) that lies on the bench beside the armrest, taped down, its
open end pointing to the robot's right. On the right, a light-blue tray holds an opened large alcohol prep pad, folded,
on its torn foil packet, and a primed 10 mL saline flush syringe (luer tip; the sim models no fluid), plunger at the
10 mL mark, lying with its tip toward the connector."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props below; nothing on disk changes)
import guardkit as gk  # noqa: E402  (the gripper's own shape, for the grip on the pad)
import flushkit as fk  # noqa: E402  (the saline: volumes, the flow's resistance on the plunger, and its drawing)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
import yam_arena.demo as D  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Flush the IV line on the training arm."

# Armrest and forearm (fixed). The forearm collides as a box whose top is the top of the arm.
REST_X, REST_L, REST_W, REST_H = 0.29, 0.36, 0.12, 0.01
ARM_X, ARM_L, ARM_R = 0.29, 0.32, 0.035
ARM_Z = TOP + REST_H + ARM_R
ARM_TOP = ARM_Z + ARM_R
CATH = (0.31, -0.012)  # where the catheter enters the skin

# The needleless connector lies on the bench along y, its open end (the hub) facing -y.
AXIS_Z = TOP + 0.009  # the connector's and the syringe's common axis height
HUB_X, HUB_BACK_Y, HUB_TIP_Y, HUB_R = 0.27, -0.074, -0.100, 0.0055
HUB = np.array([HUB_X, HUB_TIP_Y, AXIS_Z])  # the hub's open end: the surface that must be scrubbed

PAD = (0.035, 0.065, 0.010)  # the folded large prep pad, lying flat: 3.5 cm wide, 6.5 cm long, 1 cm thick
TRAY_X, TRAY_Y, TRAY_H = 0.37, -0.20, 0.008
TRAY_SIZE = (0.15, 0.15, TRAY_H)
PAD_START = (0.335, -0.20, TOP + TRAY_H + 0.002 + PAD[2] / 2)  # on its foil packet
# The syringe lies along y, its tip at +y. Its frame's origin is the middle of the barrel-and-tip body; y in that frame:
SYR_L, SYR_W = 0.080, 0.018  # barrel (with its finger flange) plus luer tip: 8 cm long, 1.8 cm across
BARREL_BACK, BARREL_FRONT = -SYR_L / 2, 0.028  # the flange end and the barrel's front shoulder
TIP_W = 0.006  # the luer tip, 6 mm across, from the shoulder to the tip end at +SYR_L / 2
SYR_START = (0.405, -0.165, TOP + TRAY_H + SYR_W / 2)

SCRUB_TILT = 20.0  # degrees; a face of the pad lies within this of the access face (flat on it, not a corner or edge)
FACE_GAP = 0.0005  # the pad's face within 0.5 mm of the access face ...
FACE_PRESS = 0.003  # ... or pressed up to 3 mm into it (the colliders give a little under a push): in contact
SCRUB_SPEED = 0.010  # the pad rubbing across the face at 1 cm/s or more: a scrub, not a pad held still on it
PAD_SHAPE = gk.Block((0.0, 0.0, 0.0), PAD)  # the pad's collider, in its own frame
PAD_GRIP = gk.band(PAD[0])  # a gripper holds the pad when both its fingers touch it (gk.gripping: within 2 mm, the
# solver's resting gap plus the fit of the hand's boxes to its meshes) with their opening stopped by the pad's 3.5 cm
# width: 0.22 to 0.40 of full opening (0 closed, 1 open, 9.4 cm). A pad pushed with one finger or the palm, an open
# hand resting on it or an empty fist beside it does not hold it.
SCRUB_STEPS = 300  # 15 s at 20 Hz, counted over the whole run
TOUCH = 0.002  # the syringe's barrel or luer tip within 2 mm of the access face (a 1.1 cm disc) touches it
SEAT_GAP = 0.0005  # seated: the luer tip's end touching the access face, from 0.5 mm short of it (the solver's
#                   resting gap between two props) ...
SEAT_PRESS = 0.003  # ... to pressed 3 mm into it (the give of the colliders under a push), the same contact band as the
#                     pad's; a tip hovering off the face, however close, is not seated ...
SEAT_RADIAL = 0.0015  # ... within 1.5 mm of the connector's axis, so the 6 mm tip sits on the port's middle ...
SEAT_TILT = 5.0  # ... its barrel within 5 degrees of the connector's axis ...
SEAT_STEPS = 10  # ... for 0.5 s in a row
PLUNGER_ANCHOR = -0.102  # the thumb press's center on the syringe's axis at the start (pulled back to 10 mL)
STROKE = 0.060  # the plunger's travel from 10 mL to empty
FLUSH_GONE = 5.0  # mL gone from the barrel (net), the usual peripheral flush: 3 cm of this syringe's 6 cm stroke ...
FLUSH_IN = 4.5  # ... and mL collected in the graduated cylinder, the real trial's rule: the 0.5 mL margin covers the
#                 reading (0.25 mL) and drops left in the tube, and caps what may run out at the hub
CYL_X, CYL_Y = 0.36, 0.12  # the 25 mL graduated cylinder the training arm's vein drains into, on the bench at the left
CYL_R, CYL_WALL, CYL_H, CYL_BASE = 0.0085, 0.0015, 0.14, 0.008  # inside radius, wall, tube height, base thickness

# -- looks and models -------------------------------------------------------------------------

SKIN = kit.Look((0.87, 0.68, 0.55), 0.65)
REST_LOOK = kit.Look((0.16, 0.18, 0.24), 0.9)
CLEAR = kit.Look((0.88, 0.92, 0.96), 0.15, opacity=0.45)
FILM = kit.Look((0.95, 0.96, 0.97), 0.2, opacity=0.35)
TUBE = kit.Look((0.90, 0.93, 0.95), 0.2, opacity=0.6)
CATH_PINK = kit.Look((0.95, 0.55, 0.70), 0.4)
BLUE = kit.Look((0.15, 0.40, 0.85), 0.4)
WHITE = kit.Look((0.96, 0.96, 0.95), 0.6)
TAPE = kit.Look((0.92, 0.91, 0.86), 0.9)
PAD_LOOK = kit.Look((0.98, 0.98, 0.97), 0.95)
FOIL = kit.Look((0.82, 0.82, 0.86), 0.25, metallic=0.8)
FOIL_PRINT = kit.Look((0.75, 0.10, 0.25), 0.5)
TRAY_LOOK = kit.Look((0.62, 0.76, 0.90), 0.5)
BARREL = kit.Look((0.94, 0.96, 0.98), 0.15, opacity=0.5)
MARK = kit.Look((0.15, 0.15, 0.18), 0.6)
PLUNGER = kit.Look((0.97, 0.97, 0.97), 0.5)
STOPPER = kit.Look((0.18, 0.18, 0.20), 0.6)
SALINE = kit.Look((0.20, 0.50, 0.95), 0.1)  # the clear saline in the cylinder, drawn blue so its level shows


def _tube(path, radius, look, n=10):
    """A round tube of `radius` along the points of `path`, with closed ends."""
    path = np.asarray(path, float)
    pts, counts, idx = [], [], []
    u = np.array([0.0, 0.0, 1.0])
    for i, p in enumerate(path):
        tan = path[min(i + 1, len(path) - 1)] - path[max(i - 1, 0)]
        tan /= np.linalg.norm(tan)
        u = u - np.dot(u, tan) * tan  # carried along the path, so the tube doesn't twist
        u /= np.linalg.norm(u)
        v = np.cross(tan, u)
        pts += [p + radius * (math.cos(2 * math.pi * k / n) * u + math.sin(2 * math.pi * k / n) * v) for k in range(n)]
    for i in range(len(path) - 1):
        for k in range(n):
            a, b = i * n + k, i * n + (k + 1) % n
            counts.append(4)
            idx += [a, b, b + n, a + n]
    for end, flip in ((0, True), (len(path) - 1, False)):
        ring = [end * n + k for k in range(n)]
        counts.append(n)
        idx += ring[::-1] if flip else ring
    return kit._mesh(pts, counts, idx, look)


def _forearm():
    """The training arm with the IV site: catheter hub and wings under a clear film, and the extension line wrapping
    down the arm's right side, onto the armrest and to the connector's back end."""
    profile = [(0.0, -0.165), (0.022, -0.163), (0.027, -0.155), (0.030, -0.10), (0.034, -0.02), (0.036, 0.06),
               (0.037, 0.15), (0.033, 0.16), (0.0, 0.162)]
    cx, cy = CATH[0] - ARM_X, CATH[1]
    top = ARM_R + 0.001
    tube_r = 0.002  # the extension line is 4 mm across
    # The line lies on the skin from the catheter's hub around the arm's right side (it never passes through the
    # arm), drops onto the armrest, then runs straight along y into the connector's back end.
    radius = lambda x: float(np.interp(x, [h for _, h in profile], [r for r, _ in profile]))  # noqa: E731
    line = [(cx - 0.013, cy, top + 0.004)]
    for k in range(9):  # 20 to 110 degrees around from the top toward the robot's right (-y), on the skin
        s = k / 8
        x = cx - 0.016 - 0.020 * s
        a = math.radians(20 + 90 * s)
        rho = radius(x) + tube_r + 0.0008
        line.append((x, -rho * math.sin(a), rho * math.cos(a)))
    rest = -ARM_R + tube_r  # the line's center lying on the armrest
    line += [(cx - 0.039, -0.042, -0.024), (cx - 0.040, -0.047, rest), (HUB_X - ARM_X, -0.056, rest),
             (HUB_X - ARM_X, -0.063, rest - 0.0005), (HUB_X - ARM_X, HUB_BACK_Y + 0.002, AXIS_Z - ARM_Z)]
    meshes = (kit.lathe(profile, SKIN, axis="x", n=48),
              kit.box((cx, cy, top + 0.0005), (0.045, 0.035, 0.001), FILM),
              kit.cylinder((cx - 0.006, cy, top + 0.004), 0.004, 0.014, CATH_PINK, axis="x"),
              kit.box((cx - 0.004, cy, top + 0.0015), (0.008, 0.022, 0.002), CATH_PINK),
              _tube(line, tube_r, TUBE))
    return kit.Prop("forearm", pos=(ARM_X, 0.0, ARM_Z), size=(ARM_L, 2 * ARM_R, 2 * ARM_R), meshes=meshes,
                    colliders=(kit.Collider((0, 0, 0), size=(ARM_L, 2 * ARM_R, 2 * ARM_R)),))


def _connector():
    """A needleless connector: clear valve body, blue collar, white luer end; taped to the bench."""
    length = HUB_BACK_Y - HUB_TIP_Y
    yc = (HUB_BACK_Y + HUB_TIP_Y) / 2
    body = [(0.0, -length / 2), (0.003, -length / 2), (0.0045, -0.008), (HUB_R, -0.004), (HUB_R, 0.006),
            (0.0045, 0.009), (0.004, length / 2), (0.0, length / 2)]
    tape_z = -(AXIS_Z - TOP) + 0.0005
    return kit.Prop("connector", pos=(HUB_X, yc, AXIS_Z), size=(0.012, length, 0.012),
                    meshes=(kit.lathe(body, CLEAR, axis=(0, -1, 0), n=32),
                            kit.cylinder((0, -0.004, 0), HUB_R + 0.0008, 0.005, BLUE, axis="y"),
                            kit.cylinder((0, -length / 2 + 0.002, 0), 0.0042, 0.004, WHITE, axis="y"),
                            kit.box((0, 0.008, tape_z), (0.022, 0.012, 0.001), TAPE)),
                    colliders=(kit.Collider((0, 0, 0), size=(0.011, length, 0.011)),))


def _pad():
    """A large alcohol prep pad folded into a soft rectangle."""
    w, l, h = PAD
    return kit.Prop("pad", pos=PAD_START, size=PAD, mass=0.025,
                    meshes=(kit.box((0, 0, -h / 4), (w, l, h / 2), PAD_LOOK),
                            kit.box((0.001, -0.001, h / 4), (w - 0.003, l - 0.002, h / 2), PAD_LOOK)),
                    colliders=(kit.Collider((0, 0, 0), size=PAD),))


def _free_root(text):
    """The syringe is loose: its articulation root goes on its barrel body (PhysX otherwise roots a floating
    articulation at its most central body, and the plunger's slide misbehaves). The plunger's slide also gets a PhysX
    drive, so its damping (the flow's resistance, see flushkit) is applied by the solver, not once per step."""
    root = 'def Xform "Root" (\n    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n)\n{\n'
    lines = text.replace(root, 'def Xform "Root"\n{\n', 1).splitlines(True)
    settings = [ln for ln in lines if ln.startswith("    ") and not ln.startswith("     ") and "physxArticulation:" in ln]
    text = "".join(ln for ln in lines if ln not in settings)
    old = ('    def Xform "base" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", '
           '"PhysxRigidBodyAPI"]\n    )\n    {\n')
    new = old.replace('"PhysxRigidBodyAPI"]', '"PhysxRigidBodyAPI", "PhysicsArticulationRootAPI", "PhysxArticulationAPI"]')
    joint = '"PhysicsJointStateAPI:linear"]\n    )\n    {\n'
    drive = ('"PhysicsJointStateAPI:linear", "PhysicsDriveAPI:linear"]\n    )\n    {\n'
             '        uniform token drive:linear:physics:type = "force"\n'
             "        float drive:linear:physics:maxForce = 1000\n"
             "        float drive:linear:physics:stiffness = 0\n"
             "        float drive:linear:physics:damping = 0\n"
             "        float drive:linear:physics:targetPosition = 0\n")
    has_drive = '"PhysicsDriveAPI:linear"]' in text  # a slide with a law already has its drive (yamkit)
    if old not in text or (joint not in text and not has_drive):
        raise RuntimeError("scrub-the-hub: unexpected yamkit USD layout")
    text = text.replace(old, new + "".join("    " + ln for ln in settings), 1)
    return text if has_drive else text.replace(joint, drive, 1)


_mech_usda = kit._mech_usda
kit._mech_usda = lambda mech: _free_root(_mech_usda(mech)) if mech.name == "syringe" else _mech_usda(mech)


def _syringe():
    """The primed 10 mL saline flush syringe: clear barrel with mL marks and a white finger flange, luer tip at +y;
    the white plunger (thumb press, cross-shaped rod, dark rubber stopper) slides 6 cm on its axis and stays where it
    is pushed. Pushing it meets the saline's flow resistance and the stopper's glide (flushkit.Flow)."""
    r = SYR_W / 2 - 0.001
    blen = BARREL_FRONT - BARREL_BACK
    stopper = 0.0675  # the stopper's middle, from the thumb press
    marks = tuple(kit.box((0, BARREL_FRONT - 0.003 - 0.006 * i, r + 0.0001), (0.006 if i % 2 else 0.009, 0.0006, 0.0003),
                          MARK) for i in range(11))  # 0 to 10 mL, every 6 mm
    plunger = kit.Link(
        "plunger", "prismatic", "Y", anchor=(0, PLUNGER_ANCHOR, 0), lower=0.0, upper=STROKE, mass=0.008,
        meshes=(kit.cylinder((0, 0, 0), 0.0082, 0.003, PLUNGER, axis="y"),
                kit.box((0, stopper / 2, 0), (0.010, stopper, 0.0015), PLUNGER),
                kit.box((0, stopper / 2, 0), (0.0015, stopper, 0.010), PLUNGER),
                kit.cylinder((0, stopper, 0), r - 0.0012, 0.005, STOPPER, axis="y")),
        colliders=(kit.Collider((0, 0, 0), size=(0.016, 0.003, 0.016)),
                   kit.Collider((0, 0.033, 0), size=(0.010, 0.060, 0.010))),
        damping=5.0, stick=True, law=fk.Flow())
    return kit.Mechanism(
        "syringe", pos=SYR_START, size=(SYR_W, SYR_L, SYR_W), mass=0.02,
        meshes=(kit.cylinder((0, (BARREL_FRONT + BARREL_BACK) / 2, 0), r, blen, BARREL, axis="y"),
                kit.lathe([(r, 0), (0.003, 0.004), (TIP_W / 2, 0.006), (TIP_W / 2 - 0.001, SYR_L / 2 - BARREL_FRONT),
                           (0.0, SYR_L / 2 - BARREL_FRONT)], BARREL, center=(0, BARREL_FRONT, 0), axis="y"),
                kit.box((0, BARREL_BACK + 0.00125, 0), (0.028, 0.0025, 0.014), WHITE)) + marks,
        colliders=(kit.Collider((0, (BARREL_FRONT + BARREL_BACK) / 2, 0), size=(SYR_W, blen, SYR_W)),
                   kit.Collider((0, (SYR_L / 2 + BARREL_FRONT) / 2, 0), size=(TIP_W, SYR_L / 2 - BARREL_FRONT, TIP_W))),
        links=(plunger,))


def _tray():
    foil = kit.box((PAD_START[0] - TRAY_X, 0, TRAY_H / 2 + 0.001), (0.05, 0.06, 0.002), FOIL)
    foil_print = kit.box((PAD_START[0] - TRAY_X, 0.022, TRAY_H / 2 + 0.0021), (0.04, 0.012, 0.0002), FOIL_PRINT)
    return kit.Prop("tray", pos=(TRAY_X, TRAY_Y, TOP + TRAY_H / 2), size=TRAY_SIZE,
                    meshes=(kit.box((0, 0, 0), TRAY_SIZE, TRAY_LOOK), foil, foil_print),
                    colliders=(kit.Collider((0, 0, 0), size=TRAY_SIZE),
                               kit.Collider((PAD_START[0] - TRAY_X, 0, TRAY_H / 2 + 0.001), size=(0.05, 0.06, 0.002))))


def _cylinder():
    """The 25 mL graduated cylinder (clear, marks every 5 mL on the side facing the robot) on a white plastic base, with
    the training arm's outflow tube running from the arm's left side across the bench and down into its mouth. Its
    frame's origin is the middle of its height. The saline collected in it is a blue column on a slide (no collider,
    no gravity) whose top follows the collected volume (flushkit.level): at 0 mL the column sits below the inside
    floor, hidden in the base and under the bench."""
    ro = CYL_R + CYL_WALL
    top = CYL_BASE + CYL_H
    z0 = -top / 2  # the bench, in the cylinder's frame
    floor = z0 + CYL_BASE  # the inside floor: the top of the base
    per_ml = 1e-6 / (math.pi * CYL_R ** 2)  # m of height per mL
    marks = tuple(kit.box((-ro - 0.0002, 0, floor + 5 * k * per_ml), (0.0004, 0.006 if k % 2 else 0.009, 0.0006),
                          MARK) for k in range(6))
    outflow = [(ARM_X + 0.11 - CYL_X, ARM_R + 0.001 - CYL_Y, ARM_Z - TOP + z0),
               (ARM_X + 0.10 - CYL_X, REST_W / 2 + 0.006 - CYL_Y, z0 + 0.004), (0.02, -0.035, z0 + 0.003),
               (0.007, -0.02, z0 + 0.04), (0.003, -0.015, -z0 - 0.02), (0.0, -0.006, -z0 + 0.012),
               (0.0, 0.0, -z0 - 0.015)]
    meshes = (kit.cylinder((0, 0, z0 + CYL_BASE / 2), 0.022, CYL_BASE, WHITE),
              kit.lathe([(ro, 0.0), (ro, CYL_H), (CYL_R, CYL_H), (CYL_R, 0.0), (0.0, 0.0)], CLEAR,
                        center=(0, 0, floor), axis="z"),
              _tube(outflow, 0.0015, TUBE)) + marks
    column = 0.10  # m: the column's length (10 mL stands 4.4 cm)
    saline = kit.Link(
        "saline", "prismatic", "Z", anchor=(0, 0, floor - 0.0005), lower=0.0, upper=0.05, mass=0.001,
        meshes=(kit.cylinder((0, 0, -column / 2), CYL_R - 0.0004, column, SALINE),), floating=True,
        follow=lambda qs: min(0.05, fk.level(math.pi * CYL_R ** 2)), follow_rate=0.01)
    return kit.Mechanism("cylinder", pos=(CYL_X, CYL_Y, TOP + top / 2), size=(0.044, 0.044, top), mass=None,
                         meshes=meshes,
                         colliders=(kit.Collider((0, 0, z0 + CYL_BASE / 2), size=(0.044, 0.044, CYL_BASE)),
                                    kit.Collider((0, 0, floor + CYL_H / 2), size=(2 * ro, 2 * ro, CYL_H))),
                         links=(saline,))


OBJECTS = [
    kit.Prop("armrest", pos=(REST_X, 0.0, TOP + REST_H / 2), size=(REST_L, REST_W, REST_H),
             meshes=(kit.box((0, 0, 0), (REST_L, REST_W, REST_H), REST_LOOK),),
             colliders=(kit.Collider((0, 0, 0), size=(REST_L, REST_W, REST_H)),)),
    _forearm(),
    _connector(),
    _tray(),
    _pad(),
    _syringe(),
    _cylinder(),
]


def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _syringe_tip(s):
    """The syringe's tip end (the end of its luer, where it meets the connector) and its long axis."""
    r = _rot(np.asarray(s.quat_xyzw, dtype=float))
    axis = r @ np.array([0.0, 1.0, 0.0])
    return np.asarray(s.pos, dtype=float) + axis * SYR_L / 2, axis


def _face_offset(p):
    """The access face's center relative to a point, as (axial, radial): axial is along the connector's axis (+y,
    into the connector), radial is the distance from the axis in the face's plane."""
    d = HUB - np.asarray(p, dtype=float)
    return float(-d[1]), float(math.hypot(d[0], d[2]))


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()  # which trial this history belongs to
        self.scrub = 0  # control steps the pad was scrubbing the access face
        self.touched_unscrubbed = False  # the syringe touched the face before 15 s of scrubbing; never forgiven
        self.seat_run = 0  # control steps in a row the syringe has been seated
        self.connected = False  # seated for SEAT_STEPS in a row after the scrub; latched
        self.last_depth = None  # the plunger's depth at the last step
        self.vol = fk.fresh()  # mL: saline in the barrel, collected in the cylinder, spilled at the tip, gone (net)
        self.flush_done = False  # 5 mL gone and 4.5 mL collected, after the connection; latched


_H = _History()


def _held(objects):
    """Some gripper holds the pad: both fingers touching it, their opening stopped by its 3.5 cm width."""
    return bool(gk.gripping(objects, "pad", PAD_SHAPE, PAD_GRIP))


def _scrubbing(objects):
    """A scrub step: a gripper holds the pad; one of the pad's faces lies on the access face (square to the
    connector's axis within 20 degrees, from 0.5 mm short of the face to pressed 3 mm into it) and covers the whole
    1.1 cm face; and the pad rubs across it (its speed across the face, sliding plus twisting at the face's rim, is
    1 cm/s or more). Rubbing the connector's side or top, or holding the pad still on the face, is not a scrub."""
    p = objects["pad"]
    r = _rot(np.asarray(p.quat_xyzw, dtype=float))
    half = np.array(PAD) / 2
    # which of the pad's three axes points along the connector's axis (world y)
    k = int(np.argmax(np.abs(r[1, :])))
    if abs(r[1, k]) < math.cos(math.radians(SCRUB_TILT)):
        return False
    # How far the pad's face reaches past the access face where they meet: the face's plane, measured along its
    # normal, at the point of the 1.1 cm disc it reaches first (the disc's middle when the pad is square to it, a point
    # on its rim when the pad leans). A far corner of a leaning pad passing beside the hub does not count.
    n = r[:, k] * math.copysign(1.0, r[1, k])  # the face's normal, toward the connector (+y)
    face = np.asarray(p.pos, dtype=float) + n * half[k]
    reach = float(n @ (face - HUB) + HUB_R * math.hypot(n[0], n[2]))
    if not (-FACE_GAP <= reach <= FACE_PRESS):
        return False
    # the face covers the whole access face: the face's center, in the pad's frame, at least HUB_R inside both edges
    loc = r.T @ (HUB - np.asarray(p.pos, dtype=float))
    others = [i for i in range(3) if i != k]
    if any(abs(loc[i]) > half[i] - HUB_R for i in others):
        return False
    v = np.asarray(p.lin_vel, dtype=float)
    w = np.asarray(p.ang_vel, dtype=float)
    rub = math.hypot(v[0], v[2]) + abs(w[1]) * HUB_R
    return bool(rub >= SCRUB_SPEED and _held(objects))


def _touching(s):
    """Any part of the syringe's front (its 1.8 cm barrel or its 6 mm luer tip) within 2 mm of the access face's
    1.1 cm disc."""
    r = _rot(np.asarray(s.quat_xyzw, dtype=float))
    loc = r.T @ (HUB - np.asarray(s.pos, dtype=float))  # the face's center in the syringe's frame
    face_r = np.array([HUB_R, 0.0, HUB_R])  # the face is a disc square to y: it reaches HUB_R across, not along y
    for center, size in (((BARREL_FRONT + BARREL_BACK) / 2, (SYR_W, BARREL_FRONT - BARREL_BACK, SYR_W)),
                         ((SYR_L / 2 + BARREL_FRONT) / 2, (TIP_W, SYR_L / 2 - BARREL_FRONT, TIP_W))):
        d = loc - np.array([0.0, center, 0.0])
        if np.all(np.abs(d) <= np.array(size) / 2 + TOUCH + face_r):
            return True
    return False


def _seated(s):
    """The syringe's luer tip touching the port's access face, centered and lined up: its end from 0.5 mm short of the
    face to pressed 3 mm into it (contact, not a hover), within 1.5 mm of the connector's axis, its barrel within 5
    degrees of the axis. The sim's connector is solid, so the tip pressed square on its port stands in for the luer
    slipped into the valve; a real trial checks the connection with a tug on the barrel."""
    tip, axis = _syringe_tip(s)
    axial, radial = _face_offset(tip)  # axial < 0: the tip end short of the face; > 0: pressed past it
    return bool(-SEAT_GAP <= axial <= SEAT_PRESS and radial <= SEAT_RADIAL
                and axis[1] >= math.cos(math.radians(SEAT_TILT)))


def _plunger_depth(objects):
    """How far the plunger has been pushed into the barrel since the start (m), along the syringe's axis; None if the
    scene has no plunger."""
    pl = objects.get("syringe/plunger")
    if pl is None:
        return None
    s = objects["syringe"]
    _, axis = _syringe_tip(s)
    return float(np.dot(np.asarray(pl.pos, dtype=float) - np.asarray(s.pos, dtype=float), axis) - PLUNGER_ANCHOR)


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    if _scrubbing(objects):
        _H.scrub += 1
    s = objects["syringe"]
    clean = _H.scrub >= SCRUB_STEPS
    if _touching(s) and not clean:
        _H.touched_unscrubbed = True
    seated = _seated(s)
    _H.seat_run = _H.seat_run + 1 if seated else 0
    if clean and not _H.touched_unscrubbed and _H.seat_run >= SEAT_STEPS:
        _H.connected = True
    depth = _plunger_depth(objects)
    if depth is not None:
        if _H.last_depth is not None:  # saline pushed through the seated tip enters the line, else it spills
            fk.account(_H.vol, depth - _H.last_depth, seated)
        _H.last_depth = depth
        v = _H.vol
        if (_H.connected and v["gone"] >= FLUSH_GONE and v["cylinder"] >= FLUSH_IN
                and v["gone"] - v["cylinder"] <= FLUSH_GONE - FLUSH_IN):  # at most 0.5 mL not collected
            _H.flush_done = True
    # what the plunger's law and the drawing read (flushkit): the valve open or not, and the volumes
    fk.STATE.update(episode=kit.episode(), seated=seated, **_H.vol)


def _score(objects):
    if _H.touched_unscrubbed or _H.scrub < SCRUB_STEPS:
        return 0.0
    if not _H.connected:
        return 0.25
    return 1.0 if _H.flush_done else 0.5


def success(objects):
    """1.0: the access face scrubbed for 15 s before the syringe touched it, the syringe seated, and the flush
    collected (5 mL gone from the barrel, 4.5 mL in the cylinder). Called first each step, so it also records the
    step's history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if the syringe's barrel or tip came within 2 mm of the access face before the face had been scrubbed for 15
    s in all, even if it was scrubbed later; 0 also if the face was never scrubbed for 15 s. Otherwise 0.25 for the
    scrub alone; 0.5 once the syringe was seated on the face (its luer tip touching it: the tip end from 0.5 mm
    short of the face to pressed 3 mm in, within 1.5 mm of the axis, lined up within 5 degrees) for 0.5 s in a row;
    1.0 once, after that, at least 5 mL had gone from the barrel (net) and at least 4.5 mL had been collected in the
    graduated cylinder, with no more than 0.5 mL of the difference lost (flushkit: saline pushed while the tip is
    seated enters the primed line and the same volume runs out into the cylinder; pushed unseated, it spills at the
    tip). A scrub step is a control step in which a gripper holds the pad (both fingers touching it, their opening
    stopped by its 3.5 cm width) with a face of it on the access face, covering it, and rubs it across the face at
    1 cm/s or more (see _scrubbing).
    Possible scores: 0, 0.25, 0.5, 1."""
    return _score(objects)


def demo():
    """Right arm: pick up the prep pad by one end, turn it on its edge and scrub the hub's open end face-on with small
    side-to-side strokes for 20 s, put the pad back on the tray, then pick up the syringe, lay it in line with the
    connector, push its tip home and let go, and push the plunger in about 3.4 cm with the closed fingers, slowly and in
    seven pulses with short pauses (the push-pause flush), at about 0.5 mL/s while pushing."""

    def grip(top, below=0.012):  # TCP height that puts the finger pads `below` an object's top
        return top - below + 0.013

    travel = TOP + 0.20
    lag = 0.012  # the arm falls about 1.2 cm short toward its own base (-y) when it pushes along y low on the bench
    o, c = FINGER_OPEN, FINGER_CLOSED
    yaw = np.pi / 2  # fingers close along x
    # The grip turned on its side, 84 degrees about x: the pad's underside faces the connector (+y), leaning 6 degrees
    # (the arm's wrist reaches that here with room to spare; 90 is at the end of its travel).
    roll = math.radians(84.0)
    pad_top = PAD_START[2] + PAD[2] / 2
    end = 0.020  # the pad is held this far from its middle (toward +y on the tray), so its far end reaches the hub
    pick_tcp = np.array([PAD_START[0], PAD_START[1] + end, grip(pad_top, PAD[2] / 2)])
    pick_pad = pick_place_waypoints(tuple(pick_tcp), (0.0, 0.0, 0.0), travel_z=travel, yaw=yaw)[:4]
    r0 = D.grip_rotation(yaw)
    r1 = kit._rot_xyz(roll, 0.0) @ r0
    local = r0.T @ (np.array(PAD_START) - pick_tcp)  # the pad's middle in the grip's frame
    turn = r1 @ r0.T  # the pad's turn from lying on the tray to held on its side
    n = turn @ np.array([0.0, 0.0, -1.0])  # its underside's normal, now toward the connector
    long_ = turn @ np.array([0.0, 1.0, 0.0])  # its long side, now pointing up (and leaning toward +y)
    press = 0.0008
    # the pad's middle: its low end 1.5 mm above the bench, its face pressed `press` past the disc's nearest rim point
    cz = TOP + 0.0015 + abs(long_[2]) * PAD[1] / 2 + abs(n[2]) * PAD[2] / 2
    mid = np.array([HUB_X, 0.0, cz])
    # n . (mid + n * h - HUB) + HUB_R * |n_xz| = press, solved for mid's y
    mid[1] = (press - HUB_R * math.hypot(n[0], n[2]) - PAD[2] / 2 - n[2] * (mid[2] - HUB[2])) / n[1] + HUB[1]
    # the arm stops about 6 mm short toward its base here; 3 mm toward -x centers the strokes on the hub
    tcp = mid - r1 @ local + np.array([-0.003, 0.006, 0.0])
    back = -0.04 * n
    stroke = 0.009  # the strokes reach about 7 mm each way: the pad still covers the 1.1 cm face
    scrub = [kit.Tilted(2.0, tuple(tcp + 2 * back + (0, 0, 0.06)), c, "turn the pad on its side", yaw, roll=roll),
             kit.Tilted(1.5, tuple(tcp + back), c, "line up with the hub", yaw, roll=roll),
             kit.Tilted(1.0, tuple(tcp), c, "pad on the access face", yaw, roll=roll)]
    for _ in range(20):
        scrub += [kit.Tilted(0.5, tuple(tcp + (stroke, 0, 0)), c, "scrub", yaw, roll=roll),
                  kit.Tilted(0.5, tuple(tcp - (stroke, 0, 0)), c, "scrub", yaw, roll=roll)]
    put_back = [kit.Tilted(1.0, tuple(tcp + 2 * back + (0, 0, 0.04)), c, "off the hub", yaw, roll=roll),
                Waypoint(1.5, (PAD_START[0], PAD_START[1] + end, travel - 0.08), c, "over the tray", yaw),
                Waypoint(1.0, (PAD_START[0], PAD_START[1] + end, grip(pad_top, PAD[2] / 2) + 0.01), c, "lower", yaw),
                Waypoint(0.5, (PAD_START[0], PAD_START[1] + end, grip(pad_top, PAD[2] / 2) + 0.01), o, "let go", yaw),
                Waypoint(0.8, (PAD_START[0], PAD_START[1] + end, travel), o, "up", yaw)]
    syr_top = SYR_START[2] + SYR_W / 2
    sy_seat = HUB_TIP_Y - SYR_L / 2  # syringe center when its tip meets the hub
    sx = HUB_X + 0.0028  # the syringe settles about 2.8 mm toward -x of the grip (its tip 0.8 to 1.6 mm toward -x of
    #                      the axis with 2 mm, against the seat's 1.5 mm)
    held = grip(TOP + SYR_W, 0.008)
    syringe = pick_place_waypoints((SYR_START[0], SYR_START[1], grip(syr_top, 0.008)),
                                   (sx, sy_seat - 0.02 + lag, held + 0.003), travel_z=travel, yaw=yaw)[:6]
    syringe += [Waypoint(1.5, (sx, sy_seat + lag + 0.006, held + 0.003), c, "push the tip home", yaw),
                Waypoint(0.8, (sx, sy_seat + lag + 0.006, held + 0.003), c, "seat", yaw),
                Waypoint(0.6, (sx, sy_seat + lag + 0.006, held + 0.003), o, "let go", yaw),
                Waypoint(1.0, (sx, sy_seat + lag, travel), o, "up", yaw)]
    press_y = sy_seat + PLUNGER_ANCHOR  # the thumb press, plunger pulled back
    pz = TOP + 0.022  # closed fingertips about 9 mm above the bench, level with the press's middle
    push = 0.030  # the hand's last target past the start: the plunger, pushed against the saline's resistance, ends
    #               about 3.4 cm in (the fingers meet the press about 5 mm on, and the hand lags behind its targets)
    syringe += [Waypoint(1.5, (HUB_X, press_y - 0.03, travel), c, "behind the thumb press", 0.0),
                Waypoint(1.2, (HUB_X, press_y - 0.03 + lag, pz), c, "down", 0.0),
                Waypoint(1.0, (HUB_X, press_y - 0.012 + lag, pz), c, "on the thumb press", 0.0)]
    # A slow push-pause flush, as nurses push a peripheral flush: seven pushes of about 0.8 mL, 1.6 s each, with
    # 0.4 s pauses between them; about 0.5 mL/s while pushing, so the plunger meets about 4 N (flushkit), not ~7 N.
    pulses = 7
    for k in range(1, pulses + 1):
        y = press_y - 0.012 + (push + 0.012) * k / pulses + lag
        syringe.append(Waypoint(1.6, (HUB_X, y, pz), c, "push about 0.8 mL", 0.0))
        if k < pulses:
            syringe.append(Waypoint(0.4, (HUB_X, y, pz), c, "pause", 0.0))
    syringe += [Waypoint(1.0, (HUB_X, press_y + push + lag, pz), c, "hold", 0.0),
                Waypoint(1.0, (HUB_X, press_y, travel), c, "up", 0.0)]
    park = (0.16, -0.26, TOP + 0.22)
    right = pick_pad + scrub + put_back + syringe + [Waypoint(1.2, park, o, "park"), Waypoint(2.0, park, o, "hold")]
    return {"right": right}
