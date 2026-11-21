"""Take a photo at 2x zoom with an iPhone 16's Camera Control.

An Apple iPhone 16 (147.6 x 71.6 x 7.80 mm, 170 g: Apple's tech specs and its accessory dimensional drawing) stands in
landscape in a desk phone stand, its screen towards the arms and its rear cameras towards a slice of cake on the table.
The Camera app is open at 1x, and Zoom is the setting the Camera Control last adjusted, so (Apple's iPhone User Guide)
a light press on the Camera Control shows the zoom overlay, sliding a finger along the control zooms, and a click
(a full press) takes the photo. In landscape, the Camera Control is on the top edge: a 17.1 mm sapphire strip whose
centre is 96.23 mm from the phone's top end (Apple's drawing), 51.4 mm from its right-hand end.

Model (declared in code):
- Phone ("phone"): a free articulation (170 g). Its frame: origin at the phone's centre; X through its thickness (+X:
  the back), Y along its length (+Y: the phone's top end, with the cameras), Z across its width (+Z: the Camera Control's
  edge, up). It leans back LEAN degrees in the stand. Its one link is the Camera Control's cap, a slider along Z on a
  spring (K_CAP): its press depth stands for the force on the control's sensor. Light press: LIGHT_D (0.5 N) or deeper,
  but short of the click, for 2 control steps. Click: CLICK_D (2.0 N), re-armed once the cap is back above REARM_D.
- The touch position is where a fingertip is on the cap, along the phone's length: the mean Y, in the phone's frame,
  of the fingertip points (mechkit.fingertips) over the cap, counted only while the cap is pressed at least TOUCH_D
  (contact, not proximity). A contact lost for less than LIFT_STEPS (0.15 s) is the same touch.
- Zoom: while the overlay is open, the zoom is the zoom when the touch began times 2 ** (slide / SLIDE_PER_DOUBLING),
  sliding towards the phone's top end (the cameras) to zoom in, from 0.5x to 10x. The dial has a detent at each
  preset (1x and 2x): within SNAP (in log2 units) of one, it reads exactly that preset. The overlay closes 2 s after
  the finger leaves the control; the zoom stays.
- The screen shows the viewfinder, the zoom overlay (a bar with a marker that is yellow at 2x), the zoom chip (1x,
  2x or other), a white flash at each photo, and the last photo's thumbnail.
- A click takes a photo at the zoom then. It is sharp if the phone moved slower than SHARP_V and turned slower than
  SHARP_W at that step, and clear if no fingertip point was within LENS_CLEAR of a rear lens's centre.

    inspect-robots run ... -E task=tasks/take-2x-photo-with-iphone-camera-control/task.py (README)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Use the phone's Camera Control to take a photo at 2x zoom."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the phone (Apple iPhone 16), in its own frame ----------------------------------------------------------------------
PH_T, PH_L, PH_W = 0.0078, 0.1476, 0.0716  # thickness (X), length (Y), width (Z)
MASS = 0.170
Y_TOPEND = PH_L / 2  # the phone's top end (the cameras' end) is +Y
CC_Y = Y_TOPEND - 0.09623  # the Camera Control's centre: 96.23 mm from the top end
CC_LEN, CC_WID = 0.0171, 0.0030  # 2 x 8.55 mm long; about 3 mm across the edge
CC_GAP = 0.0011  # the cap's collider starts this far out from the frame's edge (more than its travel)
CC_COL_T = 0.0010  # the cap collider's thickness
CAP_TOP = PH_W / 2 + CC_GAP + CC_COL_T  # the cap's top, unpressed, in the phone's frame
CC_TRAVEL = 0.0010
K_CAP = 2500.0  # N/m (estimate): 0.5 N at 0.2 mm, 2.0 N at 0.8 mm
TOUCH_D, LIGHT_D, CLICK_D, REARM_D = 0.00004, 0.0002, 0.0008, 0.0004  # m pressed: contact, light press (0.5 N), click (2.0 N), re-arm
SLIDE_PER_DOUBLING = 0.006  # m of slide for each doubling of the zoom (estimate)
SNAP = 0.15  # log2 units: the detent at 1x and 2x (2x reads from 1.80x to 2.22x: 0.9 mm of slide either side)
LZ_MIN, LZ_MAX = -1.0, math.log2(10.0)  # 0.5x to 10x
OVERLAY_IDLE_STEPS = 40  # 2 s without a touch closes the overlay
LIFT_STEPS = 3  # the touch controller's debounce: a contact lost for less than 0.15 s is the same touch
SHARP_V, SHARP_W = 0.02, math.radians(10.0)  # m/s and rad/s at the click
LENS_CLEAR = 0.015
# the two rear cameras: 13.62 and 31.34 mm from the top end, about 11.8 mm in from the Camera Control's edge (drawing, detail D)
LENSES = np.array([[PH_T / 2, Y_TOPEND - 0.01362, PH_W / 2 - 0.0118], [PH_T / 2, Y_TOPEND - 0.03134, PH_W / 2 - 0.0118]])

# -- where the phone stands ----------------------------------------------------------------------------------------------
LEAN = 15.0  # degrees back from upright (the stand's recline)
PX, PY = 0.30, 0.0
LEDGE_UNDER = 0.017  # the ledge's lowest point above the table
_s, _c = math.sin(math.radians(LEAN)), math.cos(math.radians(LEAN))
ROT = np.array([[_c, 0.0, _s], [0.0, 1.0, 0.0], [-_s, 0.0, _c]])  # phone frame -> world: +Z leans towards +x
QUAT = (0.0, math.sin(math.radians(LEAN) / 2), 0.0, math.cos(math.radians(LEAN) / 2))
_ledge_low = (ROT @ np.array([-PH_T / 2 - 0.003, 0.0, -PH_W / 2 - 0.005]))[2]
PZ = TOP + LEDGE_UNDER - _ledge_low
CENTRE = np.array([PX, PY, PZ])


def world(p_local) -> np.ndarray:
    return CENTRE + ROT @ np.asarray(p_local, float)


LOOKS = {
    "alu": m.Look(color=m.rgb(0.80, 0.82, 0.86), metallic=0.85, roughness=0.3),  # the frame (iPhone 16 in white)
    "back": m.Look(color=m.rgb(0.93, 0.93, 0.94), roughness=0.25),
    "glass": m.Look(color=m.rgb(0.02, 0.02, 0.025), roughness=0.08),
    "sapphire": m.Look(color=m.rgb(0.55, 0.57, 0.62), metallic=0.6, roughness=0.15),
    "lens": m.Look(color=m.rgb(0.03, 0.03, 0.05), roughness=0.05),
    "plateau": m.Look(color=m.rgb(0.88, 0.88, 0.90), roughness=0.2),
    "view": m.Look(color=m.rgb(0.32, 0.40, 0.38), roughness=0.6, emissive=m.rgb(0.10, 0.13, 0.12)),
    "dial": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.5, emissive=m.rgb(0.05, 0.05, 0.05)),
    "tick": m.Look(color=m.rgb(0.95, 0.95, 0.95), emissive=m.rgb(0.6, 0.6, 0.6)),
    "yellow": m.Look(color=m.rgb(1.0, 0.80, 0.0), emissive=m.rgb(0.9, 0.7, 0.0)),
    "white": m.Look(color=m.rgb(1.0, 1.0, 1.0), emissive=m.rgb(1.0, 1.0, 1.0)),
    "thumb": m.Look(color=m.rgb(0.75, 0.55, 0.45), emissive=m.rgb(0.3, 0.2, 0.15)),
    "stand": m.Look(color=m.rgb(0.22, 0.22, 0.24), metallic=0.7, roughness=0.4),
    "pad": m.Look(color=m.rgb(0.08, 0.08, 0.08), roughness=0.9),
}

# the zoom overlay's markers: log2 zoom from -1 (0.5x) to 3.25 (9.5x) in quarters
MARKS = [-1.0 + 0.25 * k for k in range(18)]
DIAL_Y0, DIAL_Y1 = CC_Y - 0.034, CC_Y + 0.034
SCREEN_X = -PH_T / 2 - 0.0003  # just in front of the cover glass


def _mark_y(lz: float) -> float:
    return DIAL_Y0 + (lz - LZ_MIN) / (LZ_MAX - LZ_MIN) * (DIAL_Y1 - DIAL_Y0)


def _screen_box(name, y0, y1, z0, z1, look, hidden=False, dx=0.0):
    mesh = m.box((0.0002, y1 - y0, z1 - z0), (SCREEN_X - dx, (y0 + y1) / 2, (z0 + z1) / 2))
    return (name, mesh, look, True) if hidden else (name, mesh, look)


def _phone_frame():
    parts = [
        ("body", m.box((PH_T - 0.0006, PH_L, PH_W), (0.0, 0.0, 0.0)), "alu"),
        ("back", m.box((0.0004, PH_L - 0.004, PH_W - 0.004), (PH_T / 2 - 0.0001, 0.0, 0.0)), "back"),
        ("glass", m.box((0.0004, PH_L - 0.002, PH_W - 0.002), (-PH_T / 2 + 0.0001, 0.0, 0.0)), "glass"),
        # the rear camera plateau (1.85 mm proud) and its two lenses (16 mm, 3.48 mm proud of the back glass)
        ("plateau", m.box((0.00185, 0.040, 0.020), (PH_T / 2 + 0.0009, Y_TOPEND - 0.0225, PH_W / 2 - 0.0118)), "plateau"),
    ]
    for i, c in enumerate(LENSES):
        parts.append((f"lens{i}", m.cylinder(0.0065, 0.0035, (c[0] + 0.00175, c[1], c[2]), axis="x", segments=24), "lens"))
    # the screen: the viewfinder, the zoom overlay, the zoom chip, the flash and the last photo's thumbnail
    parts.append(_screen_box("view", -0.066, 0.066, -0.031, 0.031, "view"))
    parts.append(_screen_box("dial", DIAL_Y0 - 0.003, DIAL_Y1 + 0.003, 0.0245, 0.0305, "dial", hidden=True, dx=0.0002))
    for k, lz in enumerate(MARKS):
        y = _mark_y(lz)
        parts.append(_screen_box(f"mark{k}", y - 0.0015, y + 0.0015, 0.0255, 0.0295, "yellow" if lz == 1.0 else "tick", hidden=True, dx=0.0004))
    chip_y, chip_z = 0.0, -0.022
    parts.append(_screen_box("chip_1x", chip_y - 0.004, chip_y + 0.004, chip_z - 0.003, chip_z + 0.003, "tick", dx=0.0002))
    parts.append(_screen_box("chip_2x", chip_y - 0.004, chip_y + 0.004, chip_z - 0.003, chip_z + 0.003, "yellow", hidden=True, dx=0.0002))
    parts.append(_screen_box("chip_other", chip_y - 0.006, chip_y + 0.006, chip_z - 0.002, chip_z + 0.002, "tick", hidden=True, dx=0.0002))
    parts.append(_screen_box("flash", -0.066, 0.066, -0.031, 0.031, "white", hidden=True, dx=0.0006))
    parts.append(_screen_box("thumb", -0.062, -0.052, -0.029, -0.019, "thumb", hidden=True, dx=0.0002))
    cols = [m.col_box("body_c", (PH_T, PH_L, PH_W), (0.0, 0.0, 0.0)),
            m.col_box("plateau_c", (0.00185, 0.040, 0.020), (PH_T / 2 + 0.0009, Y_TOPEND - 0.0225, PH_W / 2 - 0.0118))]
    return parts, cols


def _cap_link():
    look_cap = m.box((CC_WID, CC_LEN, CC_GAP + CC_COL_T + 0.0005), (0.0, CC_Y, PH_W / 2 + (CC_GAP + CC_COL_T - 0.0005) / 2))
    col = m.col_box("cc_c", (CC_WID + 0.0006, CC_LEN, CC_COL_T), (0.0, CC_Y, PH_W / 2 + CC_GAP + CC_COL_T / 2))
    return m.slider("cc", "Z", -CC_TRAVEL, 0.0, [("cap", look_cap, "sapphire")], [col], at=(0.0, CC_Y, PH_W / 2), spring=K_CAP, damping=0.5,
                    friction=0.0, mass=0.004, armature=1e-5, effort=50.0)


_pp, _pc = _phone_frame()
_INERTIA = (MASS / 12 * (PH_L ** 2 + PH_W ** 2), MASS / 12 * (PH_W ** 2 + PH_T ** 2), MASS / 12 * (PH_L ** 2 + PH_T ** 2))
PHONE = m.mechanism("phone", tuple(CENTRE), [_cap_link()], LOOKS, frame_parts=_pp, frame_colliders=_pc, free_mass=MASS, start_quat=QUAT,
                    frame_inertia=_INERTIA, friction=(0.35, 0.35))


# -- the stand: an aluminium desk phone stand with a silicone-lined ledge, reclined 15 degrees (custom structural part) --------
def _in_stand(mesh: m.Mesh) -> m.Mesh:
    """A mesh authored in the phone's frame, into the stand's frame (origin on the table under the phone's centre)."""
    return m.Mesh(mesh.v @ ROT.T + (CENTRE - np.array([PX, PY, TOP])), mesh.f)


def _stand():
    g = 0.0003  # the phone starts this far off the ledge and the back rest, and settles onto them
    ledge = m.box((PH_T + 0.007, 0.11, 0.005), (0.0005, 0.0, -PH_W / 2 - g - 0.0025))
    lip = m.box((0.003, 0.11, 0.008), (-PH_T / 2 - g - 0.0015, 0.0, -PH_W / 2 - g - 0.001))
    rest = m.box((0.006, 0.09, 0.045), (PH_T / 2 + g + 0.003, 0.0, -PH_W / 2 + 0.0175))
    pieces = {"ledge": _in_stand(ledge), "lip": _in_stand(lip), "rest": _in_stand(rest)}
    back_face = pieces["rest"].v[pieces["rest"].v[:, 0] > np.median(pieces["rest"].v[:, 0])]
    plate = m.box((0.085, 0.11, 0.018), (0.0075, 0.0, 0.009))
    strut_pts = np.vstack([back_face, back_face * np.array([1.0, 1.0, 0.0]) + np.array([0.0, 0.0, 0.017])])
    strut = _hull_mesh(strut_pts)
    parts = [("plate", plate, "stand"), ("strut", strut, "stand"), ("ledge", pieces["ledge"], "pad"), ("lip", pieces["lip"], "pad"),
             ("rest", pieces["rest"], "pad")]
    cols = [m.col_box("plate_c", (0.085, 0.11, 0.018), (0.0075, 0.0, 0.009)), m.col_hull("strut_c", strut)] + [m.col_hull(f"{k}_c", v) for k, v in pieces.items()]
    return parts, cols


def _hull_mesh(points: np.ndarray) -> m.Mesh:
    from scipy.spatial import ConvexHull

    h = ConvexHull(points)
    faces = []
    c = points.mean(axis=0)
    for f in h.simplices:
        a, b, d = points[f]
        if np.dot(np.cross(b - a, d - a), a - c) < 0:
            f = f[[0, 2, 1]]
        faces.append(f)
    return m.Mesh(points.copy(), np.array(faces))


_sp, _sc = _stand()
STAND = m.fixed("stand", (PX, PY, TOP), _sp, _sc, LOOKS, friction=(1.2, 1.0))

# -- the subject and the distractors (SimReady) -----------------------------------------------------------------------------
CAKE = SimReady("cake", pos=(0.56, 0.06, TOP), size=(0.086, 0.104, 0.070), mass=0.10, package="food_prepared_cake_a01",
                usd="sm_food_prepared_cake_a01_01.usd", yaw=0.4)
MUG = SimReady("mug", pos=(0.24, 0.24, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
               yaw=2.2)
GLASSES = SimReady("sunglasses", pos=(0.50, -0.24, TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
                   usd="sm_accessory_eyeglasses_a01_01.usd", yaw=1.2)

OBJECTS = [PHONE, STAND, CAKE, MUG, GLASSES]


# -- the phone's software: the Camera Control and the Camera app ---------------------------------------------------------
def shown_zoom(lz: float) -> float:
    """The zoom the dial reads at log2 zoom `lz`: the preset within its detent, else the continuous value."""
    for preset in (0.0, 1.0):
        if abs(lz - preset) <= SNAP:
            return 2.0 ** preset
    return round(2.0 ** lz, 2)


def _tip_points(tips: np.ndarray) -> np.ndarray:
    return tips[0::2]  # each finger's tip row (the other row is 3 cm up the finger)


def _box_gap(local: np.ndarray) -> np.ndarray:
    """Each point's distance from the phone's box (cap included), in the phone's frame."""
    half = np.array([PH_T / 2, PH_L / 2, PH_W / 2])
    c = np.array([0.0, 0.0, 0.0])
    d = np.maximum(np.abs(local - c) - half, 0.0)
    d_cap = np.maximum(np.abs(local - np.array([0.0, CC_Y, PH_W / 2 + (CC_GAP + CC_COL_T) / 2])) - np.array([CC_WID / 2, CC_LEN / 2, (CC_GAP + CC_COL_T) / 2]), 0.0)
    return np.minimum(np.linalg.norm(d, axis=1), np.linalg.norm(d_cap, axis=1))


def _rot_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1.0) / 2.0, -1.0, 1.0))))


def _path(prim: str) -> str:
    return f"{m.ENV_PATH}/phone/frame/visual/{prim}"


class Phone:
    """The Camera Control's light press, slide and click, the zoom, the photos taken, and the screen. Also the score:
    `harm` (the phone knocked out of its place in the stand) is recorded across steps and cleared only in `reset`."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.lz = 0.0  # the Camera app opens at 1x
        self.overlay = False
        self.light_steps = 0
        self.idle = 0
        self.anchor = None  # (touch position, log2 zoom) where the current touch began
        self.armed = True
        self.photos: list[dict] = []
        self.flash = 0
        self.start = None  # the settled phone's pose, read at the first control step
        self.harm = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}
        if env is not None:
            self._show()

    # -- the screen
    def _show(self):
        zoom = shown_zoom(self.lz)
        m.set_visible(_path("dial"), self.overlay)
        near = min(range(len(MARKS)), key=lambda k: abs(MARKS[k] - (1.0 if zoom == 2.0 else 0.0 if zoom == 1.0 else self.lz)))
        for k in range(len(MARKS)):
            m.set_visible(_path(f"mark{k}"), self.overlay and k == near)
        m.set_visible(_path("chip_1x"), zoom == 1.0)
        m.set_visible(_path("chip_2x"), zoom == 2.0)
        m.set_visible(_path("chip_other"), zoom not in (1.0, 2.0))
        m.set_visible(_path("flash"), self.flash > 0)
        m.set_visible(_path("thumb"), bool(self.photos))

    def step(self, env):
        s = m.state(env, "phone")
        R = s.rotation()
        pos = np.asarray(s.pos, float)
        if self.start is None:
            self.start = (pos.copy(), R.copy())
        tips = m.fingertips(env)
        local = s.to_local(tips)
        depth = -s.joint("cc")
        tl = _tip_points(local)
        over = (np.abs(tl[:, 1] - CC_Y) <= CC_LEN / 2 + 0.003) & (np.abs(tl[:, 0]) <= 0.02) & (tl[:, 2] >= CAP_TOP - 0.012) & (tl[:, 2] <= CAP_TOP + 0.004)
        touch = bool(depth >= TOUCH_D and over.any())
        where = float(tl[over, 1].mean()) if touch else None

        # the light press opens the zoom overlay; sliding then zooms; 2 s without a touch closes it
        if not self.overlay:
            self.light_steps = self.light_steps + 1 if (touch and LIGHT_D <= depth < CLICK_D) else 0
            if self.light_steps >= 2:
                self.overlay, self.idle, self.anchor = True, 0, (where, self.lz)
        else:
            if touch:
                if self.anchor is None:
                    self.anchor = (where, self.lz)
                self.lz = float(np.clip(self.anchor[1] + (where - self.anchor[0]) / SLIDE_PER_DOUBLING, LZ_MIN, LZ_MAX))
                self.idle = 0
            else:
                self.idle += 1
                if self.idle >= LIFT_STEPS:  # a touch ends once the finger has been off the control for 0.15 s
                    self.anchor = None
                if self.idle >= OVERLAY_IDLE_STEPS:
                    self.overlay = False

        # the click takes a photo at the zoom the dial shows
        if self.armed and depth >= CLICK_D:
            self.armed = False
            v, w = float(np.linalg.norm(s.lin_vel)), float(np.linalg.norm(s.ang_vel))
            lens_gap = float(min(np.linalg.norm(local[:, None, :] - LENSES[None], axis=2).min(), 1.0))
            self.photos.append({"zoom": shown_zoom(self.lz), "sharp": v < SHARP_V and w < SHARP_W, "clear": lens_gap > LENS_CLEAR, "v": v, "w": w,
                                "lens_gap": lens_gap})
            self.flash = 4
        elif depth < REARM_D:
            self.armed = True
        if self.flash:
            self.flash -= 1

        # the phone knocked out of its place in the stand: more than 2 cm or 10 degrees from its settled start, before full credit
        moved = float(np.linalg.norm(pos - self.start[0]))
        turned = _rot_angle_deg(R, self.start[1])
        if (moved > 0.02 or turned > 10.0) and not self.full:
            self.harm = True

        # the score
        last = self.photos[-1] if self.photos else None
        good = last is not None and last["sharp"] and last["clear"]
        clear = float(_box_gap(local).min()) >= 0.03
        in_stand = moved <= 0.005 and turned <= 3.0 and float(np.linalg.norm(s.lin_vel)) < 0.01 and float(np.linalg.norm(s.ang_vel)) < 0.05
        ok = good and last["zoom"] == 2.0 and clear and in_stand and not self.harm
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.full:
            self.score = 1.0
        elif self.harm or not good:
            self.score = 0.0
        elif last["zoom"] == 2.0:
            self.score = 0.75
        elif 1.5 <= last["zoom"] <= 3.0:
            self.score = 0.5
        else:
            self.score = 0.25
        self.last = {"depth_mm": depth * 1000, "touch": touch, "where_mm": None if where is None else where * 1000, "overlay": self.overlay,
                     "zoom": shown_zoom(self.lz), "photos": len(self.photos), "moved_mm": moved * 1000, "turned_deg": turned, "clear": clear,
                     "harm": self.harm, "score": self.score}
        self._show()


PHONE_SW = Phone()
m.on_step(PHONE_SW.step)
m.on_reset(PHONE_SW.reset)


def success(objects) -> bool:
    return bool(PHONE_SW.full)


def score(objects) -> float:
    return float(PHONE_SW.score)


# -- the demonstration (open loop, right arm) ---------------------------------------------------------------------------------
# The right gripper, open and pointing down with its fingers closing along y, puts its left finger (the +y one) on the cap,
# the other finger out past the phone's right-hand end. FINGER_Y: the left finger's tip from the TCP along y.
FINGER_Y = abs(FINGER_OPEN) + 0.0045
TIP_Z = 0.0125  # the finger's underside below the TCP (measured: first contact on the cap in a run)
PRESS_LIGHT, PRESS_SLIDE, PRESS_CLICK = 0.0025, 0.0012, 0.0080  # how far the TCP is commanded below the cap's top: the arm's stiffness turns this into force (measured)
START_ON_CAP = -0.0045  # where along the cap the finger first lands, from its centre (towards the phone's right-hand end)
SLIDE = 0.0075  # towards the cameras' end: 1x to 2x (6 mm, plus the finger's lag behind the command, measured)


def _tcp_at(along: float, below_top: float) -> tuple:
    p = world((0.0, CC_Y + along, CAP_TOP))
    return (float(p[0]), float(p[1] - FINGER_Y), float(p[2] + TIP_Z - below_top))


def demo():
    o = FINGER_OPEN
    a0, a1 = START_ON_CAP, START_ON_CAP + SLIDE
    hi = _tcp_at(a0, -0.04)
    right = [
        Waypoint(2.5, hi, o, "over the Camera Control"),
        Waypoint(1.5, _tcp_at(a0, -0.004), o, "just above it"),
        Waypoint(1.0, _tcp_at(a0, PRESS_LIGHT), o, "light press: the zoom overlay opens"),
        Waypoint(0.6, _tcp_at(a0, PRESS_LIGHT), o, "hold"),
        Waypoint(0.5, _tcp_at(a0, PRESS_SLIDE), o, "ease off to a light touch"),
        Waypoint(3.0, _tcp_at(a1, PRESS_SLIDE), o, "slide towards the cameras to 2x"),
        Waypoint(0.6, _tcp_at(a1, PRESS_SLIDE), o, "hold at 2x"),
        Waypoint(0.6, _tcp_at(a1, PRESS_CLICK), o, "click: take the photo"),
        Waypoint(0.4, _tcp_at(a1, PRESS_CLICK), o, "hold the click"),
        Waypoint(0.8, _tcp_at(a1, -0.004), o, "let go"),
        Waypoint(1.5, _tcp_at(a1, -0.06), o, "up and away"),
        Waypoint(1.5, (0.22, -0.22, TOP + 0.20), o, "clear"),
        Waypoint(3.0, (0.22, -0.22, TOP + 0.20), o, "wait"),
    ]
    return {"right": right}
