"""On the station table, at full size: an adult venipuncture training arm (about 80 cm shoulder to fingertips) lies
palm up across the table in front of the arms, shoulder to the left. A blood-collection needle (21G, 1 inch) is
already in the vein at the crease of the elbow, entering at 25 deg and pointing up the arm; on its back end is a
single-use tube holder (clear plastic barrel 19 mm across, 48 mm long, a 34 x 24 mm finger flange at its open end),
and in the holder sits a 13 x 75 mm evacuated blood tube (no additive, red stopper), pushed in only to the guide
line: its stopper rests on the rubber sleeve over the back needle, not yet pierced. To the right, in front of the
hand, is a white tube tray with two open slots. The job: fill the tube, take it off and lay it in the tray, without
the needle leaving the vein.

Physics (mechkit.py, in this folder, adds the joints to the stock runtime in memory; the contact readouts and the
fill counter are added below in the same way):
- The needle in the arm is a slide along the needle's own line with 0.5 N of Coulomb friction (tissue drag); the
  holder and needle are one rigid body on it. The vein's lumen is +-2 mm of that slide. Push the tube in, or pull
  it out, without holding the holder, and the needle goes deeper or comes back with it. Because it is a slide, the
  sim cannot move the needle sideways or pivot it; the scorer still reads both from the holder's pose (gates: the
  hub 2 mm off the needle's start line, the needle's direction 3 deg off its start), and the real trial checks them
  on video.
- The stopper is pierced only when the measured contact force pressing the tube onto the back needle's sleeve
  reaches 4 N (PhysX contact readout, every physics step). From then on the tube is on the needle: it slides on it
  with up to 1.5 N of drag, ramped over the 4 mm of stopper the needle passes through, to a stop 8 mm in. Pulled
  back off the needle tip, the drag is gone; once the stopper is out of the holder's mouth the tube is free.
- The fill is a counter, not a fluid: blood flows (8 s for the full 4 mL) only while the needle tip is through the
  stopper and the needle is in the vein.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/venipuncture-tube-exchange/venipuncture_tube_exchange.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (sliders with friction, per-episode memory, installed in memory)

from yam_arena import demo as _demo  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = bool(os.environ.get("VP_DEBUG"))
M = "vp"

# -- colours ---------------------------------------------------------------------------------------------------------
SKIN = (0.86, 0.70, 0.60)
VEIN = (0.36, 0.42, 0.68)
CLEAR = (0.84, 0.88, 0.90)  # clear polypropylene / glass, as it renders
STEEL = (0.78, 0.80, 0.83)
HUB_GREEN = (0.20, 0.62, 0.32)  # 21G hub colour code
SLEEVE_C = (0.80, 0.74, 0.62)
STOPPER_RED = (0.78, 0.12, 0.12)
BLOOD = (0.42, 0.02, 0.04)
TRAY_C = (0.95, 0.95, 0.94)

# -- frames ----------------------------------------------------------------------------------------------------------
# World: x forward from the arm bases, y left, z up, metres. The needle's line A points up the arm (+y) and down
# 25 deg into it; U is "up" across the needle; the holder and tube are built in the frame (A, -x_world, U), origin at
# the skin puncture SITE. Local x runs along A: +9 mm is the needle tip in the vein, -72 mm the holder's open end.
ANGLE = math.radians(25.0)
A = np.array([0.0, math.cos(ANGLE), -math.sin(ANGLE)])
U = np.array([0.0, math.sin(ANGLE), math.cos(ANGLE)])
YL = np.cross(U, A)  # = (-1, 0, 0): local y points toward the robot
R_H = np.column_stack([A, YL, U])
SKIN_Z = 0.060  # skin over the vein at the crease, above the table
SITE = np.array([0.25, 0.06, TOP + SKIN_Z])


def _qx(ang):
    """xyzw quaternion of a turn ``ang`` rad about local x."""
    return (math.sin(ang / 2), 0.0, 0.0, math.cos(ang / 2))


def _rx(ang):
    return mk.axis_angle((1.0, 0.0, 0.0), ang)


def world(p_local):
    """A point given in the holder frame (as spawned) in world coordinates."""
    return SITE + R_H @ np.asarray(p_local, float)


def _oct_solid(x0, x1, apo, color=None):
    """A solid regular octagonal prism along local x (apothem ``apo``): four boxes turned 0/45/90/135 deg."""
    side = 2 * apo * math.tan(math.radians(22.5))
    return [mk.box((x1 - x0, 2 * apo, side), at=((x0 + x1) / 2, 0.0, 0.0), color=color, quat=_qx(math.radians(k)))
            for k in (0.0, 45.0, 90.0, 135.0)]


def _oct_tube(x0, x1, r_out, r_in, color=None):
    """A hollow octagonal tube along local x: eight wall slabs."""
    side = 2 * r_out * math.tan(math.radians(22.5)) + 0.0004
    rm, t = (r_out + r_in) / 2, r_out - r_in
    out = []
    for k in range(8):
        ang = math.radians(45.0 * k)
        out.append(mk.box((x1 - x0, t, side), at=((x0 + x1) / 2, rm * math.cos(ang), rm * math.sin(ang)), color=color,
                          quat=_qx(ang)))
    return out


# -- the training arm (fixed) ----------------------------------------------------------------------------------------
# Palm up along y, shoulder at +y. Each segment is a box with its upper corners stepped off (a rounded limb). Its frame
# is world-aligned with the origin under the puncture site on the table top.
def _limb(y0, y1, half_w, top, color=SKIN):
    ch = 0.008
    yc, ly = (y0 + y1) / 2, y1 - y0
    return [mk.box((2 * half_w, ly, top - ch), at=(0.0, yc, (top - ch) / 2), color=color),
            mk.box((2 * (half_w - ch), ly, top), at=(0.0, yc, top / 2), color=color)]


ARM_BOXES = [
    *_limb(0.03, 0.36, 0.048, 0.066),  # upper arm
    *_limb(-0.035, 0.03, 0.046, SKIN_Z),  # elbow, the crease (antecubital fossa)
    *_limb(-0.17, -0.035, 0.042, 0.056),  # forearm
    *_limb(-0.25, -0.17, 0.036, 0.046),  # wrist
    *_limb(-0.36, -0.25, 0.045, 0.030),  # palm (palm up)
    *[mk.box((0.016, 0.075, 0.018), at=(dx, -0.40, 0.009)) for dx in (-0.030, -0.010, 0.010, 0.030)],  # fingers
    mk.box((0.020, 0.055, 0.020), at=(0.050, -0.30, 0.012)),  # thumb, on the far side (palm up, left arm)
    # the vein: a slightly raised blue line along the top, from the forearm up past the crease
    mk.box((0.007, 0.065, 0.001), at=(0.0, -0.0025, SKIN_Z + 0.0004), color=VEIN),
    mk.box((0.007, 0.135, 0.001), at=(0.0, -0.1025, 0.056 + 0.0004), color=VEIN),
    mk.box((0.007, 0.080, 0.001), at=(0.0, 0.070, 0.066 + 0.0004), color=VEIN),
]

# -- holder + needle (one free body) ---------------------------------------------------------------------------------
NEEDLE_TIP = 0.009  # the front needle's tip, 9 mm in along its line (3.8 mm under the skin): in the vein
HUB = (-0.022, -0.016)  # the needle hub outside the holder
WALL = (-0.024, -0.022)  # the holder's front wall (the hub screws into it)
BORE = (-0.070, -0.024)  # the holder's bore: 46 mm deep
FLANGE = (-0.072, -0.070)
R_OUT, R_IN = 0.0095, 0.0085  # barrel 19 mm across, bore 17 mm
FLANGE_H, FLANGE_W = 0.034, 0.024  # finger flange, long side across U (rotated PSI about the axis, see below)
SLEEVE = (-0.035, -0.024)  # rubber sleeve over the 11 mm back needle
SLEEVE_HALF = 0.0012
PSI = math.radians(40.0)  # the flange's long side (and the right hand's squeeze) turned this far from U toward the robot
HOLDER_KG = 0.008


def _flange_boxes():
    r = _rx(-PSI)  # turns local z (U) toward local +y (the robot): the direction UP_PSI below
    x = sum(FLANGE) / 2
    th = FLANGE[1] - FLANGE[0]
    parts = [  # (size y, size z, centre y, centre z) in the flange's own (unturned) frame, round a 17 mm opening
        (FLANGE_W, FLANGE_H / 2 - R_IN, 0.0, (FLANGE_H / 2 + R_IN) / 2),
        (FLANGE_W, FLANGE_H / 2 - R_IN, 0.0, -(FLANGE_H / 2 + R_IN) / 2),
        (FLANGE_W / 2 - R_IN, 2 * R_IN, (FLANGE_W / 2 + R_IN) / 2, 0.0),
        (FLANGE_W / 2 - R_IN, 2 * R_IN, -(FLANGE_W / 2 + R_IN) / 2, 0.0),
    ]
    return [mk.box((th, sy, sz), at=tuple(r @ np.array([x, cy, cz])), color=CLEAR, quat=_qx(-PSI))
            for sy, sz, cy, cz in parts]


HOLDER_BOXES = [
    *_oct_tube(BORE[0], BORE[1], R_OUT, R_IN, CLEAR),
    *_oct_solid(WALL[0], WALL[1], R_OUT, CLEAR),
    *_flange_boxes(),
    *_oct_solid(HUB[0], HUB[1], 0.0035, HUB_GREEN),
    mk.box((NEEDLE_TIP - HUB[1], 0.0008, 0.0008), at=((HUB[1] + NEEDLE_TIP) / 2, 0.0, 0.0), color=STEEL),
    mk.box((SLEEVE[1] - SLEEVE[0], 2 * SLEEVE_HALF, 2 * SLEEVE_HALF), at=(sum(SLEEVE) / 2, 0.0, 0.0), color=SLEEVE_C),
]

# -- the tube (free body) --------------------------------------------------------------------------------------------
CAP_FACE0 = SLEEVE[0] - 0.0002  # pre-seated to the guide line: the stopper face 0.2 mm off the sleeve's end
TUBE_LEN = 0.081  # 75 mm tube plus the stopper's head
CAP_LEN, R_CAP, R_TUBE = 0.017, 0.00775, 0.0065
TUBE_END0 = CAP_FACE0 - TUBE_LEN
N_SEG = 5  # the tube body in five lengths, each shown red once filled that far (from the stopper end down)
SEG_X = np.linspace(CAP_FACE0 - CAP_LEN + 0.002, TUBE_END0, N_SEG + 1)
TUBE_KG = 0.007
TUBE_BOXES = [
    *[b for k in range(N_SEG) for b in _oct_solid(SEG_X[k + 1], SEG_X[k], R_TUBE, CLEAR)],
    *_oct_solid(CAP_FACE0 - CAP_LEN, CAP_FACE0, R_CAP, STOPPER_RED),
]
SEG_BOXES = [list(range(4 * k, 4 * k + 4)) for k in range(N_SEG)]  # box indices of each length

# -- the tray (fixed) ------------------------------------------------------------------------------------------------
TRAY_XY = (0.420, -0.130)
SLOT_W, SLOT_L, RIB, FLOOR_T, TRAY_H = 0.040, 0.098, 0.006, 0.004, 0.012
SLOT_X = (TRAY_XY[0] - (SLOT_W + RIB) / 2, TRAY_XY[0] + (SLOT_W + RIB) / 2)
FLOOR_TOP = TOP + FLOOR_T


def _tray():
    tx, ty = TRAY_XY
    wx = 2 * SLOT_W + 3 * RIB
    wy = SLOT_L + 2 * RIB
    zc = TOP + TRAY_H / 2
    out = [Box("tray_floor", pos=(tx, ty, TOP + FLOOR_T / 2), size=(wx, wy, FLOOR_T), color=TRAY_C)]
    for k, x in enumerate((tx - wx / 2 + RIB / 2, tx, tx + wx / 2 - RIB / 2)):
        out.append(Box(f"tray_rib{k}", pos=(x, ty, zc), size=(RIB, wy, TRAY_H), color=TRAY_C))
    for k, y in enumerate((ty - wy / 2 + RIB / 2, ty + wy / 2 - RIB / 2)):
        out.append(Box(f"tray_end{k}", pos=(tx, y, zc), size=(wx, RIB, TRAY_H), color=TRAY_C))
    return out


# -- the mechanism ---------------------------------------------------------------------------------------------------
TISSUE_F = 0.5  # N: drag of the tissue on the needle (UNVERIFIED order of magnitude)
BAND = 0.002  # m: the needle's travel along its line that keeps its tip in the vein's lumen
F_PIERCE = 4.0  # N: push that drives the stopper onto the needle (UNVERIFIED; ISO 8871-5 caps vial closures at 10 N)
F_DRAG = 1.5  # N: the stopper's grip on the needle once through it (UNVERIFIED)
K_RAIL = F_DRAG / mk.STICK_M  # N/m: the tube's slide drive spring (mechkit's friction stick for F_DRAG): 3000 N/m
F_BORE = 0.3  # N: the unpierced tube's drag in the bore when drawn back (UNVERIFIED; > its 0.07 N weight, so a seated
# tube stays at the guide line when the holder is jostled)
SEAT_ZONE = -0.002  # m: within 2 mm of the guide line the unpierced stopper is seated on the sleeve
T_STOP = 0.004  # m: stopper the needle passes through; flow starts once the needle tip is past it
TRAVEL = 0.008  # m: the tube's stop on the needle (sleeve squashed against the holder's wall)
Q_OUT = -0.039  # m: the stopper 2 mm out of the holder's mouth: the tube is free
T_FILL = 8.0  # s for the full 4 mL draw (UNVERIFIED)
FULL = 0.9  # blood up to the fill mark (+-10 %)
SIDE_MAX = 0.002  # m: the hub's sideways offset from the needle's start line (any direction across it) that latches 0
PIVOT_MAX = 3.0  # deg: the needle's direction off its start direction (any pivot; spin about its own axis excluded)
HUB_FACE = HUB[1]  # the hub's face toward the skin, 16 mm out along the needle: where the sideways offset is read

kit = mk.Kit()
kit.part("arm", (SITE[0], SITE[1], TOP), ARM_BOXES, mass=None, color=SKIN)
QH = tuple(float(v) for v in mk.mat_to_quat(R_H))
kit.part("holder", tuple(SITE), HOLDER_BOXES, mass=HOLDER_KG, color=CLEAR, quat=QH)
kit.part("tube", tuple(SITE), TUBE_BOXES, mass=TUBE_KG, color=CLEAR, quat=QH)
kit.slider("tissue", "arm", "holder", axis=tuple(A), lower=-0.012, upper=0.008, friction=TISSUE_F, damping=0.2)
kit.slider("skewer", "holder", "tube", axis=tuple(A), lower=-0.045, upper=TRAVEL, friction=F_DRAG)

OBJECTS = [*kit.objects(), *_tray()]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


# -- per-episode state -----------------------------------------------------------------------------------------------
def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"t": 0, "n": 0, "pierced": None, "rail": True, "removed": None, "fill": 0.0,
                            "out": None, "let_go": None, "first_held": None, "unheld": 0, "held": False,
                            "q_t": 0.0, "q_s": 0.0, "peak_push": 0.0, "out_dirs": set(),
                            "touch": [0.0] * 8, "seg_shown": 0, "label": "",
                            "side": None, "side_kinds": set(), "lat_max": 0.0, "piv_max": 0.0}
    return m


_RT: dict = {}


def _drive(name):
    from pxr import UsdPhysics

    path = mk._SIM["joint_paths"].get((kit.id, name))
    return UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "linear") if path else None


def _set_rail(on: bool) -> None:
    mk._set_joint_enabled(kit, "skewer", on)


def _set_force_cap(name, f) -> None:
    d = _drive(name)
    if d is None:
        return
    a = d.GetMaxForceAttr() or d.CreateMaxForceAttr()
    a.Set(float(max(f, 1e-4)))


def _scene_setup(stage) -> None:
    """Once the scene is built: metal/plastic parts at friction 0.3 (not the rubbery default), the drives' stiffness
    and force caps (tissue drag; the stopper's 4 N give-way before it is pierced)."""
    from pxr import UsdPhysics

    paths = mk._SIM["paths"]
    n = 0
    for part in ("holder", "tube"):
        root = stage.GetPrimAtPath(paths.get(part, ""))
        if not root.IsValid():
            continue
        for prim in [root, *root.GetAllChildren()]:
            for p in [prim, *_descendants(prim)]:
                if p.HasAPI(UsdPhysics.MaterialAPI):
                    mat = UsdPhysics.MaterialAPI(p)
                    mat.GetStaticFrictionAttr().Set(0.3)
                    mat.GetDynamicFrictionAttr().Set(0.3)
                    n += 1
    from pxr import PhysxSchema

    for part in ("holder", "tube"):  # contact reports (the readouts below need them); light parts: more solver passes
        rep = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(paths[part]))
        rep.CreateThresholdAttr().Set(0.0)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(paths[part]))
        rb.CreateSolverPositionIterationCountAttr().Set(64)
        rb.CreateSolverVelocityIterationCountAttr().Set(8)
    (_drive("skewer").GetDampingAttr() or _drive("skewer").CreateDampingAttr()).Set(2.0)
    _set_force_cap("tissue", TISSUE_F)
    _set_force_cap("skewer", F_PIERCE + 0.05)
    _set_rail(True)
    print(f"[vp] scene: {n} holder/tube materials at mu 0.3; tissue drag {TISSUE_F} N; stopper gives way at {F_PIERCE} N",
          flush=True)


def _descendants(prim):
    out = []
    for c in prim.GetAllChildren():
        out.append(c)
        out.extend(_descendants(c))
    return out


def _patch_scene(mod) -> None:
    from isaaclab.scene import interactive_scene as isc

    cls = isc.InteractiveScene
    if getattr(cls, "_vp_setup", False):
        return
    init0 = cls.__init__

    def __init__(self, *a, **k):
        init0(self, *a, **k)
        try:
            _scene_setup(self.stage)
        except Exception as e:  # noqa: BLE001
            import traceback

            traceback.print_exc()
            print(f"[vp] scene setup failed: {e!r}", flush=True)

    cls.__init__ = __init__
    cls._vp_setup = True


def _install_scene_patch() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
        return
    for f in sys.meta_path:  # run after mechkit's own patch of yam_arena.scene (it authors the joints we need)
        if isinstance(f, mk._AfterImport) and f.name == "yam_arena.scene":
            p0 = f.patch

            def both(mod, p0=p0):
                p0(mod)
                _patch_scene(mod)

            f.patch = both
            return


_install_scene_patch()

_friction_reset0 = mk._friction_reset


def _friction_reset():  # every episode reset, before the scene is put back: a fresh tube in the holder again
    _friction_reset0()
    _set_rail(True)
    _set_force_cap("tissue", TISSUE_F)
    _set_force_cap("skewer", F_PIERCE + 0.05)
    _RT.pop("cap_s", None)


mk._friction_reset = _friction_reset


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def drag_cap(q_s: float) -> float:
    """The stopper's grip on the needle (N) with the tube ``q_s`` m along it from the guide line: none until the
    needle's tip is in the stopper, rising to F_DRAG over the stopper's thickness."""
    return F_DRAG * min(1.0, max(0.0, q_s / T_STOP))


def _latch_out(m, q_t) -> None:
    """Latched for the trial at the first exit; each direction (deeper / back) is logged once."""
    way = "deeper" if q_t > 0 else "back"
    if m["out"] is None or way not in m["out_dirs"]:
        m["out"] = m["t"] if m["out"] is None else m["out"]
        m["out_dirs"].add(way)
        print(f"[vp] LATCH needle-out: the needle moved {q_t * 1000:+.2f} mm along its line "
              f"({'deeper, through the vein' if q_t > 0 else 'back, out of the vein'}; lumen +-{BAND * 1000:.0f} mm) "
              f"at physics step {m['t']}, waypoint '{m['label']}'", flush=True)


def needle_pose(p_h, rot) -> tuple[float, float, float]:
    """From the holder's (= the needle's) origin ``p_h`` (world, m) and rotation matrix ``rot``: travel along the
    needle's start line (m, + deeper), the hub's sideways offset from that line (m, any direction across it) and the
    needle's pivot off its start direction (deg). Spin about the needle's own axis moves neither."""
    p_h = np.asarray(p_h, float)
    hub = p_h + np.asarray(rot, float) @ np.array([HUB_FACE, 0.0, 0.0])
    d = hub - (SITE + HUB_FACE * A)
    lat = float(np.linalg.norm(d - (A @ d) * A))
    piv = math.degrees(math.acos(max(-1.0, min(1.0, float(np.asarray(rot, float)[:, 0] @ A)))))
    return float(A @ (p_h - SITE)), lat, piv


def _check_side(m, lat, piv) -> None:
    """The sideways and pivot gates, every physics step for the whole trial (before, during and after the fill):
    latched 0 at the first exceedance; each kind is logged once."""
    m["lat_max"], m["piv_max"] = max(m["lat_max"], lat), max(m["piv_max"], piv)
    for kind, bad, txt in (("sideways", lat > SIDE_MAX, f"the hub moved {lat * 1000:.2f} mm sideways off the needle's "
                            f"start line (limit {SIDE_MAX * 1000:.0f} mm)"),
                           ("pivot", piv > PIVOT_MAX, f"the needle pivoted {piv:.2f} deg off its start direction "
                            f"(limit {PIVOT_MAX:.0f} deg)")):
        if bad and kind not in m["side_kinds"]:
            m["side"] = m["t"] if m["side"] is None else m["side"]
            m["side_kinds"].add(kind)
            print(f"[vp] LATCH needle-{kind}: {txt} at physics step {m['t']}, waypoint '{m['label']}'", flush=True)


def _physics_step() -> None:
    """Every physics step: the needle's and the tube's travel, the tissue and stopper drag, the piercing rule, the
    tube leaving the holder, the needle-out latch and the fill counter."""
    rt, m = _RT, _mem()
    m["t"] += 1
    x_h = _arr(rt["b_holder"].get_transforms()).reshape(-1, 7)[0]  # pos, quat xyzw
    p_h = x_h[:3] - rt["origin"]
    p_t = _arr(rt["b_tube"].get_transforms()).reshape(-1, 7)[0][:3] - rt["origin"]
    q_t, lat, piv = needle_pose(p_h, mk.quat_to_mat(x_h[3:7]))
    q_s = float(A @ (p_t - p_h))
    m["q_t"], m["q_s"] = q_t, q_s
    if abs(q_t) > BAND:
        _latch_out(m, q_t)
    _check_side(m, lat, piv)
    if m["rail"] and q_s <= Q_OUT:
        m["rail"], m["removed"] = False, m["t"]
        _set_rail(False)
        print(f"[vp] tube out of the holder at physics step {m['t']} (pierced {bool(m['pierced'])}, fill "
              f"{m['fill'] * 100:.0f} %), needle {q_t * 1000:+.2f} mm; so far hub sideways max "
              f"{m['lat_max'] * 1000:.3f} mm, pivot max {m['piv_max']:.3f} deg", flush=True)
    fm = _arr(rt["c_tube"].get_contact_force_matrix(dt=rt["dt"])).reshape(-1, 3)
    fh = _arr(rt["c_holder"].get_contact_force_matrix(dt=rt["dt"])).reshape(-1, 3)
    for k in range(4):  # finger tips touching: [holder: LL, LR, RL, RR] + [tube: LL, LR, RL, RR]
        m["touch"][k] = max(m["touch"][k], float(np.linalg.norm(fh[k])))
        m["touch"][4 + k] = max(m["touch"][4 + k], float(np.linalg.norm(fm[1 + k])))
    if m["pierced"] and m["rail"] and q_s >= T_STOP and abs(q_t) <= BAND and m["out"] is None and m["side"] is None \
            and m["fill"] < 1.0:
        m["fill"] = min(1.0, m["fill"] + rt["dt"] / T_FILL)


def _install_runtime(objects) -> None:
    if _RT.get("failed"):
        return
    try:
        _install_runtime0(objects)
    except Exception as e:  # noqa: BLE001
        import traceback

        traceback.print_exc()
        print(f"[vp] runtime setup failed: {e!r}", flush=True)
        _RT["failed"] = True


def _install_runtime0(_objects) -> None:
    if _RT.get("ok") or mk._EP.get("env") is None:
        return
    from isaaclab_physx.physics import PhysxManager

    base = mk._EP["env"].unwrapped
    paths = mk._SIM["paths"]
    view = PhysxManager.get_physics_sim_view()
    tips = []
    for arm in ("left_robot", "right_robot"):
        lp = base.scene[arm].root_physx_view.link_paths
        lp = lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp
        for want in ("tip_left", "tip_right"):
            tips.append([p for p in lp if p.rsplit("/", 1)[-1] == want][0])
    _RT["origin"] = _arr(base.scene.env_origins)[0]
    _RT["c_tube"] = view.create_rigid_contact_view([paths["tube"]], filter_patterns=[[paths["holder"], *tips]],
                                                   max_contact_data_count=256)
    _RT["c_holder"] = view.create_rigid_contact_view([paths["holder"]], filter_patterns=[tips],
                                                     max_contact_data_count=256)
    _RT["b_holder"] = view.create_rigid_body_view([paths["holder"]])
    _RT["b_tube"] = view.create_rigid_body_view([paths["tube"]])
    _RT["dt"] = float(base.physics_dt)
    isc = base.scene
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        try:
            _physics_step()
        except Exception as e:  # noqa: BLE001
            if not _RT.get("err"):
                import traceback

                traceback.print_exc()
                print(f"[vp] physics hook failed: {e!r}", flush=True)
            _RT["err"] = True
        return write(*a, **k)

    isc.write_data_to_sim = write_data_to_sim
    _RT["ok"] = True
    print(f"[vp] contact views on tube/holder vs {len(tips)} finger tips, dt {_RT['dt']:.5f}", flush=True)


TOUCH_N = 0.05  # N: a finger tip touching the holder or the tube
UNHELD_STEPS = 5  # control steps (0.25 s at 20 Hz) with the holder in no hand: let go


def holder_held(touch) -> bool:
    """The holder is held when one gripper has both finger tips on it, or one on it and the other on the tube (the
    thumb-and-fingers squeeze of flange and tube end)."""
    for g in (0, 2):
        on_h = [touch[g] > TOUCH_N, touch[g + 1] > TOUCH_N]
        on_t = [touch[4 + g] > TOUCH_N, touch[4 + g + 1] > TOUCH_N]
        if any(on_h) and all(h or t for h, t in zip(on_h, on_t)):
            return True
    return False


def _show_fill(m) -> None:
    """Colour the tube's lengths red as the counter fills them (looks only)."""
    k = int(m["fill"] * N_SEG + 1e-6)
    if k <= m["seg_shown"] or _RT.get("no_colour"):
        return
    try:
        from pxr import Gf, UsdShade

        stage = mk._SIM["stage"]
        root = mk._SIM["paths"]["tube"]
        for s in range(m["seg_shown"], k):
            for i in SEG_BOXES[s]:
                for p in [stage.GetPrimAtPath(f"{root}/box{i}"), *_descendants(stage.GetPrimAtPath(f"{root}/box{i}"))]:
                    if p.IsA(UsdShade.Shader):
                        inp = UsdShade.Shader(p).GetInput("diffuseColor")
                        if inp:
                            inp.Set(Gf.Vec3f(*[c ** 2.2 for c in BLOOD]))
        m["seg_shown"] = k
    except Exception as e:  # noqa: BLE001
        _RT["no_colour"] = True
        print(f"[vp] fill colour unavailable: {e!r}", flush=True)


def _reset_colour() -> None:
    try:
        from pxr import Gf, UsdShade

        stage = mk._SIM["stage"]
        root = mk._SIM["paths"]["tube"]
        for s in range(N_SEG):
            for i in SEG_BOXES[s]:
                for p in _descendants(stage.GetPrimAtPath(f"{root}/box{i}")):
                    if p.IsA(UsdShade.Shader):
                        inp = UsdShade.Shader(p).GetInput("diffuseColor")
                        if inp:
                            inp.Set(Gf.Vec3f(*[c ** 2.2 for c in CLEAR]))
    except Exception:  # noqa: BLE001
        pass


def _stopper(objects, m) -> None:
    """Control rate (as mechkit's friction): the tube's slide drive. Before the stopper is pierced, while seated at
    the guide line the spring's anchor stays there (the stopper sits on the sleeve); drawn back clear of it, bore
    friction (the anchor lags the tube by at most F_BORE / K_RAIL). The anchor moves only while a finger tip touches the
    tube (static friction: with no applied load the tube stays put). Pushing in past the guide line compresses the spring; once the push has passed F_PIERCE (the tube F_PIERCE / K_RAIL in) the
    stopper has given way. After that the anchor follows the tube, lagging at most drag_cap / K_RAIL (Coulomb drag
    of the stopper on the needle)."""
    if not m["rail"]:
        return
    q = kit.q(objects, "skewer")
    key = (kit.id, "skewer")
    st = mk._SIM["stick"].get(key, 0.0)
    if not m["pierced"] and q >= F_PIERCE / K_RAIL:
        m["pierced"] = m["t"]
        print(f"[vp] stopper pierced at physics step {m['t']}: the tube gave way {q * 1000:+.2f} mm past the guide line "
              f"(push {K_RAIL * q:.1f} N >= {F_PIERCE} N), needle {m['q_t'] * 1000:+.2f} mm, waypoint '{m['label']}'",
              flush=True)
    touched = m.get("tube_touched", False)
    if m["pierced"]:
        lim = drag_cap(q) / K_RAIL
        new = min(max(st, q - lim), q + lim) if touched else st  # static: the anchor moves only under a finger's load
        cap = drag_cap(q) + 0.05
    else:
        if q > SEAT_ZONE:  # seated on the sleeve at the guide line: held there (pushing in: the piercing load)
            new = 0.0
        else:  # drawn back clear of the sleeve: bore friction
            lim = F_BORE / K_RAIL
            new = min(min(max(st, q - lim), q + lim), 0.0) if touched else min(st, 0.0)
        cap = F_PIERCE + 0.05
    mk._SIM["stick"][key] = new
    d = _drive("skewer")
    if d is not None:
        d.GetTargetPositionAttr().Set(float(new))
    if abs(cap - _RT.get("cap_s", -1.0)) > 0.02:
        _set_force_cap("skewer", cap)
        _RT["cap_s"] = cap


def _on_step(objects) -> None:
    """Every control step: who holds the holder (the let-go latch), the left hand's closed loop, the fill colour."""
    if mk._EP.get("env") is None or not _RT.get("ok"):
        return
    m = _mem()
    m["n"] += 1
    if m["n"] == 1:
        _reset_colour()
    held = holder_held(m["touch"])
    m["held"] = held
    if held and m["first_held"] is None:
        m["first_held"] = m["t"]
        print(f"[vp] holder held from physics step {m['t']} ({m['label']})", flush=True)
    if m["first_held"] is not None and not held:
        m["unheld"] += 1
        if m["unheld"] >= UNHELD_STEPS and m["let_go"] is None:
            m["let_go"] = m["t"]
            print(f"[vp] LATCH holder-let-go: neither hand on the holder for {UNHELD_STEPS} control steps, at physics "
                  f"step {m['t']} (needle {m['q_t'] * 1000:+.2f} mm), waypoint '{m['label']}'", flush=True)
    else:
        m["unheld"] = 0
    if m["n"] % 200 == 0:  # every 10 s: the sideways and pivot gates' running maxima (whole trial)
        print(f"[vp] k={m['n']} needle sideways/pivot so far: hub max {m['lat_max'] * 1000:.3f} mm (limit "
              f"{SIDE_MAX * 1000:.0f}), pivot max {m['piv_max']:.3f} deg (limit {PIVOT_MAX:.0f}), latched "
              f"{m['side'] is not None}", flush=True)
    if DEBUG and m["n"] % 10 == 0:
        print(f"[vp] k={m['n']} needle {m['q_t'] * 1000:+.2f} mm tube {m['q_s'] * 1000:+.2f} mm fill "
              f"{m['fill'] * 100:.0f} % held {int(held)} touch {[round(v, 2) for v in m['touch']]} pierced "
              f"{int(bool(m['pierced']))} rail {int(m['rail'])} corr {np.round(_cl()['corr'] * 1000, 1)} | {m['label']}",
              flush=True)
    m["tube_touched"] = max(m["touch"][4:8]) > TOUCH_N
    m["touch"] = [0.0] * 8
    _stopper(objects, m)
    cl = _cl()
    cl["corr"] = np.clip(cl["corr"] - CL_GAIN * m["q_t"] * A, -0.04, 0.04) if cl["on"] else cl["corr"]
    _show_fill(m)


kit.on_step(_install_runtime)
kit.on_step(_on_step)

# -- readouts and score ----------------------------------------------------------------------------------------------
_TEST_TCPS = None


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _released(st, margin=0.025) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def in_tray(objects) -> bool:
    """The tube lying in a tray slot: its whole footprint inside the slot, its lowest point within 3 mm of the
    slot's floor, let go (no gripper within 2.5 cm) and still."""
    st = objects["tube"]
    lo, hi = np.asarray(st.low, float), np.asarray(st.high, float)
    for sx in SLOT_X:
        if (lo[0] >= sx - SLOT_W / 2 - 0.001 and hi[0] <= sx + SLOT_W / 2 + 0.001
                and lo[1] >= TRAY_XY[1] - SLOT_L / 2 - 0.001 and hi[1] <= TRAY_XY[1] + SLOT_L / 2 + 0.001
                and lo[2] <= FLOOR_TOP + 0.003):
            return bool(st.resting() and _released(st))
    return False


def score(objects) -> float:
    """0 if the needle ever left the vein (moved more than 2 mm along its line), or its hub ever moved more than 2 mm
    sideways off the needle's start line, or the needle ever pivoted more than 3 deg (whole trial, including after the
    fill), or the holder was let go of after it was first held (no hand on it for 0.25 s), or the stopper was never
    pierced. Otherwise 0.25 (pierced in the vein), 0.5 the tube filled to its mark (>= 90 %), 0.75 that and the tube
    off the needle and out of the holder, 1.0 that and the tube lying in a tray slot, let go and still, with the holder
    still held."""
    m = _mem()
    if m["out"] is not None or m["side"] is not None or m["let_go"] is not None or not m["pierced"]:
        return 0.0
    if m["fill"] < FULL:
        return 0.25
    if not m["removed"]:
        return 0.5
    if in_tray(objects) and m["held"]:
        return 1.0
    return 0.75


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    m = _mem()
    p = np.asarray(objects["tube"].pos, float)
    return float((1.0 - m["fill"]) * 0.05 + (0.0 if m["removed"] else 0.05)
                 + math.dist(p[:2], (SLOT_X[0], TRAY_XY[1])))


# -- tilted grips and the closed loop for the scripted demo ----------------------------------------------------------
# yam_arena's Waypoint carries only a yaw about the straight-down grip; a Waypoint's yaw may be a Tilt: a float (0.0)
# that also carries the full TCP rotation, which ArmPlan uses instead (in-memory patch of grip_rotation).
class Tilt(float):
    def __new__(cls, rot):
        self = super().__new__(cls, 0.0)
        self.rot = np.asarray(rot, dtype=float)
        return self


def _patch_grip_rotation() -> None:
    if getattr(_demo, "_tilt_patch", False):
        return
    base = _demo.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot.copy() if rot is not None else base(yaw)

    _demo.grip_rotation = grip_rotation
    _demo._tilt_patch = True


_patch_grip_rotation()

# The demo may read sim state (the Guide; v21). The left hand's "[hold]" waypoints are corrected live: every control
# step its target moves against the needle's measured travel (an integrator), so it keeps the holder where it was
# while the other hand pulls the tube. Kept in mechkit's per-episode MEMORY.
CL_GAIN = 0.4
CL_KEY = "vp_cl"


def _cl() -> dict:
    return mk.MEMORY.setdefault(CL_KEY, {"corr": np.zeros(3), "on": False})


def _patch_arm_plan() -> None:
    cls = _demo.ArmPlan
    if getattr(cls, "_vp_cl", False):
        return
    target0 = cls.target

    def target(self, t):
        pos, rot, finger, label = target0(self, t)
        if mk._EP.get("env") is None:
            return pos, rot, finger, label
        cl = _cl()
        if self.base_pos[1] > 0:
            cl["on"] = label.startswith("[hold]")
            if cl["on"]:
                pos = pos + cl["corr"]
            _mem()["label_l"] = label
        else:
            _mem()["label"] = label
        return pos, rot, finger, label

    cls.target = target
    cls._vp_cl = True


_patch_arm_plan()


# -- the scripted demo -----------------------------------------------------------------------------------------------
def _frame(x_axis, z_axis):
    x = np.asarray(x_axis, float) / np.linalg.norm(x_axis)
    z = np.asarray(z_axis, float) / np.linalg.norm(z_axis)
    return np.column_stack([x, np.cross(z, x), z])


def fw(w):
    """Finger joint target for a pad opening ``w`` (m)."""
    return -w / 2


TIP = PAD_TIP_PAST_TCP
UP_PSI = R_H @ np.array([0.0, math.sin(PSI), math.cos(PSI)])  # the flange's long side, turned PSI toward the robot

# Squeeze (the right hand): one pad behind the flange's upper wing, the other on the tube's end, closing along C (the
# needle's line tipped THETA toward UP_PSI) so the flange pad's tip clears the barrel while the other pad's tip is on
# the tube's end. Solved in the plane (A, UP_PSI).
WING = FLANGE_H / 2  # the wing's edge, 17 mm off the axis
TIP_F = R_OUT + 0.0035  # the flange pad's tip: 3.5 mm over the barrel
H_T = 0.0015  # the tube pad's tip, 1.5 mm past the tube's axis
GAP = 0.001


def _squeeze_geometry():
    tan = 0.25
    for _ in range(50):
        xf = FLANGE[1] + (WING - TIP_F) * tan + GAP
        xt = TUBE_END0 - GAP
        tan = (TIP_F - H_T) / (xf - xt)
    th = math.atan(tan)
    xf = FLANGE[1] + (WING - TIP_F) * tan + GAP
    xt = TUBE_END0 - GAP
    w = math.hypot(xf - xt, TIP_F - H_T)
    up = np.array([0.0, math.sin(PSI), math.cos(PSI)])
    c_l = math.cos(th) * np.array([1.0, 0, 0]) + math.sin(th) * up
    d_l = math.sin(th) * np.array([1.0, 0, 0]) - math.cos(th) * up
    ef = np.array([xf, 0, 0]) + TIP_F * up
    et = np.array([xt, 0, 0]) + H_T * up
    tcp_l = (ef + et) / 2 - TIP * d_l
    return th, w, R_H @ c_l, R_H @ d_l, world(tcp_l)


THETA, W_SQ, C_SQ, D_SQ, TCP_SQ = _squeeze_geometry()
ROT_SQ = _frame(-C_SQ, D_SQ)
SQ_OPEN = W_SQ + 0.006  # approach opening
SQ_CLOSE = W_SQ - 2 * GAP - TRAVEL / math.cos(THETA) - 0.002  # past the tube's stop: ~5 N per pad at the end
SQ_SHIFT = (W_SQ - SQ_CLOSE) / 2 - GAP - 0.001  # the TCP's move along C so the flange pad stays on the flange

# The left hand holds the barrel near the hub, closing across it, coming from above and a little in front.
XL_GRIP = -0.042
PHI_L = math.radians(30.0)
D_L = -(math.cos(PHI_L) * U + math.sin(PHI_L) * np.array([1.0, 0, 0]))
ROT_L = _frame(np.cross(A, D_L), D_L)
TCP_L = world((XL_GRIP, 0, 0)) - 0.005 * D_L  # the barrel 5 mm toward the pad tips
W_L_OPEN, W_L_HOLD = 0.042, R_OUT * 2 - 0.002  # ~1 mm past the barrel per pad: a firm hold, less twist

# The right hand's tube grip (to take it off): across the tube 16 mm from its end once pushed home, coming from
# above and from the robot's side; the tube sits 4 mm toward the pad tips.
XR_GRIP = TUBE_END0 + TRAVEL + 0.010  # 10 mm from its end
PHI_R = math.radians(-45.0)
D_R = -(math.cos(PHI_R) * U + math.sin(PHI_R) * np.array([1.0, 0, 0]))
ROT_R = _frame(-np.cross(A, D_R), D_R)
TCP_R = world((XR_GRIP, 0, 0)) - 0.004 * D_R
W_R_OPEN, W_R_HOLD, W_R_LET = 0.040, 2 * R_TUBE - 0.004, 0.018
ROT_R_FLAT = mk.axis_angle((1.0, 0.0, 0.0), ANGLE) @ ROT_R  # the same grip with the tube turned level (cap to +y)
TUBE_MID_FROM_GRIP = (CAP_FACE0 + TRAVEL + TUBE_END0 + TRAVEL) / 2 - XR_GRIP  # m along the tube, toward the cap


def _go(plan, dur, pos, w, label, rot):
    plan.append(Waypoint(float(dur), tuple(float(v) for v in pos), fw(w), label, Tilt(rot)))


def _left_grip(plan, hold_s, label="[hold] keep the holder still"):
    _go(plan, 2.5, TCP_L - 0.045 * D_L, W_L_OPEN, "left: over the holder's barrel", ROT_L)
    _go(plan, 1.5, TCP_L - 0.012 * D_L, W_L_OPEN, "left: down round the barrel", ROT_L)
    _go(plan, 0.6, TCP_L, W_L_OPEN, "left: pads at the barrel", ROT_L)
    _go(plan, 1.0, TCP_L, W_L_HOLD, "left: close on the barrel", ROT_L)
    _go(plan, hold_s, TCP_L, W_L_HOLD, label, ROT_L)


def _squeeze(plan, wait_s):
    p0 = TCP_SQ + C_SQ * (SQ_OPEN - W_SQ) / 2 * 0.0  # centred: both pads 4 mm off
    _go(plan, 3.0, p0 - 0.06 * D_SQ, SQ_OPEN, "right: over flange and tube end", ROT_SQ)
    _go(plan, wait_s, p0 - 0.06 * D_SQ, SQ_OPEN, "right: wait for the left hand", ROT_SQ)
    _go(plan, 1.5, p0, SQ_OPEN, "right: pads down behind the flange and at the tube's end", ROT_SQ)
    _go(plan, 0.6, p0, W_SQ, "right: pads onto flange and tube end", ROT_SQ)
    _go(plan, 2.5, p0 + SQ_SHIFT * C_SQ, SQ_CLOSE, "[squeeze] right: squeeze the tube onto the needle", ROT_SQ)
    _go(plan, 1.0, p0 + SQ_SHIFT * C_SQ, SQ_CLOSE, "[squeeze] right: hold it home", ROT_SQ)
    _go(plan, 0.8, p0, W_SQ + 0.002, "right: let go of flange and tube", ROT_SQ)
    _go(plan, 1.2, p0 - 0.06 * D_SQ, W_SQ + 0.002, "right: up", ROT_SQ)


def _tube_grip(plan):
    _go(plan, 2.0, TCP_R - 0.06 * D_R, W_R_OPEN, "right: over the tube", ROT_R)
    _go(plan, 1.5, TCP_R, W_R_OPEN, "right: down round the tube", ROT_R)
    _go(plan, 1.0, TCP_R, W_R_HOLD, "right: close on the tube", ROT_R)


def _pull(plan, slow=0.014, fast=0.040, v_slow=0.002, v_fast=0.010, label="[pull]"):
    p = TCP_R - slow * A
    _go(plan, slow / v_slow, p, W_R_HOLD, f"{label} right: ease the tube off the needle", ROT_R)
    p2 = p - fast * A
    _go(plan, fast / v_fast, p2, W_R_HOLD, f"{label} right: draw it out of the holder", ROT_R)
    return p2


def _to_tray(plan, p_out):
    p_up = p_out - 0.015 * A + np.array([0, 0, 0.04])
    _go(plan, 1.5, p_up, W_R_HOLD, "right: clear of the holder", ROT_R)
    tray = np.array([SLOT_X[0], TRAY_XY[1] - TUBE_MID_FROM_GRIP, FLOOR_TOP + R_CAP + 0.008])
    d_flat = ROT_R_FLAT[:, 2]
    tcp = tray - 0.004 * d_flat
    _go(plan, 3.0, tcp - 0.06 * d_flat, W_R_HOLD, "right: carry it level to the tray", ROT_R_FLAT)
    _go(plan, 1.5, tcp, W_R_HOLD, "right: lower it into the slot", ROT_R_FLAT)
    _go(plan, 0.5, tcp, W_R_HOLD, "settle", ROT_R_FLAT)
    _go(plan, 0.8, tcp, W_R_LET, "right: let go", ROT_R_FLAT)
    _go(plan, 1.5, tcp - 0.08 * d_flat, W_R_LET, "right: up and away", ROT_R_FLAT)
    _go(plan, 1.5, np.array([0.20, -0.25, TOP + 0.20]), W_R_OPEN, "right: park", ROT_R_FLAT)


T_LEFT_GRIP = 5.6  # s: the left hand is closed on the barrel
T_FILL_WAIT = 3.0  # s waited after the grip on the tube, so the fill is complete before the pull


def demo():
    """Left: hold the holder's barrel (from 5.6 s to the end, closed loop on the needle's travel). Right: squeeze the
    tube onto the needle (pierce, push home), let go, grip the tube, wait for the fill, ease it off the needle and out
    of the holder, carry it level to the tray and lay it in a slot."""
    right: list = []
    _squeeze(right, wait_s=T_LEFT_GRIP + 0.4 - 3.0)
    _tube_grip(right)
    _go(right, T_FILL_WAIT, TCP_R, W_R_HOLD, "right: hold while the tube fills", ROT_R)
    p_out = _pull(right)
    _to_tray(right, p_out)
    t_right = sum(w.duration for w in right)
    left: list = []
    _left_grip(left, hold_s=t_right - T_LEFT_GRIP + 6.0)
    return {"left": left, "right": right}


def _ready_tcp(arm):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), np.array(station.BASES[arm]))


# Unbraced push (control a): the closed finger tips on the tube's end, pushing along the needle's line.
ROT_PUSH = _frame(np.array([1.0, 0.0, 0.0]), A)
TCP_PUSH = world((TUBE_END0 - 0.001, 0.0, 0.0)) - (TIP + 0.0015) * A


def controls_push():
    """Control (a): nothing holds the holder; the right hand's closed finger tips push the tube in by its end."""
    right: list = []
    _go(right, 3.0, TCP_PUSH - 0.05 * A, 0.0, "right: behind the tube's end, fingers closed", ROT_PUSH)
    _go(right, 1.5, TCP_PUSH, 0.0, "right: finger tips at the tube's end", ROT_PUSH)
    _go(right, 3.0, TCP_PUSH + 0.020 * A, 0.0, "[ctl-a] right: push the tube in, holder free", ROT_PUSH)
    _go(right, 1.0, TCP_PUSH + 0.020 * A, 0.0, "[ctl-a] hold", ROT_PUSH)
    _go(right, 1.5, TCP_PUSH - 0.05 * A, 0.0, "right: back", ROT_PUSH)
    left = [Waypoint(sum(w.duration for w in right), tuple(float(v) for v in _ready_tcp("left")[0]), fw(0.03),
                     "left: stays in its ready pose", Tilt(_ready_tcp("left")[1]))]
    return {"left": left, "right": right}


def controls_letgo_yank():
    """Controls (c) then (b): as the demo up to the right hand's grip on the filling tube, needle in the vein; then
    the left hand lets go of the holder (let-go latch) and the right hand at once yanks the tube out with the holder
    free (needle-out latch, the needle dragged back by the stopper)."""
    right: list = []
    _squeeze(right, wait_s=T_LEFT_GRIP + 0.4 - 3.0)
    _tube_grip(right)
    _go(right, 1.0, TCP_R, W_R_HOLD, "right: hold the tube", ROT_R)
    t_yank = sum(w.duration for w in right)
    _go(right, 0.6, TCP_R, W_R_HOLD, "right: hold the tube (left lets go)", ROT_R)
    _go(right, 1.0, TCP_R - 0.050 * A, W_R_HOLD, "[ctl-b] right: yank the tube out, holder free", ROT_R)
    _go(right, 1.0, TCP_R - 0.050 * A - 0.05 * D_R, W_R_HOLD, "right: up", ROT_R)
    left: list = []
    _left_grip(left, hold_s=t_yank - T_LEFT_GRIP)
    _go(left, 0.4, TCP_L, W_L_OPEN, "[ctl-c] left: let go of the holder", ROT_L)
    _go(left, 1.5, TCP_L - 0.045 * D_L, W_L_OPEN, "[ctl-c] left: away", ROT_L)
    _go(left, 3.0, TCP_L - 0.045 * D_L, W_L_OPEN, "left: wait", ROT_L)
    return {"left": left, "right": right}


def _ready_tcp(arm):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), np.array(station.BASES[arm]))


def controls():
    """Sim controls in one trial, each logging its latch: (c) the left hand grips the holder, then lets go of it;
    (a) the right hand grips the tube's body and pushes it in with the holder free (the needle goes deeper; at its stop
    the push pierces the stopper); (b) the right hand yanks the tube back out, holder still free (the needle is
    dragged back out of the vein)."""
    left: list = []
    _left_grip(left, hold_s=2.0)
    _go(left, 0.8, TCP_L, W_L_OPEN, "[ctl-c] left: let go of the holder", ROT_L)
    _go(left, 1.5, TCP_L - 0.045 * D_L, W_L_OPEN, "[ctl-c] left: away", ROT_L)
    right: list = [Waypoint(sum(w.duration for w in left) - 0.5, tuple(float(v) for v in _ready_tcp("right")[0]),
                            fw(0.03), "right: wait in the ready pose", Tilt(_ready_tcp("right")[1]))]
    _tube_grip(right)
    _go(right, 0.5, TCP_R, W_R_HOLD, "settle", ROT_R)
    _go(right, 3.0, TCP_R + 0.024 * A, W_R_HOLD, "[ctl-a] right: push the tube in, holder free", ROT_R)
    _go(right, 1.0, TCP_R + 0.024 * A, W_R_HOLD, "[ctl-a] hold", ROT_R)
    _go(right, 1.5, TCP_R - 0.050 * A, W_R_HOLD, "[ctl-b] right: yank the tube out, holder free", ROT_R)
    _go(right, 1.0, TCP_R - 0.050 * A - 0.05 * D_R, W_R_HOLD, "right: up", ROT_R)
    _go(left, sum(w.duration for w in right) - sum(w.duration for w in left) + 1.0, TCP_L - 0.045 * D_L, W_L_OPEN,
        "left: wait", ROT_L)
    return {"left": left, "right": right}


def _ready_tcp(arm):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), np.array(station.BASES[arm]))


# -- offline self-test -----------------------------------------------------------------------------------------------
def _states(q_t=0.0, q_s=0.0, tube=None):
    moved = {}
    if tube is not None:
        moved["tube"] = tube
    return kit.fake_states(q={"tissue": q_t, "skewer": q_s}, moved=moved)


def _tube_in_tray(slot=0, lift=0.0, dx=0.0):
    r = mk.axis_angle((1.0, 0.0, 0.0), ANGLE) @ R_H  # the tube level, cap to +y
    # place the tube's local midpoint over the slot, its lowest point on the floor
    mid_local = np.array([(CAP_FACE0 + TUBE_END0) / 2, 0.0, 0.0])
    want = np.array([SLOT_X[slot] + dx, TRAY_XY[1], FLOOR_TOP + R_CAP + 0.0005 + lift])
    return (want - r @ mid_local, mk.mat_to_quat(r))


def _geometry_check() -> None:
    """Fit checks: the needle's tip in the vein, the tube's stopper through the flange's opening and the bore, the
    squeeze span inside the gripper's 94 mm, the squeeze pads clear of the barrel, the tube in a slot."""
    assert R_CAP < R_IN - 0.0005 and R_CAP * 2 < 2 * R_IN, "stopper must pass the bore"
    tip = world((NEEDLE_TIP, 0, 0))
    assert 0.002 < SITE[2] - tip[2] < 0.006, tip  # 2-6 mm under the skin
    assert 0.0 < W_SQ < 0.09 and SQ_OPEN < 0.0939 and SQ_CLOSE > 0.0, (W_SQ, SQ_OPEN, SQ_CLOSE)
    assert TIP_F > R_OUT + 0.002
    top = max(R_H @ (mk.quat_to_mat(b.quat) @ np.array([0, 0, b.size[2] / 2]) + np.asarray(b.at)) @ UP_PSI
              for b in _flange_boxes())
    assert abs(top - WING) < 1e-6, ("flange wing not along the squeeze's up", top)
    st = _states(tube=_tube_in_tray())
    lo, hi = st["tube"].low, st["tube"].high
    assert hi[0] - lo[0] < SLOT_W and hi[1] - lo[1] < SLOT_L, (lo, hi)
    print(f"squeeze: closing axis tipped {math.degrees(THETA):.1f} deg off the needle's line, pads {W_SQ * 1000:.1f} mm "
          f"apart at touch (open {SQ_OPEN * 1000:.1f}, commanded {SQ_CLOSE * 1000:.1f}); needle tip "
          f"{(SITE[2] - tip[2]) * 1000:.1f} mm under the skin; tube end {(-(TUBE_END0 - FLANGE[0])) * 1000:.0f} mm out "
          f"of the flange", file=sys.stderr)


def self_test():
    """The scoring rules on hand-built states (memory flags as the sim's physics hook sets them)."""
    global _TEST_TCPS
    import dataclasses

    _geometry_check()
    base = _states()
    on = _states(q_s=TRAVEL)
    out_tube = _states(q_s=-0.06)
    tray = _states(tube=_tube_in_tray())
    tray_hi = _states(tube=_tube_in_tray(lift=0.01))
    tray_off = _states(tube=_tube_in_tray(dx=(SLOT_W + RIB) / 2))  # on the middle rib
    moving = dict(tray)
    moving["tube"] = dataclasses.replace(moving["tube"], lin_vel=np.array([0.0, 0.0, -0.05]))
    near = [np.asarray(tray["tube"].pos)]
    F = dict(pierced=1, fill=1.0, removed=1, held=True)

    cases = {  # name: (state, memory flags, tcps, score)
        "start": (base, {}, [], 0.0),
        "pierced, filling 40 %": (on, dict(pierced=1, fill=0.4, held=True), [], 0.25),
        "pierced, full, still on the needle": (on, dict(pierced=1, fill=0.95, held=True), [], 0.5),
        "full, off the needle, in the hand": (out_tube, F, [], 0.75),
        "full, in the tray, let go, holder held": (tray, F, [], 1.0),
        "full, in the tray, holder not held now": (tray, {**F, "held": False}, [], 0.75),
        "full, in the tray but 1 cm up (on a rib)": (tray_hi, F, [], 0.75),
        "full, beside the tray slot": (tray_off, F, [], 0.75),
        "full, in the tray, still moving": (moving, F, [], 0.75),
        "full, in the tray, gripper still on it": (tray, F, near, 0.75),
        "underfilled (60 %), in the tray": (tray, {**F, "fill": 0.6}, [], 0.25),
        "needle left the vein (latched)": (tray, {**F, "out": 5}, [], 0.0),
        "holder let go (latched)": (tray, {**F, "let_go": 5}, [], 0.0),
        "never pierced, tube in the tray": (tray, {**F, "pierced": None, "fill": 0.0}, [], 0.0),
        "unbraced push: pierced but needle out": (on, dict(pierced=1, fill=0.0, out=3, held=False), [], 0.0),
        "unbraced pull: full, removed, needle out": (out_tube, {**F, "out": 9}, [], 0.0),
        "needle pushed sideways after the fill (latched)": (tray, {**F, "side": 7}, [], 0.0),
    }
    try:
        for name, (st, flags, tcps, want) in cases.items():
            mk.new_episode()
            _mem().update(flags)
            _TEST_TCPS = tcps
            got = score(st)
            assert abs(got - want) < 1e-9 and success(st) == (want >= 1.0), (name, got, want)
        # readouts and rules
        assert abs(kit.q(on, "skewer") - TRAVEL) < 1e-9 and abs(kit.q(_states(q_t=0.003), "tissue") - 0.003) < 1e-9
        assert drag_cap(-0.01) == 0.0 and abs(drag_cap(0.002) - F_DRAG / 2) < 1e-9 and drag_cap(0.006) == F_DRAG
        # the sideways and pivot gates: pose readouts, then the latch firing from hand-built holder poses
        def pose(shift=(0.0, 0.0, 0.0), piv_deg=0.0, axis=(1.0, 0.0, 0.0), spin_deg=0.0):
            r = mk.axis_angle(axis, math.radians(piv_deg)) @ mk.axis_angle(A, math.radians(spin_deg)) @ R_H
            return SITE + np.asarray(shift), r  # pivoting about the skin point (the holder's origin)
        q0, l0, v0 = needle_pose(*pose())
        assert abs(q0) < 1e-9 and l0 < 1e-9 and v0 < 1e-6
        q1, l1, v1 = needle_pose(*pose(shift=0.0015 * A, spin_deg=30.0))  # along the line and spun: neither gate
        assert abs(q1 - 0.0015) < 1e-9 and l1 < 1e-9 and v1 < 1e-6
        _, l2, _ = needle_pose(*pose(shift=0.0025 * np.array([1.0, 0.0, 0.0])))  # 2.5 mm across (away from the robot)
        assert abs(l2 - 0.0025) < 1e-9
        _, l3, v3 = needle_pose(*pose(piv_deg=4.0))  # 4 deg about the vertical through the skin point
        assert abs(v3 - 4.0) < 1e-6 and abs(l3 - abs(HUB_FACE) * math.sin(math.radians(4.0))) < 1e-9
        for p_h, rot, kinds in ((*pose(shift=0.0019 * U), set()), (*pose(piv_deg=2.9, axis=U), set()),
                                (*pose(shift=0.0021 * U), {"sideways"}), (*pose(piv_deg=3.1, axis=U), {"pivot"}),
                                (*pose(shift=0.0025 * YL, piv_deg=8.0, axis=YL), {"sideways", "pivot"})):
            mk.new_episode()
            m = _mem()
            m.update(F)
            m["t"] = 4000  # after the fill
            _, lat, piv = needle_pose(p_h, rot)
            _check_side(m, lat, piv)
            assert m["side_kinds"] == kinds and (m["side"] is not None) == bool(kinds), (kinds, lat, piv)
            _TEST_TCPS = []
            assert (score(tray) == 0.0) == bool(kinds), kinds
        z = [0.0] * 8
        assert holder_held([1, 1, 0, 0, 0, 0, 0, 0]) and holder_held([0, 0, 0, 1, 0, 0, 1, 0])
        assert not holder_held(z) and not holder_held([0, 0, 0, 0, 0, 0, 1, 1]) and not holder_held([1, 0, 0, 0, 0, 0, 0, 0])
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 4


if __name__ == "__main__":
    # Offline checks, no GPU: geometry and the scoring rules on hand-built states, then both arms' plans (demo and
    # sim controls) replayed through the kinematics at 20 Hz, every joint inside the URDF limits, IK under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan

    print(f"scoring: {self_test()} cases pass")
    hz = 20.0
    for kind, fn in (("demo", demo), ("control a (not the demo)", controls_push),
                     ("controls c+b (not the demo)", controls_letgo_yank)):
        for arm, waypoints in fn().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (kind, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (kind, arm)
            print(f"{kind} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
