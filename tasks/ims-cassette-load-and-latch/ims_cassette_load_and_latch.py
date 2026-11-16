"""Fix an instrument misloaded across a 5-instrument sterilization cassette, seat both loose ones, close the lid flat and
hook its latch clip under the keeper lip.

Real product: stainless 5-instrument exam cassette, 76 x 203 x 32 mm closed (maker catalog, IMS 2020 p.6): two silicone
rails across the 76 mm width, instruments lying along the 203 mm length, lid hinged on the back long edge, a small
pivoting latch clip at the middle of the lid's front edge (catalog p.5). The catalog's troubleshooting table (p.55):
"improperly placed instruments" break hinges, "instruments should not protrude from cassette", "only light force is
needed to close the cassette". Estimates (not in the catalog): 9.5 mm handles, 12.7 mm notch pitch, rail spacing,
lid pad height, clip size and pivot friction.

Bench layout in station coordinates (TOP = station.TABLE_TOP, +X forward, +Y left; arm bases at y = +-0.305 m):
- cassette (fixed in a low bench holder): base with two rails (5 notches each) at y = +-60 mm, lid on a hinge "lid"
  along the back long edge (+X), standing open at 120 deg; latch clip on a hinge "clip" riding on the lid, open: leaning back over
  the lid top at 125 deg.
- keeper: the latch lip on the cassette's front wall, a separate fixed body so the clip's hook tab collides with it
  (the clip is a link of the cassette articulation, whose links do not collide with each other).
- inst1, inst3, inst5: loose instruments already snapped into slots 1, 3, 5 (held only by the sprung rail jaws);
  inst2: loose on the instrument tray in front of the cassette; inst4 (the explorer): misloaded, lying across the
  cassette on the front rim and the seated handles, so the lid's pads cannot come down. Free slots: 2 and 4.
- A handle lying across the rail tops instead of down in its notches holds the lid's pads up: the lid then stays
  open at the front and cannot be closed flat (collision geometry, not script).
- Optional diagnostic (env IMS_PULL_TEST=1, never in the official runs): an upward pull on the lid, once while it is
  closed but unlatched and once after the clip is hooked; the log lines PULL ... give the lid's lift each time.

Run it:
  inspect-robots run --embodiment yam_arena -E task=ims_cassette_load_and_latch.py -E object_jitter=0 \\
    -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<see task.json>" --scorer task_score ...
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import labkit  # noqa: E402
from labkit import Joint, PartState, on_reset, on_step, part  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

# -- geometry (single source of truth for score(), success(), demo()) ---------------------------------------------

SCENE = "station"
TOP = station.TABLE_TOP

CXY = (0.300, 0.000)  # cassette centre
HOLD = 0.006  # holder plate under the cassette (and the instrument tray beside it)
CAS_X, CAS_Y = 0.076, 0.203  # closed footprint (catalog)
BASE_H = 0.017  # base rim height above its floor plate's underside
LID_H = 0.015  # lid depth (closed total 32 mm)
WALL = 0.0015  # perforated stainless sheet
PITCH, N_SLOT = 0.0127, 5
SLOT_X = tuple((i - 2) * PITCH for i in range(N_SLOT))  # slot 1..5 centres along X (cassette frame)
RAIL_Y, RAIL_T = 0.060, 0.008  # rail centres at y = +-60 mm, 8 mm thick along Y
NOTCH_W = 0.0105
FLOOR_Z, TOOTH_Z = 0.004, 0.016  # notch floor and tooth top above the cassette floor
TOOTH_W = PITCH - NOTCH_W  # 2.2 mm
HANDLE_R, HANDLE_L = 0.00475, 0.130  # 9.5 mm round handle (estimate); its ends reach 5 mm past the rail centres
TIP_R, TIP_L = 0.0015, 0.0175  # working end / shank, each side (165 mm overall)
INST_M = 0.020
PAD_Z = 0.0143  # lid pad underside when closed, above the cassette floor (0.8 mm over a seated handle)
PAD_Y = 0.040  # pads span y = +-40 mm (between the rails)
LID_OPEN = math.radians(120.0)  # leaning well back on its stop
CLIP_L, CLIP_W, CLIP_T = 0.023, 0.012, 0.002  # latch clip (estimate)
CLIP_UP = math.radians(125.0)  # open: leaning back over the lid top against its stop (gravity holds it there)
CLIP_OUT = 0.0025  # clip pivot this far in front of the lid's front face
TAB = (0.0015, 0.0012)  # hook tab at the clip's tip (depth, height): swung down, it sits under the base lip
LIP = (0.0012, 0.024, 0.0012)  # base lip on the front wall (out, along Y, height), 4 mm under the rim
# Rail grip (catalog: silicone rails hold the instruments): one sprung jaw per rail at the mouth of every notch
# on the notch's +X side; it narrows the 10.5 mm mouth to 8.0 mm, so a 9.5 mm handle rests on it until
# pressed home, then snaps back over the handle (proof: seat force 0.6-2.6 N over friction 0.39-1.43). The 0.75 mm
# overlap per side is well above contact softness, so both MuJoCo and PhysX have to push the jaw back (1.5 mm overlap
# with a 15 kN/m jaw passed in MuJoCo by soft contact alone and jammed in Isaac).
JAW_R, JAW_IN, JAW_Z = 0.0010, 0.0025, 0.0146  # jaw tip radius, protrusion into the mouth, tip height
JAW_K, JAW_TRAVEL = 3000.0, 0.0026  # N/m (estimate: the lid's weight on its pads must not push a handle home), retract travel
WORK = (1, 3)  # slot indices of the two free slots (2 and 4)
LIPPED = tuple(range(N_SLOT))  # every notch has its sprung lips (a real rail grips all five)
TRAY_X, TRAY_W = -0.078, 0.064  # instrument tray in front of the cassette (cassette frame X, width along X)
SPARE_X = (-0.060, -0.096)  # tray places 36 mm apart (the open jaw spans 42 mm); only inst2 starts on the tray
MIS_X, MIS_Y = -0.030, 0.047  # misloaded inst4: centre in the cassette frame (back tip clear of the open lid)
MIS_GRIP_X, MIS_GRIP_Z = -0.0127, 0.0210  # grip point along it (over the empty slot 2) and its centre height there

Z0 = TOP + HOLD  # cassette floor underside (world)
SEAT_Z = FLOOR_Z + HANDLE_R  # seated handle centre above the floor (8.75 mm)


def _rest_on_mouth():
    """Handle resting on a notch's mouth: on the -X tooth's rounded top and on the jaw (not pressed home)."""
    ta = (-PITCH / 2, JAW_Z - JAW_R - 0.0002 - TOOTH_W / 2, TOOTH_W / 2 + HANDLE_R)  # -X tooth cap (cut down under its own jaw)
    jb = (NOTCH_W / 2 - JAW_IN + JAW_R, JAW_Z, JAW_R + HANDLE_R)
    best = None
    for k in range(4001):
        x = ta[0] + (jb[0] - ta[0]) * k / 4000
        za = ta[1] + math.sqrt(max(ta[2] ** 2 - (x - ta[0]) ** 2, 0.0))
        zb = jb[1] + math.sqrt(max(jb[2] ** 2 - (x - jb[0]) ** 2, 0.0))
        z = max(za, zb)
        if best is None or z < best[1]:
            best = (x, z)
    return best


REST_DX, REST_Z = _rest_on_mouth()  # ~(-0.6, 16.6) mm: the handle's top stands ~12 mm above a seated one's
REST_Z += 0.0003

# -- scoring thresholds (shared by on_step, score, success) -------------------------------------------------------

SEAT_TOL_XY = 0.003  # handle centre within 3 mm of its slot line
SEAT_TOL_Y = 0.008  # along the slot: both handle ends at least 1 mm onto their rail (rail inner edges at +-56 mm)
SEAT_TOL_Z = 0.0010  # and down in the notches: centre no more than 1 mm above the seated height
LID_FLAT_RAD = math.atan2(0.001, CAS_X)  # front edge gap <= 1 mm
CLIP_LOCK_RAD = math.radians(88.0)  # clip swung down >= 88 deg
HOOK_OVERLAP = 0.0005  # hooked: the tab's top edge under the keeper lip, overlapping it by >= 0.5 mm along X,
HOOK_GAP = 0.0020  # and no more than 2 mm below the lip's underside
HAND_CLEAR_M = 0.050
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
PASS_SCORE = 1.0 - 1e-9
S_EACH, S_LID, S_LATCH = 0.2, 0.3, 0.3
NAMES = ("inst1", "inst2", "inst3", "inst4", "inst5")
SLOT_OF = {n: i for i, n in enumerate(NAMES)}  # home slot (index into SLOT_X); any free slot counts, see _slot_of

# -- colours ------------------------------------------------------------------------------------------------------

STEEL = (0.74, 0.75, 0.77)
RAIL_BLUE = (0.16, 0.38, 0.78)
DARK = (0.12, 0.12, 0.13)
CLIP_RED = (0.78, 0.16, 0.14)


def _base_boxes():
    hx, hy = CAS_X / 2, CAS_Y / 2
    b = [
        ((0.0, 0.0, -HOLD / 2), (CAS_X + 0.024, CAS_Y + 0.024, HOLD), DARK),  # holder plate
        ((0.0, 0.0, 0.0005), (CAS_X, CAS_Y, 0.001), STEEL),  # floor
        ((hx - WALL / 2, 0.0, BASE_H / 2), (WALL, CAS_Y, BASE_H), STEEL),  # back wall
        ((-hx + WALL / 2, 0.0, BASE_H / 2), (WALL, CAS_Y, BASE_H), STEEL),  # front wall
        ((0.0, hy - WALL / 2, BASE_H / 2), (CAS_X, WALL, BASE_H), STEEL),
        ((0.0, -hy + WALL / 2, BASE_H / 2), (CAS_X, WALL, BASE_H), STEEL),
    ]
    edges = [-(N_SLOT / 2) * PITCH + k * PITCH for k in range(N_SLOT + 1)]  # tooth centres
    for sy in (-1, 1):
        y = sy * RAIL_Y
        b.append(((0.0, y, FLOOR_Z / 2), (CAS_X - 2 * WALL, RAIL_T, FLOOR_Z), RAIL_BLUE))
        for xc in edges:
            zt = _tooth_top(xc)
            b.append(((xc, y, (FLOOR_Z + zt) / 2), (TOOTH_W, RAIL_T, zt - FLOOR_Z), RAIL_BLUE))
        for xo in (-1, 1):  # end blocks between the outer teeth and the walls
            x0 = xo * (N_SLOT / 2 * PITCH + TOOTH_W / 2)
            x1 = xo * (CAS_X / 2 - WALL)
            b.append((((x0 + x1) / 2, y, (FLOOR_Z + TOOTH_Z) / 2), (abs(x1 - x0), RAIL_T, TOOTH_Z - FLOOR_Z), RAIL_BLUE))
    return tuple(b)


def _jaw_side_teeth():
    return {round(SLOT_X[i] + PITCH / 2, 6) for i in LIPPED}


def _tooth_top(xc):
    """Box top of a tooth (its rounded cap sits on it): the tooth on each notch's +X side is cut down under its jaw."""
    if round(xc, 6) in _jaw_side_teeth():
        return JAW_Z - JAW_R - 0.0002 - TOOTH_W / 2
    return TOOTH_Z - TOOTH_W / 2


def _base_cyls():
    """Rounded tooth tops on every tooth (guide a handle into its notch). With lips at every notch the inner teeth are
    cut down under their jaw; a square corner there held a handle 5 mm above its seat in Isaac (build2 r1)."""
    edges = [-(N_SLOT / 2) * PITCH + k * PITCH for k in range(N_SLOT + 1)]
    return tuple(((xc, sy * RAIL_Y, _tooth_top(xc)), TOOTH_W / 2, RAIL_T, RAIL_BLUE, "Y")
                 for sy in (-1, 1) for xc in edges)


def _lid_boxes():
    """Lid at the closed position (joint 0), cassette frame."""
    hx, hy = CAS_X / 2, CAS_Y / 2
    zt = BASE_H + LID_H
    return (
        ((0.0, 0.0, zt - 0.0005), (CAS_X, CAS_Y, 0.001), STEEL),  # top sheet
        ((hx - WALL / 2, 0.0, BASE_H + LID_H / 2 + 0.0002), (WALL, CAS_Y, LID_H - 0.0004), STEEL),
        ((-hx + WALL / 2, 0.0, BASE_H + LID_H / 2 + 0.0002), (WALL, CAS_Y, LID_H - 0.0004), STEEL),
        ((0.0, hy - WALL / 2, BASE_H + LID_H / 2 + 0.0002), (CAS_X, WALL, LID_H - 0.0004), STEEL),
        ((0.0, -hy + WALL / 2, BASE_H + LID_H / 2 + 0.0002), (CAS_X, WALL, LID_H - 0.0004), STEEL),
        # press pads between the rails, hanging from the top sheet down to PAD_Z
        ((0.0, 0.0, (PAD_Z + zt - 0.001) / 2), (CAS_X - 2 * WALL - 0.002, 2 * PAD_Y, zt - 0.001 - PAD_Z), RAIL_BLUE),
    )


CLIP_PIVOT = (-CAS_X / 2 - CLIP_OUT, BASE_H + LID_H + 0.001)  # (x, z) on the lid's top front edge, cassette frame


def _clip_boxes():
    """Clip at joint 0 (horizontal, pointing -X), with a hook tab under its tip. It starts at CLIP_UP, leaning back over
    the lid top (over centre: gravity holds it on its stop); pushed forward past upright it swings down to -92 deg,
    where the tab sits under the base lip, so the lid's front edge cannot lift more than the ~0.9 mm gap above the
    tab. Cassette frame."""
    xp, zp = CLIP_PIVOT
    return (
        ((xp - CLIP_L / 2, 0.0, zp), (CLIP_L, CLIP_W, CLIP_T), CLIP_RED),
        ((xp - CLIP_L + TAB[0] / 2, 0.0, zp - CLIP_T / 2 - TAB[1] / 2), (TAB[0], CLIP_W, TAB[1]), CLIP_RED),
    )


_LID = Joint(
    "lid", kind="hinge", axis="Y", anchor=(CAS_X / 2, 0.0, BASE_H), boxes=_lid_boxes(),
    lower=0.0, upper=LID_OPEN, stiffness=0.0, damping=0.03, friction=0.002, initial=LID_OPEN, target=LID_OPEN, mass=0.12,
)
_CLIP = Joint(
    "clip", kind="hinge", axis="Y", parent="lid", anchor=(CLIP_PIVOT[0], 0.0, CLIP_PIVOT[1]),
    boxes=_clip_boxes(), lower=-math.radians(92.0), upper=CLIP_UP, stiffness=0.0, damping=0.0001, friction=0.0,
    initial=CLIP_UP, target=CLIP_UP, mass=0.004,
)


def _jaws():
    out = []
    for i in LIPPED:
        xj = SLOT_X[i] + NOTCH_W / 2 - JAW_IN + JAW_R
        for sy in (-1, 1):
            out.append(Joint(
                f"jaw{i + 1}{'l' if sy > 0 else 'r'}", kind="slide", axis="X", anchor=(xj, sy * RAIL_Y, JAW_Z),
                cylinders=(((xj, sy * RAIL_Y, JAW_Z), JAW_R, RAIL_T, RAIL_BLUE, "Y"),),
                lower=0.0, upper=JAW_TRAVEL, stiffness=JAW_K, damping=20.0, target=0.0, initial=0.0, mass=0.2))
    return tuple(out)


JAWS = _jaws()
CASSETTE = part("cassette", (*CXY, Z0), fixed=True, friction=0.5, boxes=_base_boxes(), cylinders=_base_cyls(),
                joints=(_LID, _CLIP, *JAWS))
# Cradle ridges on the tray (a round handle on a flat tray rolls off in PhysX: Isaac r3, inst4 off the tray front by
# step 40): two short ridges 5.5 mm either side of each spare handle (room for the +-5 mm jitter) near its ends, clear
# of the fingers at y = +-30 mm.
CRADLE_GAP, CRADLE_T, CRADLE_H, CRADLE_Y = 0.0055, 0.0015, 0.003, (0.048, 0.072)


def _tray_boxes():
    white = (0.82, 0.83, 0.85)
    b = [((0.0, 0.0, HOLD / 2), (TRAY_W, 0.190, HOLD), white), ((TRAY_W / 2, 0.0, HOLD + 0.002), (0.003, 0.190, 0.004), white)]
    yc, yl = (CRADLE_Y[0] + CRADLE_Y[1]) / 2, CRADLE_Y[1] - CRADLE_Y[0]
    for s in SPARE_X:
        for sx in (-1, 1):
            for sy in (-1, 1):
                b.append(((s - TRAY_X + sx * (HANDLE_R + CRADLE_GAP + CRADLE_T / 2), sy * yc, HOLD + CRADLE_H / 2),
                          (CRADLE_T, yl, CRADLE_H), white))
    return tuple(b)


# Keeper: the latch lip on the front wall, its own fixed body (see the module docstring); tab gap under it 0.9 mm.
LIP_Z = BASE_H - 0.004  # lip centre height (cassette frame)
KEEPER = part("keeper", (*CXY, Z0), fixed=True, friction=0.5, boxes=(((-CAS_X / 2 - LIP[0] / 2, 0.0, LIP_Z), LIP, STEEL),))
TRAY_PART = part("tray", (CXY[0] + TRAY_X, CXY[1], TOP), fixed=True, friction=0.5, boxes=_tray_boxes())


def _instrument(name, x, z, held=False, y=0.0, yaw=0.0):
    """A loose instrument (held=True: fixed in place, used only by offline controls)."""
    cyl = [((0.0, 0.0, 0.0), HANDLE_R, HANDLE_L, STEEL, "Y")]
    for s in (-1, 1):
        cyl.append(((0.0, s * (HANDLE_L / 2 + TIP_L / 2), 0.0), TIP_R, TIP_L, DARK, "Y"))
    if held:
        return part(name, (CXY[0] + x, CXY[1] + y, z), fixed=True, friction=0.5, cylinders=tuple(cyl), yaw=yaw)
    return part(name, (CXY[0] + x, CXY[1] + y, z), mass=INST_M, friction=0.5, cylinders=tuple(cyl), yaw=yaw)


INSTS = {
    "inst1": _instrument("inst1", SLOT_X[0], Z0 + SEAT_Z + 0.0002),
    "inst3": _instrument("inst3", SLOT_X[2], Z0 + SEAT_Z + 0.0002),
    "inst5": _instrument("inst5", SLOT_X[4], Z0 + SEAT_Z + 0.0002),
    "inst2": _instrument("inst2", SPARE_X[0], TOP + HOLD + HANDLE_R),
    # misloaded: lying across the cassette (handle along x) on the front rim and the seated instrument 5, so the pads
    # cannot come down -- the maker's "improperly placed instruments" case
    "inst4": _instrument("inst4", MIS_X, Z0 + BASE_H + HANDLE_R + 0.0003, y=MIS_Y, yaw=math.pi / 2),
}
OBJECTS = (CASSETTE, KEEPER, TRAY_PART, *INSTS.values())
# Pre-check start jitter (mj_check): the pre-seated instruments start in their notches and move with the cassette
# (fixed in its holder, so they keep their seats); the misloaded explorer and the mirror keep their own jitter.
SEATED_IN = {"inst1": "cassette", "inst3": "cassette", "inst5": "cassette"}

# Visual meshes (assets/visual, built by assets_build/build_visuals.py); the shapes above stay as invisible colliders.
_VIS = pathlib.Path(_HERE) / "assets" / "visual"
labkit.VISUALS.update({
    "cassette/body": str(_VIS / "cassette.usdc"),
    "cassette/lid": str(_VIS / "lid.usdc"),
    "cassette/clip": str(_VIS / "clip.usdc"),
    "tray/body": str(_VIS / "tray.usdc"),
    **{f"cassette/{j.name}": str(_VIS / f"{j.name}.usdc") for j in JAWS},
    **{f"{n}/body": str(_VIS / f"{n}.usdc") for n in NAMES},
})

# -- trial state ---------------------------------------------------------------------------------------------------

_trial: dict = {}
_ids: dict = {}


def _fresh() -> dict:
    return {"done_s": 0.0, "tcps": None, "step": -1, "flat_n": 0, "pull": None, "pull_max": 0.0, "pull_n": 0, "pulls": 0}


# Diagnostic only (env IMS_PULL_TEST=1; never set in the official runs): PULL_N upward on the lid's centre of mass,
# (a) for PULL_PULSE steps once the lid has lain flat for 1 s with the clip still up (unlatched), and (b) from the
# first step the latch scores until the run ends (hooked). Each logs the lid's largest angle as a PULL line.
PULL_TEST = bool(int(__import__("os").environ.get("IMS_PULL_TEST", "0") or 0))
PULL_N, PULL_PULSE = 2.0, 4


_trial.update(_fresh())


def _set_test_trial(**kw) -> None:
    """Explicit test hook for offline scoring tests (never called by demo())."""
    for k, v in kw.items():
        if k not in _trial:
            raise KeyError(k)
        _trial[k] = tuple(np.asarray(p, float) for p in v) if k == "tcps" and v is not None else v


def _drive_targets(base) -> None:
    """Isaac aims every implicit drive at the default joint position; the lid and the clip have no spring (stiffness
    0), so this only pins their targets to the start pose -- kept explicit so no drive pulls the lid open or shut."""
    import torch

    asset = base.scene["cassette"]
    if "joints" not in _ids:
        names = list(asset.joint_names)
        _ids["joints"] = [names.index(_LID.joint_name), names.index(_CLIP.joint_name)]
    asset.set_joint_position_target(torch.tensor([[LID_OPEN, CLIP_UP]], device=asset.device), joint_ids=_ids["joints"])


def _lid_force(base, fz: float) -> None:
    """Upward world force fz (N) on the lid link's centre of mass (Isaac Lab wrench API, body frame)."""
    import torch

    asset = base.scene["cassette"]
    if "lid_body" not in _ids:
        _ids["lid_body"] = asset.find_bodies("lid")[0]
    ids = _ids["lid_body"]
    if not ids:
        raise RuntimeError("no lid body in this simulator's cassette asset")
    w, x, y, z = (float(v) for v in labkit._np(asset.data.body_quat_w)[0, ids[0]])
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    f = r.T @ np.array([0.0, 0.0, fz])
    asset.set_external_force_and_torque(torch.tensor([[f.tolist()]], device=asset.device),
                                        torch.zeros((1, 1, 3), device=asset.device), body_ids=ids)


def _pull_test(base, objs) -> None:
    cas = objs["cassette"]
    lid = math.degrees(_lid_q(cas))
    t = _trial
    t["flat_n"] = t["flat_n"] + 1 if _lid_flat(cas) else 0
    try:
        if t["pull"] is None and t["pulls"] == 0 and t["flat_n"] >= 20 and math.degrees(_clip_q(cas)) > 60.0:
            t.update(pull="unlatched", pull_max=lid, pull_n=0)
        elif t["pull"] is None and t["pulls"] == 1 and score(objs) >= PASS_SCORE:
            t.update(pull="hooked", pull_max=lid, pull_n=0)
        if t["pull"] is not None:
            t["pull_max"] = max(t["pull_max"], lid)
            t["pull_n"] += 1
            on = t["pull"] == "hooked" or t["pull_n"] <= PULL_PULSE
            _lid_force(base, PULL_N if on else 0.0)
            if t["pull"] == "unlatched" and t["pull_n"] >= 30:
                print("PULL " + json.dumps({"case": "unlatched", "force_N": PULL_N, "steps_on": PULL_PULSE,
                                            "lid_max_deg": round(t["pull_max"], 2), "step": t["step"]}), flush=True)
                t.update(pull=None, pulls=1)
            elif t["pull"] == "hooked" and t["pull_n"] % 10 == 0:
                print("PULL " + json.dumps({"case": "hooked", "force_N": PULL_N, "steps_on": t["pull_n"],
                                            "lid_max_deg": round(t["pull_max"], 2), "step": t["step"]}), flush=True)
    except Exception as e:  # diagnostic only: never break the run
        print(f"PULL error {type(e).__name__}: {e}", flush=True)
        t["pulls"] = 9


@on_reset
def _reset(_env) -> None:
    _trial.clear()
    _trial.update(_fresh())
    _ids.clear()


def _arm_tcps(env):
    try:
        base = env.unwrapped
        out = []
        for side in ("left", "right"):
            asset = base.scene[f"{side}_robot"]
            key = f"arm_{side}"
            if key not in _ids:
                _ids[key] = asset.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = labkit._np(asset.data.joint_pos)[0, _ids[key]]
            tcp_p, _ = ik.tcp_pose(q, station.BASES[side])
            out.append(tcp_p)
        return (out[0], out[1])
    except Exception:
        return None


def _local(ps: PartState):
    """Instrument centre in the cassette frame (x, y, z above the cassette floor)."""
    p = np.asarray(ps.pos, float)
    return p[0] - CXY[0], p[1] - CXY[1], p[2] - Z0


def _axis_ok(ps: PartState) -> bool:
    """Handle axis lies along Y (within 10 deg) and level."""
    x, y, z, w = (float(v) for v in ps.quat_xyzw)
    ay = np.array([2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)])  # body Y axis in world
    return abs(ay[1]) >= math.cos(math.radians(10.0))


def _slot_of(ps: PartState):
    """Index of the slot this instrument is seated in, or None."""
    x, y, z = _local(ps)
    if abs(y) > SEAT_TOL_Y or z - SEAT_Z > SEAT_TOL_Z or z < SEAT_Z - 0.003 or not _axis_ok(ps):
        return None
    for i, sx in enumerate(SLOT_X):
        if abs(x - sx) <= SEAT_TOL_XY:
            return i
    return None


def _seated(objects) -> dict:
    """{name: slot} for seated instruments, one per slot (a second one in the same slot does not count)."""
    out, used = {}, set()
    for n in NAMES:
        s = _slot_of(objects[n])
        if s is not None and s not in used:
            out[n] = s
            used.add(s)
    return out


def _lid_q(cas: PartState) -> float:
    return float(cas.joints.get("lid", LID_OPEN))


def _clip_q(cas: PartState) -> float:
    return float(cas.joints.get("clip", CLIP_UP))


def _lid_flat(cas: PartState) -> bool:
    return _lid_q(cas) <= LID_FLAT_RAD


def _ry(q, x, z):
    """Rotate (x, z) by a hinge angle q about +Y."""
    return x * math.cos(q) + z * math.sin(q), -x * math.sin(q) + z * math.cos(q)


def _tab_top(cas: PartState):
    """(x0, x1, z) of the hook tab's top edge in the cassette frame, from the lid and clip hinge angles."""
    ql, qc = _lid_q(cas), _clip_q(cas)
    ax, az = CAS_X / 2, BASE_H  # lid hinge
    out = []
    for zt in (-CLIP_T / 2, -CLIP_T / 2 - TAB[1]):  # the tab's top edge in the clip frame (x = -CLIP_L + TAB depth)
        x, z = _ry(qc, -CLIP_L + TAB[0], zt)
        x, z = _ry(ql, CLIP_PIVOT[0] + x - ax, CLIP_PIVOT[1] + z - az)
        out.append((x + ax, z + az))
    (xa, za), (xb, zb) = out
    return min(xa, xb), max(xa, xb), max(za, zb)


def _hooked(cas: PartState) -> bool:
    """The hook tab sits under the keeper lip: overlapping it along X and just below its underside."""
    x0, x1, z = _tab_top(cas)
    lx0, lx1, lz = -CAS_X / 2 - LIP[0], -CAS_X / 2, LIP_Z - LIP[2] / 2
    return min(x1, lx1) - max(x0, lx0) >= HOOK_OVERLAP and lz - HOOK_GAP <= z <= lz + 0.0001


def _latched(cas: PartState) -> bool:
    return _lid_flat(cas) and -_clip_q(cas) >= CLIP_LOCK_RAD and _hooked(cas)


def _hands_clear(tcps) -> bool:
    """Both tool-centre points outside the cassette + tray outline grown by 5 cm, up to 5 cm above the open lid."""
    if tcps is None or len(tcps) < 2:
        return False
    cx, cy = CXY
    lo = np.array([cx + TRAY_X - TRAY_W / 2, cy - CAS_Y / 2, TOP]) - HAND_CLEAR_M
    hi = np.array([cx + CAS_X / 2 + 0.020, cy + CAS_Y / 2, Z0 + BASE_H + CAS_X]) + HAND_CLEAR_M
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


SUBACTIONS = (
    ("seat instrument 2 in a free slot", "inst2 slot"),
    ("seat instrument 4 in a free slot", "inst4 slot"),
    ("close the lid flat", "cassette.lid (deg)"),
    ("lock the latch clip", "cassette.clip (deg)"),
)


def _log_subaction_end(objects, step: int, run_end: bool) -> None:
    seated = _seated(objects)
    cas = objects["cassette"]
    ends = (seated.get("inst2", -1) + 1, seated.get("inst4", -1) + 1,
            round(math.degrees(_lid_q(cas)), 2), round(math.degrees(_clip_q(cas)), 2))
    for (name, key), end in zip(SUBACTIONS, ends):
        rec = {"name": name, "joint_or_body": key, "end_value": end, "step": step, "run_end": run_end}
        print("SUBACTION_END " + json.dumps(rec), flush=True)


@on_step
def _watch(env) -> None:
    base = env.unwrapped
    objs = {n: labkit._part_state(env, labkit._SPECS[n]) for n in ("cassette", *NAMES)}
    _trial["step"] += 1
    _trial["tcps"] = _arm_tcps(env)
    if _trial["step"] % 40 == 39:  # observe-only state log every 2 s
        cas = objs["cassette"]
        print("STATE " + json.dumps({"step": _trial["step"] + 1, "seated": {n: s + 1 for n, s in _seated(objs).items()},
                                     "lid_deg": round(math.degrees(_lid_q(cas)), 1),
                                     "clip_deg": round(math.degrees(_clip_q(cas)), 1),
                                     "pos_mm": {n: [round(v * 1000, 1) for v in _local(objs[n])] for n in ("inst2", "inst4")},
                                     "axis_ok": {n: bool(_axis_ok(objs[n])) for n in ("inst2", "inst4")}}), flush=True)
    dt = float(getattr(base, "step_dt", 0.05))
    if PULL_TEST:
        _pull_test(base, objs)
    _trial["done_s"] = _trial["done_s"] + dt if score(objs) >= PASS_SCORE else 0.0
    if _trial["done_s"] > 0.0:
        _log_subaction_end(objs, _trial["step"], _trial["done_s"] >= SUCCESS_HOLD_S - 1.5 * dt)
    _drive_targets(base)


# -- scoring -------------------------------------------------------------------------------------------------------

def score(objects: dict[str, PartState]) -> float:
    """End state only (eye, ruler, 1 mm feeler gauge); no history:
    - 0.2 for each of instruments 2 and 4 seated in a free slot (handle down in both rail notches, along the slot).
    - +0.3 lid closed flat (front gap <= 1 mm) and +0.3 latch clip swung down with its hook tab under the keeper lip
      (overlap >= 0.5 mm, within 2 mm below it) -- both count only when all five instruments are seated; the latch
      only on a flat lid, and only with both grippers off the cassette.
    Possible: 0, 0.2, 0.4, 0.7, 1.0.
    """
    seated = _seated(objects)
    s = S_EACH * sum(1 for n in ("inst2", "inst4") if n in seated)
    if len(seated) < N_SLOT:
        return s
    cas = objects["cassette"]
    if not _lid_flat(cas):
        return s
    s += S_LID
    if _latched(cas) and _hands_clear(_trial["tcps"]):
        s += S_LATCH
    return min(s, 1.0)


def success(objects: dict[str, PartState]) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict[str, PartState]) -> float:
    cas = objects["cassette"]
    return float((N_SLOT - len(_seated(objects))) * PITCH + _lid_q(cas) * CAS_X
                 + max(0.0, CLIP_LOCK_RAD + _clip_q(cas)) * CLIP_L)


# -- scripted demo (fixed scene geometry only) ---------------------------------------------------------------------

def demo():
    """Both arms (open loop, fixed scene geometry only):
    1. Left arm grips the misloaded explorer (wrist 0), lifts it out, turns it a quarter turn so it lies along its slot,
       sets it on slot 4's notch mouths, lets go and presses it home past the rail jaws at its middle.
    2. Left arm presses each pre-seated instrument (slots 5, 3, 1) home, then parks.
    3. Right arm picks the mirror off the tray, sets it on slot 2's notch mouths and presses it home; parks.
    4. Left arm, fingers closed, comes round the cassette's far end and pushes the open lid from behind past upright; it
       swings shut onto its pads under its own weight; the left arm parks.
    5. Right arm pushes the clip (leaning back over the lid top) forward past upright; it swings down over centre and
       its hook tab goes under the keeper lip; both arms park clear.
    """
    W = _demo.Waypoint
    c = FINGER_CLOSED
    wide = 0.45 * FINGER_OPEN
    cx, cy = CXY
    rest_top = Z0 + REST_Z + HANDLE_R  # top of a handle resting on its notch mouth
    seat_top = Z0 + SEAT_Z + HANDLE_R  # top of a seated handle (under the jaws)
    hover = Z0 + TOOTH_Z + 0.05
    park_r = (0.16, -0.32, TOP + 0.20)
    park_l = (0.20, +0.26, TOP + 0.20)

    def load(x_from, x_to, y, yaw):
        """Pick the instrument off the tray (jaws across the handle), set it on its slot's notch mouths, let go (it rests
        on the rail jaws), then press it down with closed finger tips past the jaws until it bottoms; lift."""
        yp = 0.0  # press at the handle's middle, between the rails: pressed at y +-30 mm one end seated and the other
        # stayed on its lips (Isaac r4, handle 3.8 deg tilted)
        grip_tray = TOP + HOLD + HANDLE_R - 0.001 + PAD_TIP_PAST_TCP - 0.004
        set_z = Z0 + REST_Z + 0.002 - 0.001 + PAD_TIP_PAST_TCP - 0.004  # 2 mm above its resting height on the mouth
        return [
            W(1.2, (cx + x_from, cy + y, hover), wide, "over the instrument", yaw=yaw),
            W(0.8, (cx + x_from, cy + y, grip_tray), wide, "around the handle", yaw=yaw),
            W(0.8, (cx + x_from, cy + y, grip_tray), c, "grip the handle", yaw=yaw),
            W(0.8, (cx + x_from, cy + y, hover), c, "lift", yaw=yaw),
            W(1.2, (cx + x_to, cy + y, hover), c, "over its slot", yaw=yaw),
            W(0.8, (cx + x_to, cy + y, set_z), c, "down onto the notch mouths", yaw=yaw),
            W(0.5, (cx + x_to, cy + y, set_z), wide, "let go: it rests on the rail jaws", yaw=yaw),
            W(0.5, (cx + x_to, cy + y, rest_top + PAD_TIP_PAST_TCP + 0.010), wide, "up a little", yaw=yaw),
            W(0.4, (cx + x_to, cy + yp, rest_top + PAD_TIP_PAST_TCP + 0.010), c, "close the fingers", yaw=yaw),
            W(0.5, (cx + x_to, cy + yp, rest_top + PAD_TIP_PAST_TCP + 0.002), c, "tips on the handle", yaw=yaw),
            W(1.2, (cx + x_to, cy + yp, seat_top + PAD_TIP_PAST_TCP - 0.030), c, "press it home past the jaws", yaw=yaw),
            W(0.5, (cx + x_to, cy + yp, seat_top + PAD_TIP_PAST_TCP - 0.030), c, "hold", yaw=yaw),
            W(0.6, (cx + x_to, cy + yp, hover), c, "lift", yaw=yaw),
        ]

    r_load = load(SPARE_X[0], SLOT_X[1], -0.030, math.pi / 2)  # right arm: instrument 2 (wrist +90: jaws across)

    def unload_misloaded():
        """Left arm: jaws across the misloaded handle (wrist 0), lift it out, turn it a quarter turn in the air (wrist
        -90) so it lies along its slot, set it on slot 4's notch mouths, let go, press it home at its middle."""
        yaw0, yaw1 = 0.0, -math.pi / 2
        grip = Z0 + MIS_GRIP_Z - 0.001 + PAD_TIP_PAST_TCP - 0.004
        # held off-centre by (MIS_X - MIS_GRIP_X) along x; after the -90 deg turn that offset lies along -y
        ty = MIS_X - MIS_GRIP_X - 0.004  # TCP y that puts the handle centre on y = 0 (build2 ended 4 mm to +y)
        set_z = Z0 + REST_Z + 0.002 - 0.001 + PAD_TIP_PAST_TCP - 0.004
        gx, gy, sx = cx + MIS_GRIP_X, cy + MIS_Y, cx + SLOT_X[3]
        return [
            W(1.2, (gx, gy, hover), wide, "over the misloaded instrument", yaw=yaw0),
            W(0.8, (gx, gy, grip), wide, "around its handle", yaw=yaw0),
            W(0.8, (gx, gy, grip), c, "grip it", yaw=yaw0),
            W(0.8, (gx, gy, hover + 0.03), c, "lift it out", yaw=yaw0),
            W(1.5, (sx, cy + ty, hover + 0.03), c, "turn it along its slot", yaw=yaw1),
            W(0.8, (sx, cy + ty, set_z), c, "down onto the notch mouths", yaw=yaw1),
            W(0.5, (sx, cy + ty, set_z), wide, "let go: it rests on the rail lips", yaw=yaw1),
            W(0.5, (sx, cy + ty, rest_top + PAD_TIP_PAST_TCP + 0.010), wide, "up a little", yaw=yaw1),
            W(0.4, (sx, cy, rest_top + PAD_TIP_PAST_TCP + 0.010), c, "close the fingers", yaw=yaw1),
            W(0.5, (sx, cy, rest_top + PAD_TIP_PAST_TCP + 0.002), c, "tips on the handle", yaw=yaw1),
            W(1.2, (sx, cy, seat_top + PAD_TIP_PAST_TCP - 0.030), c, "press it home past the lips", yaw=yaw1),
            W(0.5, (sx, cy, seat_top + PAD_TIP_PAST_TCP - 0.030), c, "hold", yaw=yaw1),
            W(0.6, (sx, cy, hover), c, "lift", yaw=yaw1),
        ]

    def press_seated():
        """Left arm, fingers closed: press each pre-seated instrument (slots 5, 3, 1) home at its middle -- lifting the
        explorer off them or a knock can leave one riding on its rail jaws."""
        out = []
        for i in (4, 2, 0):
            sx = cx + SLOT_X[i]
            out += [
                W(0.8, (sx, cy, rest_top + PAD_TIP_PAST_TCP + 0.010), c, f"over slot {i + 1}", yaw=-math.pi / 2),
                W(0.8, (sx, cy, seat_top + PAD_TIP_PAST_TCP - 0.030), c, f"press slot {i + 1} home", yaw=-math.pi / 2),
                W(0.3, (sx, cy, seat_top + PAD_TIP_PAST_TCP - 0.030), c, "hold", yaw=-math.pi / 2),
                W(0.5, (sx, cy, rest_top + PAD_TIP_PAST_TCP + 0.010), c, "up", yaw=-math.pi / 2),
            ]
        return out

    l_fix = unload_misloaded()[:-1] + press_seated() + [W(0.6, (cx, cy, hover), c, "lift", yaw=-math.pi / 2)]
    # lid: hinge at the back edge; open 120 deg, its front edge stands ~66 mm above the hinge, 38 mm behind it
    hinge_x = cx + CAS_X / 2
    push_z = Z0 + BASE_H + 0.055 + PAD_TIP_PAST_TCP  # closed tips on the lid's outer face, 55 mm up the open lid
    side_y = cy + CAS_Y / 2 + 0.045  # beyond the cassette's +Y end
    close = [  # left arm: around the +Y end, low, then behind the lid, then push it over upright
        W(1.5, (hinge_x + 0.045, side_y, push_z + 0.02), c, "beside the cassette's far end"),
        W(1.0, (hinge_x + 0.045, cy + 0.060, push_z), c, "behind the open lid"),
        W(1.5, (hinge_x - 0.012, cy + 0.060, push_z), c, "push it past upright"),
        W(1.0, (hinge_x - 0.012, cy + 0.060, push_z + 0.03), c, "let it swing shut"),
        W(1.5, park_l, c, "park clear"),
    ]
    # clip: leaning back over the lid top at 125 deg; the closed finger tips push its upper part forward (-X) past
    # upright, then move back and up out of its way; it swings down over centre and its hook tab drops under the lip
    xp, zp = cx + CLIP_PIVOT[0], Z0 + CLIP_PIVOT[1]
    zc = zp + 0.015  # contact height on the clip
    flip = [  # right arm
        W(1.2, (xp + 0.030, cy, zc + PAD_TIP_PAST_TCP + 0.03), c, "over the lid, behind the clip"),
        W(0.6, (xp + 0.030, cy, zc + PAD_TIP_PAST_TCP), c, "down behind the clip"),
        W(1.0, (xp - 0.004, cy, zc + PAD_TIP_PAST_TCP), c, "push it forward past upright"),
        W(0.5, (xp + 0.030, cy, zc + PAD_TIP_PAST_TCP + 0.04), c, "back and up out of its way"),
        W(0.8, (xp + 0.030, cy, zc + PAD_TIP_PAST_TCP + 0.04), c, "let it swing down"),
        W(1.0, (xp - 0.035, cy, zp - 0.016 + PAD_TIP_PAST_TCP + 0.03), c, "in front of the clip"),
        W(0.6, (xp - 0.035, cy, zp - 0.016 + PAD_TIP_PAST_TCP), c, "down in front of it"),
        W(0.8, (xp - 0.013, cy, zp - 0.016 + PAD_TIP_PAST_TCP), c, "press it home over the lip"),
        W(0.4, (xp - 0.013, cy, zp - 0.016 + PAD_TIP_PAST_TCP), c, "hold"),
        W(0.5, (xp - 0.040, cy, zp - 0.016 + PAD_TIP_PAST_TCP), c, "off the clip"),
        W(0.6, (xp - 0.040, cy, zp + PAD_TIP_PAST_TCP + 0.03), c, "up"),
        W(1.5, park_r, c, "park clear"),
    ]
    left = [*l_fix, W(1.5, park_l, wide, "park clear")]
    right = [W(sum(w.duration for w in left), park_r, c, "wait"), *r_load, W(1.5, park_r, c, "park clear")]
    left.append(W(sum(w.duration for w in right) - sum(w.duration for w in left), park_l, wide, "wait"))
    left += close
    right.append(W(sum(w.duration for w in left) - sum(w.duration for w in right) + 3.0, park_r, c, "wait for the lid to settle"))
    right += flip
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    end = max(tl, tr) + 2.5
    left.append(W(end - tl, park_l, c, "hold clear"))
    right.append(W(end - tr, park_r, c, "hold clear"))
    return {"left": left, "right": right}
