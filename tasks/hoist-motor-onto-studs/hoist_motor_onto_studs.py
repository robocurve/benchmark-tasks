"""A 33 kg electric motor (3 hp TEFC, NEMA 182T frame) hangs from a 1/4-ton electric chain hoist over its motor base.

The base is a fixed grey steel plate 26 x 20 x 1.2 cm centred 30 cm in front of the arm bases, with four 3/8-16 steel
studs standing 3.8 cm proud of it in the NEMA 182T foot pattern (19.05 cm across the shaft, 11.43 cm along it), each
tip chamfered 3 mm x 45 deg. The motor hangs with its shaft along y (shaft end toward the right arm), its four cast
feet (10.3 mm holes) 6 cm above the stud tips. Its hook point is not over the studs: left alone, the motor hangs
1.0 cm toward the arms and 0.5 cm to the left of the stud pattern, turned 6 deg about vertical. A cast conduit box
(7 x 7 x 6 cm) sticks out of the motor's side toward the arms. The hoist chain runs from the hook straight up out of
view; the hoist moves the hook down (or up) at 2.0 cm/s only while the green DOWN (or black UP) button of its pendant
is held pressed. The pendant lies in a fixed cradle on the right of the table, buttons up.

Scored from the poses: the motor set down with all four studs through its feet, feet flat on the base, chain slack,
both grippers clear of the motor and off the pendant, and nothing moving. Whole-trial zero gates: any part of the robot
under the motor before all four feet are down on the plate, any part of the robot between a foot and the plate while
that foot is under 5 cm above it, or a foot touching a stud while moving faster than 5 cm/s.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory; this file
adds the chain (a slack-capable distance joint) and the hoist's hold-to-run logic.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/hoist-motor-onto-studs/hoist_motor_onto_studs.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (joints and per-episode hooks, installed in memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_DT = 1.0 / 20.0  # run with -E control_hz=20

# -- geometry (m). Frame: x forward from the arm bases, y left, z up. ---------------------------------------------
PLATE = (0.26, 0.20, 0.012)
BASE_XY = (0.300, 0.000)
PLATE_TOP = TOP + PLATE[2]
STUD_D = 0.0095  # 3/8 in stud (across flats of the octagonal model)
STUD_H = 0.038  # proud of the plate
CHAMFER = 0.003
STUD_TIP = PLATE_TOP + STUD_H
HOLE = 0.0103  # 13/32 in foot hole
E2, F2 = 0.1905, 0.1143  # NEMA 182T: 2E (across the shaft, x here), 2F (along the shaft, y)
FEET = [(sx * E2 / 2, sy * F2 / 2) for sx in (-1, 1) for sy in (-1, 1)]  # foot holes, motor frame
STUDS = [(BASE_XY[0] + fx, BASE_XY[1] + fy) for fx, fy in FEET]  # world (same pattern)
SHAFT_H = 0.1143  # NEMA 182T "D": foot underside to shaft axis
FOOT = (0.045, 0.045, 0.012)  # cast foot pad
FRAME_A = 0.22  # frame across flats (octagon)
FRAME_L = 0.26
MOTOR_KG = 33.0
STEEL, CAST, BLUE, DARK = (0.62, 0.63, 0.65), (0.38, 0.40, 0.43), (0.16, 0.30, 0.55), (0.12, 0.12, 0.13)

# Start: motor (frame origin on the shaft axis, mid-frame) displaced from its seated pose
SEAT = np.array([BASE_XY[0], BASE_XY[1], PLATE_TOP + SHAFT_H])
START_OFF = np.array([-0.010, 0.005, (STUD_TIP + 0.060) - PLATE_TOP])  # feet 6 cm above the stud tips
START_YAW = math.radians(6.0)
START = SEAT + START_OFF
CHAIN_L = 0.85  # hook to eyebolt eye (hook to centre of mass about 1.0 m)
HOOK_KG = 20.0  # hook block (moves only vertically, on the hoist)
HOIST_SPEED = 0.020  # m/s
BTN_ON = 0.0025  # button pressed this far runs the hoist
BTN_TRAVEL = 0.004


def _octagon(across, length, at, axis, color=None):
    """Four bands whose union is a regular octagonal prism, ``axis`` 'y' or 'z' along its length."""
    w = across * math.tan(math.pi / 8)
    out = []
    for k in range(4):
        a = k * math.pi / 4
        if axis == "z":
            out.append(mk.box((across, w, length), at=at, yaw=a, color=color))
        else:  # prism along y: rotate the band about y
            q = (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))
            out.append(mk.box((across, length, w), at=at, quat=q, color=color))
    return out


def _motor_boxes():
    z_foot = -SHAFT_H + FOOT[2] / 2
    out = _octagon(FRAME_A, FRAME_L, (0.0, 0.0, 0.0), "y", BLUE)
    out += [mk.box((0.17, 0.05, 0.17), at=(0.0, FRAME_L / 2 + 0.025, 0.0), color=BLUE)]  # fan cover
    out += [mk.box((0.0286, 0.07, 0.0286), at=(0.0, -FRAME_L / 2 - 0.035, 0.0), color=STEEL)]  # shaft 1-1/8 in
    s = (FOOT[0] - HOLE) / 2
    for fx, fy in FEET:  # foot pad with a square 10.3 mm hole (four bars round it)
        for d in (-1, 1):
            out.append(mk.box((s, FOOT[1], FOOT[2]), at=(fx + d * (HOLE + s) / 2, fy, z_foot), color=CAST))
            out.append(mk.box((HOLE, s, FOOT[2]), at=(fx, fy + d * (HOLE + s) / 2, z_foot), color=CAST))
    rail_z0, rail_z1 = -SHAFT_H + FOOT[2], -0.050
    for sx in (-1, 1):  # cast foot rails: frame to the inner half of each pad (the column above a hole is open)
        out.append(mk.box((0.030, 0.16, rail_z1 - rail_z0), at=(sx * 0.060, 0.0, (rail_z0 + rail_z1) / 2),
                          color=BLUE))
    for deg in (22.5, 67.5, 112.5, -22.5):  # cast cooling fins along the frame corners (TEFC), clear of the grip
        ang = math.radians(deg)
        q = (0.0, math.sin(-ang / 2), 0.0, math.cos(-ang / 2))
        out.append(mk.box((0.018, FRAME_L - 0.03, 0.006), at=(0.124 * math.cos(ang), 0.0, 0.124 * math.sin(ang)),
                          quat=q, color=BLUE))
    out.append(mk.box((0.006, 0.05, 0.005), at=(0.0, -FRAME_L / 2 - 0.040, 0.0158), color=STEEL))  # shaft key
    # conduit box on a mounting pad, toward the arms (-x)
    out.append(mk.box((0.020, 0.06, 0.06), at=(-0.115, 0.030, -0.064), color=BLUE))
    out.append(mk.box((0.070, 0.07, 0.06), at=(-0.160, 0.030, -0.064), color=BLUE))
    return out


CONDUIT = np.array([-0.160, 0.030, -0.064])  # conduit box centre, motor frame
CONDUIT_SIZE = (0.070, 0.07, 0.06)
EYE_BOXES = [  # forged eyebolt: shank and a square ring, eye centre 4 cm above the frame
    mk.box((0.012, 0.012, 0.025), at=(0.0, 0.0, FRAME_A / 2 + 0.010), color=STEEL),
    mk.box((0.008, 0.006, 0.030), at=(-0.013, 0.0, FRAME_A / 2 + 0.040), color=STEEL),
    mk.box((0.008, 0.006, 0.030), at=(0.013, 0.0, FRAME_A / 2 + 0.040), color=STEEL),
    mk.box((0.034, 0.006, 0.008), at=(0.0, 0.0, FRAME_A / 2 + 0.055), color=STEEL),
]


def _centroid(boxes):
    v = np.array([np.prod(b.size) for b in boxes])
    c = np.array([b.at for b in boxes])
    return (v[:, None] * c).sum(axis=0) / v.sum()


_BODY = _motor_boxes()
CG = _centroid(_BODY + EYE_BOXES)  # PhysX takes the centre of mass from the colliders at uniform density
EYE = np.array([CG[0], CG[1], FRAME_A / 2 + 0.040])  # the eyebolt is screwed in over the centre of mass
MOTOR_BOXES = _BODY + [mk.box(b.size, at=(b.at[0] + CG[0], b.at[1] + CG[1], b.at[2]), color=STEEL)
                       for b in EYE_BOXES]


def _stud_boxes(x, y):
    h = STUD_H - CHAMFER
    out = _octagon(STUD_D, h, (x, y, PLATE[2] / 2 + h / 2), "z", STEEL)
    c = CHAMFER * math.sqrt(2)
    t = 0.001
    for k in range(4):  # four plates at 45 deg: the chamfered tip
        a = k * math.pi / 2
        d = STUD_D / 2 - CHAMFER / 2 - t / 2 / math.sqrt(2)
        zc = PLATE[2] / 2 + h + CHAMFER / 2 - t / 2 / math.sqrt(2)
        r = mk.axis_angle((0, 0, 1), a) @ mk.axis_angle((0, 1, 0), math.pi / 4)
        out.append(mk.box((c, STUD_D * 0.75, t), at=(x + d * math.cos(a), y + d * math.sin(a), zc),
                          quat=mk.mat_to_quat(r), color=STEEL))
    return out


kit = mk.Kit()
_base = [mk.box(PLATE, at=(0, 0, 0), color=(0.50, 0.51, 0.53))]
for _x, _y in FEET:
    _base += _stud_boxes(_x, _y)
kit.part("motor_base", (BASE_XY[0], BASE_XY[1], TOP + PLATE[2] / 2), _base)
kit.part("motor", tuple(START), MOTOR_BOXES, mass=MOTOR_KG, color=BLUE, yaw=START_YAW)
_eye_start = START + mk.axis_angle((0, 0, 1), START_YAW) @ EYE
HOOK_PT0 = _eye_start + np.array([0.0, 0.0, CHAIN_L])  # chain attachment on the hook block at the start
HOOK_HALF = 0.04
CHAIN_VIS = CHAIN_L - 0.02  # the chain's hanging length drawn from the hook block down to just above the eye
kit.part("hook", tuple(HOOK_PT0 + np.array([0, 0, HOOK_HALF])),
         [mk.box((0.06, 0.04, 2 * HOOK_HALF)),
          mk.box((0.008, 0.008, CHAIN_VIS), at=(0.0, 0.0, -HOOK_HALF - CHAIN_VIS / 2), color=(0.25, 0.25, 0.27))],
         mass=HOOK_KG, color=(0.85, 0.65, 0.10))
# the hoist: the hook block on a vertical slide; its drive target is moved by the hold-to-run logic in on_step
kit.slider("hoist", None, "hook", axis=(0, 0, -1), lower=-0.05, upper=0.40, start=0.0, stiffness=4.0e5,
           rest=0.0, damping=2.0e4)

# pendant (lying in its cradle, buttons up) and its two hold-to-run buttons
PENDANT_XY = (0.200, -0.250)
CRADLE_H, PEND = 0.060, (0.200, 0.060, 0.040)
PEND_TOP = TOP + CRADLE_H + PEND[2]
BTN = {"down": (0.170, -0.250), "up": (0.230, -0.250)}
BTN_CAP = (0.022, 0.022, 0.010)
kit.part("pendant", (PENDANT_XY[0], PENDANT_XY[1], TOP), [
    mk.box((0.22, 0.08, CRADLE_H), at=(0, 0, CRADLE_H / 2), color=(0.30, 0.30, 0.32)),
    mk.box(PEND, at=(0, 0, CRADLE_H + PEND[2] / 2), color=DARK),
])
for _n, _c in (("down", (0.10, 0.62, 0.20)), ("up", DARK)):
    _bx, _by = BTN[_n]
    kit.part(f"btn_{_n}", (_bx, _by, PEND_TOP + BTN_CAP[2] / 2 + 0.004), [mk.box(BTN_CAP, color=_c),
             mk.box((0.012, 0.012, 0.012), at=(0, 0, -0.010), color=_c)], mass=0.03, color=_c)
    # spring 1.2 N/mm preloaded 1.2 N against the top stop: about 6 N at full travel (4 mm)
    kit.slider(f"{_n}_button", "pendant", f"btn_{_n}", axis=(0, 0, -1), lower=0.0, upper=BTN_TRAVEL, start=0.0,
               stiffness=1200.0, rest=-0.001, damping=2.0)

OBJECTS = [*kit.objects()]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- sim-only additions: the chain and steel-on-steel friction -------------------------------------------------------


def _author_extra(stage, env_path):
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade

    paths = mk._SIM["paths"]
    if "hook" not in paths or "motor" not in paths:
        return
    # a D6 joint with only a distance limit (GPU PhysX takes D6 joints, not distance joints): a chain that goes slack
    j = UsdPhysics.Joint.Define(stage, f"{env_path}/hoist_chain")
    j.CreateBody0Rel().SetTargets([Sdf.Path(paths["hook"])])
    j.CreateBody1Rel().SetTargets([Sdf.Path(paths["motor"])])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -HOOK_HALF))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in EYE]))
    lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), UsdPhysics.Tokens.distance)
    lim.CreateLowAttr().Set(-1.0)
    lim.CreateHighAttr().Set(float(CHAIN_L))
    j.CreateCollisionEnabledAttr().Set(False)
    j.CreateExcludeFromArticulationAttr().Set(True)
    # the drawn chain (a box on the hook block) never collides with the motor
    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(paths["hook"]))
    fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(paths["motor"]))
    # steel on steel (machined studs, cast feet): mu 0.25 static / 0.2 dynamic, "min" so it wins the pair
    mat = UsdShade.Material.Define(stage, f"{env_path}/steel_physics")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr().Set(0.25)
    api.CreateDynamicFrictionAttr().Set(0.20)
    api.CreateRestitutionAttr().Set(0.0)
    px = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
    px.CreateFrictionCombineModeAttr().Set("min")
    n = 0
    for part, skip in (("motor_base", ()), ("motor", tuple(range(len(_BODY) - 2, len(_BODY))))):
        for i in range(len(kit.parts[part].boxes)):
            if i in skip:  # the conduit box keeps its painted (grippy) surface
                continue
            for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{paths[part]}/box{i}")):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.strongerThanDescendants,
                                                                 "physics")
                    n += 1
    print(f"[hoist] chain {CHAIN_L} m hook->eye; steel material on {n} colliders", flush=True)


_author0 = mk._author_all


def _author_all(stage, env_path):
    _author0(stage, env_path)
    _author_extra(stage, env_path)


mk._author_all = _author_all


def _set_hook(d):
    path = mk._SIM["joint_paths"].get((kit.id, "hoist"))
    if not path or mk._SIM["stage"] is None:
        return
    from pxr import UsdPhysics

    UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "linear").GetTargetPositionAttr().Set(float(d))


def _robot_points():
    """World positions of both arms' link origins plus points down each gripper: finger middle, the TCP and the pad
    tips, centre and at each finger ([] offline)."""
    if _TEST_ROBOT is not None:
        return [np.asarray(p, float) for p in _TEST_ROBOT]
    env = mk._EP["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        raw = art.data.body_pos_w
        pos = (getattr(raw, "torch", raw)[0] - origin).detach().cpu().numpy().astype(float)
        out.extend(list(pos))
        names = list(art.body_names)
        g = pos[names.index("gripper")]
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]) / 2
        d = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        for k in (0.067, mk.TCP_PAST_GRIPPER, mk.TCP_PAST_GRIPPER + PAD_TIP_PAST_TCP):  # finger mid, TCP, pad tips
            out.append(g + d * k)
            if k > 0.1:  # each finger's tip, one either side
                for t in (pos[names.index("tip_left")], pos[names.index("tip_right")]):
                    side = t - mid - np.dot(t - mid, d) * d
                    out.append(g + d * k + side)
    return out


def _tcps():
    return [np.asarray(t, float) for t in (mk.tcps() if _TEST_TCPS is None else _TEST_TCPS)]


# -- readouts (all geometry from the recorded poses) ------------------------------------------------------------------

_TEST_TCPS = None
_TEST_ROBOT = None


def _T(objects, part):
    return kit._pose_T(objects, part)


def foot_holes(objects):
    """World positions of the four hole centres at the foot underside."""
    t = _T(objects, "motor")
    return [t[:3, :3] @ np.array([fx, fy, -SHAFT_H]) + t[:3, 3] for fx, fy in FEET]


def slack(objects) -> float:
    """How far (m) the chain is from taut: chain length minus the hook-to-eye distance."""
    hook = _T(objects, "hook")[:3, 3] + np.array([0, 0, -HOOK_HALF])
    eye = kit.point(objects, "motor", EYE)
    return float(CHAIN_L - np.linalg.norm(hook - eye))


def button_q(objects, name) -> float:
    return kit.q(objects, f"{name}_button")


def _through(objects):
    """Per stud: through its hole (hole centre within 3 mm of the stud seen from above, foot underside below the
    stud tip)."""
    return [bool(math.dist(h[:2], s) <= 0.003 and h[2] < STUD_TIP) for h, s in zip(foot_holes(objects), STUDS)]


def _flat(objects) -> bool:
    return all(h[2] - PLATE_TOP <= 0.001 for h in foot_holes(objects))


def _local(objects, p):
    t = _T(objects, "motor")
    return t[:3, :3].T @ (np.asarray(p, float) - t[:3, 3])


MOTOR_LO = kit.parts["motor"].lo.copy()
MOTOR_HI = kit.parts["motor"].hi.copy()


def _near_motor(p, margin) -> bool:
    return bool(np.all(p >= MOTOR_LO - margin) and np.all(p <= MOTOR_HI + margin))


SUSPENDED_SLACK = 0.005  # resting on the base: chain at least 5 mm from taut


def _track(objects) -> None:
    """Per-step latches (sim: every control step; self-test: called by hand)."""
    m = mk.MEMORY
    fired = m.setdefault("fired", {})
    # not yet set down: some foot more than 1 mm above the plate (hanging, on the stud tips or on the way down)
    suspended = not all(h[2] - PLATE_TOP <= 0.001 for h in foot_holes(objects))
    holes = foot_holes(objects)
    pts = _robot_points()
    low_z = float(objects["motor"].low[2])
    step = mk._EP.get("step", 0)

    def fire(name, why):
        if name not in fired:
            fired[name] = why
            print(f"[hoist] LATCH {name} at step {step}: {why}", file=sys.stderr, flush=True)

    # 1. under the load: a robot link below the motor's lowest point, inside its plan outline + 1 cm, until set down
    if suspended:
        for p in pts:
            lp = _local(objects, p)
            if (p[2] < low_z and MOTOR_LO[0] - 0.01 <= lp[0] <= MOTOR_HI[0] + 0.01
                    and MOTOR_LO[1] - 0.01 <= lp[1] <= MOTOR_HI[1] + 0.01):
                fire("under_load", f"robot point {np.round(p, 3).tolist()} under the hung motor (lowest z "
                                   f"{low_z:.3f})")
                break
    # 2. pinch: a robot link between a foot and the plate while that foot is under 5 cm above the plate
    for (fx, fy), h in zip(FEET, holes):
        if h[2] - PLATE_TOP >= 0.05:
            continue
        for p in pts:
            lp = _local(objects, p)
            if (abs(lp[0] - fx) <= FOOT[0] / 2 + 0.01 and abs(lp[1] - fy) <= FOOT[1] / 2 + 0.01
                    and PLATE_TOP - 0.005 < p[2] < h[2]):
                fire("pinch", f"robot point {np.round(p, 3).tolist()} under a foot {1000 * (h[2] - PLATE_TOP):.0f}"
                              f" mm above the plate")
                break
    # 3. stud strike: a foot touching a stud while the foot moves faster than 5 cm/s
    st = objects["motor"]
    for (fx, fy), h, s in zip(FEET, holes, STUDS):
        lp = _local(objects, (s[0], s[1], h[2]))
        dxy = np.array([lp[0] - fx, lp[1] - fy])
        z_overlap = h[2] < STUD_TIP + 0.001
        reach = FOOT[0] / 2 + STUD_D / 2 + 0.001
        touching = z_overlap and np.max(np.abs(dxy)) <= reach and np.max(np.abs(dxy)) >= (HOLE - STUD_D) / 2 - 1e-4
        if touching:
            r = h - np.asarray(st.pos, float)
            v = np.asarray(st.lin_vel, float) + np.cross(np.asarray(st.ang_vel, float), r)
            if float(np.linalg.norm(v)) > 0.05:
                fire("stud_strike", f"foot at {np.round(h, 3).tolist()} touching a stud at "
                                    f"{100 * np.linalg.norm(v):.1f} cm/s")
    # progress: all four holes within 3 mm (from above) of their studs with the feet under 1 cm above the tips
    if all(math.dist(h[:2], s) <= 0.003 and h[2] - STUD_TIP < 0.01 for h, s in zip(holes, STUDS)):
        m["aligned"] = True


def score(objects) -> float:
    """0 if any gate fired (under the load / pinch / stud strike). Else 1.0 all four studs through, feet flat
    (within 1 mm), chain slack 1 cm or more, both grippers 5 cm or more from the motor, both buttons up and the
    motor still; 0.75 through and flat but not all of that (or chain run out more than 15 cm past slack); 0.5 resting
    on the base (chain slack) with two or more studs through; 0.25 at some point all four holes within 3 mm of
    their studs with the feet under 1 cm above the tips; else 0."""
    if mk.MEMORY.get("fired"):
        return 0.0
    thr = _through(objects)
    sl = slack(objects)
    if all(thr) and _flat(objects):
        clear = all(not _near_motor(_local(objects, t), 0.05) for t in _tcps())
        buttons_up = all(button_q(objects, b) < 0.001 for b in ("down", "up"))
        if sl >= 0.01 and sl <= 0.16 and clear and buttons_up and objects["motor"].resting():
            return 1.0
        return 0.75
    if sl >= SUSPENDED_SLACK and sum(thr) >= 2:
        return 0.5
    if mk.MEMORY.get("aligned"):
        return 0.25
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Sum over the feet of each hole's distance to its seated position (m)."""
    return float(sum(math.dist(h, (s[0], s[1], PLATE_TOP)) for h, s in zip(foot_holes(objects), STUDS)))


# -- hoist logic (the hoist's contactor: runs only while a button is held) --------------------------------------------


def _hoist_step(objects) -> None:
    m = mk.MEMORY
    d = m.get("hook_d", 0.0)
    down, up = button_q(objects, "down") >= BTN_ON, button_q(objects, "up") >= BTN_ON
    if down and not up:
        d = min(d + HOIST_SPEED * CONTROL_DT, 0.40)
    elif up and not down:
        d = max(d - HOIST_SPEED * CONTROL_DT, -0.05)
    if d != m.get("hook_d", 0.0) or "hook_d" not in m:
        _set_hook(d)
    m["hook_d"] = d
    step = mk._EP.get("step", 0)
    if os.environ.get("HOIST_DEBUG") and (step % 10 == 0 or step < 40 and step % 2 == 0):
        h = foot_holes(objects)
        err = [1000 * math.dist(a[:2], s) for a, s in zip(h, STUDS)]
        cen = 1000 * (np.mean(h, axis=0)[:2] - np.asarray(BASE_XY))
        yaw = math.degrees(math.atan2(_T(objects, "motor")[1, 0], _T(objects, "motor")[0, 0]))
        print(f"[hoist] step {step} hook_d {d:.3f} slack {1000 * slack(objects):.1f}mm feet-z "
              f"{[round(1000 * (a[2] - PLATE_TOP), 1) for a in h]} xy-err-mm {[round(e, 1) for e in err]} "
              f"cen-err {np.round(cen, 1).tolist()} yaw {yaw:.2f} btn {1000 * button_q(objects, 'down'):.1f} score {score(objects)}",
              file=sys.stderr, flush=True)
        for (fx, fy), hh in zip(FEET, h):  # debug: robot points in the pinch window's footprint, local mm
            for i, p in enumerate(_robot_points()):
                lp = _local(objects, p)
                if abs(lp[0] - fx) <= FOOT[0] / 2 + 0.01 and abs(lp[1] - fy) <= FOOT[1] / 2 + 0.01:
                    print(f"[hoist]   foot ({fx:+.3f},{fy:+.3f}) at {1000 * (hh[2] - PLATE_TOP):.1f} mm: point {i} "
                          f"z {1000 * (p[2] - PLATE_TOP):.1f} mm", file=sys.stderr, flush=True)
        near = [i for i, p in enumerate(_robot_points()) if _near_motor(_local(objects, p), 0.02)]
        if near:
            print(f"[hoist]   robot points near the motor: {near}", file=sys.stderr, flush=True)


def _on_step(objects) -> None:
    if "hook_d" not in mk.MEMORY:
        _set_hook(0.0)  # a new episode: the hook back at its parked height
    _hoist_step(objects)
    _guide_step(objects)
    _track(objects)


kit.on_step(_on_step)

# -- the scripted demo ------------------------------------------------------------------------------------------------
# Left arm: grip the conduit box from above (fingers closing along the box's y faces), carry the motor over the studs
# and turn it the 6 deg square, then follow it down as the hoist lowers it, let go and back off. Right arm: hold DOWN
# on the pendant through the lowering until the chain is 3 cm slack, then lift off.

TRAVEL = TOP + 0.150
GRIP_DEPTH = 0.022  # TCP this far below the conduit box top
PRESS_Z = PEND_TOP + 0.004 + BTN_CAP[2] + PAD_TIP_PAST_TCP - 0.012  # 12 mm past the cap's top: bottoms the button
LOWER_TO_FLAT = (STUD_TIP + 0.060) - PLATE_TOP  # 9.8 cm
EXTRA_SLACK = 0.030
PRESS_S = (LOWER_TO_FLAT + EXTRA_SLACK) / HOIST_SPEED + 0.3
# The guiding hand closes the loop on the motor's pose (read from the sim each control step, which the demo may do):
# the arm lags its target by about 7 mm per newton against the hanging motor's 3.6 N pull toward the hook, so the
# guided waypoints get a PI correction that drives the four holes' centre onto the studs' centre and the yaw to 0.
# It freezes once a foot is within 2 mm of the stud tips: from there the 3 mm chamfers centre the feet.
KP, KI, I_MAX = 0.8, 1.5, 0.05
_GUIDE: dict = {}


def _grip_world(motor_pos, yaw):
    r = mk.axis_angle((0, 0, 1), yaw)
    top = CONDUIT + np.array([0, 0, CONDUIT_SIZE[2] / 2 - GRIP_DEPTH])
    return r @ top + motor_pos


def _guide_step(objects) -> None:
    g = _GUIDE
    if not g or g.get("frozen"):
        return
    t = mk._EP.get("step", 0) * CONTROL_DT
    if t < g["t_on"]:
        return
    holes = foot_holes(objects)
    if t >= g["t_off"] or min(h[2] for h in holes) < STUD_TIP + 0.002:
        g["frozen"] = True
        print(f"[hoist] guide frozen at t {t:.2f}: correction {np.round(1000 * g['i'], 1).tolist()} mm, "
              f"{math.degrees(g['iy']):.2f} deg", file=sys.stderr, flush=True)
        return
    tm = _T(objects, "motor")
    yaw = math.atan2(tm[1, 0], tm[0, 0])
    c = np.mean(holes, axis=0)
    e_xy, e_yaw = np.asarray(BASE_XY) - c[:2], -yaw
    ga = _grip_world(tm[:3, 3], yaw)
    gd = mk.axis_angle((0, 0, 1), e_yaw) @ (ga - c) + c
    d = gd[:2] + e_xy - ga[:2]
    g["i"] = np.clip(g["i"] + KI * d * CONTROL_DT, -I_MAX, I_MAX)
    g["iy"] = float(np.clip(g["iy"] + KI * e_yaw * CONTROL_DT, -0.25, 0.25))
    for w, (p0, y0) in zip(g["wps"], g["nominal"]):
        w.pos = (float(p0[0] + g["i"][0] + KP * d[0]), float(p0[1] + g["i"][1] + KP * d[1]), float(p0[2]))
        w.yaw = float(y0 + g["iy"] + KP * e_yaw)


def _left_guide(press_at, side=1):
    g0 = _grip_world(START, START_YAW)
    seat_hi = SEAT + np.array([0, 0, START_OFF[2]])
    g1 = _grip_world(seat_hi, 0.0)
    g2 = _grip_world(SEAT, 0.0)
    o, c = FINGER_OPEN, FINGER_CLOSED
    y0 = START_YAW
    plan = [
        Waypoint(1.2, (0.24, 0.31 * side, TOP + 0.20), c, "out wide of the motor", 0.0),
        Waypoint(1.3, (0.08, 0.21 * side, TOP + 0.18), c, "round the motor, not over it", 0.0),
        Waypoint(1.2, (0.07, g0[1], g0[2] + 0.04), c, "in front of the conduit box", y0),
        Waypoint(0.8, (g0[0], g0[1], g0[2] + 0.04), c, "above the conduit box", y0),
        Waypoint(0.6, (g0[0], g0[1], g0[2] + 0.04), o, "open", y0),
        Waypoint(1.0, tuple(g0), o, "fingers round the conduit box", y0),
        Waypoint(1.0, tuple(g0), c, "grip", y0),
    ]
    t_on = sum(w.duration for w in plan)
    plan.append(Waypoint(3.0, tuple(g1), c, "carry the motor over the studs and square it", 0.0))
    t = sum(w.duration for w in plan)
    plan.append(Waypoint(max(0.5, press_at - t), tuple(g1), c, "hold it there", 0.0))
    lower_s = LOWER_TO_FLAT / HOIST_SPEED + 0.5
    g_land = g2 + np.array([0, 0, 0.004])
    plan += [
        Waypoint(lower_s, tuple(g_land), c, "follow it down", 0.0),
        Waypoint(1.5, tuple(g_land), c, "seated", 0.0),
    ]
    t_off = sum(w.duration for w in plan)
    guided = plan[7:]
    plan += [
        Waypoint(1.0, tuple(g_land), o, "let go", 0.0),
        Waypoint(1.5, (g_land[0] - 0.05, g_land[1], TRAVEL), o, "up and back", 0.0),
        Waypoint(1.5, (0.08, 0.26, TRAVEL), o, "clear of the motor (over 5 cm)", 0.0),
        Waypoint(1.5, (0.08, 0.26, TRAVEL), o, "wait", 0.0),
    ]
    guided.append(plan[-4])
    _GUIDE.clear()
    _GUIDE.update(wps=guided, nominal=[(np.array(w.pos), w.yaw) for w in guided], t_on=t_on, t_off=t_off,
                  i=np.zeros(2), iy=0.0)
    return plan


def _right_press(press_at, hold_s, release=True):
    bx, by = BTN["down"]
    above = (bx, by, PEND_TOP + 0.06)
    plan = [
        Waypoint(1.5, (0.24, -0.30, TOP + 0.20), FINGER_CLOSED, "out wide of the shaft end"),
        Waypoint(1.5, above, FINGER_CLOSED, "over DOWN"),
        Waypoint(max(0.5, press_at - 4.0), above, FINGER_CLOSED, "wait for the motor to be lined up"),
        Waypoint(1.0, (bx, by, PRESS_Z), FINGER_CLOSED, "press DOWN"),
        Waypoint(hold_s, (bx, by, PRESS_Z), FINGER_CLOSED, "hold DOWN while it lowers"),
    ]
    if release:
        plan += [Waypoint(1.0, above, FINGER_CLOSED, "let go of DOWN"),
                 Waypoint(1.0, (bx + 0.03, by - 0.06, PEND_TOP + 0.06), FINGER_CLOSED, "clear of the motor (over 5 cm)")]
    return plan


PRESS_AT = 11.0  # s: DOWN goes in once the motor is lined up


def demo():
    """Left arm guides the motor over the studs and down; right arm runs the hoist. About 25 s."""
    plan = os.environ.get("HOIST_PLAN", "demo")
    if plan != "demo":
        return CONTROLS[plan]()
    return {"left": _left_guide(PRESS_AT), "right": _right_press(PRESS_AT, PRESS_S)}


# -- negative controls (HOIST_PLAN=<name>; not the shipped demo) ------------------------------------------------------


def _ctl_down_only():
    """Hold DOWN with no guiding: lands on the stud tips, out of line."""
    return {"right": _right_press(3.5, PRESS_S)}


def _ctl_keep_pressing():
    """The full demo but DOWN never let go."""
    return {"left": _left_guide(PRESS_AT), "right": _right_press(PRESS_AT, PRESS_S + 8.0, release=False)}


def _ctl_one_arm():
    """One arm (the right) only: line it up, let go, then go and press DOWN (the motor swings back)."""
    g = _left_guide(PRESS_AT, side=-1)[:9]
    last = g[-1]
    o = FINGER_OPEN
    plan = g + [Waypoint(0.8, last.pos, o, "let go", last.yaw),
                Waypoint(1.5, (last.pos[0] - 0.03, last.pos[1], TRAVEL), o, "up", 0.0)]
    t = sum(w.duration for w in plan)
    bx, by = BTN["down"]
    plan += [Waypoint(2.5, (bx, by, PEND_TOP + 0.06), FINGER_CLOSED, "over DOWN"),
             Waypoint(1.0, (bx, by, PRESS_Z), FINGER_CLOSED, "press DOWN"),
             Waypoint(PRESS_S, (bx, by, PRESS_Z), FINGER_CLOSED, "hold"),
             Waypoint(1.0, (bx, by, PEND_TOP + 0.06), FINGER_CLOSED, "release")]
    return {"right": plan}


def _ctl_under():
    """Steady the motor from below: fingers under the near-left foot while it is hung."""
    fx, fy = STUDS[1]
    z = PLATE_TOP + 0.030
    o, c = FINGER_OPEN * 0.5, FINGER_CLOSED
    left = [Waypoint(1.5, (0.09, 0.21, TOP + 0.18), c, "round the motor"),
            Waypoint(1.5, (0.09, 0.10, TOP + 0.06), c, "low, in front of it"),
            Waypoint(1.5, (fx - 0.03, fy + 0.02, z), o, "beside the foot"),
            Waypoint(1.5, (fx, fy + 0.01, z), o, "under the foot"),
            Waypoint(8.0, (fx, fy + 0.01, z), o, "hold under it")]
    return {"left": left, "right": _right_press(3.5, 4.0)}


def _ctl_pinch():
    """The demo until the studs are in the holes (feet about 3.4 cm up), then DOWN is let go, the left hand lets go
    and puts its fingers on the plate under the near-left foot, and DOWN is pressed again."""
    plan = _left_guide(PRESS_AT)
    follow = plan[9]
    k = next(i for i, w in enumerate(_GUIDE["wps"]) if w is follow)
    p0, y0 = _GUIDE["nominal"][k]
    p0 = p0.copy()
    p0[2] = _grip_world(START, 0.0)[2] - 0.064  # feet about 3.4 cm above the plate, the studs already in the holes
    _GUIDE["nominal"][k] = (p0, y0)
    follow.pos, follow.duration = tuple(p0), 3.4
    left = plan[:10]
    last = left[-1]
    fx, fy = STUDS[1]
    zl = PLATE_TOP + PAD_TIP_PAST_TCP + 0.004
    o, yx = FINGER_OPEN, -math.pi / 2  # fingers spread along x; one fingertip at the foot's outer edge
    left += [Waypoint(0.5, last.pos, o, "let go", 0.0),
             Waypoint(1.0, (last.pos[0] - 0.05, last.pos[1], last.pos[2] + 0.03), o, "up and back", 0.0),
             Waypoint(1.2, (fx - 0.055, fy + 0.028, zl + 0.04), o, "in front of the foot", yx),
             Waypoint(1.0, (fx - 0.055, fy + 0.028, zl), o, "a fingertip on the plate at the foot's edge", yx),
             Waypoint(10.0, (fx - 0.055, fy + 0.028, zl), o, "leave it there", yx)]
    bx, by = BTN["down"]
    above = (bx, by, PEND_TOP + 0.06)
    right = _right_press(PRESS_AT, 2.9, release=False) + [
        Waypoint(0.8, above, FINGER_CLOSED, "let go of DOWN"),
        Waypoint(3.0, above, FINGER_CLOSED, "wait for the hand to be at the foot"),
        Waypoint(1.0, (bx, by, PRESS_Z), FINGER_CLOSED, "press DOWN again"),
        Waypoint(6.0, (bx, by, PRESS_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.0, above, FINGER_CLOSED, "let go")]
    return {"left": left, "right": right}


def _ctl_pinch_park():
    """A fingertip parked on the plate at the near-right foot's near edge before DOWN is pressed: the motor, not
    guided, comes down beside the finger."""
    t = _T(kit.fake_states(), "motor")
    fx, fy = (t[:3, :3] @ np.array([*FEET[0], 0.0]) + t[:3, 3])[:2]  # near-right foot at the start, from above
    zl = PLATE_TOP + PAD_TIP_PAST_TCP + 0.004
    cx, cy = fx - 0.032, fy - 0.047  # the +y finger on the plate just off the pad's near edge (inside the 1 cm margin)
    o, c = FINGER_OPEN, FINGER_CLOSED
    left = [Waypoint(1.5, (0.09, 0.21, TOP + 0.18), c, "round the motor"),
            Waypoint(1.0, (0.04, 0.12, TOP + 0.10), c, "low, close to the robot, clear of the conduit box"),
            Waypoint(1.5, (0.04, -0.13, TOP + 0.10), c, "across, in front of the conduit box"),
            Waypoint(1.0, (0.04, -0.13, TOP + 0.10), o, "open"),
            Waypoint(1.5, (cx - 0.04, cy, zl + 0.01), o, "down in front of the foot"),
            Waypoint(1.5, (cx, cy, zl), o, "a fingertip on the plate under the foot"),
            Waypoint(12.0, (cx, cy, zl), o, "leave it there")]
    return {"left": left, "right": _right_press(8.0, PRESS_S)}


def _ctl_swing():
    """Shove the hung motor and let it swing while DOWN lowers it onto the studs."""
    g0 = _grip_world(START, START_YAW)
    o, c = FINGER_OPEN, FINGER_CLOSED
    y0 = START_YAW
    left = [Waypoint(1.2, (0.24, 0.31, TOP + 0.20), c, "out wide of the motor", 0.0),
            Waypoint(1.3, (0.08, 0.21, TOP + 0.18), c, "round the motor, not over it", 0.0),
            Waypoint(1.2, (0.07, g0[1], g0[2] + 0.04), c, "in front of the conduit box", y0),
            Waypoint(0.8, (g0[0], g0[1], g0[2] + 0.04), c, "above the conduit box", y0),
            Waypoint(0.6, (g0[0], g0[1], g0[2] + 0.04), o, "open", y0),
            Waypoint(1.0, tuple(g0), o, "fingers round it", y0),
            Waypoint(1.0, tuple(g0), c, "grip", y0),
            Waypoint(2.0, (g0[0] + 0.045, g0[1] - 0.02, g0[2]), c, "push it out", y0),
            Waypoint(0.25, (g0[0] + 0.045, g0[1] - 0.02, g0[2]), o, "let go: it swings", y0),
            Waypoint(1.0, (g0[0] - 0.08, g0[1], TRAVEL), o, "out of the way", 0.0),
            Waypoint(8.0, (g0[0] - 0.08, g0[1], TRAVEL), o, "wait", 0.0)]
    return {"left": left, "right": _right_press(8.5, PRESS_S)}


CONTROLS = {"idle": lambda: {}, "down_only": _ctl_down_only, "keep_pressing": _ctl_keep_pressing, "one_arm": _ctl_one_arm,
            "under": _ctl_under, "pinch": _ctl_pinch, "pinch_park": _ctl_pinch_park, "swing": _ctl_swing}

# -- offline self-test ------------------------------------------------------------------------------------------------


def _states(motor_pos, yaw=0.0, hook_d=0.0, down=0.0, up=0.0, motor_vel=(0, 0, 0)):
    import dataclasses

    hook = HOOK_PT0 + np.array([0, 0, HOOK_HALF - hook_d])
    st = kit.fake_states(q={"down_button": down, "up_button": up},
                         moved={"motor": (np.asarray(motor_pos, float), mk.yaw_quat(yaw)),
                                "hook": (hook, (0.0, 0.0, 0.0, 1.0))})
    st["motor"] = dataclasses.replace(st["motor"], lin_vel=np.asarray(motor_vel, float))
    return st


def _hook_for(motor_pos, yaw, slack_m):
    """Hook travel that leaves the chain ``slack_m`` slack with the motor at this pose (hook straight above)."""
    eye = np.asarray(motor_pos) + mk.axis_angle((0, 0, 1), yaw) @ EYE
    hook_xy = HOOK_PT0[:2]
    dz = math.sqrt(max((CHAIN_L - slack_m) ** 2 - float(np.sum((hook_xy - eye[:2]) ** 2)), 0.0))
    return float(HOOK_PT0[2] - (eye[2] + dz))


def _geometry_check():
    st = _states(START, START_YAW)
    assert abs(slack(st)) < 1e-6, slack(st)
    holes = foot_holes(st)
    assert all(abs(h[2] - STUD_TIP - 0.060) < 1e-6 for h in holes)
    seated = _states(SEAT, 0.0)
    assert all(math.dist(h[:2], s) < 1e-9 and abs(h[2] - PLATE_TOP) < 1e-9 for h, s in zip(foot_holes(seated), STUDS))
    # the stud passes the hole: 0.4 mm radial clearance; the open column above each hole clears the stud tip
    assert (HOLE - STUD_D) / 2 > 0.0003
    rail_inner = 0.060 + 0.015
    assert E2 / 2 - HOLE / 2 > rail_inner, "rail over the hole"
    # restoring pull at the start offset (pendulum hook-to-CG)
    lever = CHAIN_L + EYE[2] - CG[2]
    pull = MOTOR_KG * 9.81 * math.hypot(*START_OFF[:2]) / lever
    eye0 = START + mk.axis_angle((0, 0, 1), START_YAW) @ EYE
    print(f"motor CG {np.round(CG * 1000, 1)} mm (frame); hook-to-CG {lever:.2f} m; restoring pull at the seat "
          f"{pull:.1f} N; hook at z {HOOK_PT0[2] - TOP:.2f} m above the table; eye {eye0[2] - TOP:.3f}; "
          f"conduit top {(_grip_world(START, START_YAW)[2] + GRIP_DEPTH - TOP):.3f}", file=sys.stderr)


def self_test():
    global _TEST_TCPS, _TEST_ROBOT
    _geometry_check()
    far = [(0.05, 0.30, TOP + 0.30), (0.05, -0.30, TOP + 0.30)]
    ok_robot = [(0.05, 0.2, TOP + 0.3)]
    sl_ok = _hook_for(SEAT, 0.0, 0.03)
    taut_seat = _hook_for(SEAT, 0.0, 0.0)
    near = SEAT + np.array([0.0012, -0.001, 0.0])  # 1.6 mm off: inside the 0.4 mm clearance band? still "through"
    cases = {  # name: (sequence of states, tcps, robot points, want)
        "start: hung, nothing done": ([_states(START, START_YAW)], far, ok_robot, 0.0),
        "seated, flat, slack, released": ([_states(SEAT, 0.0, sl_ok)], far, ok_robot, 1.0),
        "seated 1.6 mm off-centre (within 3 mm), slack": ([_states(near, 0.0, sl_ok)], far, ok_robot, 1.0),
        "seated but chain still taut": ([_states(SEAT, 0.0, taut_seat)], far, ok_robot, 0.75),
        "seated, slack, DOWN still held": ([_states(SEAT, 0.0, sl_ok, down=0.003)], far, ok_robot, 0.75),
        "seated, slack, left gripper on the conduit box": (
            [_states(SEAT, 0.0, sl_ok)], [_grip_world(SEAT, 0.0), far[1]], ok_robot, 0.75),
        "seated, chain run out 20 cm": ([_states(SEAT, 0.0, _hook_for(SEAT, 0.0, 0.20))], far, ok_robot, 0.75),
        "seated, slack, still sliding": ([_states(SEAT, 0.0, sl_ok, motor_vel=(0.0, 0.0, -0.03))], far, ok_robot,
                                         0.75),
        "on the studs but held 4 mm off the plate (not flat), slack": (
            [_states(SEAT + np.array([0, 0, 0.004]), 0.0, sl_ok)], far, ok_robot, 0.5),
        "on the stud tips, not through, slack": (
            [_states(SEAT + np.array([0.012, 0.0, STUD_H]), 0.0, _hook_for(SEAT + np.array([0.012, 0, STUD_H]),
                                                                          0.0, 0.02))], far, ok_robot, 0.0),
        "aligned over the studs then landed on the tips": (
            [_states(SEAT + np.array([0.0, 0.0, STUD_H + 0.008]), 0.0, _hook_for(SEAT + np.array([0, 0, STUD_H + 0.008]),
                                                                                0.0, 0.0)),
             _states(SEAT + np.array([0.012, 0.0, STUD_H]), 0.0, _hook_for(SEAT + np.array([0.012, 0, STUD_H]),
                                                                          0.0, 0.02))], far, ok_robot, 0.25),
        "gate under the load: fingertip under a hung foot": (
            [_states(START, START_YAW), _states(SEAT, 0.0, sl_ok)], far,
            [ok_robot[0], (STUDS[1][0] - 0.01, STUDS[1][1], PLATE_TOP + 0.02)], 0.0),
        "gate pinch: fingertip under a foot 2 cm above the plate, then seated": (
            [_states(SEAT + np.array([0, 0, 0.02]), 0.0, sl_ok + 0.02), _states(SEAT, 0.0, sl_ok)], far,
            [ok_robot[0], (STUDS[1][0], STUDS[1][1] + 0.012, PLATE_TOP + 0.005)], 0.0),
        "gate under the load: hand under a motor resting on the stud tips (chain slack)": (
            [_states(SEAT + np.array([0.012, 0.0, STUD_H]), 0.0, _hook_for(SEAT + np.array([0.012, 0, STUD_H]), 0.0,
                                                                          0.02))], far,
            [ok_robot[0], (0.30, 0.0, PLATE_TOP + 0.02)], 0.0),
        "gate stud strike: foot hits a stud at 8 cm/s, then seated": (
            [_states(SEAT + np.array([0.004, 0, STUD_H - 0.002]), 0.0, 0.0, motor_vel=(0.08, 0, 0)),
             _states(SEAT, 0.0, sl_ok)], far, ok_robot, 0.0),
        "slow slide down the studs (2 cm/s) is not a strike": (
            [_states(SEAT + np.array([0.0006, 0, 0.02]), 0.0, _hook_for(SEAT + np.array([0, 0, 0.02]), 0.0, 0.0),
                     motor_vel=(0, 0, -0.02)), _states(SEAT, 0.0, sl_ok)], far, ok_robot, 1.0),
    }
    try:
        for name, (seq, tcps, robot, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS, _TEST_ROBOT = tcps, robot
            for st in seq:
                _track(st)
            got = score(seq[-1])
            assert abs(got - want) < 1e-9 and success(seq[-1]) == (want >= 1.0), (name, got, want, mk.MEMORY)
    finally:
        mk.new_episode()
        _TEST_TCPS = _TEST_ROBOT = None
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    names = ["demo", *CONTROLS]
    for name in names:
        os.environ["HOIST_PLAN"] = name
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert np.all(q[:, :6] >= ik.LOWER - 1e-9) and np.all(q[:, :6] <= ik.UPPER + 1e-9), (name, arm)
            if name == "demo":
                assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
            print(f"{name:14s} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
    os.environ.pop("HOIST_PLAN")
