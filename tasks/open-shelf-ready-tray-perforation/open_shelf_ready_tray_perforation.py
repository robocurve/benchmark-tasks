"""On the station table, at full size: a 12-can shelf-ready tray (B-flute corrugated board, 283 x 216 x 110 mm,
250 g) holding twelve 10.75 oz soup cans (68 x 102 mm, 0.36 kg each) three deep and four across, its front wall
toward the robot. The upper 70 mm of the front wall is the tear-away panel: it is joined to the rest of the tray only
by a perforation that runs down both front corners and along the 40 mm front lip. At the centre of its top edge the
panel stands over the gap between the two middle front cans, where a finger fits behind it (marked with a printed
thumb mark); the panel's top 15 mm is folded over outward (double board, a stiff pull edge). The job: open the tray so it can go straight on a shelf, i.e. take the panel
off along the perforation, leave the lip, walls and cans as they are, and let go.

Physics (mechkit.py, in this folder, adds the joints to the stock runtime in memory; the perforation ties are added
below in the same way):
- The tray (floor, back wall, side walls, front lip) is one rigid body; the cans are twelve free cylinders standing on
  its floor. Tray-on-table friction 0.30: the loaded tray (4.7 kg) starts to slide at about 13.7 N.
- The panel is four full-width rows of board (17.5 mm tall), each a rigid body, joined edge to edge by hinges with
  a light bending spring, so it can curl down as it tears.
- The perforation is eight ties: rows 1-3 are tied to the side walls at both corners, row 4 to the lip at two points
  on the bottom line. Each tie is a PhysX D6 joint whose three linear axes are springs (4000 N/m); its rotations are
  free. After every control step the force in each tie is read from the stretch of its spring (from the two bodies'
  poses) in the tray's frame and split into the part pulling the panel OUT of the wall (peel) and the part in the
  wall's plane (up, down, sideways):
    * peel force >= the tie's strength (side ties 9 N, lip ties 8 N): the tie tears cleanly along the perforation
      (its joint is switched off for the rest of the trial);
    * in-plane force >= twice that strength, first: the board beside the tie tears instead (the tear leaves the
      perforation): latched as wall damage, the tie lets go too. In-plane the tie itself is 3x stronger, so the
      board always goes first.
  The panel's rows share in-plane loads between all remaining ties, while a pull out of the wall concentrates on the
  top row's two ties: pulled straight up or sideways the panel does not open; pulled out it peels row by row.
  Pulled out at the top centre, the first row needs about 16.4 N, more than the 13.7 N that slides the tray, so pulling
  without holding the tray drags it instead of tearing.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/open-shelf-ready-tray-perforation/open_shelf_ready_tray_perforation.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (multi-box parts, hinges, per-episode memory, installed in memory)

from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = bool(os.environ.get("TRAY_DEBUG"))

KRAFT = (0.66, 0.50, 0.33)
KRAFT_IN = (0.58, 0.44, 0.29)
PANEL_C = (0.70, 0.54, 0.36)
PERF_C = (0.30, 0.22, 0.14)
CAN_C = (0.72, 0.30, 0.16)
CAN_TOP_C = (0.80, 0.81, 0.83)

# World: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
# -- the tray: front face (the tear panel) at x = XF, toward the robot -----------------------------------------------
BT = 0.003  # B-flute board
CAN_R, CAN_H, CAN_KG = 0.034, 0.102, 0.36  # 10.75 oz condensed soup can (211 x 400): 68 mm x 102 mm
PITCH = 2 * CAN_R + 0.001
IN_W = 4 * PITCH + 0.001  # inner width (y): 277 mm
IN_D = 3 * PITCH + 0.003  # inner depth (x): 210 mm
TW, TD, TH = IN_W + 2 * BT, IN_D + 2 * BT, 0.110  # outer 283 x 216 x 110 mm
LIP = 0.040  # the front lip left after opening
XF, YC = 0.165, 0.0
XC = XF + TD / 2
TRAY_KG = 0.25
N_ROW, ROW_H = 4, (TH - LIP) / 4  # 17.5 mm rows
ROW_W = IN_W - 0.002  # 1 mm clear of each side wall
FOLD_H = 0.015  # the top row's folded-over pull edge (double board)
ROW_KG = 0.025  # (real board ~3 g per row: weighted 8x for the solver; 0.25 N, against tearing forces of 10-20 N)
SIDE_Y = IN_W / 2 - 0.0005  # side ties: midway across the 1 mm gap between a row end and the side wall
LIP_Y = 0.070  # lip ties at +-70 mm on the bottom line
K_TIE, C_TIE = 4000.0, 2.0  # N/m, N*s/m per linear axis
S_SIDE, S_LIP = 9.0, 8.0  # N: peel force that tears a tie cleanly
W_FACTOR = 2.0  # in-plane force (x the tie's peel strength) at which the board beside it tears instead
K_ROW_HINGE = 0.03  # N*m/rad: board bending at the row lines
MU_TRAY = 0.30

kit = mk.Kit()
_tray = [
    mk.box((TD, TW, BT), at=(0, 0, BT / 2), color=KRAFT_IN),  # floor
    mk.box((BT, TW, TH), at=(TD / 2 - BT / 2, 0, TH / 2)),  # back wall
    mk.box((TD, BT, TH), at=(0, TW / 2 - BT / 2, TH / 2)),  # left side wall
    mk.box((TD, BT, TH), at=(0, -(TW / 2 - BT / 2), TH / 2)),  # right side wall
    mk.box((BT, TW, LIP), at=(-TD / 2 + BT / 2, 0, LIP / 2)),  # front lip
    # the perforation, printed: a dark line along the lip's top edge and down both front corners
    mk.box((0.0006, TW - 0.002, 0.0015), at=(-TD / 2 - 0.0003, 0, LIP - 0.00075), color=PERF_C),
    mk.box((0.0006, 0.0015, TH - LIP), at=(-TD / 2 - 0.0003, TW / 2 - 0.0015, (TH + LIP) / 2), color=PERF_C),
    mk.box((0.0006, 0.0015, TH - LIP), at=(-TD / 2 - 0.0003, -(TW / 2 - 0.0015), (TH + LIP) / 2), color=PERF_C),
]
kit.part("tray", (XC, YC, TOP), _tray, mass=TRAY_KG, color=KRAFT)

ROWS = tuple(f"row{i + 1}" for i in range(N_ROW))  # row1 = top
ROW_X = XF + BT / 2  # the panel's mid-plane


def _row_z0(i):  # bottom edge of row i (0 = top row), above the table
    return LIP + (N_ROW - 1 - i) * ROW_H


for i, name in enumerate(ROWS):
    boxes = [mk.box((BT, ROW_W, ROW_H), at=(0, 0, ROW_H / 2))]
    if i == 0:  # the folded-over pull edge (outer face, 15 mm) and the printed thumb mark on it (0.3 mm proud)
        boxes.append(mk.box((BT, ROW_W, FOLD_H), at=(-BT, 0, ROW_H - FOLD_H / 2)))
        boxes.append(mk.box((0.0003, 0.020, 0.008), at=(-1.5 * BT - 0.00015, 0, ROW_H - 0.006), color=PERF_C))
    kit.part(name, (ROW_X, YC, TOP + _row_z0(i)), boxes, mass=ROW_KG, color=PANEL_C)
# Row lines: row i hinged on the top edge of row i+1; + angle = row i's top swings OUT (toward -x).
for i in range(N_ROW - 1):
    z = TOP + _row_z0(i)
    kit.hinge(f"crease{i + 1}", ROWS[i + 1], ROWS[i], anchor=(ROW_X, YC, z), axis=(0, -1, 0),
              lower_deg=-10.0, upper_deg=175.0, start_deg=0.0, stiffness=K_ROW_HINGE, rest_deg=0.0)


def _can_xy(r, c):  # r: 0 front .. 2 back, c: 0..3 from -y
    return (XF + BT + 0.002 + CAN_R + r * PITCH, YC + (c - 1.5) * PITCH)


CANS = tuple(f"can{r}{c}" for r in range(3) for c in range(4))
_cans = [Cylinder(name=f"can{r}{c}", pos=(*_can_xy(r, c), TOP + BT + CAN_H / 2), radius=CAN_R, height=CAN_H,
                  mass=CAN_KG, color=CAN_C) for r in range(3) for c in range(4)]
OBJECTS = [*kit.objects(), *_cans]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the perforation ties -----------------------------------------------------------------------------------------
# (name, row, anchor in world at spawn, peel strength N)
TIES = []
for i in range(N_ROW - 1):
    z = TOP + _row_z0(i) + ROW_H / 2
    for s, tag in ((1, "L"), (-1, "R")):
        TIES.append((f"{ROWS[i]}{tag}", ROWS[i], np.array([ROW_X, YC + s * SIDE_Y, z]), S_SIDE))
for s, tag in ((1, "L"), (-1, "R")):
    TIES.append((f"lip{tag}", ROWS[-1], np.array([ROW_X, YC + s * LIP_Y, TOP + LIP]), S_LIP))
TIE_NAMES = tuple(t[0] for t in TIES)
OUT = np.array([-1.0, 0.0, 0.0])  # tray frame: out of the front wall
M = "tray"
_TEST_TCPS = None


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"torn": {}, "damage": None, "tipped": False, "lifted": False, "can_out": None,
                            "drag": 0.0, "frame": None, "n": 0, "pose": None, "prev_pose": None,
                            "peak": {n: (0.0, 0.0) for n in TIE_NAMES}}
    return m


def _tie_path(name):
    return f"{mk._SIM['paths']['tray'].rsplit('/', 1)[0]}/tray_ties/{name}"


def set_tray_material(stage, tray_path):
    """Bind the board-on-table material (static = dynamic = MU_TRAY, friction combine "min", so the table's 0.5 does
    not average it up) to EVERY collider anywhere under the tray prim (spawned boxes nest their collision mesh two
    levels down: box*/geometry/mesh). Returns (number bound, report lines: each collider and the physics material it
    now resolves to, with that material's friction). Pure pxr: also runs on a stand-in stage offline."""
    from pxr import Sdf, Usd, UsdPhysics, UsdShade

    mat_path = f"{tray_path}/board_material"
    mat = UsdShade.Material.Define(stage, mat_path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr().Set(MU_TRAY)
    api.CreateDynamicFrictionAttr().Set(MU_TRAY)
    api.CreateRestitutionAttr().Set(0.0)
    prim = mat.GetPrim()
    try:
        from pxr import PhysxSchema

        px = PhysxSchema.PhysxMaterialAPI.Apply(prim)
        px.CreateFrictionCombineModeAttr().Set("min")
    except ImportError:  # offline stand-in: same attribute, by name
        prim.AddAppliedSchema("PhysxMaterialAPI")
        prim.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("min")
    n, report = 0, []
    root = stage.GetPrimAtPath(tray_path)
    colliders = [g for g in Usd.PrimRange(root) if g.HasAPI(UsdPhysics.CollisionAPI)]
    targets = {}
    for g in colliders:  # the collider and every prim above it up to the tray (an ancestor's binding can win)
        p = g
        while p and p.GetPath().HasPrefix(root.GetPath()):
            targets[str(p.GetPath())] = p
            p = p.GetParent()
    for p in targets.values():
        UsdShade.MaterialBindingAPI.Apply(p).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics")
    for g in colliders:
        n += 1
        bound = UsdShade.MaterialBindingAPI(g).ComputeBoundMaterial("physics")[0]
        bp = bound.GetPrim()
        sf = bp.GetAttribute("physics:staticFriction").Get() if bp else None
        df = bp.GetAttribute("physics:dynamicFriction").Get() if bp else None
        cm = bp.GetAttribute("physxMaterial:frictionCombineMode").Get() if bp else None
        report.append(f"[tray] collider {g.GetPath()} -> physics material {bp.GetPath() if bp else None} "
                      f"(static {sf}, dynamic {df}, combine {cm})")
    return n, report


def _author_ties(stage) -> None:
    """Eight D6 spring joints, row -> tray (the kit's joint tree cannot close these loops); the rows do not collide
    with the tray (they are held only by the ties, 1 mm clear of the walls); the tray gets its board-on-table friction.
    Runs once the scene is built, before the simulation starts."""
    from pxr import Gf, Sdf, UsdPhysics

    paths = mk._SIM["paths"]
    if "tray" not in paths:
        return
    poses = kit._spawn_poses()
    tray_T = poses["tray"]
    for name, row, anchor, _ in TIES:
        row_T = poses[row]
        j = UsdPhysics.Joint.Define(stage, _tie_path(name))
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[row])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(paths["tray"])])
        p0 = (anchor - row_T[:3, 3]) @ row_T[:3, :3]
        p1 = (anchor - tray_T[:3, 3]) @ tray_T[:3, :3]
        q0 = mk.mat_to_quat(row_T[:3, :3].T)  # joint frame = the tray's axes at spawn
        q1 = mk.mat_to_quat(tray_T[:3, :3].T)
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in p0]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(float(q0[3]), Gf.Vec3f(*[float(v) for v in q0[:3]])))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in p1]))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(float(q1[3]), Gf.Vec3f(*[float(v) for v in q1[:3]])))
        j.CreateCollisionEnabledAttr().Set(False)
        j.CreateExcludeFromArticulationAttr().Set(True)
        j.CreateJointEnabledAttr().Set(True)
        prim = j.GetPrim()
        for ax in ("transX", "transY", "transZ"):
            d = UsdPhysics.DriveAPI.Apply(prim, ax)
            d.CreateTypeAttr().Set("force")
            d.CreateStiffnessAttr().Set(K_TIE)
            d.CreateDampingAttr().Set(C_TIE)
            d.CreateTargetPositionAttr().Set(0.0)
            d.CreateTargetVelocityAttr().Set(0.0)
    for row in ROWS:
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(paths[row]))
        fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(paths["tray"]))
    n_bound, report = set_tray_material(stage, paths["tray"])
    for line in report:
        print(line, flush=True)
    if n_bound == 0:
        raise RuntimeError("[tray] no collider found under the tray: board material not applied")
    print(f"[tray] ties {len(TIES)} (k {K_TIE:.0f} N/m), rows filtered from the tray, tray friction {MU_TRAY}",
          flush=True)


def _patch_scene(mod) -> None:
    from isaaclab.scene import interactive_scene as isc

    cls = isc.InteractiveScene
    if getattr(cls, "_tray_ties", False):
        return
    init0 = cls.__init__

    def __init__(self, *a, **k):
        init0(self, *a, **k)
        _author_ties(self.stage)

    cls.__init__ = __init__
    cls._tray_ties = True


def _install_scene_patch() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
        return
    for f in sys.meta_path:  # run after mechkit's own patch of yam_arena.scene (it records the prim paths we need)
        if isinstance(f, mk._AfterImport) and f.name == "yam_arena.scene":
            p0 = f.patch

            def both(mod, p0=p0):
                p0(mod)
                _patch_scene(mod)

            f.patch = both
            return


_install_scene_patch()


def _set_tie(name, on: bool) -> None:
    stage = mk._SIM.get("stage")
    if stage is None:
        return
    from pxr import UsdPhysics

    prim = stage.GetPrimAtPath(_tie_path(name))
    if prim.IsValid():
        UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(bool(on))


_friction_reset0 = mk._friction_reset


def _friction_reset():  # every episode reset, before the scene is put back: the perforation whole again
    _friction_reset0()
    for n in TIE_NAMES:
        _set_tie(n, True)


mk._friction_reset = _friction_reset


# -- readouts ------------------------------------------------------------------------------------------------------
def _T(objects, part):
    pos, quat = kit.pose(objects, part)
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = mk.quat_to_mat(quat), pos
    return t


_REF = kit._spawn_poses()


def tie_force(objects, name) -> tuple:
    """(peel N, in-plane N) carried by tie ``name``: its spring stretch in the tray's frame times K_TIE."""
    _, row, anchor, _ = next(t for t in TIES if t[0] == name)
    tr, rw = _T(objects, "tray"), _T(objects, row)
    a_tray = tr @ np.linalg.inv(_REF["tray"]) @ np.r_[anchor, 1.0]
    a_row = rw @ np.linalg.inv(_REF[row]) @ np.r_[anchor, 1.0]
    d = tr[:3, :3].T @ (a_row[:3] - a_tray[:3])  # tray frame
    f = K_TIE * d
    return float(f @ OUT), float(np.hypot(f[1], f[2]))


def tray_tilt_deg(objects) -> float:
    z = _T(objects, "tray")[:3, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


_CORNERS = np.array([[sx * TD / 2, sy * TW / 2, 0.0, 1.0] for sx in (-1, 1) for sy in (-1, 1)])


def tray_drag(objects) -> float:
    """Largest horizontal shift (m) of any floor corner from where the tray started."""
    now = (_T(objects, "tray") @ np.linalg.inv(_REF["tray"]) @ (_REF["tray"] @ _CORNERS.T)).T
    ref = (_REF["tray"] @ _CORNERS.T).T
    return float(np.max(np.linalg.norm((now - ref)[:, :2], axis=1)))


def tray_lift(objects) -> float:
    now = (_T(objects, "tray") @ _CORNERS.T).T
    return float(np.max(now[:, 2]) - TOP)


def _in_tray(objects, p, margin=0.0) -> bool:
    """Point p (world) inside the tray's walls (tray frame), at or above its floor."""
    tr = _T(objects, "tray")
    q = tr[:3, :3].T @ (np.asarray(p, float) - tr[:3, 3])
    return bool(abs(q[0]) <= TD / 2 - margin and abs(q[1]) <= TW / 2 - margin and -0.01 <= q[2] <= TH + 0.05)


def cans_in(objects) -> bool:
    return all(_in_tray(objects, objects[c].pos, margin=BT) for c in CANS)


def cans_upright(objects) -> bool:
    for c in CANS:
        z = mk.quat_to_mat(objects[c].quat_xyzw)[:, 2]
        if z[2] < math.cos(math.radians(20.0)):
            return False
    return True


def panel_in_tray(objects) -> bool:
    """A removed row lying in or on the tray (debris on the product): its centre inside the walls' footprint, more
    than 1 cm behind the front wall's plane."""
    tr = _T(objects, "tray")
    for r in ROWS:
        q = tr[:3, :3].T @ (np.asarray(objects[r].pos, float) - tr[:3, 3])
        if -TD / 2 + 0.01 < q[0] <= TD / 2 and abs(q[1]) <= TW / 2 and q[2] > -0.01:
            return True
    return False


def released(objects) -> bool:
    tcps = _TEST_TCPS if _TEST_TCPS is not None else mk.tcps()
    for part in ("tray", *ROWS):
        st = objects[part]
        lo, hi = np.asarray(st.low) - 0.025, np.asarray(st.high) + 0.025
        if any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps):
            return False
    return True


def _say(msg) -> None:
    if mk._EP.get("env") is not None or DEBUG:
        print(msg, flush=True)


def _tie_rule(objects) -> None:
    """The perforation, every control step (kit.on_step in sim; called by score() offline too)."""
    m = _mem()
    for name, row, anchor, s in TIES:
        if name in m["torn"]:
            continue
        peel, inplane = tie_force(objects, name)
        pk = m["peak"][name]
        m["peak"][name] = (max(pk[0], peel), max(pk[1], inplane))
        if inplane >= W_FACTOR * s:
            m["torn"][name] = ("wall", m["n"], peel, inplane)
            if m["damage"] is None:
                m["damage"] = (name, m["n"], inplane)
            _set_tie(name, False)
            _say(f"[tray] WALL TORN beside tie {name} at step {m['n']}: in-plane {inplane:.1f} N, peel {peel:.1f} N")
        elif peel >= s:
            m["torn"][name] = ("clean", m["n"], peel, inplane)
            _set_tie(name, False)
            _say(f"[tray] tie {name} torn cleanly at step {m['n']}: peel {peel:.1f} N, in-plane {inplane:.1f} N")


def _update(objects) -> dict:
    m = _mem()
    key = mk._EP.get("step") if mk._EP.get("env") is not None else id(objects)
    if m["frame"] == key:
        return m
    m["frame"] = key
    m["n"] += 1
    _tie_rule(objects)
    if tray_tilt_deg(objects) > 10.0:
        m["tipped"] = True
    if tray_lift(objects) > 0.015:
        m["lifted"] = True
    m["drag"] = max(m["drag"], tray_drag(objects))
    if m["can_out"] is None:
        out = [c for c in CANS if not _in_tray(objects, objects[c].pos, margin=BT)]
        if out:
            m["can_out"] = (out[0], m["n"])
    pose = {n: _T(objects, n) for n in ("tray", *ROWS)}
    pose.update({c: np.asarray(objects[c].pos, float) for c in CANS})
    m["prev_pose"], m["pose"] = m["pose"], pose
    if DEBUG and m["n"] % 10 == 0:
        f = {n: tie_force(objects, n) for n in TIE_NAMES if n not in m["torn"]}
        print(f"[tray] k={m['n']} drag {m['drag'] * 1000:.1f} mm tilt {tray_tilt_deg(objects):.1f} torn "
              f"{sorted(m['torn'])} forces " + " ".join(f"{n}:{p:.1f}/{q:.1f}" for n, (p, q) in f.items()) +
              f" | row1 {np.round(objects['row1'].pos, 3)} row4 {np.round(objects['row4'].pos, 3)} tcps "
              f"{[np.round(t, 3).tolist() for t in mk.tcps()]}", flush=True)
    return m


def _on_step(objects) -> None:
    _update(objects)


kit.on_step(_on_step)


def _still(objects) -> bool:
    m = _update(objects)
    a, b = m["prev_pose"], m["pose"]
    if a is None or b is None:
        return False
    for n in ("tray", *ROWS):
        d = np.linalg.inv(a[n]) @ b[n]
        turn = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(d[:3, :3]) - 1) / 2))))
        if np.linalg.norm(b[n][:3, 3] - a[n][:3, 3]) > 0.001 or turn > 1.0:
            return False
    for c in CANS:
        if np.linalg.norm(b[c] - a[c]) > 0.001:
            return False
    return True


def score(objects) -> float:
    """0 for the trial if the board ever tore off the perforation (a tie's in-plane load reached twice its peel
    strength), the tray ever tipped more than 10 deg or lifted 15 mm off the table, or a can ever left the tray.
    Otherwise: 0 nothing torn; 0.25 the tear started (at least one tie torn) but the panel is still on; 0.5 the panel
    fully off (all eight ties torn); 1.0 the panel off, the tray never shifted more than 30 mm, all cans upright in
    it, no part of the panel left in or on the tray, both grippers clear (2.5 cm) of tray and panel, everything still
    (success when held 1 s)."""
    m = _update(objects)
    if m["damage"] is not None or m["tipped"] or m["lifted"] or m["can_out"] is not None:
        return 0.0
    n_torn = len(m["torn"])
    if n_torn == 0:
        return 0.0
    if n_torn < len(TIES):
        return 0.25
    if (m["drag"] <= 0.030 and cans_in(objects) and cans_upright(objects) and not panel_in_tray(objects)
            and released(objects) and _still(objects)):
        return 1.0
    return 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Ties still holding (one each) plus metres of extra tray shift."""
    m = _update(objects)
    return float(len(TIES) - len(m["torn"]) + max(0.0, m["drag"] - 0.030))


# -- scripted demo ------------------------------------------------------------------------------------------------
YAW_R, YAW_L = math.pi / 2, -math.pi / 2  # both grips close along world x (across the board)
PAD_TIP = 0.013  # pad tips below the TCP
F_OPEN, F_NEAR, F_SHUT = -0.035, -0.008, 0.0  # pads 70 / 16 / 0 mm apart (0: 15 N each on the 6 mm edge)
# right arm: the panel's top centre, over the gap between the middle cans (pads 15 mm into the 17.5 mm top row)
GRIP_Z = TOP + TH - 0.015 + PAD_TIP
GRIP_X = XF  # the middle of the 6 mm double board at the pull edge
GRIP = np.array([GRIP_X, YC, GRIP_Z])
# left arm: the back wall, inner pad in the gap between the two back-row cans at y +0.069
PIN = np.array([XF + TD - BT / 2, YC + PITCH, TOP + TH - 0.016 + PAD_TIP])
PIN_HOVER = 0.03  # the pin hand is near its reach limit: approach from 3 cm above
LAG = 0.08  # m the soft arm trails its target at ~18 N (wrist 10 N*m/rad); the targets lead the hand by this much


def _go(W, dur, p, f, label, yaw):
    W.append(Waypoint(float(dur), tuple(float(v) for v in p), float(f), label, float(yaw)))


def _pin_plan(L):
    _go(L, 2.5, PIN + [0, 0, PIN_HOVER], F_NEAR, "left: over the back wall", YAW_L)
    _go(L, 1.5, PIN, F_NEAR, "left: inner finger down between two back cans", YAW_L)
    _go(L, 0.8, PIN, F_SHUT, "left: hold the back wall", YAW_L)


def _grip_plan(R):
    _go(R, 2.5, GRIP + [0, 0, 0.06], F_NEAR, "right: over the top centre of the panel", YAW_R)
    _go(R, 1.5, GRIP, F_NEAR, "right: inner finger down behind the panel, between the middle cans", YAW_R)
    _go(R, 0.8, GRIP, F_SHUT, "right: pinch the panel's top edge", YAW_R)


def _peel_plan(R):
    """Out of the wall first, then out and down after the tear, row by row, then the lip line from level."""
    legs = [
        (3.0, (XF - LAG, GRIP_Z), "right: pull the top row straight out (corner ties tear)"),
        (2.0, (XF - LAG - 0.02, TOP + 0.090), "right: out and down: row 2 peels"),
        (2.0, (XF - LAG - 0.03, TOP + 0.070), "right: out and down: row 3 peels"),
        (2.0, (XF - LAG - 0.045, TOP + 0.050), "right: panel hanging from the lip line"),
        (2.5, (XF - LAG - 0.085, TOP + LIP + 0.005), "right: pull it out level with the lip line (lip ties tear)"),
        (1.0, (XF - LAG - 0.085, TOP + LIP + 0.005), "settle"),
    ]
    for dur, (x, z), lab in legs:
        _go(R, dur, (x, YC, z), F_SHUT, lab, YAW_R)


def _dispose_plan(R):
    _go(R, 2.0, (0.12, -0.20, TOP + 0.10), F_SHUT, "right: carry the panel clear of the tray", YAW_R)
    _go(R, 1.5, (0.12, -0.20, TOP + 0.04), F_SHUT, "right: lower it to the table", YAW_R)
    _go(R, 0.8, (0.12, -0.20, TOP + 0.04), F_OPEN, "right: let go", YAW_R)
    _go(R, 1.5, (0.12, -0.25, TOP + 0.16), F_OPEN, "right: up and away", YAW_R)


def _release_pin(L):
    _go(L, 0.8, PIN, F_NEAR, "left: open", YAW_L)
    _go(L, 1.2, PIN + [0, 0, PIN_HOVER], F_NEAR, "left: up out of the tray", YAW_L)
    _go(L, 1.5, (0.30, 0.30, TOP + 0.20), F_NEAR, "left: clear", YAW_L)


def _dur(W):
    return sum(w.duration for w in W)


def _wait(W, until, label, yaw):
    if until > _dur(W) + 1e-9:
        _go(W, until - _dur(W), W[-1].pos, W[-1].finger, label, yaw)


def demo():
    """Left: down onto the back wall, inner finger in the gap between two back cans, and hold it (the tray cannot
    slide toward the robot). Right: pinch the panel's top edge at the centre, pull it straight out of the wall until
    the top row's corner ties tear, then keep pulling out and down so the rows peel one by one along the corner
    perforations; with the panel hanging from the lip line, pull it out level with that line until the lip ties tear.
    Carry the panel clear, lay it on the table, let go; the left lets go of the tray. About 30 s."""
    R, L = [], []
    _pin_plan(L)
    _grip_plan(R)
    _wait(R, _dur(L), "right: wait for the hold", YAW_R)
    _wait(L, _dur(R), "left: hold", YAW_L)
    _peel_plan(R)
    _wait(L, _dur(R), "left: hold the tray while the panel tears", YAW_L)
    _dispose_plan(R)
    _release_pin(L)
    end = max(_dur(R), _dur(L)) + 2.0
    _wait(R, end, "right: still", YAW_R)
    _wait(L, end, "left: still", YAW_L)
    return {"left": L, "right": R}


def naive_demo():
    """The run-1 controls in one trial. (B) tray held: pinch the panel's top centre and pull it straight UP, then
    SIDEWAYS (in the wall's plane); (A) the left lets go of the tray, the right pinches again and pulls the panel
    straight out with nothing holding the tray."""
    R, L = [], []
    _pin_plan(L)
    _grip_plan(R)
    _wait(R, _dur(L), "right: wait for the hold", YAW_R)
    _wait(L, _dur(R), "left: hold", YAW_L)
    _go(R, 3.0, GRIP + [0, 0, 0.10], F_SHUT, "B: rip the panel straight up", YAW_R)
    _go(R, 1.5, GRIP + [0, 0, 0.0], F_NEAR, "B: back, open", YAW_R)
    _go(R, 0.8, GRIP, F_SHUT, "B: pinch again", YAW_R)
    _go(R, 3.0, GRIP + [0, 0.10, 0.0], F_SHUT, "B: rip it sideways along the wall", YAW_R)
    _go(R, 1.5, GRIP, F_NEAR, "B: back, open", YAW_R)
    _wait(L, _dur(R), "left: hold", YAW_L)
    _release_pin(L)
    _wait(R, _dur(L), "right: wait (nothing holds the tray now)", YAW_R)
    _go(R, 0.8, GRIP, F_SHUT, "A: pinch the top centre", YAW_R)
    _go(R, 4.0, (XF - LAG - 0.06, YC, GRIP_Z), F_SHUT, "A: pull the panel straight out, tray not held", YAW_R)
    _go(R, 1.0, R[-1].pos, F_SHUT, "A: hold", YAW_R)
    _go(R, 0.8, R[-1].pos, F_OPEN, "A: let go", YAW_R)
    _go(R, 1.5, (0.12, -0.25, TOP + 0.16), F_OPEN, "A: away", YAW_R)
    end = max(_dur(R), _dur(L)) + 1.0
    _wait(R, end, "right: still", YAW_R)
    _wait(L, end, "left: still", YAW_L)
    return {"left": L, "right": R}


# -- offline self-test ---------------------------------------------------------------------------------------------
def _states(q=None, moved=None, pull=None, tray=None, cans=None):
    """Kit states with crease angles q (deg) and whole parts moved; ``pull`` (m, world) shifts the rows not in
    ``moved`` rigidly (stretching their ties); ``tray`` (4x4 world transform) moves the tray with the cans and the
    unmoved rows; ``cans`` {name: (pos, quat)} overrides single cans."""
    moved = dict(moved or {})
    user_moved = set(moved)
    tr = np.eye(4) if tray is None else np.asarray(tray, float)
    if tray is not None:
        t1 = tr @ _REF["tray"]
        moved["tray"] = (t1[:3, 3], mk.mat_to_quat(t1[:3, :3]))
        for r in ROWS:
            if r not in moved:
                t1 = tr @ _REF[r]
                moved[r] = (t1[:3, 3], mk.mat_to_quat(t1[:3, :3]))
    st = kit.fake_states(q=q or {}, moved=moved, deg=True)
    if pull is not None:
        for r in ROWS:
            if r not in user_moved:
                _shift(st, r, np.asarray(pull, float))
    for o in _cans:
        p, quat = (tr @ np.r_[o.pos, 1.0])[:3], mk.mat_to_quat(tr[:3, :3])
        if cans and o.name in cans:
            p, quat = np.asarray(cans[o.name][0], float), cans[o.name][1]
        h = np.array([o.radius, o.radius, o.height / 2])
        st[o.name] = _obj(p, quat, p - h, p + h)
    return st


def _obj(pos, quat, lo, hi):
    from yam_arena.task_file import ObjectState

    z = np.zeros(3)
    return ObjectState(pos=np.asarray(pos, float), quat_xyzw=np.asarray(quat, float), lin_vel=z, ang_vel=z.copy(),
                       low=np.asarray(lo, float), high=np.asarray(hi, float))


def _shift(st, part, d):
    s = st[part]
    st[part] = _obj(np.asarray(s.pos) + d, s.quat_xyzw, np.asarray(s.low) + d, np.asarray(s.high) + d)
    return st


def _row_moved(i, deg, dx=0.0, dz=0.0):
    """Row i turned ``deg`` out about its own bottom edge and shifted (dx, dz): (origin pos, quat)."""
    r = mk.axis_angle((0, -1, 0), math.radians(deg))
    p0 = _REF[ROWS[i]][:3, 3]
    return (p0 + np.array([dx, 0.0, dz]), mk.mat_to_quat(r))


def _off_table():
    """The panel lying flat on the table at x 0.12, y -0.20 (all ties torn)."""
    out = {}
    for i, r in enumerate(ROWS):
        out[r] = (np.array([0.12 - i * ROW_H, -0.20, TOP + BT / 2]), mk.mat_to_quat(mk.axis_angle((0, -1, 0),
                                                                                                  math.radians(90))))
    return out


def _geometry_check() -> None:
    # cans fit: four across, three deep, 1 mm apart, 2 mm behind the front wall
    assert IN_W >= 4 * 2 * CAN_R + 0.003 and IN_D >= 3 * 2 * CAN_R + 0.004
    # reach: the tray rim and the grips are under 18 cm above the table
    assert TH < 0.18 and GRIP_Z - TOP < 0.18 and PIN[2] - TOP < 0.18
    # pull side: the inner pad (6.4 mm, faces 7 mm behind the panel's mid-plane when near-closed) fits between the
    # two middle front cans over its 16 mm width: free depth behind the panel there
    def free_depth(u):  # behind the front wall's inner face, at lateral offset u from the gap between two cans
        return 0.002 + CAN_R - math.sqrt(max(0.0, CAN_R ** 2 - (PITCH / 2 - abs(u)) ** 2))
    inner_back = GRIP_X + (-F_NEAR) + 0.0064 - (XF + BT)
    assert min(free_depth(u) for u in np.linspace(-0.008, 0.008, 9)) > inner_back + 0.001, inner_back
    # the pads go 15 mm into the top row, which is 17.5 mm tall (2.5 mm clear of row 2)
    assert ROW_H - (TH - (GRIP_Z - PAD_TIP - TOP)) > 0.002
    # ties: the statics of the panel (rows rigid, hinged, on the eight springs) -> first-row tear load vs tray drag
    drag = MU_TRAY * (TRAY_KG + N_ROW * ROW_KG + 12 * CAN_KG) * 9.81
    f_start = S_SIDE / 0.55  # statics (statics.py): a pull out 7.5 mm below the top edge puts 0.55 of it on each top-row corner tie
    assert f_start > 1.15 * drag, (f_start, drag)
    print(f"tray drag {drag:.1f} N; first-row tear at about {f_start:.1f} N (x{f_start / drag:.2f}); "
          f"inner pad back {inner_back * 1000:.1f} mm behind the wall, free depth "
          f"{min(free_depth(u) for u in np.linspace(-0.008, 0.008, 9)) * 1000:.1f} mm", file=sys.stderr)


def self_test():
    """The tie rule and the scoring rules on hand-built sequences of states, one trial each."""
    global _TEST_TCPS
    _geometry_check()
    start = _states()
    # tie readout: the panel pulled rigidly out 1.5 mm -> each tie 6 N peel, nothing torn; 3.5 mm -> 14 N tears the
    # side ties (9 N) and the lip ties (8 N); lifted 6 mm in-plane -> 24 N >= 2 x 9 N: the wall tears
    st = _states(pull=np.array([-0.0015, 0, 0]))
    assert all(abs(tie_force(st, n)[0] - 6.0) < 1e-6 and tie_force(st, n)[1] < 1e-6 for n in TIE_NAMES)
    open_rows = {r: _row_moved(i, 100.0, dx=-0.03, dz=-0.01 * i) for i, r in enumerate(ROWS)}
    off = _off_table()
    cases = {  # name: ([state builders], TCPs at the last state, expected score)
        "start, untouched": ([lambda: start, lambda: start], [], 0.0),
        "panel pulled out 1.5 mm (6 N per tie): holds": ([lambda: start, lambda: _states(pull=np.array([-0.0015, 0, 0]))],
                                                      [], 0.0),
        "top row peeled out (its corner ties torn), rest on": (
            [lambda: start, lambda: _states(moved={"row1": _row_moved(0, 0.0, dx=-0.0035)}),
             lambda: _states(moved={"row1": _row_moved(0, 60.0, dx=-0.01)})], [], 0.25),
        "demo: all ties torn by peeling, panel on the table, hands clear, still": (
            [lambda: start, lambda: _states(pull=np.array([-0.0035, 0, 0])), lambda: _states(moved=off),
             lambda: _states(moved=off)], [], 1.0),
        "panel off but a gripper still holds it": (
            [lambda: start, lambda: _states(pull=np.array([-0.0035, 0, 0])), lambda: _states(moved=off),
             lambda: _states(moved=off)], [np.array([0.12, -0.20, TOP + 0.02])], 0.5),
        "panel off but dropped into the tray on the cans": (
            [lambda: start, lambda: _states(pull=np.array([-0.0035, 0, 0])),
             lambda: _states(moved={r: (np.array([XC, YC, TOP + 0.11]), (0, 0, 0, 1)) for r in ROWS}),
             lambda: _states(moved={r: (np.array([XC, YC, TOP + 0.11]), (0, 0, 0, 1)) for r in ROWS})], [], 0.5),
        "panel off, tray dragged 5 cm on the way (latched max)": (
            [lambda: start, lambda: _states(tray=_tx(-0.05)), lambda: _states(pull=np.array([-0.0035, 0, 0])),
             lambda: _states(moved=off), lambda: _states(moved=off)], [], 0.5),
        "ripped straight up hard: board torn off the line (in-plane 24 N)": (
            [lambda: start, lambda: _states(pull=np.array([0, 0, 0.006])), lambda: _states(moved=off),
             lambda: _states(moved=off)], [], 0.0),
        "ripped sideways hard: board torn off the line": (
            [lambda: start, lambda: _states(pull=np.array([0, 0.006, 0])), lambda: _states(moved=off),
             lambda: _states(moved=off)], [], 0.0),
        "tray tipped 15 deg to get at the panel, set back": (
            [lambda: start, lambda: _states(tray=_tilt(15.0)), lambda: _states(pull=np.array([-0.0035, 0, 0])),
             lambda: _states(moved=off), lambda: _states(moved=off)], [], 0.0),
        "tray lifted 3 cm (to shake the cans out), set back": (
            [lambda: start, lambda: _states(tray=_tx(0.0, dz=0.03)), lambda: _states(pull=np.array([-0.0035, 0, 0])),
             lambda: _states(moved=off), lambda: _states(moved=off)], [], 0.0),
        "a can pulled out of the tray and put back": (
            [lambda: start, lambda: _states(cans={"can01": (np.array([0.10, 0.0, TOP + CAN_H / 2]), (0, 0, 0, 1))}),
             lambda: _states(pull=np.array([-0.0035, 0, 0])), lambda: _states(moved=off),
             lambda: _states(moved=off)], [], 0.0),
        "a can knocked over inside the tray": (
            [lambda: start, lambda: _states(pull=np.array([-0.0035, 0, 0])),
             lambda: _states(moved=off, cans={"can01": (np.array([*_can_xy(0, 1), TOP + 0.04]),
                                                        mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(80))))}),
             lambda: _states(moved=off, cans={"can01": (np.array([*_can_xy(0, 1), TOP + 0.04]),
                                                        mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(80))))})],
            [], 0.5),
        "still moving at the end": (
            [lambda: start, lambda: _states(pull=np.array([-0.0035, 0, 0])), lambda: _states(moved=off),
             lambda: _states(moved={**off, "row1": (off["row1"][0] + [0.01, 0, 0], off["row1"][1])})], [], 0.5),
    }
    try:
        for name, (seq, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            states = [b() for b in seq]
            for s in states[:-1]:
                score(s)
            _TEST_TCPS = tcps
            got = score(states[-1])
            assert abs(got - want) < 1e-9 and success(states[-1]) == (want == 1.0), (name, got, want, _mem()["torn"])
        # the latches never clear: a wall tear early, then a clean finish, still 0
        mk.new_episode()
        _TEST_TCPS = []
        for s in [start, _states(pull=np.array([0, 0, 0.006])), start, _states(moved=off), _states(moved=off)]:
            got = score(s)
        assert got == 0.0, "damage latch"
        # a new trial forgets the last one
        mk.new_episode()
        for s in [start, _states(pull=np.array([-0.0035, 0, 0])), _states(moved=off), _states(moved=off)]:
            got = score(s)
        assert got == 1.0, "reset"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 3


def _tx(dx, dz=0.0):
    t = np.eye(4)
    t[0, 3], t[2, 3] = dx, dz
    return t


def _tilt(deg):
    t = np.eye(4)
    r = mk.axis_angle((0, 1, 0), math.radians(deg))
    pivot = np.array([XF + TD, YC, TOP])
    t[:3, :3], t[:3, 3] = r, pivot - r @ pivot
    return t


if __name__ == "__main__":
    # Offline checks, no GPU: geometry, the tie rule and the scoring rules on hand-built states, then both arms'
    # plans (demo and the run-1 control) replayed through the kinematics at 20 Hz, every joint inside its limits.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for tag, plans in (("demo", task.demo()), ("naive", naive_demo())):
        for arm, waypoints in plans.items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (tag, arm, plan.worst_ik_mm,
                                                                          plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (tag, arm)
            assert np.all((q[:, 6] >= 0.0) & (q[:, 6] <= 1.0)), (tag, arm)
            print(f"{tag} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
