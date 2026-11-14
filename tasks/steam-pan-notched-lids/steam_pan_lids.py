"""On the station table: a serving-line row of two clear polycarbonate food pans (a GN 1/3 pan, 325 x 176 x 65 mm, and a
GN 1/6 pan, 176 x 162 x 65 mm) with food in them, and three clear lids lying beside them: a notched GN 1/3 lid, a plain
GN 1/3 lid and a plain GN 1/6 lid. A serving spoon lies in the GN 1/3 pan, its handle resting over the pan's end rim.
The food in both pans is to be covered: each pan gets a lid seated flat on its rim, and the spoon stays in its pan.

The lids are 2 mm polycarbonate sheet (0.08-0.15 kg) with an 80 mm bridge handle, 20 mm high. The notched lid has a
50 x 35 mm slot in the middle of one short end. Everything is full size. The pans are fixtures; the lids and the
spoon are free rigid bodies. A plain lid set on the spoon's pan rests on the spoon's handle (about 3 mm proud of the
rim, tilted); the notched lid's notch has to sit over the handle. The notched lid on the GN 1/6 pan leaves its notch
open over the food.

Run it:  inspect-robots run --embodiment yam_arena -E task=steam_pan_lids.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Asset, ObjectState
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S: the run ends when it holds
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
TAG = "steam_pan_lids"

# -- the pans and lids (EN 631-1 GN sizes; a pan's frame: origin at the underside centre, +x along the 325 mm length) --
PAN_LEN = {"third": 325 * MM, "sixth": 176 * MM}  # GN 1/3 and GN 1/6 (EN 631-1: 325 x 176 and 176 x 162 mm)
PAN_W = {"third": 176 * MM, "sixth": 162 * MM}
PAN_H = 65 * MM  # the rim's top face is PAN_H above the table
FLANGE_W = 8 * MM  # the rim flange the lid rests on; the pan's opening is inside it
LID_T = 2.0 * MM
LID_MASS = {"third_notched": 0.145, "third_solid": 0.149, "sixth_solid": 0.080}
HANDLE_H, HANDLE_L, HANDLE_W, BAR_T = 20 * MM, 80 * MM, 12 * MM, 7 * MM  # the lid's bridge handle
NOTCH_W, NOTCH_D = 50 * MM, 35 * MM  # the notch in the middle of a lid's +x end

# -- the spoon (its frame: origin at the bowl's centre; it rests with its handle rising SPOON_TILT toward the pan's +x end,
# the handle's underside on the end rim's inner edge, the bowl on the pan's floor) ---------------------------------------
BOWL_R = 30 * MM
SPOON_TILT = math.radians(19.7)
SPOON_X = 54.5 * MM  # the bowl's centre from the pan's centre, along its length
SPOON_Z = 2 * MM + BOWL_R + 0.3 * MM  # the bowl's centre above the table (floor 2 mm thick; 0.3 mm to settle)
SPOON_MASS = 0.09

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
# One row across the table, every long axis along x (away from the robot); the notch end and the spoon's handle at +x.
ROW_X = 0.31
PAN_XY = {"third": (ROW_X, -0.098), "sixth": (ROW_X, 0.287)}
LID_XY = {"sixth_solid": (ROW_X, 0.469), "third_solid": (ROW_X, 0.098), "third_notched": (ROW_X, -0.294)}


def _ry(a: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), a)


def _rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw)


def lid_size(name: str) -> str:
    return "third" if "third" in name else "sixth"


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: one rigid body, or a fixture (``kinematic``).
    ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    fixed_base: bool = False
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed_base)


MAX_DEPENETRATION = 0.2  # m/s


def _quat_y(angle: float) -> tuple[float, float, float, float]:
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


PANS = {k: UsdPart(f"pan_{k}", asset=f"pan_{k}", xy=PAN_XY[k], extent=PAN_LEN[k], mass=None,
                   usd_path=str(ASSETS / f"pan_{k}.usda"), pos=(*PAN_XY[k], TOP), kinematic=True) for k in PAN_W}
LIDS = {k: UsdPart(f"lid_{k}", asset=f"lid_{k}", xy=LID_XY[k], extent=PAN_LEN[lid_size(k)], mass=LID_MASS[k],
                   usd_path=str(ASSETS / f"lid_{k}.usda"), pos=(*LID_XY[k], TOP + 0.0002)) for k in LID_XY}
SPOON_POS = (PAN_XY["third"][0] + SPOON_X, PAN_XY["third"][1], TOP + SPOON_Z)
SPOON_QUAT = _quat_y(-SPOON_TILT)  # +x of the spoon's frame rises
SPOON = UsdPart("spoon", asset="spoon", xy=PAN_XY["third"], extent=0.27, mass=SPOON_MASS,
                usd_path=str(ASSETS / "spoon.usda"), pos=SPOON_POS, rot_xyzw=SPOON_QUAT)
OBJECTS = [*PANS.values(), *LIDS.values(), SPOON]
LID_NAMES = tuple(f"lid_{k}" for k in LIDS)


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon=addon,
    )


@dataclass(frozen=True)
class GripperState:
    """One gripper after a control step (world frame, m): the span of its finger pads along its pointing axis (``back``
    to the pad ``tips``), how far apart the pads are, and the smaller of the two fingers' grip forces (N): each
    finger's drive stiffness times how far something holds it back from its commanded position."""

    back: np.ndarray
    tips: np.ndarray
    opening: float
    contact: float


PAD_BACK = 0.025  # m: the pads reach this far behind the TCP


def _gripper_states(env) -> dict:
    """Each gripper's ``GripperState`` and the episode's time (s), keyed ``left_gripper``, ``right_gripper`` and
    ``episode_time``."""
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {"episode_time": float(base.episode_length_buf[0]) * float(base.step_dt)}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        axis = runtime.link_poses(robot)["gripper"][:3, 2]
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)  # fingers open to -q each; a closing finger is held back
        out[f"{side}_gripper"] = GripperState(
            back=tcp - PAD_BACK * axis,
            tips=tcp + PAD_TIP_PAST_TCP * axis,
            opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])),
        )
    return out


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root pose and speeds (Arena's geometry queries are left at the root), plus the grippers and
        the episode time."""
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        out = {}
        for name in names:
            data = base.scene[name].data

            def arr(t):
                return t.torch[0].detach().cpu().numpy().astype(np.float64)

            pose = arr(data.root_link_pose_w)
            pos = pose[:3] - origin
            out[name] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=arr(data.root_lin_vel_w),
                                    ang_vel=arr(data.root_ang_vel_w), low=pos.copy(), high=pos.copy())
        out.update(_gripper_states(env))
        return out

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states


class _AfterSceneImport(importlib.abc.MetaPathFinder):
    """Patches ``yam_arena.scene`` right after it is first imported (it needs Isaac Sim, which starts later)."""

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        spec = importlib.util.find_spec(fullname)
        run = spec.loader.exec_module

        def exec_module(module):
            run(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _AfterSceneImport) for f in sys.meta_path):
    sys.meta_path.insert(0, _AfterSceneImport())


# -- scoring ----------------------------------------------------------------------------------------------------------
ON_PAN = 0.03  # m: a lid whose underside is within this of the rim's top face (and nearly level) is over the pan
LEVEL = math.radians(10.0)  # ... "nearly level"
COVER_SPOON = 0.95  # the spoon pan's opening covered at least this much: the notch may stay open over the handle
COVER_FULL = 0.995  # a pan without a utensil: no opening (a notch leaves about 6% of a GN 1/6 pan open)
SEAT_Z = 0.0015  # m: seated: the lid's underside within this of the rim's top face,
SEAT_TILT = math.radians(1.0)  # ... tipped less than this,
SEAT_XY = 0.006  # ... centred on the pan within this (both ways), and
SEAT_YAW = math.radians(2.5)  # ... square to it within this (a lid is the same turned third a turn, apart from its notch)
SPOON_MOVE = 0.02  # m: the spoon's bowl within this of where it started, and
SPOON_TURN = math.radians(6.0)  # ... turned less than this from its resting angle: still in its pan
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
CLEAR = 0.03  # m: released: every gripper's pads at least this far from every lid and the spoon (none touching)
# Held: a gripper's pads span the part (its bounding box, grown by GRIP_SLACK), both fingers press on something with at
# least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING (not closed on nothing).
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
STILL_S, STILL_MOVE, STILL_TURN = 1.0, 0.002, math.radians(2.0)  # settled: within 2 mm and 2 degrees over 1 s
TRACE_S = 1.0  # s between trace lines in a run
STEPS = ("spoon pan covered", "spoon pan's lid seated flat, spoon still in", "second pan covered fully",
         "second pan's lid seated flat")
POINTS = 0.2  # per step; only success scores 1.0
GRID = (np.linspace(-0.5, 0.5, 31), np.linspace(-0.5, 0.5, 25))  # samples of a pan's opening (fractions of its size)


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _lid_kind(name: str) -> str:
    return name.split("_", 1)[1] if name.startswith("lid_") else name


def _lid_box(name):
    w, ln = PAN_W[lid_size(name)], PAN_LEN[lid_size(name)]
    return (np.array([-ln / 2, -w / 2, 0.0]), np.array([ln / 2, w / 2, LID_T + HANDLE_H]))


def _box(pose, box) -> tuple[np.ndarray, np.ndarray]:
    """The world-aligned bounds of ``box`` (low, high corners in the part's frame) at ``pose``."""
    lo, hi = box
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = corners @ _rot(pose).T + pose.pos
    return w.min(0), w.max(0)


def _held(bounds, gripper) -> bool:
    """``gripper`` holds the part with world ``bounds``: its pads reach them and both fingers press."""
    if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
        return False
    low, high = bounds[0] - GRIP_SLACK, bounds[1] + GRIP_SLACK
    span = (gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11))
    return any(bool(np.all(q >= low) and np.all(q <= high)) for q in span)


def lid_held(objects, name) -> bool:
    bounds = _box(objects[name], _lid_box(name))
    return any(_held(bounds, objects.get(g)) for g in ("left_gripper", "right_gripper"))


def lid_on_pan(objects, name, pan) -> dict:
    """Where a lid is on a pan (the pan's frame, m and rad): ``rise``, the lid's underside above the rim's top face;
    ``off``, its centre's offset (x, y); ``yaw``, its turn from square (modulo a third turn); ``tilt``, its tip from
    level; ``cover``, the fraction of the pan's opening under the lid's plate (the notch is open)."""
    lid = objects[name]
    pan_pos = np.array([*PAN_XY[pan], TOP])
    r = _rot(lid)
    rel = lid.pos - pan_pos
    yaw = math.atan2(r[1, 0], r[0, 0])
    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
    # coverage: sample the opening, test each point in the lid's frame
    w_open = PAN_W[pan] - 2 * FLANGE_W
    xs = GRID[0] * (PAN_LEN[pan] - 2 * FLANGE_W)
    ys = GRID[1] * w_open
    px, py = np.meshgrid(xs, ys, indexing="ij")
    pts = np.stack([px.ravel() + pan_pos[0], py.ravel() + pan_pos[1], np.full(px.size, lid.pos[2])], axis=1)
    q = (pts - lid.pos) @ r  # to the lid's frame
    wl, ll = PAN_W[lid_size(name)], PAN_LEN[lid_size(name)]
    inside = (np.abs(q[:, 0]) <= ll / 2) & (np.abs(q[:, 1]) <= wl / 2)
    if lid_kind_notched(name):
        inside &= ~((q[:, 0] >= ll / 2 - NOTCH_D) & (np.abs(q[:, 1]) <= NOTCH_W / 2))
    return {
        "rise": float(rel[2] - PAN_H),
        "off": (float(rel[0]), float(rel[1])),
        "yaw": float(yaw),
        "tilt": float(math.acos(min(1.0, max(-1.0, r[2, 2])))),
        "cover": float(inside.mean()),
    }


def lid_kind_notched(name: str) -> bool:
    return "notched" in name


def _over(c) -> bool:
    return bool(abs(c["rise"]) < ON_PAN and c["tilt"] < LEVEL)


def _seated(c) -> bool:
    return bool(abs(c["rise"]) < SEAT_Z and c["tilt"] < SEAT_TILT and abs(c["off"][0]) < SEAT_XY
                and abs(c["off"][1]) < SEAT_XY and abs(c["yaw"]) < SEAT_YAW)


def spoon_in(objects) -> bool:
    """The spoon still lies in its pan, where it started: its bowl within SPOON_MOVE and its angle within SPOON_TURN."""
    s = objects["spoon"]
    turn = 2 * math.acos(min(1.0, abs(float(np.dot(s.quat_xyzw, SPOON_QUAT)))))
    return bool(np.linalg.norm(np.asarray(s.pos) - np.asarray(SPOON_POS)) <= SPOON_MOVE and turn <= SPOON_TURN)


def covering(objects, pan, threshold, skip=()) -> tuple[str | None, str | None]:
    """The lid resting on ``pan`` (over it, nearly level, in no gripper's hold, its plate over at least ``threshold`` of
    the opening), preferring a seated one, and the lid (if any) that is also seated. Lids in ``skip`` are not
    considered."""
    best, seated = None, None
    for name in LID_NAMES:
        if name in skip or lid_held(objects, name):
            continue
        c = lid_on_pan(objects, name, pan)
        if _over(c) and c["cover"] >= threshold:
            best = best or name
            if _seated(c):
                seated = best = name
    return best, seated


def _pads(g):
    return [g.back + f * (g.tips - g.back) for f in np.linspace(0.0, 1.0, 11)]


def _dist_box(q, bounds) -> float:
    lo, hi = bounds
    return float(np.linalg.norm(np.maximum(np.maximum(lo - q, 0.0), q - hi)))


def released(objects) -> bool:
    """No gripper touches a lid or the spoon: every gripper's pads are at least ``CLEAR`` from each of them."""
    parts = [_box(objects[n], _lid_box(n)) for n in LID_NAMES]
    parts.append(_box(objects["spoon"], (np.array([-BOWL_R, -0.012, -BOWL_R]), np.array([0.23, 0.012, 0.15]))))
    for side in ("left", "right"):
        g = objects.get(f"{side}_gripper")
        if g is None:
            continue
        for q in _pads(g):
            if min(_dist_box(q, b) for b in parts) < CLEAR:
                return False
    return True


def _poses(objects) -> tuple:
    """The poses whose stillness success needs: each lid and the spoon (position, quaternion)."""
    return tuple((np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float))
                 for n in (*LID_NAMES, "spoon"))


def still(objects, anchor) -> bool:
    """Every lid and the spoon within STILL_MOVE and STILL_TURN of their poses in ``anchor``. Judged from the poses,
    not the simulator's instantaneous speeds."""
    for (pos, quat), (pos0, quat0) in zip(_poses(objects), anchor, strict=True):
        turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
        if np.linalg.norm(pos - pos0) > STILL_MOVE or turn > STILL_TURN:
            return False
    return True


def _steps(objects) -> list[bool]:
    """Each of ``STEPS`` as things stand. The spoon pan is covered by a lid over it; the lid is seated flat and the
    spoon still in its pan; the second pan is covered fully (no opening) by another lid; that lid is seated flat."""
    cover1, seat1 = covering(objects, "third", COVER_SPOON)
    in1 = spoon_in(objects)
    s2 = bool(seat1 and in1)
    cover2, seat2 = covering(objects, "sixth", COVER_FULL, skip=(cover1,) if cover1 else ())
    s3 = bool(cover2)
    return [bool(cover1), s2, s3, bool(s3 and seat2)]


class _Progress:
    """What has happened so far this episode, updated once per control step: since when every full-success condition
    has held without a break (``met_since``, ``None`` while one fails) with the poses when they began to hold
    (``anchor``). Any lapse (a step undone, a violation, a gripper near a lid, or a lid or the spoon moving more than
    STILL_MOVE or turning more than STILL_TURN from the anchor) restarts the count. Starts over when the episode time
    goes back."""

    def __init__(self):
        self.time = -1.0
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:  # this control step already counted
                return
            if t < self.time:
                self.__init__()
            self.time = t
        if t is None:  # no clock: nothing can be timed
            return
        met = bool(all(_steps(objects)) and not _violation(objects) and released(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'parts moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        """How long every full-success condition has held without a break (s), 0 while one fails."""
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):  # in a run: the step timeline, a trace
        bits = []
        for pan in ("third", "sixth"):
            for n in LID_NAMES:
                c = lid_on_pan(objects, n, pan)
                if abs(c["rise"]) < 0.05 and abs(c["off"][0]) < 0.2 and abs(c["off"][1]) < 0.2:
                    bits.append(f"{n} on {pan}: {c['rise'] * 1000:+.1f} mm, tilt {math.degrees(c['tilt']):.1f} deg, "
                                f"cover {c['cover']:.3f}, held {lid_held(objects, n)}")
        s = objects["spoon"]
        line = f"({'; '.join(bits) or 'no lid over a pan'}; spoon {np.linalg.norm(np.asarray(s.pos) - np.asarray(SPOON_POS)) * 1000:.1f} mm moved)"
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: a lid or the spoon off the table, the spoon thrown out of its pan, or a state that is not finite."""
    ps = [np.asarray(objects[n].pos, float) for n in (*LID_NAMES, "spoon")]
    if not np.all(np.isfinite(ps)):
        return True
    if any(p[2] < TOP + FALLEN_Z for p in ps):
        return True
    lp = np.asarray(objects["spoon"].pos) - np.array([*PAN_XY["third"], 0.0])
    return bool(abs(lp[0]) > PAN_LEN["third"] / 2 or abs(lp[1]) > PAN_W["third"] / 2)


def violation(objects) -> bool:
    """``_violation`` after counting this control step."""
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: all four steps done, every gripper clear of the lids and the spoon, no violation, and the lids and the spoon
    within STILL_MOVE and STILL_TURN of where they were when that second began. The arms' return is not scored."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise 0.2 per step done (``STEPS``, at most
    0.8, also while the second is being timed); 0 for any violation."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Each arm in turn: from above, the fingers closing across the lid's bridge handle (12 mm wide, 20 mm high; the pad tips
# end below its bar), lift the lid, carry it level over its pan, lower it onto the rim, open and lift away, go home.
# The left arm takes the plain GN 1/6 lid to the GN 1/6 pan, then the right arm takes the notched GN 1/3 lid to the spoon
# pan (notch end over the spoon's handle).
SQUEEZE = 0.006  # the grip closes the fingers this far inside the handle's width (about 15 N a finger)
HANDLE_HOLD = -(HANDLE_W - SQUEEZE) / 2
HANDLE_RELEASE = HANDLE_HOLD - 0.008  # opens each finger 8 mm
BAR_CENTRE = LID_T + HANDLE_H - BAR_T / 2  # the bar's middle above the lid's underside
TIPS_BELOW = 0.006  # the pad tips this far below the bar's middle
CARRY_UNDER = 0.09  # m: the carried lid's underside above the table (25 mm above the rims)
TOUCH = 0.0005  # m: the lid is lowered to this far above the rim before the fingers open


@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None  # a full gripper (TCP) rotation; replaces ``yaw`` when set


def _target(self, t: float):
    """``ArmPlan.target`` that also follows a waypoint's ``rot``."""
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
        rot = w.rot if getattr(w, "rot", None) is not None else yam_demo.grip_rotation(w.yaw)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (
                prev_pos + s * (goal - prev_pos),
                ik.slerp_matrix(prev_rot, rot, s),
                prev_finger + s * (w.finger - prev_finger),
                w.label,
            )
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


yam_demo.ArmPlan.target = _target

_Z = np.array([0.0, 0.0, 1.0])
GRIP_ROT = yam_demo.grip_rotation(0.0)  # pointing down, the fingers closing along the table's y: across the handle


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ _Z)


def _tcp_lid(xy, under: float) -> np.ndarray:
    """The TCP holding a lid whose centre is over ``xy`` with its underside ``under`` above the table."""
    return _tcp_with_tips((xy[0], xy[1], TOP + under + BAR_CENTRE), TIPS_BELOW, GRIP_ROT)


def lid_plan(lid_xy, pan_xy, start_delay: float, start_tcp, start_rot) -> list[Waypoint]:
    seat = PAN_H + TOUCH
    wps = []
    if start_delay > 0:
        wps.append(Waypoint(start_delay, tuple(start_tcp), FINGER_OPEN, "wait for the other arm", rot=start_rot))
    wps += [
        Waypoint(3.0, _tcp_lid(lid_xy, 0.0) + (0, 0, 0.08), FINGER_OPEN, "over the lid's handle", rot=GRIP_ROT),
        Waypoint(1.5, _tcp_lid(lid_xy, 0.0), FINGER_OPEN, "down around the handle", rot=GRIP_ROT),
        Waypoint(0.8, _tcp_lid(lid_xy, 0.0), HANDLE_HOLD, "grip the handle", rot=GRIP_ROT),
        Waypoint(1.6, _tcp_lid(lid_xy, CARRY_UNDER), HANDLE_HOLD, "lift the lid", rot=GRIP_ROT),
        Waypoint(2.5, _tcp_lid(pan_xy, CARRY_UNDER), HANDLE_HOLD, "carry it over the pan", rot=GRIP_ROT),
        Waypoint(1.6, _tcp_lid(pan_xy, seat + 0.01), HANDLE_HOLD, "lower it", rot=GRIP_ROT),
        Waypoint(1.0, _tcp_lid(pan_xy, seat), HANDLE_HOLD, "onto the rim", rot=GRIP_ROT),
        Waypoint(0.6, _tcp_lid(pan_xy, seat), HANDLE_HOLD, "settle", rot=GRIP_ROT),
        Waypoint(0.8, _tcp_lid(pan_xy, seat), HANDLE_RELEASE, "let go", rot=GRIP_ROT),
        Waypoint(1.2, _tcp_lid(pan_xy, CARRY_UNDER), HANDLE_RELEASE, "lift away", rot=GRIP_ROT),
    ]
    return wps


def demo():
    """The left arm covers the GN 1/6 pan with the plain GN 1/6 lid; then the right arm covers the spoon pan with the
    notched lid. Both go home."""
    left = lid_plan(LID_XY["sixth_solid"], PAN_XY["sixth"], 0.0, _START_TCP["left"], _START_ROT["left"])
    t_left = sum(w.duration for w in left)
    left.append(Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]))
    right = lid_plan(LID_XY["third_notched"], PAN_XY["third"], t_left, _START_TCP["right"], _START_ROT["right"])
    right.append(Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]))
    return {"left": left, "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
