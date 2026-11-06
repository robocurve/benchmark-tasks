"""Camera turntable on the station table. A grey turntable disc (120 mm diameter, 20 mm thick, 100 g) turns on a fixed
vertical axle at x 30 cm, y -7 cm, resting on the table (light bearing drag). A red arrow on its top (where the camera
would face) points away from the robot (+x); a grey knob (20 mm diameter, 25 mm tall, on its own pin) stands on the
disc 4 cm from the axle on the right (-y). A green tape mark lies on the table at the disc's robot-side edge, opposite
the arrow. The black 6 mm camera cable (56 cm) leaves a swivelling strain relief on the disc's far rim (x 36.35, y -7)
tangentially to the left (+y), passes 3 cm beyond a fixed grey post (25 mm diameter, 45 mm tall, at x 33.3, y 7) and
runs through a fixed cable clip (18 x 16 mm opening) at y 18; beyond the clip it lies loose to y 49. Turning the disc
half a turn clockwise winds the cable round the disc; turning it anticlockwise unwinds it but sweeps it into the post,
where it hooks round the post's far side unless it is first moved over the post to the robot's side.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: TT_DEMO=correct (default) | nothing | ccw_no_reroute | cw | reroute_only | hold.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Turn the camera turntable half a turn so its red arrow points at the green mark, then let go. Its "
               "cable must not end up wound round the turntable or wrapped round the grey post.")

GREY = (0.55, 0.56, 0.58)
DARK = (0.30, 0.31, 0.33)
RED = (0.85, 0.10, 0.08)
GREEN = (0.15, 0.65, 0.25)
BLACK = (0.04, 0.04, 0.04)

# -- turntable: free disc on a vertical D6 axle (see _install_spawn_hook) ------------------------------------------
C = np.array([0.30, -0.07])  # axle
DISC_R, DISC_H = 0.06, 0.020
DISC_Z = TOP + 0.0005 + DISC_H / 2  # settles onto the table: its weight gives bearing friction
DISC = Cylinder("turntable", pos=(float(C[0]), float(C[1]), DISC_Z), radius=DISC_R, height=DISC_H, mass=0.10,
                color=GREY)
DISC_TOP = DISC_Z + DISC_H / 2
KNOB_RK = 0.040  # knob axis distance from the axle
KNOB_A0 = -math.pi / 2  # knob starts on the right (-y) side
KNOB_R, KNOB_H = 0.010, 0.025
KNOB_P0 = np.array([C[0] + KNOB_RK * math.cos(KNOB_A0), C[1] + KNOB_RK * math.sin(KNOB_A0), DISC_TOP + KNOB_H / 2 + 0.0003])
KNOB = Cylinder("knob", pos=tuple(float(v) for v in KNOB_P0), radius=KNOB_R, height=KNOB_H, mass=0.01, color=DARK)
DAMPING = 0.004  # N m s/rad on the axle (light bearing friction)

# -- fixed post, cable clip and green mark ----------------------------------------------------------------------------
P = C + np.array([0.033, 0.14])
POST_R, POST_H = 0.0125, 0.045
POST = Cylinder("post", pos=(float(P[0]), float(P[1]), TOP + POST_H / 2), radius=POST_R, height=POST_H, color=DARK)

R = 0.003  # 6 mm cable
CORD_X = C[0] + DISC_R + R + 0.0005  # 0.3635: leaves the far rim tangentially
CORD_Z = TOP + R + 0.0005
CORD_START = (float(CORD_X), float(C[1]), float(CORD_Z))
CORD_YAW = math.pi / 2
CORD_LEN = 0.56
CORD = cable("camera_cable", start=CORD_START, yaw=CORD_YAW, length=CORD_LEN, radius=R, anchor_start=False,
             bend_stiffness=0.01, bend_damping=0.001, color=BLACK)

CLIP_Y = float(C[1]) + 0.25
B = np.array([CORD_X, CLIP_Y])  # the clip's centre: the cable's fixed pass-through point
CLIP_W, CLIP_HGT, CLIP_T, CLIP_L = 0.018, 0.016, 0.006, 0.020


def _clip() -> list[Box]:
    x, y = B
    wx = CLIP_W / 2 + CLIP_T / 2
    return [
        Box("clip_left", pos=(x - wx, y, TOP + CLIP_HGT / 2), size=(CLIP_T, CLIP_L, CLIP_HGT), color=DARK),
        Box("clip_right", pos=(x + wx, y, TOP + CLIP_HGT / 2), size=(CLIP_T, CLIP_L, CLIP_HGT), color=DARK),
        Box("clip_top", pos=(x, y, TOP + CLIP_HGT + 0.002), size=(CLIP_W + 2 * CLIP_T, CLIP_L, 0.004), color=DARK),
    ]


MARK = Box("green_mark", pos=(float(C[0] - DISC_R - 0.012), float(C[1]), TOP + 0.0005), size=(0.014, 0.024, 0.001),
           color=GREEN)

OBJECTS = [DISC, KNOB, POST, *_clip(), MARK, CORD]


# -- axle, knob pin, cable weld and the arrow / connector decals ------------------------------------------------------
# cablekit can only weld a cable end to the world, so the cable's spawn function is wrapped: right after the cable is
# authored (it is the last object), the disc gets its vertical axle to the world (D6 joint, yaw free), the knob its
# own vertical pin on the disc (a firm pinch must not lock the disc), and the cable's first segment is pinned to the
# disc's rim by a vertical swivel (revolute, -200..+20 deg). All are excluded from the cable articulation (PhysX solves them as external constraints).

def _install_spawn_hook() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_turntable_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        disc, knob = f"{parent}/{DISC.name}", f"{parent}/{KNOB.name}"
        w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()

        # axle: a D6 joint free in yaw and in height (+/-3 mm) only, so the disc's weight rests on the table and table
        # friction (a turntable's bearing drag) holds it wherever it is left; plus light viscous damping
        ax = UsdPhysics.Joint.Define(stage, f"{prim_path}/turntable_axle")
        ax.CreateBody1Rel().SetTargets([Sdf.Path(disc)])
        ax.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + C[0]), float(w[1] + C[1]), float(w[2] + DISC_Z)))
        ax.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        ax.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        ax.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        ax.CreateExcludeFromArticulationAttr(True)
        for dof in ("transX", "transY", "rotX", "rotY"):
            lim = UsdPhysics.LimitAPI.Apply(ax.GetPrim(), dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        lim = UsdPhysics.LimitAPI.Apply(ax.GetPrim(), "transZ")
        lim.CreateLowAttr(-0.003)
        lim.CreateHighAttr(0.003)
        drv = UsdPhysics.DriveAPI.Apply(ax.GetPrim(), "rotZ")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(0.0)
        drv.CreateDampingAttr(DAMPING * math.pi / 180)  # USD angular gains are per degree

        pin = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/knob_pin")
        pin.CreateBody0Rel().SetTargets([Sdf.Path(disc)])
        pin.CreateBody1Rel().SetTargets([Sdf.Path(knob)])
        pin.CreateAxisAttr("Z")
        off = KNOB_P0 - np.array([C[0], C[1], DISC_Z])
        pin.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in off)))
        pin.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        pin.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        pin.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        pin.CreateExcludeFromArticulationAttr(True)

        # the cable leaves the rim through a swivelling strain relief: a vertical pin that lets the cable exit turn
        # from tangential (as spawned) through radial to the other tangent, never into the disc
        weld = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/disc_weld")
        weld.CreateAxisAttr("Z")
        weld.CreateLowerLimitAttr(-200.0)
        weld.CreateUpperLimitAttr(20.0)
        weld.CreateBody0Rel().SetTargets([Sdf.Path(disc)])
        weld.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        off = np.asarray(CORD_START) - np.array([C[0], C[1], DISC_Z])
        weld.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in off)))
        weld.CreateLocalRot0Attr(Gf.Quatf(math.cos(CORD_YAW / 2), 0.0, 0.0, math.sin(CORD_YAW / 2)))
        weld.CreateLocalPos1Attr(Gf.Vec3f(-CORD.seg_len / 2, 0.0, 0.0))
        weld.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        weld.CreateExcludeFromArticulationAttr(True)
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(disc))
        fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])

        # decals: visual-only children of the disc body (no collider), so they turn with it
        for nm, pos, size, col in (
            ("arrow", (DISC_R * 0.45, 0.0, DISC_H / 2 + 0.001), (DISC_R * 0.9, 0.010, 0.002), RED),
            ("arrow_tip", (DISC_R * 0.80, 0.0, DISC_H / 2 + 0.0015), (0.014, 0.022, 0.002), RED),
            ("connector", (DISC_R - 0.002, 0.0, -0.004), (0.010, 0.014, 0.010), BLACK),
        ):
            cp = f"{disc}/{nm}"
            cube = UsdGeom.Cube.Define(stage, cp)
            cube.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(cube.GetPrim())
            xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
            xf.AddScaleOp().Set(Gf.Vec3f(*size))
            look = f"{disc}/Looks_{nm}"
            mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=0.5)
            mat.func(look, mat)
            bind_visual_material(cp, look, stage=stage)
        return prim

    spawn._turntable_hook = True
    cablekit._spawn_cable = spawn


_install_spawn_hook()

# -- grippers (score() needs them for the release check) --------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_turntable_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._turntable_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (0.134 m along the gripper axis); [] if unknown (offline)."""
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        a = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + a * 0.134)
    return out


# -- scoring ----------------------------------------------------------------------------------------------------------
FULL_DEG = 10.0  # arrow within 10 deg of the mark: 1.0
HALF_DEG = 30.0  # within 30 deg: 0.5
WIND_MAX = 180.0  # cable wound half a turn or more round the turntable axle: 0
RELEASE_DIST = 0.03
CLIP_TOL = 0.015


def disc_deg(objects) -> float:
    """Turntable yaw (deg, -180..180; 0 = arrow away from the robot, 180 = arrow at the green mark)."""
    x, y, z, w = (float(v) for v in objects[DISC.name].quat_xyzw)
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def arrow_error(objects) -> float:
    return abs(((disc_deg(objects) - 180.0) + 180.0) % 360.0 - 180.0)


def _signed_sweep(poly2: np.ndarray, q: np.ndarray) -> float:
    v = poly2 - q
    a = np.arctan2(v[:, 1], v[:, 0])
    d = (np.diff(a) + np.pi) % (2 * np.pi) - np.pi
    return float(np.degrees(d.sum()))


def cable_to_clip(objects) -> np.ndarray | None:
    """Top-view cable path from the disc connector to the clip (polyline cut at the point nearest the clip, then the
    clip centre); None if no part of the cable is within 1.5 cm of the clip centre (cable out of the clip)."""
    poly = objects[CORD.name].polyline()[:, :2]
    d = np.linalg.norm(poly - B, axis=1)
    i = int(np.argmin(d))
    if d[i] > CLIP_TOL:
        return None
    return np.vstack([poly[: i + 1], B])


def wind_deg(objects) -> float | None:
    """Signed angle (deg) the cable sweeps round the turntable axle from its connector to the clip (+ = anticlockwise
    seen from above). About +76 at the start; +251 after a clockwise half turn; -97 after the demo's anticlockwise turn."""
    p = cable_to_clip(objects)
    return None if p is None else _signed_sweep(p, C)


def post_loops(objects) -> int | None:
    """How many times the closed loop 'cable from connector to clip, then straight back to the connector' encircles
    the post's axis: 0 = the post is not enclosed between the cable and that straight line (not wrapped)."""
    p = cable_to_clip(objects)
    return None if p is None else int(round(_signed_sweep(np.vstack([p, p[:1]]), P) / 360.0))


def released(objects) -> bool:
    pts = objects[CORD.name].polyline()
    for g in grippers():
        if float(np.min(np.linalg.norm(pts - g, axis=1))) < RELEASE_DIST:
            return False
        for o in (KNOB, DISC):
            st = objects[o.name]
            if float(np.linalg.norm(np.maximum(np.maximum(st.low - g, g - st.high), 0.0))) < RELEASE_DIST:
                return False
    return True


def _finite(objects) -> bool:
    return all(np.isfinite(objects[o.name].pos).all() for o in (DISC, KNOB)) and bool(
        np.isfinite(objects[CORD.name].points).all())


def score(objects) -> float:
    """0 unless, at the end: no NaN, both grippers at least 3 cm from the cable, disc and knob, the cable still runs
    through the clip, it sweeps less than 180 deg round the turntable axle (connector to clip) and the post is not
    enclosed between the cable and the straight connector-clip line. Then 1.0 if the arrow is within 10 deg of the
    green mark, 0.5 if within 30 deg, else 0."""
    if not _finite(objects) or not released(objects):
        return 0.0
    w, n = wind_deg(objects), post_loops(objects)
    if w is None or abs(w) >= WIND_MAX or n != 0:
        return 0.0
    e = arrow_error(objects)
    return 1.0 if e <= FULL_DEG else 0.5 if e <= HALF_DEG else 0.0


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and objects[DISC.name].resting() and objects[KNOB.name].resting())


def distance(objects) -> float:
    """Arrow error (in half turns), plus 1 per post wrap and 1 if wound past half a turn."""
    if not _finite(objects):
        return 10.0
    w, n = wind_deg(objects), post_loops(objects)
    return arrow_error(objects) / 180.0 + (3.0 if w is None else float(abs(w) >= WIND_MAX) + abs(n))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    if not _finite(objects):
        return {"finite": 0.0}
    w, n = wind_deg(objects), post_loops(objects)
    return {
        "cable_robot_side_of_post": float(n is not None and n != 0 and arrow_error(objects) > 90.0),
        "turned": float(max(0.0, 1.0 - arrow_error(objects) / 180.0)),
        "not_wound": float(w is not None and abs(w) < WIND_MAX),
        "post_clear": float(n == 0),
        "released": float(released(objects)),
    }


VARIATION = {
    "cable:camera_cable": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # swivel-pinned to the disc rim: moves with the turntable
    "obj:post": {"dx": 0.005, "dy": 0.01, "yaw": 0.0},  # for a later seeded runner; the demo runs with no jitter
}

# -- closed-loop scripted demo (cabledemo.py) -------------------------------------------------------------------------
# The right arm pinches the knob (wrist camera looking across the disc, +y) and swings it anticlockwise round the
# measured axle, 12 deg per step, about 60 deg: the connector swings towards the clip and the cable goes slack. It holds
# the turntable there while the left arm takes the cable 3 cm past the post (wrist camera looking back along it at the
# post and disc), lifts it over the post, lays it down on the robot's side well clear of the post (pulling spare
# cable through the clip) and lets go. Then the right arm finishes the half turn and lets go.

FINGER_KNOB = -0.030
RYAW = math.pi / 2
GRAB_Y = P[1] + 0.03  # take the cable 3 cm past the post (fingers clear of it)
LAY_X, LAY_Y = 0.215, P[1] + 0.045  # robot side, well clear of the post: the detour pulls spare cable through the clip
LIFT_Z = TOP + 0.075
PRE_TURN = 60.0


def _log(ctx, tag=""):
    st = ctx.states()
    print(f"[t={ctx.t:5.2f}] {tag} disc={disc_deg(st):7.1f} err={arrow_error(st):5.1f} wind={wind_deg(st)} "
          f"loops={post_loops(st)} rel={released(st)} score={score(st)} prog={progress(st)}", flush=True)
    print(f"    knob={np.round(st[KNOB.name].pos, 4)} grippers={[np.round(g, 3).tolist() for g in grippers()]}",
          flush=True)


def _dbg_cable(ctx, tag):
    if os.environ.get("TT_DBG"):
        poly = ctx.states()[CORD.name].polyline()
        print(f"[dbg] {tag} cable xy z: {np.round(poly[:14], 3).tolist()} tcpL {np.round(ctx.left.tcp(), 3)}", flush=True)


def _grip_yaw(seg, p) -> float:
    """Grip yaw across the cable at ``p`` with the wrist camera looking back along it towards the turntable and
    post (heading of the cable reversed, in (-pi, 0])."""
    i = int(np.argmin(np.linalg.norm(seg - p, axis=1)))
    d = seg[min(i + 1, len(seg) - 1)] - seg[max(i - 1, 0)]
    a = math.atan2(d[1], d[0]) - math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _reroute(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L = ctx.left
    poly = ctx.states()[CORD.name].polyline()
    seg = poly[: int(np.argmin(np.linalg.norm(poly[:, :2] - B, axis=1)))]
    p = seg[int(np.argmin(np.abs(seg[:, 1] - GRAB_Y)))]
    print(f"[demo] cable grip at {np.round(p, 4)}", flush=True)
    gy = _grip_yaw(seg, p)
    yield from L.move((p[0], p[1], TOP + 0.06), yaw=gy, finger=FINGER_OPEN, duration=1.4)
    yield from L.move((p[0], p[1], p[2] + 0.009), yaw=gy, duration=0.8)
    yield from L.wait(0.2)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from L.wait(0.2)
    yield from L.move((p[0] - 0.01, p[1], LIFT_Z), yaw=gy, duration=1.2)
    _dbg_cable(ctx, "lifted")
    yield from L.move((LAY_X, LAY_Y, LIFT_Z), yaw=gy, duration=1.6)
    yield from L.move((LAY_X, LAY_Y, CORD_Z + 0.012), yaw=gy, duration=1.0)
    yield from L.wait(0.3)
    _dbg_cable(ctx, "laid")
    yield from L.grip(FINGER_OPEN, 0.4)
    yield from L.move((LAY_X, LAY_Y, TOP + 0.09), yaw=gy, duration=0.6)
    yield from L.move((0.16, 0.24, TOP + 0.15), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
    _log(ctx, "rerouted")


def _knob_target(ctx, delta_deg: float) -> np.ndarray:
    """Closed loop: where the knob goes if the disc turns ``delta_deg`` further about the axle."""
    k = np.asarray(ctx.states()[KNOB.name].pos, dtype=float)
    v = k[:2] - C
    t = math.radians(delta_deg)
    q = C + np.array([v[0] * math.cos(t) - v[1] * math.sin(t), v[0] * math.sin(t) + v[1] * math.cos(t)])
    return np.array([q[0], q[1], k[2]])


_GRASP = {"off": np.zeros(3)}


def _take_knob(ctx):
    """Right arm pinches the knob from above (it then holds the turntable still until it turns it)."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    k = ctx.states()[KNOB.name].pos
    zg = DISC_TOP + KNOB_H - 0.006  # pads pinch most of the knob height
    yield from Rt.move((k[0], k[1], zg + 0.05), yaw=RYAW, finger=FINGER_KNOB, duration=1.4)
    yield from Rt.move((k[0], k[1], zg), yaw=RYAW, duration=0.8)
    yield from Rt.wait(0.2)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    _GRASP["off"] = np.array([k[0], k[1], zg]) - np.asarray(ctx.states()[KNOB.name].pos)


def _turn(ctx, sign: float, total=180.0, step=12.0, dur=0.35, hold=False, release=True):
    """Swing the knob ``total`` deg round the axle (sign +1 = anticlockwise), tracking the measured disc angle."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    off = _GRASP["off"]
    start = disc_deg(ctx.states())
    done, last, stall, prev = 0.0, start, 0, -99.0
    for _ in range(60):
        now = disc_deg(ctx.states())
        done += ((now - last + 180.0) % 360.0) - 180.0
        last = now
        rem = total - sign * done
        stall = stall + 1 if abs(done - prev) < 0.5 else 0
        prev = done
        if rem < 1.5 or stall >= 4:  # there, or the turntable no longer follows (e.g. the cable snagged)
            break
        yield from Rt.move(_knob_target(ctx, sign * min(step, rem)) + off, yaw=RYAW, duration=dur)
        if os.environ.get("TT_DBG"):
            st = ctx.states()
            print(f"[turn] disc {disc_deg(st):6.1f} done {done:6.1f} knob {np.round(st[KNOB.name].pos, 4)} "
                  f"tcp {np.round(Rt.tcp(), 4)} wind {wind_deg(st):.1f} loops {post_loops(st)} clip_i "
                  f"{-1 if cable_to_clip(st) is None else len(cable_to_clip(st)) - 2}", flush=True)
    yield from Rt.wait(0.3)
    _log(ctx, "turned")
    if hold or not release:
        return
    yield from Rt.grip(FINGER_KNOB, 0.4)
    p = Rt.tcp()
    yield from Rt.move((p[0], p[1], p[2] + 0.07), yaw=RYAW, duration=0.8)
    yield from Rt.move((0.16, -0.24, TOP + 0.15), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)


def demo_script(ctx):
    mode = os.environ.get("TT_DEMO", "correct")  # test only: correct|nothing|ccw_no_reroute|cw|reroute_only|hold
    _log(ctx, f"start ({mode})")
    if mode == "nothing":
        for _ in range(5):
            yield from ctx.left.wait(1.0)
        _log(ctx, "end")
        return
    if mode == "reroute_only":
        yield from _reroute(ctx)
    else:
        sign = -1.0 if mode == "cw" else 1.0
        yield from _take_knob(ctx)
        if mode in ("correct", "hold"):
            # turn a third of the way first (the connector swings towards the clip, so the cable goes slack), hold
            # the turntable there while the left arm lifts the slack cable over the post, then finish the turn
            yield from _turn(ctx, sign, total=PRE_TURN, release=False)
            yield from _reroute(ctx)
            yield from _turn(ctx, sign, total=180.0 - PRE_TURN, hold=(mode == "hold"))
        else:
            yield from _turn(ctx, sign)
    yield from ctx.left.wait(2.0)
    _log(ctx, "end")
