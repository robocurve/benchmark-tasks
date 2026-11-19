"""A back-yard patio. A 6.3 L blue plastic pail (22 cm across the brim inside, 20 cm deep) stands empty on the
pavers in front of the robot. A green 5/8 inch garden hose, about 1 m of it, runs from a brass tap on a brick pillar
at the back left across the pavers to a pistol-grip spray nozzle lying on its side on the ground at the front left.
The tap is open: water comes out only while the nozzle's trigger is squeezed, more the further it is squeezed, up to
12 L a minute.

Job: pick the nozzle up, aim it into the pail and squeeze the trigger until the pail is full to the brim, then let
go of the trigger. The robot holds the nozzle with one hand and squeezes the trigger with the other. Water that
misses the pail or runs over its brim is spilled.

Scored by the water: what is in the pail, how much was spilled, and whether the water is off at the end.
"""

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import hoseskin  # noqa: E402
import kit  # noqa: E402
import meshes as m  # noqa: E402
import water as w  # noqa: E402
from parts import (  # noqa: E402,F401
    BRIM_Z, BUCKET, BUCKET_KG, COUPLING_END, FLOOR_Z, GRASP_X, GROUND, HEAD_R, HOSE_LINKS, HOSE_NODES, HOSE_R, NOZZLE,
    NOZZLE_POS0, NOZZLE_ROT0, OBJECTS, OUTLET, SCENE, TRIGGER, r_in,
)

_LINK_LEN = np.linalg.norm(np.diff(HOSE_NODES, axis=0), axis=1)


def _draw_hose(positions, rotations, end=None):
    pts, side = hoseskin.centre_line(positions, rotations, _LINK_LEN, start=HOSE_NODES[0], end=end)
    tube, stripe = hoseskin.skin(pts, side, HOSE_R)
    kit.set_mesh("hose_skin", *tube)
    kit.set_mesh("hose_stripe", *stripe)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

kit.allow_matrix_grips()

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring rules -------------------------------------------------------------------------------------------------

FULL_GAP = 0.015  # full: the water's surface within 15 mm of the brim
SPILL_OK_L = 0.25  # at most this much spilled (missed or run over) for full credit
SPILL_FAIL_L = 1.0  # more than this spilled: 0
TIP_FAIL_DEG = 15.0  # the pail tipped this far at any moment: 0 (water sloshes out)
OFF_S = 1.0  # the water must have been off this long to count as stopped
TIERS = ((0.85, 0.75), (0.50, 0.50), (0.25, 0.25))  # (fraction of the brimful volume in the pail, score)


def _disc(centre, radius, z, wobble=0.0, phase=0.0, n=28, thick=0.0012):
    """A thin flat disc (world), its rim wavy by ``wobble``: foam or a wet patch."""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    r = radius * (1.0 + wobble * np.sin(3 * a + phase) + 0.5 * wobble * np.sin(7 * a - 2 * phase))
    ring = np.column_stack([centre[0] + r * np.cos(a), centre[1] + r * np.sin(a)])
    pts = np.vstack([np.c_[ring, np.full(n, z)], np.c_[ring, np.full(n, z + thick)],
                     [[centre[0], centre[1], z]], [[centre[0], centre[1], z + thick]]])
    faces = []
    for j in range(n):
        k = (j + 1) % n
        faces += [(j, k, n + k), (j, n + k, n + j), (2 * n + 1, n + j, n + k), (2 * n, k, j)]
    return pts, np.array(faces)


class Puddles:
    """Wet patches on the ground where spilled water landed: litres per spot, merged within 4 cm."""

    def __init__(self):
        self.spots: list[list[float]] = []  # [x, y, litres]

    def add(self, x, y, litres):
        for spot in self.spots:
            if math.hypot(spot[0] - x, spot[1] - y) < 0.04:
                spot[2] += litres
                return
        self.spots.append([x, y, litres])

    def mesh(self):
        if not self.spots:
            return None
        parts = [_disc((x, y), min(0.16, 0.02 + 0.20 * math.sqrt(litres)), GROUND + 0.0004 + 0.0001 * i,
                       wobble=0.12, phase=i) for i, (x, y, litres) in enumerate(self.spots)]
        return m.merge(*parts)


class Episode:
    def __init__(self):
        self.tank = w.Tank()
        self.tipped = False
        self.trigger_deg = 0.0
        self.flow = 0.0
        self.landing = "none"
        self.t = 0.0
        self.max_tilt = 0.0
        self.puddles = Puddles()


EP = Episode()
_TRACE = None


def trigger_angle(nozzle_rot, trigger_rot) -> float:
    """How far the trigger is squeezed (deg): its turn about the nozzle's y axis."""
    rel = np.asarray(nozzle_rot).T @ np.asarray(trigger_rot)
    return math.degrees(math.atan2(rel[0, 2], rel[0, 0]))


def score_of(ep: Episode) -> float:
    """The score of an episode's water so far (see task.json)."""
    tank = ep.tank
    if ep.tipped or tank.spilled > SPILL_FAIL_L:
        return 0.0
    if (tank.level >= w.DEPTH - FULL_GAP and tank.off_s >= OFF_S and tank.spilled <= SPILL_OK_L):
        return 1.0
    fraction = tank.in_bucket / w.BRIM_L
    for need, value in TIERS:
        if fraction >= need:
            return value
    return 0.0


def advance(ep: Episode, dt, nozzle_pos, nozzle_rot, trigger_rot, bucket_pos, bucket_rot):
    """One control step of water: the flow the trigger lets through, where it lands, and the pail's state."""
    ep.t += dt
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.asarray(bucket_rot)[2, 2])))))
    ep.max_tilt = max(ep.max_tilt, tilt)
    if tilt > TIP_FAIL_DEG:
        ep.tipped = True
    ep.trigger_deg = trigger_angle(nozzle_rot, trigger_rot)
    ep.flow = w.flow(ep.trigger_deg)
    landing = None
    if ep.flow > 0.0:
        rot = np.asarray(nozzle_rot)
        outlet = np.asarray(nozzle_pos) + rot @ OUTLET
        floor = np.asarray(bucket_pos) + np.asarray(bucket_rot) @ np.array([0.0, 0.0, FLOOR_Z])
        landing = w.trace(outlet, rot[:, 0], w.exit_speed(ep.flow), floor, bucket_rot, ep.tank.level, GROUND)
        before = ep.tank.overflow
        ep.tank.add(ep.flow * dt, landing.into_bucket)
        ep.landing = landing.what
        if not landing.into_bucket:
            ep.puddles.add(float(landing.point[0]), float(landing.point[1]), ep.flow * dt)
        if ep.tank.overflow > before:  # over the brim: it runs down the pail's outside onto the ground
            side = np.asarray(outlet[:2]) - np.asarray(bucket_pos[:2])
            side = side / max(np.linalg.norm(side), 1e-9)
            spot = np.asarray(bucket_pos[:2]) - side * (w.R_TOP + 0.02)
            ep.puddles.add(float(spot[0]), float(spot[1]), ep.tank.overflow - before)
        ep.tank.off_s = 0.0
    else:
        ep.landing = "none"
        ep.tank.off_s += dt
    ep.tank.flowing = ep.flow > 0.0
    return landing


# -- in the simulator ----------------------------------------------------------------------------------------------


def _water_mesh(level: float, t: float, flowing: bool):
    """The water in the pail (bucket frame): a frustum up to the level, its surface rippling while water pours in."""
    top = FLOOR_Z + level
    bottom = FLOOR_Z + 0.0006
    prof = [(0.0, bottom), (r_in(bottom) - 0.0008, bottom), (r_in(top) - 0.0008, top)]
    rings = 10
    for k in range(rings - 1, 0, -1):
        r = (r_in(top) - 0.0008) * k / rings
        ripple = 0.0009 * math.sin(55.0 * r - 14.0 * t) * (1.0 - k / rings) if flowing else 0.0
        prof.append((r, top + ripple))
    prof.append((0.0, top + (0.0012 if flowing else 0.0)))
    return m.lathe(prof, 72)


STREAM_R = 0.0075  # the flood pattern's soft column leaves the dial about 15 mm across


def _stream_mesh(landing, speed0, t):
    """The stream along its path (world), thinning as it speeds up, its surface wavering a little."""
    path = landing.path
    if len(path) < 2:
        return None
    if len(path) > 80:
        path = path[np.linspace(0, len(path) - 1, 80).round().astype(int)]
    drop = np.maximum(path[0, 2] - path[:, 2], 0.0)
    speed = np.sqrt(speed0 ** 2 + 2 * w.G * drop)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    radius = STREAM_R * np.sqrt(speed0 / np.maximum(speed, 1e-6)) * (1.0 + 0.08 * np.sin(90.0 * s - 40.0 * t))
    return m.tube(path, radius, 16, cap=True)




def _render_settings():
    """Real-time RTX draws glass (the water) opaque and dark unless translucency is on. Rendering only: no effect
    on the physics or the score."""
    try:
        import carb

        settings = carb.settings.get_settings()
        settings.set("/rtx/translucency/enabled", True)
        settings.set("/rtx/translucency/maxRefractionBounces", 6)
        settings.set("/rtx/translucency/reflectionCutoff", 0.0)
        settings.set("/rtx/raytracing/fractionalCutoutOpacity", True)
        # the cameras path-trace (RealTimePathTracing) with 3 bounces: too few to see through water to the pail
        settings.set("/rtx/rtpt/maxBounces", 6)
        settings.set("/rtx/rtpt/maxSpecularAndTransmissionBounces", 8)
    except Exception:  # noqa: BLE001 -- rendering only
        pass


@kit.on_reset
def _reset():
    global EP, _TRACE
    _render_settings()
    EP = Episode()
    EP.tank.in_bucket = float(os.environ.get("HOSE_PREFILL_L", "0"))  # previews only; trials start empty
    kit.set_mesh("water_body", visible=False)
    kit.set_mesh("stream", visible=False)
    kit.set_mesh("foam", visible=False)
    kit.set_mesh("puddles", visible=False)
    if _TRACE is not None:
        _TRACE.close()
    path = os.environ.get("HOSE_TRACE")
    _TRACE = open(path, "w") if path else None  # noqa: SIM115 -- closed at the next reset


_MASS_VIEW = {}


def _set_bucket_mass(env, kg: float) -> None:
    """The pail weighs what it holds: its empty mass plus 1 kg per litre of water."""
    try:
        obj = env.unwrapped.scene[BUCKET.name]
        view = obj.root_physx_view
        masses = view.get_masses()
        masses[:] = kg
        view.set_masses(masses, _MASS_VIEW.setdefault("ids", np.arange(masses.shape[0])))
    except Exception:  # noqa: BLE001 -- the mass is cosmetic for scoring; never fail the episode over it
        pass


@kit.on_step
def _observe(env):
    base = env.unwrapped
    noz = kit._body_state(env, NOZZLE)
    trig = kit._body_state(env, TRIGGER)
    buck = kit._body_state(env, BUCKET)
    links = [kit._body_state(env, lk) for lk in HOSE_LINKS]
    _draw_hose([st.pos for st in links], [st.rotation() for st in links],
               end=noz.pos + noz.rotation() @ COUPLING_END)
    spots = len(EP.puddles.spots), sum(s[2] for s in EP.puddles.spots)
    landing = advance(EP, base.step_dt, noz.pos, noz.rotation(), trig.rotation(), buck.pos, buck.rotation())
    if landing is not None:
        mesh = _stream_mesh(landing, w.exit_speed(EP.flow), EP.t)
        kit.set_mesh("stream", *(mesh if mesh is not None else (None, None)), visible=mesh is not None)
        if landing.what == "water":
            size = 0.016 + 0.05 * EP.flow
            kit.set_mesh("foam", *_disc(landing.point[:2], size, float(landing.point[2]) + 0.0005, 0.25,
                                         EP.t * 9.0, thick=0.002))
        else:
            kit.set_mesh("foam", visible=False)
    else:
        kit.set_mesh("stream", visible=False)
        kit.set_mesh("foam", visible=False)
    if (len(EP.puddles.spots), sum(s[2] for s in EP.puddles.spots)) != spots:
        kit.set_mesh("puddles", *EP.puddles.mesh())
    if EP.tank.in_bucket > 0.002:
        kit.set_mesh("water_body", *_water_mesh(EP.tank.level, EP.t, EP.tank.flowing))
    step = round(EP.t / base.step_dt)
    if step % 10 == 0:
        _set_bucket_mass(env, BUCKET_KG + EP.tank.in_bucket)
    if _TRACE is not None and step % 2 == 0:
        import json

        _TRACE.write(json.dumps({
            "t": round(EP.t, 3), "trigger_deg": round(EP.trigger_deg, 2), "flow_lps": round(EP.flow, 4),
            "landing": EP.landing, "in_bucket_l": round(EP.tank.in_bucket, 4), "missed_l": round(EP.tank.missed, 4),
            "overflow_l": round(EP.tank.overflow, 4), "level_mm": round(EP.tank.level * 1000, 1),
            "tilt_deg": round(EP.max_tilt, 2), "score": score_of(EP),
            "nozzle": [round(v, 4) for v in noz.pos.tolist()], "bucket": [round(v, 4) for v in buck.pos.tolist()],
            "nozzle_x": [round(v, 3) for v in noz.rotation()[:, 0].tolist()],
            "nozzle_y": [round(v, 3) for v in noz.rotation()[:, 1].tolist()],
            "tcp": _tcps(env),
        }) + "\n")
        _TRACE.flush()


def _tcps(env):
    """Both grippers' TCP (world) and finger opening, for the trace."""
    from yam_arena.constants import TCP_OFFSET

    out = {}
    base = env.unwrapped
    for arm in ("left", "right"):
        robot = base.scene[arm + "_robot"]
        ids, _ = robot.find_bodies(["gripper"])
        pose = kit.as_numpy(robot.data.body_link_pose_w)[0, ids[0]]
        origin = kit.as_numpy(base.scene.env_origins[0])
        p = pose[:3] - origin + kit.rotation(pose[3:7]) @ np.array(TCP_OFFSET)
        fingers, _ = robot.find_joints(["joint7", "joint8"])
        q = kit.as_numpy(robot.data.joint_pos)[0, fingers]
        out[arm] = [round(v, 4) for v in p.tolist()] + [round(float(q.mean()), 4)]
    return out


def score(objects) -> float:
    return score_of(EP)


def success(objects) -> bool:
    return score_of(EP) >= 1.0


# -- the scripted demo ------------------------------------------------------------------------------------------
# Left hand (A): take the nozzle's barrel from above as it lies on its side, lift it, turn it handle-down and hold it
# with the dial just inside the pail's near rim, aimed down into it. Right hand (B): come in from the right and close
# round the handle and the trigger together, squeezing the trigger to its stop; open when the pail is full.

# The holding hand's grip in the nozzle's frame: approach along -y_N, fingers closing along z_N.
GRIP_A = np.column_stack([(0.0, 0.0, -1.0), (1.0, 0.0, 0.0), (0.0, -1.0, 0.0)])
# The squeezing hand's grip: approach along +y_N, fingers closing along x_N (trigger front to handle back).
GRIP_B = np.column_stack([(1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)])
SQUEEZE_AT = np.array([0.000, 0.0, -0.068])  # the squeezing hand's TCP, nozzle frame

AIM_PITCH_DEG = 45.0
AIM_TIP = np.array([0.400, -0.040 + float(os.environ.get("HOSE_AIM_DY", "0")),  # shortcut trials move the aim
                    BRIM_Z + 0.060])  # the dial's centre: over the near rim, 6 cm above the brim
FILL_S = float(os.environ.get("HOSE_FILL_S", "28.8"))  # squeeze time (open loop, from the flow rate)


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def hold_pose(pitch_deg=AIM_PITCH_DEG, tip=AIM_TIP):
    """The nozzle's (position, rotation) with its dial at ``tip`` (world, z above the pail's base) pitched down."""
    rot = _rot_y(math.radians(pitch_deg))  # x_N forward and down, y_N left, handle down and back
    tip = np.array([tip[0], tip[1], GROUND + tip[2]])
    return tip - rot @ np.array([OUTLET[0], 0.0, 0.0]), rot


def _tcp(pos, rot, local, grip):
    return tuple((pos + rot @ local).tolist()), rot @ grip


def demo():
    o, c = FINGER_OPEN, FINGER_CLOSED
    p0, r0 = NOZZLE_POS0, NOZZLE_ROT0
    grasp = np.array([GRASP_X, 0.0, 0.0])
    at, ra = _tcp(p0, r0, grasp, GRIP_A)
    up = np.array([0.0, 0.0, 1.0])
    ph, rh = hold_pose()
    hold, rah = _tcp(ph, rh, grasp, GRIP_A)
    lifted = tuple((np.array(at) + 0.12 * up).tolist())
    above_hold = tuple((np.array(hold) + np.array([-0.05, 0.0, 0.06])).tolist())
    left = [
        Waypoint(2.5, tuple((np.array(at) + 0.08 * up).tolist()), o, "over the nozzle", ra),
        Waypoint(1.5, at, o, "down round the barrel", ra),
        Waypoint(1.0, at, c, "close on the barrel", ra),
        Waypoint(0.5, at, c, "settle", ra),
        Waypoint(2.0, lifted, c, "lift the nozzle", ra),
        Waypoint(3.5, above_hold, c, "turn it handle-down over the pail", rah),
        Waypoint(1.5, hold, c, "aim into the pail", rah),
    ]
    t_aim = sum(x.duration for x in left)
    bp, br = _tcp(ph, rh, SQUEEZE_AT, GRIP_B)
    back = tuple((np.array(bp) - 0.07 * (rh @ np.array([0.0, 1.0, 0.0]))).tolist())
    right = [
        Waypoint(t_aim + 0.5, back, o, "wait beside the nozzle", br),
        Waypoint(2.0, bp, o, "fingers round handle and trigger", br),
        Waypoint(1.0, bp, c, "squeeze the trigger", br),
        Waypoint(FILL_S, bp, c, "hold: fill", br),
        Waypoint(0.8, bp, o, "let go of the trigger", br),
        Waypoint(1.5, back, o, "back off", br),
    ]
    t_end = sum(x.duration for x in right)
    left.append(Waypoint(t_end - t_aim + 2.0, hold, c, "hold the nozzle", rah))
    return {"left": left, "right": right}
