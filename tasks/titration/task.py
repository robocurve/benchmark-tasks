"""On the station table: a clear plastic burette of amber iodine solution clamped to a stand, with a silicone hose and a red spring pinch clip at its foot, over a white tile. On the tile stands a
plastic Erlenmeyer flask holding a milky solution of vitamin C and starch. Squeezing the clip's two red tabs runs iodine into the flask; letting go shuts it.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 --seed 1 --instruction "<INSTRUCTION below>"
The demo:  python demokit.py task.py OUT --seed 1
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import mechkit as m  # noqa: E402  (hooks yam_arena so that the parts below spawn from USD)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = (
    "Titrate the vitamin C solution in the flask with the iodine in the burette. Hold the burette's spring clip squeezed with one arm to run iodine into the flask, while the other arm swirls the flask under it. "
    "Release the clip as soon as the whole solution stays blue-black. Stand the flask back on the white tile and let go with both arms."
)
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

BX, BY = 0.32, 0.0  # the burette's axis
PHI_L, PHI_R = -80.0, 80.0  # the direction each arm's level grip points in the demo, from the side, so that neither hides the flask from the top camera; the clip's tabs squeeze across PHI_L

TILE = (0.14, 0.14, 0.005)
R_BASE, R_NECK, Z_SHOULDER, Z_RIM, MOUTH_R, LIQ_Z = 0.0425, 0.019, 0.10, 0.14, 0.0165, 0.05  # a 250 mL flask, from its underside
FLASK_MASS = 0.19  # 35 g of plastic and about 150 mL of solution
NECK_GRIP_Z = 0.12
TIP_Z, Z_CLIP, Z_HOSE_TOP, Z_TUBE_BOTTOM, Z_TUBE_TOP = 0.165, 0.205, 0.235, 0.245, 0.525  # above the table
R_TUBE, R_IN = 0.008, 0.0065
ML_PER_M = math.pi * (R_IN * 100.0) ** 2 * 100.0
CAPACITY_ML = 25.0
Z_ZERO = 0.505
Z_EMPTY = Z_ZERO - CAPACITY_ML / ML_PER_M

CLIP_REST, CLIP_TRAVEL, CLIP_SPRING, CLIP_DAMPING = 0.019, 0.006, 1200.0, 8.0  # each tab's outer face from the axis, and how far it moves: about 7 N a tab at full squeeze
TAB = (0.026, 0.004, 0.03)  # along the grip's axis, across, up
FLOW_FROM, FLOW_FULL, FLOW_MAX = 0.25, 0.85, 0.8  # squeeze fractions, and mL/s

E_RANGE = (8.0, 16.0)  # mL, by the seed
OVER_OK, OVER_MAX, SWIRL_MIN, SPILL_MAX = 1.0, 3.0, 0.8, 0.2
SWIRL_SPEED = 0.025  # m/s: the mouth, averaged over about half a second, moving at least this fast counts as swirling
SPEED_TAU = 0.5
K_STILL, K_SWIRL, K_SPEED = 0.01, 3.0, 0.05  # per second: how fast iodine that has landed mixes into the bulk, still and swirled at K_SPEED or more
EXCESS_PALE, EXCESS_BLUE, EXCESS_DARK = 0.0, 0.05, 0.15  # mL past the endpoint, mixed: the bulk's colour
PLUME_SMALL, PLUME_BIG, NEAR_END = 0.05, 1.0, 0.85  # mL unmixed; and the fraction of the endpoint from which even a little local colour lingers and spreads
CLEAR = 0.03
TILT_OVER, TILT_BACK, TILE_XY = 45.0, 10.0, 0.03


LOOKS = {
    "plastic": m.Look(glass=(0.94, 0.97, 0.98)),
    "iodine": m.Look(color=m.rgb(0.60, 0.30, 0.05), roughness=0.2),
    "milky": m.Look(color=m.rgb(0.93, 0.93, 0.90), roughness=0.35, emissive=(0.25, 0.25, 0.235)),
    "pale": m.Look(color=m.rgb(0.55, 0.62, 0.80), roughness=0.3, emissive=(0.06, 0.08, 0.14)),
    "blue": m.Look(color=m.rgb(0.16, 0.22, 0.52), roughness=0.3, emissive=(0.01, 0.015, 0.05)),
    "dark": m.Look(color=m.rgb(0.03, 0.04, 0.16), roughness=0.3),
    "black": m.Look(color=m.rgb(0.05, 0.05, 0.06), roughness=0.5),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.8, roughness=0.45),
    "tile": m.Look(color=m.rgb(0.95, 0.95, 0.94), roughness=0.25),
    "rim": m.Look(color=m.rgb(0.72, 0.80, 0.86), roughness=0.3),
    "clip": m.Look(color=m.rgb(0.80, 0.08, 0.06), roughness=0.5),
    "hose": m.Look(color=m.rgb(0.80, 0.78, 0.70), roughness=0.5),
    "tick": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.6),
}
LIQUIDS = ("liquid_milky", "liquid_pale", "liquid_blue", "liquid_dark")
PLUMES = ("plume_small", "plume_big")


def flask_radius(z: float) -> float:
    """The flask's outer radius at this height above its underside."""
    if z >= Z_SHOULDER:
        return R_NECK
    return R_BASE + (R_NECK - R_BASE) * max(z - 0.004, 0.0) / (Z_SHOULDER - 0.004)


def _flask_parts():
    wall = m.lathe([(0.0, 0.0), (R_BASE - 0.003, 0.0), (R_BASE, 0.004), (R_NECK, Z_SHOULDER), (R_NECK, Z_RIM - 0.003), (R_NECK + 0.002, Z_RIM - 0.002), (R_NECK + 0.002, Z_RIM), (MOUTH_R, Z_RIM), (MOUTH_R, Z_SHOULDER)], 48)
    inset = 0.0012
    body = m.lathe([(0.0, inset), (flask_radius(inset) - 2 * inset, inset), (flask_radius(LIQ_Z) - inset, LIQ_Z), (0.0, LIQ_Z)], 40)
    rim = m.lathe([(R_NECK + 0.0004, Z_RIM - 0.006), (R_NECK + 0.0022, Z_RIM - 0.006), (R_NECK + 0.0022, Z_RIM + 0.0002), (MOUTH_R, Z_RIM + 0.0002)], 40)
    foot = m.lathe([(R_BASE + 0.0002, 0.0005), (R_BASE + 0.0006, 0.004), (R_BASE - 0.0012, 0.0085)], 40)
    parts = [("wall", wall, "plastic"), ("rim", rim, "rim"), ("foot", foot, "rim")]
    for name, look in zip(LIQUIDS, ("milky", "pale", "blue", "dark")):
        parts.append((name, body, look, name != "liquid_milky"))

    def lens(r: float):
        return m.lathe([(0.0, LIQ_Z - 0.4 * r), (0.7 * r, LIQ_Z - 0.25 * r), (r, LIQ_Z - 0.0005), (0.0, LIQ_Z + 0.0006)], 24)

    parts.append(("plume_small", lens(0.011), "dark", True))
    parts.append(("plume_big", lens(0.024), "dark", True))
    return parts


def _flask_colliders():
    body = m.lathe([(0.0, 0.0), (R_BASE - 0.003, 0.0), (R_BASE, 0.004), (R_NECK, Z_SHOULDER), (0.0, Z_SHOULDER)], 24)
    return [m.col_hull("c_body", body), m.col_cylinder("c_neck", R_NECK, Z_RIM - Z_SHOULDER, (0.0, 0.0, (Z_RIM + Z_SHOULDER) / 2))]


def _column_mesh():
    """The iodine above the 25 mL mark, in its local frame: the task scales it about Z_EMPTY as the level falls."""
    return m.cylinder(R_IN - 0.0004, Z_ZERO - Z_EMPTY, (BX, BY, (Z_ZERO + Z_EMPTY) / 2), segments=16)


def _stand_parts():
    rod_x = BX + 0.135
    ticks = []
    for ml in range(0, int(CAPACITY_ML) + 1):
        z = Z_ZERO - ml / ML_PER_M
        major = ml % 5 == 0
        ticks.append(m.box((0.0008, 0.012 if major else 0.006, 0.0014 if major else 0.0009), (BX - R_TUBE - 0.0002, BY, z)))
    tube = m.lathe([(0.0, Z_TUBE_BOTTOM - 0.008), (0.003, Z_TUBE_BOTTOM - 0.008), (R_TUBE, Z_TUBE_BOTTOM), (R_TUBE, Z_TUBE_TOP), (R_IN, Z_TUBE_TOP), (R_IN, Z_TUBE_TOP - 0.01)], 24).moved((BX, BY, 0.0))
    fill = m.lathe([(0.0, TIP_Z + 0.001), (0.0012, TIP_Z + 0.001), (0.0022, Z_TUBE_BOTTOM - 0.008), (R_IN - 0.0004, Z_TUBE_BOTTOM), (R_IN - 0.0004, Z_EMPTY), (0.0, Z_EMPTY)], 16).moved((BX, BY, 0.0))
    hose = m.cylinder(0.0045, Z_HOSE_TOP - (TIP_Z + 0.012), (BX, BY, (Z_HOSE_TOP + TIP_Z + 0.012) / 2), segments=16)
    tip = m.lathe([(0.0, TIP_Z), (0.002, TIP_Z), (0.0035, TIP_Z + 0.015), (0.0, TIP_Z + 0.015)], 14).moved((BX, BY, 0.0))
    return [
        ("base", m.box((0.12, 0.14, 0.012), (rod_x, BY, 0.006)), "black"),
        ("rod", m.cylinder(0.006, 0.56, (rod_x, BY, 0.28), segments=16), "steel"),
        ("arm", m.box((0.135, 0.012, 0.012), (BX + 0.0675, BY, 0.44)), "steel"),
        ("jaws", m.box((0.024, 0.028, 0.022), (BX, BY, 0.44)), "black"),
        ("tube", tube, "plastic"),
        ("fill", fill, "iodine"),
        ("column", _column_mesh(), "iodine"),
        ("ticks", m.join(*ticks), "tick"),
        ("hose", hose, "hose"),
        ("tip", tip, "plastic"),
        ("stream", m.cylinder(0.0013, TIP_Z - LIQ_Z, (BX, BY, (TIP_Z + LIQ_Z) / 2), segments=10), "iodine", True),
    ]


def _stand_colliders():
    rod_x = BX + 0.135
    return [
        m.col_box("c_base", (0.12, 0.14, 0.012), (rod_x, BY, 0.006)),
        m.col_cylinder("c_rod", 0.006, 0.56, (rod_x, BY, 0.28)),
        m.col_box("c_arm", (0.135, 0.012, 0.012), (BX + 0.0675, BY, 0.44)),
        m.col_box("c_jaws", (0.024, 0.028, 0.022), (BX, BY, 0.44)),
        m.col_cylinder("c_tube", R_TUBE, Z_TUBE_TOP - Z_TUBE_BOTTOM, (BX, BY, (Z_TUBE_TOP + Z_TUBE_BOTTOM) / 2)),
        m.col_box("c_hose", (0.009, 0.009, Z_TUBE_BOTTOM - TIP_Z), (BX, BY, (Z_TUBE_BOTTOM + TIP_Z) / 2)),
    ]


def _clip_part():
    y = CLIP_REST - TAB[1] / 2

    def tab(sign: float, name: str):
        plate = m.box(TAB, (0.0, sign * y, 0.0))
        wire = m.tube([(0.0, sign * y, 0.0), (0.0, sign * 0.009, 0.0), (0.0, sign * 0.006, 0.0)], 0.0016, sides=8)
        return [(f"{name}_tab", plate, "clip"), (f"{name}_wire", wire, "steel")], [m.col_box(f"{name}_c", TAB, (0.0, sign * y, 0.0))]

    parts_a, col_a = tab(1.0, "a")
    parts_b, col_b = tab(-1.0, "b")
    kw = dict(spring=CLIP_SPRING, damping=CLIP_DAMPING, friction=0.0, mass=0.01, armature=1e-4)
    links = [m.slider("jaw_a", "Y", -CLIP_TRAVEL, 0.0, parts_a, col_a, **kw), m.slider("jaw_b", "Y", 0.0, CLIP_TRAVEL, parts_b, col_b, **kw)]
    return m.mechanism("clip", (BX, BY, TOP + Z_CLIP), links, LOOKS, frame_parts=[("collar", m.cylinder(0.0065, 0.012, (0.0, 0.0, 0.0), segments=16), "steel")], yaw_deg=PHI_L)


OBJECTS = [
    m.fixed("tile", (BX, BY, TOP), [("tile", m.box(TILE, (0.0, 0.0, TILE[2] / 2)), "tile")], [m.col_box("c", TILE, (0.0, 0.0, TILE[2] / 2))], LOOKS),
    m.fixed("stand", (0.0, 0.0, TOP), _stand_parts(), _stand_colliders(), LOOKS),
    _clip_part(),
    m.free("flask", (BX, BY, TOP + TILE[2] + 0.0005), _flask_parts(), _flask_colliders(), LOOKS, mass=FLASK_MASS),
]
FLASK_START_Z = TOP + TILE[2] + 0.0005


def endpoint(seed: int | None) -> float:
    return round(float(np.random.default_rng(0 if seed is None else seed).uniform(*E_RANGE)), 1)


def flow_from(squeeze: float) -> float:
    """mL/s through the hose at this fraction of the clip's travel."""
    return FLOW_MAX * float(np.clip((squeeze - FLOW_FROM) / (FLOW_FULL - FLOW_FROM), 0.0, 1.0))


def mix_step(unmixed: float, mixed: float, speed: float, dt: float) -> tuple[float, float]:
    """Iodine that has landed moves into the bulk, fast only while the flask's mouth moves at `speed` m/s."""
    k = K_STILL + (K_SWIRL - K_STILL) * min(1.0, speed / K_SPEED)
    moved = unmixed * (1.0 - math.exp(-k * dt))
    return unmixed - moved, mixed + moved


def end_state_ok(off_tile: float, z_off: float, tilt: float, to_flask: float, to_clip: float, resting: bool, flow: float) -> bool:
    """The flask back on its tile, upright and still, the clip shut, and every finger clear of both."""
    return bool(off_tile <= TILE_XY and abs(z_off) <= 0.004 and tilt <= TILT_BACK and to_flask >= CLEAR and to_clip >= CLEAR and resting and flow == 0.0)


def task_score(mixed: bool, v: float, e: float, swirled: float, spilled: float, knocked: bool, put_back: bool) -> float:
    """The scoring text, line by line. `v` the mL delivered, `e` the endpoint, `swirled` the fraction delivered while swirling, `mixed` whether the bulk reached the endpoint, `put_back` the end state held 1 s."""
    if knocked or spilled > SPILL_MAX or not mixed or v > e + OVER_MAX:
        return 0.0
    chemistry = 1.0 if v <= e + OVER_OK and swirled >= SWIRL_MIN else 0.5
    return chemistry if put_back else chemistry / 2.0


class Trial:
    """The trial's chemistry and latches. Liquid is drawn, not simulated: what lands, what has mixed, and the colour they give follow from the clip's joints and the flask's motion."""

    def __init__(self) -> None:
        self.seed: int | None = None
        self.reset()

    def reset(self, env=None) -> None:
        self.steps = 0
        self.e = endpoint(self.seed)
        self.v = self.unmixed = self.mixed = self.v_swirled = self.spilled = 0.0
        self.speed = 0.0
        self.last_mouth = None
        self.knocked = False
        self.held = 0
        self.ready = False
        self.score = 0.0
        self.shown_v = -1.0
        self.first_colour_t = None
        self.max_off_axis = 0.0
        self.first_spill_t = None
        self.report: dict = {}
        if env is not None:
            self._clear_walls_cast_no_shadow()
            self._show_liquid("liquid_milky", None, False)
            self._show_level()

    def _clear_walls_cast_no_shadow(self) -> None:
        """The renderer's direct light treats a clear wall as opaque, which would leave the liquid inside it in shadow."""
        import isaaclab.sim as sim_utils
        from pxr import Sdf, UsdGeom

        stage = sim_utils.get_current_stage()
        for path in ("flask/visual/wall", "stand/visual/tube"):
            UsdGeom.PrimvarsAPI(stage.GetPrimAtPath(f"{m.ENV_PATH}/{path}")).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)

    def _show_liquid(self, liquid: str, plume: str | None, stream: bool) -> None:
        for name in LIQUIDS:
            m.set_visible(f"{m.ENV_PATH}/flask/visual/{name}", name == liquid)
        for name in PLUMES:
            m.set_visible(f"{m.ENV_PATH}/flask/visual/{name}", name == plume)
        m.set_visible(f"{m.ENV_PATH}/stand/visual/stream", stream)

    def _show_level(self) -> None:
        """Lower the burette's column to the volume delivered, by scaling it about the 25 mL mark."""
        if abs(self.v - self.shown_v) < 0.02:
            return
        self.shown_v = self.v
        import isaaclab.sim as sim_utils
        from pxr import Gf, UsdGeom

        prim = sim_utils.get_current_stage().GetPrimAtPath(f"{m.ENV_PATH}/stand/visual/column")
        if not prim.IsValid():
            raise RuntimeError("no burette column to draw")
        xf = UsdGeom.Xformable(prim)
        ops = xf.GetOrderedXformOps()
        if not ops:
            ops = [xf.AddTranslateOp(), xf.AddScaleOp()]
        s = max((CAPACITY_ML - self.v) / CAPACITY_ML, 1e-3)
        ops[0].Set(Gf.Vec3d(0.0, 0.0, Z_EMPTY * (1.0 - s)))
        ops[1].Set(Gf.Vec3f(1.0, 1.0, float(s)))

    def _gaps(self, env, flask, clip) -> tuple[float, float]:
        """The least distance from any fingertip point to the flask and to the clip's tabs."""
        pts = m.fingertips(env)
        local = flask.to_local(pts)
        radial = np.hypot(local[:, 0], local[:, 1]) - np.array([flask_radius(z) for z in local[:, 2]])
        along = np.maximum(np.maximum(-local[:, 2], local[:, 2] - Z_RIM), 0.0)
        to_flask = float(np.hypot(np.maximum(radial, 0.0), along).min())
        c = clip.to_local(pts)
        half = np.array([TAB[0] / 2, CLIP_REST, TAB[2] / 2])
        to_clip = float(np.linalg.norm(np.maximum(np.abs(c) - half, 0.0), axis=1).min())
        return to_flask, to_clip

    def step(self, env) -> None:
        dt = float(env.unwrapped.step_dt)
        self.steps += 1
        t = self.steps * dt
        flask, clip = m.state(env, "flask"), m.state(env, "clip")
        squeeze = float(np.clip((-clip.joint("jaw_a") + clip.joint("jaw_b")) / (2.0 * CLIP_TRAVEL), 0.0, 1.0))
        flow = flow_from(squeeze)
        mouth = flask.to_world(np.array([[0.0, 0.0, Z_RIM]]))[0]
        if self.last_mouth is None:
            self.last_mouth = mouth
        self.speed += (float(np.linalg.norm((mouth - self.last_mouth)[:2])) / dt - self.speed) * min(1.0, dt / SPEED_TAU)
        self.last_mouth = mouth
        swirling = self.speed >= SWIRL_SPEED
        dv = min(flow * dt, CAPACITY_ML - self.v)
        under = bool(np.hypot(mouth[0] - BX, mouth[1] - BY) <= MOUTH_R and mouth[2] <= TOP + TIP_Z)
        if dv > 0.0:
            self.v += dv
            if under:
                self.unmixed += dv
                if swirling:
                    self.v_swirled += dv
            else:
                self.spilled += dv
                if self.first_spill_t is None:
                    self.first_spill_t = round(t, 2)
            self.max_off_axis = max(self.max_off_axis, float(np.hypot(mouth[0] - BX, mouth[1] - BY)))
        self.unmixed, self.mixed = mix_step(self.unmixed, self.mixed, self.speed, dt)
        excess = self.mixed - self.e
        if excess > EXCESS_PALE and self.first_colour_t is None:
            self.first_colour_t = round(t, 2)
            print(f"[titration] the bulk turned at {t:.2f} s, {self.v:.2f} mL delivered, endpoint {self.e} mL", flush=True)
            self._write()
        liquid = "liquid_milky" if excess <= EXCESS_PALE else "liquid_pale" if excess < EXCESS_BLUE else "liquid_blue" if excess < EXCESS_DARK else "liquid_dark"
        near = self.mixed >= NEAR_END * self.e
        plume = None
        if excess <= EXCESS_PALE and self.unmixed >= PLUME_SMALL:
            plume = "plume_big" if near or self.unmixed >= PLUME_BIG else "plume_small"
        self._show_liquid(liquid, plume, flow > 0.0)
        self._show_level()
        tilt = flask.tilt_deg()
        if tilt > TILT_OVER or flask.pos[2] < TOP - 0.02:
            self.knocked = True
        to_flask, to_clip = self._gaps(env, flask, clip)
        off_tile, z_off = float(np.hypot(flask.pos[0] - BX, flask.pos[1] - BY)), float(flask.pos[2] - FLASK_START_Z)
        on_tile = bool(off_tile <= TILE_XY and abs(z_off) <= 0.004 and tilt <= TILT_BACK)
        self.ready = end_state_ok(off_tile, z_off, tilt, to_flask, to_clip, flask.resting(), flow)
        self.held = self.held + 1 if self.ready else 0
        put_back = self.held * dt >= SUCCESS_HOLD_S
        delivered = self.v - self.spilled
        swirled = self.v_swirled / delivered if delivered > 0.0 else 0.0
        mixed = self.mixed >= self.e
        self.score = task_score(mixed, self.v, self.e, swirled, self.spilled, self.knocked, put_back)
        self.full = bool(self.ready and task_score(mixed, self.v, self.e, swirled, self.spilled, self.knocked, True) == 1.0)
        self.report = {"t": round(t, 2), "seed": self.seed, "endpoint_ml": self.e, "delivered_ml": round(self.v, 3), "over_ml": round(self.v - self.e, 3), "mixed_ml": round(self.mixed, 3), "unmixed_ml": round(self.unmixed, 3), "flow_ml_s": round(flow, 3),
                       "squeeze": round(squeeze, 3), "speed_m_s": round(self.speed, 4), "swirled_fraction": round(swirled, 3), "spilled_ml": round(self.spilled, 3), "first_spill_t": self.first_spill_t, "max_off_axis_while_dosing_m": round(self.max_off_axis, 4), "knocked": self.knocked, "first_colour_t": self.first_colour_t, "colour": liquid, "plume": plume,
                       "on_tile": on_tile, "gap_flask_m": round(to_flask, 3), "gap_clip_m": round(to_clip, 3), "tilt_deg": round(tilt, 2), "flask": [round(float(v), 4) for v in flask.pos], "mouth_off_axis_m": round(float(np.hypot(mouth[0] - BX, mouth[1] - BY)), 4), "ready": self.ready, "score": self.score, "full_now": self.full}
        if self.steps % 20 == 0:
            self._write()

    def _write(self) -> None:
        out = os.environ.get("TITRATION_SCORE_DIR", "titration_scores")
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, f"{os.environ.get('TASK_DEMO', 'run')}_seed{self.seed}.json"), "w") as f:
            json.dump(self.report, f, indent=0)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def _read_seed(module) -> None:
    """The embodiment's reset hands its seed to nothing a task file can read, so note it here. Only this file's endpoint uses it."""
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_titration_seed", False):
        return
    plain = cls.reset

    def reset(self, scene, *, seed=None):
        TRIAL.seed = seed
        return plain(self, scene, seed=seed)

    cls.reset = reset
    cls._titration_seed = True


if "yam_arena.inspect_embodiment" in sys.modules:
    _read_seed(sys.modules["yam_arena.inspect_embodiment"])


def success(objects) -> bool:
    """A 1.0 titration, with the flask back on its tile, every finger clear and all still."""
    return TRIAL.full


def score(objects) -> float:
    """See `task_score`."""
    return TRIAL.score


# -- the demo ------------------------------------------------------------------------------

DARK_BELOW = 0.6  # a pixel patch on the flask's side this far below what it was at the start means the bulk has turned
RELEASED = -(CLIP_REST + 0.004)
SQUEEZED_W = 2.0 * (CLIP_REST - CLIP_TRAVEL)
NECK_W = 2.0 * R_NECK
LIFT = 0.008
SWIRL_R, SWIRL_HZ = 0.008, 1.2
BACK = 0.12


def _pixel(world, width: int, height: int) -> tuple[int, int]:
    """Where a world point falls in the top camera's image, from the station's public calibration."""
    from yam_arena import station
    from yam_arena.urdf import quat_xyzw_to_matrix

    cam = station.top_camera()
    c = quat_xyzw_to_matrix(np.array(cam.rot_xyzw)).T @ (np.asarray(world, float) - np.asarray(cam.pos))
    lens = cam.lens
    return int(round((lens.cx + lens.fx * c[0] / -c[2]) * width / lens.width)), int(round((lens.cy - lens.fy * c[1] / -c[2]) * height / lens.height))


def _patch(ctx, world) -> float:
    """The mean brightness of a small patch of the top camera's latest image round a world point."""
    image = ctx.obs.images["top_cam"]
    h, w = image.shape[:2]
    u, v = _pixel(world, w, h)
    r = w // 320
    return float(image[max(v - r, 0):v + r + 1, max(u - r, 0):u + r + 1].mean())


def demo_script(ctx):
    """Closed loop. The arms follow the flask's pose and their own joints, and the stop comes only from the top camera's pixels on the flask's near side, where a local colour at the mouth does not show.
    `TASK_DEMO`: full; none; no_swirl (hold the clip with the flask still, to 20 mL); first_flash (stop on the first colour at the middle of the flask); fixed_mark (stop at 12 mL by the clock); late and very_late (let go 2.2 s and 5 s after the colour);
    dose_then_swirl (the flask still while dosing, then swirled to check); one_arm (the left arm alone alternates clip and swirl); spill (carry the flask aside while dosing); knock (push the flask over);
    still_held (the right arm never lets go); fingers_left (the left arm stays round the clip); off_tile (stand the flask beside the tile)."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    up = np.array([0.0, 0.0, 1.0])
    g_l, g_r = d.level(PHI_L), d.level(PHI_R)
    ax_l, ax_r = l.approach_axis(g_l), r.approach_axis(g_r)
    clip_at = np.array([BX, BY, TOP + Z_CLIP])
    tip_xy = np.array([BX, BY])
    near_side = (BX - 0.03, BY, TOP + 0.02)
    middle = (BX, BY, TOP + LIQ_Z)
    state = {"stop": False, "base": None}

    def neck():
        return ctx.pos("flask") + NECK_GRIP_Z * up

    def dark() -> bool:
        return _patch(ctx, near_side) < DARK_BELOW * state["base"]

    def take_flask(arm, grip, axis):
        yield from arm.move(neck() - BACK * axis, grip=grip, finger=FINGER_OPEN, t=3.0, label="to the flask")
        yield from arm.servo(neck() - BACK * axis, timeout=1.0, label="to the flask")
        yield from arm.move(neck(), t=1.5, label="round the neck")
        yield from arm.servo(neck(), timeout=1.0, label="round the neck")
        yield from arm.close(NECK_W, 15.0, t=0.4, label="hold the neck")
        goal = arm.at() + LIFT * up
        yield from arm.move(goal, t=0.5, label="lift")
        yield from arm.servo(goal, timeout=1.0, label="lift")

    def to_clip():
        yield from l.move(clip_at - BACK * ax_l, grip=g_l, finger=FINGER_OPEN, t=3.0, label="to the clip")
        yield from l.servo(clip_at - BACK * ax_l, timeout=1.0, label="to the clip")
        yield from l.move(clip_at, t=1.5, label="round the clip")
        yield from l.servo(clip_at, timeout=1.0, label="round the clip")

    def swirl(arm, seconds: float = 85.0, until=None):
        """Circle the held flask's mouth under the tip, re-centring the circle on where the flask is seen to be."""
        centre = arm.at().copy()
        mean = ctx.pos("flask")[:2].copy()
        clock = {"t": 0.0}

        def target():
            nonlocal mean
            clock["t"] += d.DT
            a = SWIRL_R * min(1.0, clock["t"] / 0.8)
            mean = mean + (ctx.pos("flask")[:2] - mean) * 0.05
            centre[:2] += 0.02 * (tip_xy - mean)
            w = 2.0 * math.pi * SWIRL_HZ * clock["t"]
            return centre + a * np.array([math.cos(w), math.sin(w), 0.0])

        yield from arm.follow(target, t=seconds, speed=0.25, until=until, label="swirl")
        yield from arm.move(centre, t=0.5, label="swirl: stop")

    def put_down(arm, axis, xy=None):
        over = ctx.pos("flask")[2] - FLASK_START_Z - 0.001
        goal = arm.at() - over * up
        if xy is not None:
            goal[:2] = goal[:2] + (np.asarray(xy) - ctx.pos("flask")[:2])
        yield from arm.move(goal, t=1.2, label="down on the tile")
        yield from arm.open(t=0.4, label="let the flask go")
        yield from arm.move(arm.at() - BACK * axis, t=1.2, label="back")

    def squeeze(arm):
        yield from arm.close(SQUEEZED_W, 18.0, t=0.3, label="squeeze")

    def release(arm):
        yield from arm.move(arm.at(), finger=RELEASED, t=0.1, label="release")

    def let_go_clip():
        yield from l.open(t=0.3, label="off the clip")
        yield from l.move(l.at() - BACK * ax_l, t=1.2, label="back")

    if mode == "none":
        yield from l.wait(5.0)
        return
    if mode == "knock":
        yield from r.move(ctx.pos("flask") + np.array([-0.12, -0.02, 0.11]), grip=d.level(0.0), finger=0.0, t=3.0, label="behind the flask")
        yield from r.move(ctx.pos("flask") + np.array([0.04, -0.02, 0.11]), t=1.0, label="push it over")
        yield from r.wait(3.0)
        return
    if mode == "one_arm":
        yield from to_clip()
        state["base"] = _patch(ctx, near_side)
        for _ in range(8):
            yield from l.servo(clip_at, timeout=0.5, label="at the clip")
            yield from squeeze(l)
            yield from l.wait(4.0)
            yield from release(l)
            yield from l.open(t=0.3, label="off the clip")
            yield from l.move(l.at() - BACK * ax_l, t=0.8, label="back")
            yield from take_flask(l, g_l, ax_l)
            yield from swirl(l, seconds=2.5)
            yield from put_down(l, ax_l)
            if dark():
                break
            yield from l.move(clip_at - BACK * ax_l, t=0.8, label="to the clip")
            yield from l.move(clip_at, t=0.8, label="round the clip")
        yield from l.wait(0.5)
        return

    yield from ctx.together(take_flask(r, g_r, ax_r), to_clip())
    for _ in range(10):
        yield
    state["base"] = _patch(ctx, near_side)
    print(f"[titration] the flask's side reads {state['base']:.0f} at the start", flush=True)

    if mode in ("no_swirl", "dose_then_swirl"):
        if mode == "no_swirl":
            yield from squeeze(l)
            yield from l.wait(25.0)
            yield from release(l)
        else:
            while not dark():
                yield from squeeze(l)
                yield from l.wait(3.0)
                yield from release(l)
                yield from swirl(r, seconds=2.5)
                for _ in range(6):
                    yield
    else:

        def dose():
            for _ in range(24):
                yield
            state["base"] = _patch(ctx, near_side)
            yield from squeeze(l)
            start = ctx.t
            if mode == "first_flash":
                mid = _patch(ctx, middle)
                while _patch(ctx, middle) > 0.85 * mid and ctx.t - start < 2.0:
                    yield
            elif mode == "fixed_mark":
                while ctx.t - start < 12.0 / FLOW_MAX:
                    yield
            elif mode == "spill":
                while ctx.t - start < 8.0:
                    yield
            else:
                while not dark() and ctx.t - start < 40.0:
                    yield
                print(f"[titration] the camera saw it after {ctx.t - start:.2f} s of squeezing: {_patch(ctx, near_side):.0f}", flush=True)
                if mode == "late":
                    yield from l.wait(2.2)
                elif mode == "very_late":
                    yield from l.wait(5.0)
            yield from release(l)
            if mode == "spill":
                return
            yield from l.wait(1.5)
            state["stop"] = True

        if mode == "spill":

            def carry():
                yield from r.wait(1.0)
                yield from r.move(r.at() + np.array([-0.05, -0.05, 0.0]), t=1.0, label="aside")
                yield from r.wait(3.0)
                yield from r.move(r.at() + np.array([0.05, 0.05, 0.0]), t=1.0, label="back under")

            yield from ctx.together(carry(), dose())
        else:
            yield from ctx.together(swirl(r, until=lambda: state["stop"]), dose())

    if mode == "off_tile":
        yield from ctx.together(put_down(r, ax_r, xy=(BX - 0.13, BY - 0.06)), let_go_clip())
    elif mode == "still_held":
        yield from ctx.together(r.move(r.at() - (LIFT - 0.001) * up, t=1.2, label="down on the tile"), let_go_clip())
        yield from r.wait(3.0)
    elif mode == "fingers_left":
        yield from put_down(r, ax_r)
        yield from l.wait(3.0)
    else:
        yield from ctx.together(put_down(r, ax_r), let_go_clip())
