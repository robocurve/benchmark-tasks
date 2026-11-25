"""Laundry toss: throw three rolled pairs of socks into an open pop-up mesh laundry hamper that stands on the floor beyond the table,
out of the arms' reach.

On the station table lie three rolled pairs of crew socks (each pair rolled into its own cuff, about 8 x 6 x 5.5 cm, 40 g). On the
floor beside the table's far right corner, 1.15 m from the right arm's base (100 cm forward, 56.5 cm to its right), stands a Household Essentials round pop-up mesh
hamper (16 in / 40.6 cm across, 25 in / 63.5 cm tall, 0.40 kg, open top). The arms reach about 0.6 m: the only way in is a throw.
Distractors on the table: a folded microfibre towel, a notebook and a mug.

Model (declared in code):
- Socks ("sock_a", "sock_b", "sock_c", free bodies): a rolled pair is a firm, lumpy ball; it is modelled as one rigid flattened
  ellipsoid (8.0 x 6.0 x 5.5 cm, one convex collider), 40 g, cotton-like friction 0.9/0.8 and no bounce (restitution 0), so it lands
  dead as a sock ball does. It lies flat and does not roll away.
- Hamper ("hamper", a free body, 0.40 kg): a 2 mm polyester mesh wall on two steel hoops, drawn see-through. Colliders: a 2.5 cm floor disc and
  24 wall panels, 3 cm thick outside the inner face (so a fast sock cannot tunnel through), 63.5 cm tall; rigid (the real mesh gives a little).

Scoring (code, every control step, from the sock and hamper poses):
- A pair is in when its centre is inside the hamper's wall (under 18.3 cm from the hamper's axis, i.e. the 20.3 cm inner radius less
  2 cm), at least 3 cm below the rim (lower than 60.5 cm above the hamper's floor) and above its floor, and it is still (under 5 cm/s).
- The hamper must stand where it was put: upright (axis within 10 degrees of vertical) with its base centre within 5 cm of its marks.
  If not, the score is 0.
- Score: the number of pairs in, divided by 3 (0, 0.33 or 0.67; three in before the hold completes also scores 0.67). Full credit
  (1.0) when all three are in and the hamper stands for 1 s (20 steps) with every gripper fingertip 30 cm or more from the hamper
  (always true here: the arms cannot reach it). Possible scores: 0, 0.33, 0.67, 1.0. Any number of throws within the run limit; a sock that falls off the table is lost.
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
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Throw the three pairs of socks into the laundry hamper."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
DT = 0.05
TOP = TABLE_TOP[SCENE]
FLOOR = -0.0125  # the station floor's top (station.FLOOR_Z)

# -- the hamper: Household Essentials round pop-up mesh hamper, 16 in (40.6 cm) diameter x 25 in (63.5 cm), 0.88 lb (0.40 kg) -----------
HAMPER_XY = (1.00, -0.87)
HAMPER_R = 0.203  # outer radius
HAMPER_H = 0.635
WALL_T = 0.004  # the drawn wall
WALL_COL_T = 0.030  # wall colliders: 3 cm thick, all outside the wall's inner face, so a fast sock cannot pass through in one physics step
FLOOR_COL_T = 0.025  # the floor collider, 2.5 cm thick (the drawn floor is its top)
HAMPER_MASS = 0.40
N_PANELS = 24
IN_R = HAMPER_R - WALL_T - 0.016  # a sock's centre counts as in within this radius
IN_Z = HAMPER_H - 0.030  # and below this height above the hamper's floor

# -- a rolled pair of crew socks ---------------------------------------------------------------------------------------------------------
SOCK = (0.080, 0.060, 0.055)  # x (along the roll), y, z
SOCK_MASS = 0.040
SOCKS = {"sock_a": (0.22, -0.20, 20.0), "sock_b": (0.30, -0.33, -10.0), "sock_c": (0.20, -0.44, 75.0)}  # x, y, yaw deg

LOOKS = {
    "mesh": m.Look(color=m.rgb(0.30, 0.62, 0.36), roughness=0.9, opacity=0.75),
    "hoop": m.Look(color=m.rgb(0.20, 0.42, 0.24), roughness=0.6),
    "base": m.Look(color=m.rgb(0.25, 0.52, 0.30), roughness=0.9),
    "sock_white": m.Look(color=m.rgb(0.90, 0.90, 0.88), roughness=0.95),
    "sock_grey": m.Look(color=m.rgb(0.55, 0.56, 0.58), roughness=0.95),
    "sock_black": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.95),
    "cuff_white": m.Look(color=m.rgb(0.97, 0.97, 0.95), roughness=0.95),
    "cuff_dark": m.Look(color=m.rgb(0.30, 0.30, 0.32), roughness=0.95),
}


def _ellipsoid(size) -> m.Mesh:
    s = m.sphere(0.5, rings=12, segments=28)
    return m.Mesh(s.v * np.asarray(size, float), s.f)


def _sock(name: str, x: float, y: float, yaw: float, look: str, cuff: str):
    body = _ellipsoid(SOCK)
    band = m.Mesh(_ellipsoid((SOCK[0] * 0.42, SOCK[1] * 1.02, SOCK[2] * 1.02)).v + np.array([0.012, 0.0, 0.0]), _ellipsoid(SOCK).f)
    return m.free(name, (x, y, TOP + SOCK[2] / 2), [("roll", body, look), ("cuff", band, cuff)], [m.col_hull("roll_c", _ellipsoid(SOCK))],
                  LOOKS, SOCK_MASS, friction=(0.9, 0.8), yaw_deg=yaw)


def _hamper():
    parts, cols = [], []
    r_in = HAMPER_R - 0.002
    wall = m.lathe([(r_in, 0.0), (HAMPER_R, 0.0), (HAMPER_R, HAMPER_H), (r_in, HAMPER_H), (r_in, 0.0)], 48)
    parts.append(("wall", wall, "mesh"))
    parts.append(("top_hoop", m.lathe([(HAMPER_R - 0.006, HAMPER_H - 0.012), (HAMPER_R + 0.004, HAMPER_H - 0.012), (HAMPER_R + 0.004, HAMPER_H + 0.002),
                                       (HAMPER_R - 0.006, HAMPER_H + 0.002), (HAMPER_R - 0.006, HAMPER_H - 0.012)], 48), "hoop"))
    parts.append(("bottom_hoop", m.lathe([(HAMPER_R - 0.006, 0.0), (HAMPER_R + 0.004, 0.0), (HAMPER_R + 0.004, 0.015), (HAMPER_R - 0.006, 0.015),
                                          (HAMPER_R - 0.006, 0.0)], 48), "hoop"))
    parts.append(("floor", m.cylinder(HAMPER_R - 0.003, FLOOR_COL_T, (0.0, 0.0, FLOOR_COL_T / 2), segments=48), "base"))
    cols.append(m.col_cylinder("floor_c", HAMPER_R - 0.004, FLOOR_COL_T, (0.0, 0.0, FLOOR_COL_T / 2)))
    r_face = HAMPER_R - WALL_T  # the inner face
    chord = 2 * (r_face + WALL_COL_T) * math.tan(math.pi / N_PANELS) + 0.004
    for k in range(N_PANELS):
        a = 360.0 * k / N_PANELS
        panel = m.box((WALL_COL_T, chord, HAMPER_H), (r_face + WALL_COL_T / 2, 0.0, HAMPER_H / 2)).turned(a, "z")
        cols.append(m.col_hull(f"wall_{k:02d}_c", panel))
    return m.free("hamper", (HAMPER_XY[0], HAMPER_XY[1], FLOOR), parts, cols, LOOKS, HAMPER_MASS, friction=(0.8, 0.7), com=(0.0, 0.0, 0.22))


SOCK_PARTS = [
    _sock("sock_a", *SOCKS["sock_a"], "sock_white", "cuff_white"),
    _sock("sock_b", *SOCKS["sock_b"], "sock_grey", "cuff_dark"),
    _sock("sock_c", *SOCKS["sock_c"], "sock_black", "cuff_dark"),
]
HAMPER = _hamper()

DISTRACTORS = [
    SimReady("towel", pos=(0.24, 0.20, TOP), size=(0.205, 0.212, 0.034), mass=0.10, package="consumable_microfiber_towel_folded_small_b01",
             usd="sm_consumable_microfiber_towel_folded_small_b01_01.usd", yaw=0.1),
    SimReady("notebook", pos=(0.50, 0.30, TOP), size=(0.229, 0.170, 0.010), mass=0.12, package="misc_notebook_a01", usd="sm_misc_notebook_a01_01.usd", yaw=0.3),
    SimReady("mug", pos=(0.62, 0.05, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=0.6),
]

OBJECTS = [HAMPER, *SOCK_PARTS, *DISTRACTORS]
SOCK_NAMES = ("sock_a", "sock_b", "sock_c")


# -- reading the scene -------------------------------------------------------------------------------------------------------------------


def hamper_ok(h) -> bool:
    off = math.hypot(h.pos[0] - HAMPER_XY[0], h.pos[1] - HAMPER_XY[1])
    return bool(h.tilt_deg() <= 10.0 and off <= 0.05)


def sock_in(h, s) -> bool:
    loc = h.to_local(np.asarray([s.pos], float))[0]
    r = math.hypot(loc[0], loc[1])
    still = float(np.linalg.norm(s.lin_vel)) < 0.05
    return bool(r < IN_R and 0.0 < loc[2] < IN_Z and still)


def now(env) -> dict:
    h = m.state(env, "hamper")
    socks = {n: m.state(env, n) for n in SOCK_NAMES}
    tips = m.fingertips(env)
    loc = h.to_local(tips)
    reach = float(np.min(np.hypot(loc[:, 0], loc[:, 1]) - HAMPER_R))
    return dict(h=h, socks=socks, ok=hamper_ok(h), inside={n: sock_in(h, s) for n, s in socks.items()}, reach=reach)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a trial's record is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.n_in = 0

    def step(self, env):
        c = now(env)
        n = sum(c["inside"].values()) if c["ok"] else 0
        self.n_in = n
        ok = n == 3 and c["reach"] >= 0.30
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if not c["ok"]:
            self.full = False
        self.score = 1.0 if (self.full and n == 3) else round(min(n, 2) / 3.0, 4)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full and TRIAL.n_in == 3)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration -----------------------------------------------------------------------------------------------------------------------


# The throw, in the right arm's joints (shoulder yaw, shoulder pitch, elbow, wrist pitch, wrist roll, wrist yaw): from the cocked pose
# (the pair about 30 cm over the table just in front of the arm's base, wrist bent back) shoulder pitch and wrist pitch sweep forward together, their targets moving
# at RATE rad/s; the gripper opens REL control steps into the sweep. The pair leaves at about 2.2 m/s, 45-50 degrees up.
AIM = math.atan2(HAMPER_XY[1] - (-0.305), HAMPER_XY[0])  # the right arm's base yaw that faces the hamper (-29.5 deg)
COCK = (AIM, 0.4, 1.2, -1.6, 0.0, 0.0)
THROWN = (AIM, 2.0, 1.2, 1.0, 0.0, 0.0)
RATE = (0.0, 4.0, 5.5, 20.0, 0.0, 0.0)
REL = 4


def _report(ctx, env, label=""):
    c = now(env)
    print(f"[socks] t {ctx.t:5.2f} {label}" + " ".join(f"{n} {np.round(s.pos, 3)} in {c['inside'][n]}" for n, s in c["socks"].items())
          + f" hamper {np.round(c['h'].pos, 3)} tilt {c['h'].tilt_deg():.1f} reach {c['reach']:.2f} score {TRIAL.score} hold {TRIAL.hold}", flush=True)


def _sock_yaw(ctx, name) -> float:
    """The sock's long axis's heading (deg, folded into -90..90): the grip closes across its width."""
    rot = m.quat_matrix(ctx.state(name).quat_xyzw)
    yaw = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
    return (yaw + 90.0) % 180.0 - 90.0


def _throw(ctx, env, name, params):
    """Pick one pair, cock the arm and throw it."""
    import demokit as d
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN

    r = ctx.right
    cock, thrown, rate, rel = params
    yield from r.pick(name, 0.0, width=SOCK[1], grip=d.down(_sock_yaw(ctx, name)), above=0.10, lift=0.10, force=25.0, label=f"pick {name}")
    grip_f = float(r.cmd[6])
    # to the cocked pose, smoothly in joint space
    q0 = np.array(r.q, dtype=float)
    n = 30
    for k in range(1, n + 1):
        q = q0 + (cock - q0) * ik.smoothstep(k / n)
        r.q = q.copy()
        r.cmd = np.r_[q, grip_f]
        yield
    yield from r.wait(0.5)
    q = cock.copy()
    for k in range(1, 16):
        q = np.clip(q + np.clip(thrown - q, -rate * DT, rate * DT), ik.LOWER + 0.02, ik.UPPER - 0.02)
        r.q = q.copy()
        r.cmd = np.r_[q, 1.0 if k >= rel else grip_f]
        yield
    r.finger = FINGER_OPEN
    r.q = np.array(ctx.observed("right")[:6], dtype=float)
    r.grip = d._as_grip(ik.tcp_pose(r.q, r.base)[1])
    yield from r.move(np.array([0.25, -0.30, TOP + 0.20]), grip=d.down(0.0), finger=FINGER_OPEN, t=1.5, label="back over the table")


def demo_script(ctx):
    """The right arm picks each pair from above (fingers closing across its 6 cm width, whatever its heading), raises it to a cocked pose about 30 cm
    over the table in front of its base, wrist bent back, and swings shoulder and wrist forward together; the gripper opens on the fourth control step
    of the swing.
    Failure checks (--mode): none does nothing; drop lets each pair go at the arm's farthest reach over the table; weak and hard
    throw at half and 1.25 times the sweep rates; early and late open the gripper at step 2 or 7 of the sweep instead of 4."""
    env = ctx.emb._env
    mode = ctx.mode
    if mode not in ("full", "none", "drop", "weak", "hard", "early", "late"):
        raise ValueError(mode)
    for _ in range(4):
        yield
    _report(ctx, env, f"{mode} start: ")
    if mode == "none":
        for _ in range(120):
            yield
        _report(ctx, env, "end: ")
        return
    cock, thrown, rate, rel = np.array(COCK), np.array(THROWN), np.array(RATE), REL
    if mode == "weak":
        rate = rate * 0.5
    if mode == "hard":
        rate = rate * 1.25
    if mode == "early":
        rel = 2
    if mode == "late":
        rel = 7
    names = SOCK_NAMES
    import demokit as d
    from yam_arena.constants import FINGER_OPEN
    for name in names:
        if mode == "drop":
            r = ctx.right
            yield from r.pick(name, 0.0, width=SOCK[1], grip=d.down(_sock_yaw(ctx, name)), above=0.10, lift=0.10, label=f"pick {name}")
            yield from r.move(np.array([0.48, -0.30, TOP + 0.15]), grip=d.down(0.0), t=1.5, label="far out")
            yield from r.open(t=0.4)
            yield from r.move(np.array([0.25, -0.30, TOP + 0.20]), finger=FINGER_OPEN, t=1.2, label="back")
        else:
            yield from _throw(ctx, env, name, (cock, thrown, rate, rel))
        _report(ctx, env, f"after {name}: ")
    for k in range(50):
        yield
    _report(ctx, env, "end: ")
