"""On the station table: three small dark-blue 'PV module' blocks stand in a row at the far edge of the robot's reach
(left, middle, right; each 10 cm deep x 12 cm wide x 2.5 cm tall, fixed). From the front face of each module two
short 7 mm leads, 21 cm long, lie straight on the table towards the robot, each ending in a rigid connector body
(40 x 16 x 14 mm): a RED lead (+) and a BLACK lead (-). Left module: black lead on its inner (right) side, red lead
on its outer side. Middle module: black lead left, red lead right. Right module: red lead on its inner (left)
side, black lead on its outer side. Between the modules, closer to the robot, sit two fixed grey in-line couplers
(left coupler at y +10 cm, right coupler at y -10 cm): a U-channel running left-right with a 2 mm centre stop, so
one connector enters from each side and the two meet end to end at the stop.

The robot must wire the string in series with correct polarity: in the left coupler the left module's RED lead
meets the middle module's BLACK lead; in the right coupler the middle module's RED lead meets the right module's
BLACK lead. The left module's black lead and the right module's red lead are the string's home-run ends and stay
free. The layout is a derangement: the lead nearest each coupler on the module side (left black, right red) is
always the wrong one, so pairing by position alone (or by "red meets black" alone) scores 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var STRING_DEMO): correct (default) | nothing | nearest | redred | hold | half.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Wire the solar panels in series, left to right: in each grey coupler, push a panel's red "
               "(+) lead and the black (-) lead of the panel to its right in from opposite sides until both touch "
               "the centre stop. Leave the left panel's black and the right panel's red lead free for the inverter, "
               "then let go.")

RED = (0.78, 0.07, 0.06)
BLACK = (0.04, 0.04, 0.04)
PANEL_BLUE = (0.10, 0.16, 0.38)
FRAME = (0.75, 0.76, 0.78)
COUPLER_GREY = (0.50, 0.52, 0.55)

# -- panels and their leads (x forward, y left) ---------------------------------------------------------------------
PANEL_X, PANEL_SIZE = 0.50, (0.10, 0.12, 0.025)
PANEL_FACE_X = PANEL_X - PANEL_SIZE[0] / 2  # front face (towards the robot)
PANEL_Y = {"p1": 0.21, "p2": 0.0, "p3": -0.21}  # left, middle, right
R = 0.0035  # 7 mm lead
LEAD_LEN = 0.21
PL, PW, PH = 0.040, 0.016, 0.014  # connector body: length along the lead, width, height
LEAD_Z = TOP + PH / 2 + 0.0005
ANCHOR_X = PANEL_FACE_X - 0.006
LEAD_SPLAY = math.radians(10.0)  # the middle panel's two leads fan out by 10 degrees each
# lead name -> (panel, polarity, exit y)
LEADS = {
    "p1_pos": ("p1", "+", 0.240), "p1_neg": ("p1", "-", 0.180),
    "p2_neg": ("p2", "-", 0.035), "p2_pos": ("p2", "+", -0.035),
    "p3_pos": ("p3", "+", -0.180), "p3_neg": ("p3", "-", -0.240),
}


def _lead(name: str):
    _, pol, y = LEADS[name]
    c = RED if pol == "+" else BLACK
    yaw = math.pi - math.copysign(LEAD_SPLAY, y) if name.startswith("p2") else math.pi
    return cable(name, start=(ANCHOR_X, y, LEAD_Z), yaw=yaw, length=LEAD_LEN, radius=R, anchor_start=True,
                 end_plug=Plug(size=(PL, PW, PH), mass=0.015, color=c), color=c, bend_stiffness=0.01,
                 bend_damping=0.001)


LEAD_OBJS = {n: _lead(n) for n in LEADS}
PANELS = []
for _p, _y in PANEL_Y.items():
    PANELS += [
        Box(f"panel_{_p}", pos=(PANEL_X, _y, TOP + PANEL_SIZE[2] / 2), size=PANEL_SIZE, color=PANEL_BLUE),
        Box(f"panel_{_p}_frame", pos=(PANEL_FACE_X + 0.003, _y, TOP + PANEL_SIZE[2] + 0.001),
            size=(0.006, PANEL_SIZE[1], 0.002), color=FRAME),
    ]

# -- couplers: a U-channel along y with a 2 mm centre stop ------------------------------------------------------------
COUPLER_X = 0.32
COUPLERS = {"left": 0.10, "right": -0.10}  # centre-stop y
GAP = 0.022  # inner channel width (x): 3 mm clearance each side of the connector
WALL_T, WALL_H, WALL_L = 0.005, 0.008, 0.018  # side wall thickness, height, length from the stop (each side)
STOP_T, STOP_H = 0.002, 0.012


def _coupler(tag: str) -> list[Box]:
    cy = COUPLERS[tag]
    out = [Box(f"coupler_{tag}_stop", pos=(COUPLER_X, cy, TOP + STOP_H / 2), size=(GAP, STOP_T, STOP_H),
               color=COUPLER_GREY)]
    for sx, nx in ((+1, "far"), (-1, "near")):
        out.append(Box(f"coupler_{tag}_wall_{nx}", pos=(COUPLER_X + sx * (GAP + WALL_T) / 2, cy, TOP + WALL_H / 2),
                       size=(WALL_T, 2 * WALL_L + STOP_T, WALL_H), color=COUPLER_GREY))
    return out


COUPLER_OBJS = [b for t in COUPLERS for b in _coupler(t)]
OBJECTS = [*PANELS, *LEAD_OBJS.values(), *COUPLER_OBJS]

# -- scoring ---------------------------------------------------------------------------------------------------------
CORRECT_PAIRS = ({"p1_pos", "p2_neg"}, {"p2_pos", "p3_neg"})
HOME_RUN = {"p1_neg", "p3_pos"}
TIP_TO_STOP = 0.005  # each connector's front end within 5 mm of the centre stop face (tips <= 12 mm apart)
SEAT_X = 0.008  # connector centre within 8 mm of the channel centre line (x)
SEAT_Z = 0.006  # connector centre at most 6 mm above lying on the table
ALIGN = 0.85  # cos between connector axis and channel axis (within ~30 deg)
NEAR_X, NEAR_Y, NEAR_Z = 0.03, 0.06, 0.04  # "at the coupler" zone around the stop (half sizes / height)
RELEASE_DIST = 0.03  # every gripper TCP at least 3 cm from every connector centre
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    """Keep a handle on the env so score() can see where the grippers are."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_string_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._string_hooked = True
        cls._task_step = _task_step


_install_hooks()


def plug(objects, name: str) -> tuple[np.ndarray, np.ndarray]:
    """(connector centre, unit axis pointing out through its front end)."""
    st = objects[name]
    p = np.asarray(st.end_plug if st.end_plug is not None else np.full(3, np.nan), dtype=float)
    a = p - np.asarray(st.points[-1], dtype=float)
    return p, a / (np.linalg.norm(a) + 1e-12)


def near(p: np.ndarray, tag: str) -> bool:
    """Connector centre at the coupler (generous zone: within 3 cm of the channel line, 6 cm of the stop, 4 cm up)."""
    cy = COUPLERS[tag]
    return bool(np.isfinite(p).all() and abs(p[0] - COUPLER_X) <= NEAR_X and abs(p[1] - cy) <= NEAR_Y
                and p[2] <= TOP + NEAR_Z)


def seated_side(p: np.ndarray, a: np.ndarray, tag: str) -> int:
    """+1 / -1: the connector lies in the channel on the +y / -y side of the stop, pointing at it, front end within
    5 mm of the stop face; 0 otherwise."""
    if not np.isfinite(p).all():
        return 0
    cy = COUPLERS[tag]
    if abs(p[0] - COUPLER_X) > SEAT_X or p[2] > TOP + PH / 2 + SEAT_Z:
        return 0
    s = 1 if p[1] > cy else -1
    if -s * a[1] < ALIGN:
        return 0
    tip_gap = s * (p[1] + a[1] * PL / 2 - cy) - STOP_T / 2
    return s if tip_gap <= TIP_TO_STOP else 0


def analyse(objects) -> dict:
    plugs = {n: plug(objects, n) for n in LEADS}
    held = {t: {n for n, (p, _) in plugs.items() if near(p, t)} for t in COUPLERS}
    bad = []
    for t, h in held.items():
        if h & HOME_RUN:
            bad.append(f"{t}: home-run lead {sorted(h & HOME_RUN)} connected")
        if len(h) >= 2 and h not in CORRECT_PAIRS:
            bad.append(f"{t}: wrong pair {sorted(h)}")
    good = 0
    for pair in CORRECT_PAIRS:
        a, b = sorted(pair)
        for t in COUPLERS:
            sa, sb = seated_side(*plugs[a], t), seated_side(*plugs[b], t)
            if sa and sb and sa == -sb:
                good += 1
                break
    return {"held": held, "bad": bad, "good": good}


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP; [] if unknown (offline)."""
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
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def released(objects) -> bool:
    ps = [plug(objects, n)[0] for n in LEADS]
    return all(float(np.linalg.norm(p - g)) >= RELEASE_DIST for g in grippers() for p in ps)


def score(objects) -> float:
    """0 if a gripper is within 3 cm of any connector at the end, if either home-run connector (left panel black,
    right panel red) is at a coupler, or if a coupler holds two connectors that are not a correct series pair
    (red-red, black-black, a panel's own red+black, or red/black from non-adjacent or reversed panels). Otherwise
    0.5 per correct pair (left red + middle black; middle red + right black) seated in one coupler: one connector
    from each side, both in the channel, aligned, front ends within 5 mm of the centre stop."""
    if not released(objects):
        return 0.0
    r = analyse(objects)
    if r["bad"]:
        return 0.0
    return 0.5 * r["good"]


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and all(np.linalg.norm(objects[n].lin_vel) < 0.02 for n in LEADS))


def distance(objects) -> float:
    """Sum over the four leads to connect of their connector's distance (m) to its seated pose in the demo's
    coupler; 1 m once a wrong connection is made."""
    if analyse(objects)["bad"]:
        return 1.0
    d = 0.0
    for name, (tag, s) in TARGET.items():
        d += float(np.linalg.norm(plug(objects, name)[0] - _seat_pos(tag, s)))
    return d


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
# Both arms in parallel: each arm pinches its outer panel's correct lead near the connector's lead end, turns it in
# place to point at its coupler, sets it down and grips it again from the other side, so the wrist camera (on the
# grip's yaw side) looks along the connector; carries it to 2 cm outside the channel, lines it up (closed loop on the
# connector), slides it against the centre stop (backing off and retrying if it stalls at the mouth), releases.
# Then, one at a time with the other arm parked, the right arm reaches across to plug the middle black lead into the
# left coupler and the left arm the middle red lead into the right coupler, gripping each along its tip directly.

TARGET = {"p1_pos": ("left", +1), "p2_neg": ("left", -1), "p3_neg": ("right", -1), "p2_pos": ("right", +1)}
FINGER_PINCH = -0.020
RELEASE_OPEN = -0.012  # finger joint for letting go in the coupler (~4 mm clear of the connector each side)
GRASP_BACK = 0.010  # pinch this far from the connector's lead end
GZ = TOP + PH + 0.003  # TCP height for the pinch
CARRY = TOP + 0.05
PARK = {"left": (0.20, 0.30, TOP + 0.14), "right": (0.20, -0.30, TOP + 0.14)}


def _seat_pos(tag, s):
    return np.array([COUPLER_X, COUPLERS[tag] + s * (STOP_T / 2 + PL / 2 + 0.001), TOP + PH / 2])


def _log(ctx, tag=""):
    st = ctx.states()
    r = analyse(st)
    pl = {n: np.round(plug(st, n)[0], 3).tolist() for n in TARGET}
    ax = {n: np.round(plug(st, n)[1][:2], 2).tolist() for n in TARGET}
    print(f"[t={ctx.t:5.2f}] {tag} held={ {k: sorted(v) for k, v in r['held'].items()} } bad={r['bad']} "
          f"good={r['good']} rel={released(st)} score={score(st)} plugs={pl} axes={ax}", flush=True)


def _pick(ctx, arm, name, along_tip=False):
    """Pinch the connector near its lead end and lift it. ``along_tip``: turn the grip so the wrist camera (which sits
    on the grip's yaw side) looks along the connector's front end; otherwise use the nearest yaw in [-pi/2, pi/2]."""
    from yam_arena.constants import FINGER_CLOSED

    p, a = plug(ctx.states(), name)
    g = p - a * (PL / 2 - GRASP_BACK)
    yaw = math.atan2(a[1], a[0])
    if not along_tip:
        yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
    yield from arm.move((g[0], g[1], CARRY), yaw=yaw, finger=FINGER_PINCH, duration=1.4)
    p, a = plug(ctx.states(), name)
    g = p - a * (PL / 2 - GRASP_BACK)
    yield from arm.move((g[0], g[1], GZ), yaw=yaw, duration=0.9)
    yield from arm.grip(FINGER_CLOSED, 0.5)
    if os.environ.get("STRING_DBG"):
        print(f"[pick {name}] cmd={np.round(g, 3).tolist()} tcp={[np.round(t, 3).tolist() for t in grippers()]}",
              flush=True)
    yield from arm.move((g[0], g[1], CARRY), yaw=yaw, duration=0.7)
    return yaw


def _turn_and_regrasp(ctx, arm, name, s):
    """Outer leads: the arm can't reach a grip with the camera looking along a connector that points at the robot.
    Turn the connector in place to point at its coupler (-s along y), set it down, let go, and grip it again from the
    other side (yaw turned by pi, via 0 so the wrist turns the feasible way): the camera then looks along it."""
    from yam_arena.constants import FINGER_CLOSED

    tcp = arm.tcp()
    yield from arm.move((tcp[0], tcp[1], TOP + 0.03), yaw=s * math.pi / 2, duration=1.0)
    yield from arm.move((tcp[0], tcp[1], GZ + 0.002), yaw=s * math.pi / 2, duration=0.6)
    yield from arm.grip(FINGER_PINCH, 0.4)
    yield from arm.move((tcp[0], tcp[1], CARRY), yaw=s * math.pi / 2, duration=0.6)
    yield from arm.move((tcp[0], tcp[1], CARRY), yaw=0.0, duration=0.6)
    yaw = yield from _pick(ctx, arm, name, along_tip=True)
    return yaw


def _insert(ctx, arm, name, tag, s, yaw):
    """Bring the held connector to the ``s`` side of coupler ``tag`` pointing at the stop, slide it in, release."""
    cy = COUPLERS[tag]
    pre = 0.022  # front end this far outside the stop face when set down (walls end at 18 mm)
    gy = cy + s * (STOP_T / 2 + pre + PL - GRASP_BACK)
    yield from arm.move((COUPLER_X, gy, CARRY), yaw=yaw, duration=1.6)
    dx = dy = 0.0
    for _ in range(3):  # closed loop: connector front end on the channel line, `pre` outside the stop
        p, a = plug(ctx.states(), name)
        tip = p + a * PL / 2
        ex = COUPLER_X - p[0]
        ey = (cy + s * (STOP_T / 2 + pre)) - tip[1]
        if abs(ex) < 0.001 and abs(ey) < 0.002:
            break
        dx, dy = float(np.clip(dx + ex, -0.02, 0.02)), float(np.clip(dy + ey, -0.02, 0.02))
        yield from arm.move((COUPLER_X + dx, gy + dy, CARRY), yaw=yaw, duration=0.5)
        yield from arm.wait(0.2)
    yield from arm.move((COUPLER_X + dx, gy + dy, GZ), yaw=yaw, duration=0.8)
    yield from arm.wait(0.2)
    gy2 = gy + dy
    for attempt in range(4):
        for _ in range(2):  # line the connector's front part up with the channel (the arm sags under load)
            p, a = plug(ctx.states(), name)
            ex = COUPLER_X - (p[0] + a[0] * (PL / 2 - 0.010))
            if abs(ex) < 0.0008:
                break
            dx = float(np.clip(dx + ex, -0.02, 0.02))
            yield from arm.move((COUPLER_X + dx, gy2, GZ), yaw=yaw, duration=0.4)
            yield from arm.wait(0.2)
        p, a = plug(ctx.states(), name)
        gap = s * (p[1] + a[1] * PL / 2 - cy) - STOP_T / 2
        gy2 -= s * (gap + 0.008)
        yield from arm.move((COUPLER_X + dx, gy2, GZ), yaw=yaw, duration=1.2)
        yield from arm.wait(0.3)
        p, a = plug(ctx.states(), name)
        gap = s * (p[1] + a[1] * PL / 2 - cy) - STOP_T / 2
        if os.environ.get("STRING_DBG"):
            print(f"[insert {name}] attempt {attempt} gap_mm={gap * 1000:.1f} x={p[0]:.4f}", flush=True)
        if gap <= TIP_TO_STOP - 0.002:
            break
        gy2 = gy2 + s * (gap + 0.008) + s * 0.006  # stalled at the channel mouth: back off and line up again
        yield from arm.move((COUPLER_X + dx, gy2, GZ), yaw=yaw, duration=0.5)
    yield from arm.grip(RELEASE_OPEN, 0.5)  # open only a little, so the fingers don't push the connector aside
    yield from arm.wait(0.2)
    yield from arm.move((COUPLER_X + dx, gy2, GZ + 0.03), yaw=yaw, duration=0.7)  # straight up off the connector
    yield from arm.move((COUPLER_X + dx, gy2 + s * 0.006, CARRY + 0.03), yaw=yaw, duration=0.6)


def _connect(ctx, arm, name, tag, s):
    """Plug lead ``name`` into side ``s`` of coupler ``tag``. The grip always ends with yaw -s*pi/2, so the wrist
    camera looks along the connector towards the stop while it is carried in and inserted."""
    if name.startswith("p2"):  # middle leads: taken by the arm on the far side, which can grip along the tip directly
        yield from _pick(ctx, arm, name, along_tip=True)
    else:
        yield from _pick(ctx, arm, name)
        yield from _turn_and_regrasp(ctx, arm, name, s)
    yield from _insert(ctx, arm, name, tag, s, -s * math.pi / 2)


def _park(arm, side):
    yield from arm.move(PARK[side], yaw=0.0, duration=1.2)


def demo_script(ctx):
    mode = os.environ.get("STRING_DEMO", "correct")  # test only
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":
        for _ in range(4):
            yield from L.wait(1.0)
        _log(ctx, "end")
        return
    # (lead, coupler, side). The left arm does the left outer lead and the middle red (into the right coupler); the
    # right arm the right outer lead and the middle black (into the left coupler): reaching across lets each arm hold
    # its middle connector with the wrist camera looking along it.
    if mode == "nearest":  # position only: the lead nearest each coupler side (left black, right red)
        plan_l = [("p1_neg", "left", +1), ("p2_pos", "right", +1)]
        plan_r = [("p3_pos", "right", -1), ("p2_neg", "left", -1)]
    elif mode == "redred":  # left coupler correct; right coupler gets the two red leads
        plan_l = [("p1_pos", "left", +1), ("p2_pos", "right", +1)]
        plan_r = [("p3_pos", "right", -1), ("p2_neg", "left", -1)]
    else:
        plan_l = [("p1_pos", "left", +1), ("p2_pos", "right", +1)]
        plan_r = [("p3_neg", "right", -1), ("p2_neg", "left", -1)]
    if mode == "half":  # left coupler only
        plan_l = [("p1_pos", "left", +1)]
        plan_r = [("p2_neg", "left", -1)]

    def step(arm, item, side):
        n, t, s = item
        yield from _connect(ctx, arm, n, t, s)
        _log(ctx, f"{side} {n} placed")

    def idle():
        yield

    # 1) both arms in parallel: each outer panel's lead into its own coupler
    yield from ctx.both(step(L, plan_l[0], "left"), step(Rt, plan_r[0], "right") if mode != "half" else idle())
    # 2) the middle panel's leads one arm at a time, reaching across while the other arm is parked
    yield from ctx.both(_park(L, "left"), _park(Rt, "right"))
    yield from step(Rt, plan_r[-1], "right")
    yield from _park(Rt, "right")
    if mode != "half":
        yield from step(L, plan_l[-1], "left")
        if mode != "hold":
            yield from _park(L, "left")
    if mode == "hold":  # the left arm re-grips the middle red connector and keeps holding it
        from yam_arena.constants import FINGER_CLOSED
        p, a = plug(ctx.states(), "p2_pos")
        g = p - a * (PL / 2 - GRASP_BACK)
        yield from L.move((g[0], g[1], GZ), yaw=-math.pi / 2, duration=0.8)
        yield from L.grip(FINGER_CLOSED, 0.5)
    for _ in range(2):
        yield from L.wait(1.0)
    _log(ctx, "end")
