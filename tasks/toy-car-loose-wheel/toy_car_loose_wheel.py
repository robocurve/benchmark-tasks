"""Three donated wooden toy cars: the sound ones go in the toddlers' toy basket, the one with a loose wheel in the repair box.

Three identical beech toy cars stand in a row. Each has four hardwood wheels on birch axle pegs glued into the body. On one car
(which car, and which corner, changes with the seed) one peg has come unglued: it sits dry in its hole, holds at rest,
and slides out under a pull of about 3 N. Nothing shows it until a wheel is pulled along its axle.

Mechanism: the loose wheel and its peg are one body on a sliding joint along the axle, from fully in (0) to a stop
12 mm out. A drive with no spring and a 3 N force cap resists its motion (dry friction): below 3 N it creeps at most
0.3 mm/s, above 3 N it slides. The other eleven wheels are part of their car body (glued). Nothing moves the wheel
except contact.

Frame: x forward from the arm bases, y left, z up, metres. All sizes, masses and places: layout.py.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies and joints are simulated)
import layout as L  # noqa: E402
from bodykit import Body, BoxShape, CylShape  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = L.TOP
SCORE_OVER = "end"
CONTROL_HZ = 20.0

BEECH, BIRCH, PEG_C = (0.86, 0.62, 0.40), (0.93, 0.85, 0.68), (0.80, 0.70, 0.52)
CAB_C = (0.80, 0.18, 0.15)  # the cab is painted red (water-based toy paint)
BASKET_C, BOX_C = (0.45, 0.78, 0.35), (0.52, 0.54, 0.56)
DOT_C = (0.10, 0.10, 0.10)


def _box(lo, hi, color, collide: bool = True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _wheel_shapes(cx: float, side: float, origin=(0.0, 0.0, 0.0)) -> list:
    """One wheel, its peg head and (inside the body, no collision) its peg. ``side`` +1: the car's +y side."""
    ox, oy, oz = origin
    wy = side * L.WHEEL_Y
    hy = side * (L.WHEEL_Y + L.WHEEL_W / 2 + L.HEAD_T / 2)
    py = side * (L.BODY_W / 2 + L.SPACER - L.PEG_IN / 2)
    return [CylShape((cx - ox, wy - oy, L.WHEEL_R - oz), L.WHEEL_R, L.WHEEL_W, BIRCH, axis="Y"),
            CylShape((cx - ox, hy - oy, L.WHEEL_R - oz), L.HEAD_R, L.HEAD_T, PEG_C, axis="Y"),
            CylShape((cx - ox, py - oy, L.WHEEL_R - oz), L.PEG_R, L.PEG_IN + L.SPACER, PEG_C, axis="Y", collide=False)]


def _body_shapes() -> list:
    s = [_box((-L.BODY_L / 2, -L.BODY_W / 2, L.BODY_Z[0]), (L.BODY_L / 2, L.BODY_W / 2, L.BODY_Z[1]), BEECH),
         _box((L.CAB_X - L.CAB_L / 2, -L.CAB_W / 2, L.CAB_Z[0]), (L.CAB_X + L.CAB_L / 2, L.CAB_W / 2, L.CAB_Z[1]),
              CAB_C),
         # a 10 mm marker dot under the body: the tester's mark for which car was prepared loose (no collision)
         _box((-0.005, -0.005, L.BODY_Z[0] - 0.0004), (0.005, 0.005, L.BODY_Z[0]), DOT_C, collide=False)]
    return s


LOOSE = (L.AXLE_X, 1.0)  # the loose wheel's corner on the defective car (its own frame): front, +y side


def _car_shapes(loose: bool) -> list:
    s = _body_shapes()
    for ax in (L.AXLE_X, -L.AXLE_X):
        for side in (1.0, -1.0):
            if loose and (ax, side) == LOOSE:
                continue
            s += _wheel_shapes(ax, side)
    return s


def _container_shapes(l_y: float, w_x: float, h: float, color) -> list:
    hx, hy, w, f = w_x / 2, l_y / 2, L.WALL, L.FLOOR
    s = [_box((-hx, -hy, 0.0), (hx, hy, f), color)]
    for sx in (-1.0, 1.0):
        x0, x1 = sorted((sx * hx, sx * (hx - w)))
        s.append(_box((x0, -hy, 0.0), (x1, hy, h), color))
    for sy in (-1.0, 1.0):
        y0, y1 = sorted((sy * hy, sy * (hy - w)))
        s.append(_box((-hx, y0, 0.0), (hx, y1, h), color))
    return s


# block letters on a 1 x 1 grid (x right, y up), as strokes
_GLYPHS = {
    "S": [((1, 1), (0, 1)), ((0, 1), (0, .5)), ((0, .5), (1, .5)), ((1, .5), (1, 0)), ((1, 0), (0, 0))],
    "T": [((0, 1), (1, 1)), ((.5, 1), (.5, 0))],
    "A": [((0, 0), (0, 1)), ((0, 1), (1, 1)), ((1, 1), (1, 0)), ((0, .5), (1, .5))],
    "F": [((0, 0), (0, 1)), ((0, 1), (1, 1)), ((0, .5), (.75, .5))],
    "R": [((0, 0), (0, 1)), ((0, 1), (1, 1)), ((1, 1), (1, .5)), ((1, .5), (0, .5)), ((.35, .5), (1, 0))],
    "E": [((0, 0), (0, 1)), ((0, 1), (1, 1)), ((0, .5), (.8, .5)), ((0, 0), (1, 0))],
    "P": [((0, 0), (0, 1)), ((0, 1), (1, 1)), ((1, 1), (1, .5)), ((1, .5), (0, .5))],
    "I": [((.5, 0), (.5, 1)), ((.15, 1), (.85, 1)), ((.15, 0), (.85, 0))],
}
SIGN_C, INK_C = (0.97, 0.97, 0.95), (0.75, 0.08, 0.08)


def _sign_shapes() -> list:
    """The repair box's paper sign, in the box's frame: a white card taped inside the back wall, leaning back, with
    "STAFF" over "REPAIR" in red block letters facing the robot. Visual only (paper; no collision)."""
    lean = math.radians(L.SIGN_LEAN_DEG)
    v = np.array([math.sin(lean), 0.0, math.cos(lean)])  # up the card
    n = np.array([-math.cos(lean), 0.0, math.sin(lean)])  # the card's front, toward the robot
    u = np.array([0.0, -1.0, 0.0])  # left to right as the robot sees it
    base = np.array([L.BOX_W / 2 - L.WALL - 0.012, 0.0, L.SIGN_BOTTOM])
    s = [bodykit.plate(base, base + v * L.SIGN_H, L.SIGN_W, 0.001, n, SIGN_C, collide=False)]
    gap, t = 0.007, L.STROKE
    for word, v0 in (("STAFF", 0.097), ("REPAIR", 0.050)):
        u0 = -(len(word) * L.LETTER_W + (len(word) - 1) * gap) / 2
        for k, ch in enumerate(word):
            ox = u0 + k * (L.LETTER_W + gap)
            for (x1, y1), (x2, y2) in _GLYPHS[ch]:
                a = np.array([ox + x1 * L.LETTER_W, v0 + y1 * L.LETTER_H])
                b = np.array([ox + x2 * L.LETTER_W, v0 + y2 * L.LETTER_H])
                d = (b - a) / np.linalg.norm(b - a) * t / 2  # run each stroke half a stroke past its ends
                a, b = a - d, b + d
                p1 = base + u * a[0] + v * a[1] + n * (0.0005 + t / 2)
                p2 = base + u * b[0] + v * b[1] + n * (0.0005 + t / 2)
                s.append(bodykit.segment(p1, p2, t, INK_C, collide=False))
    return s


CAR_NAMES = ("car_bad", "car_a", "car_b")
WHEEL_C = (L.AXLE_X, L.WHEEL_Y, L.WHEEL_R)  # the loose wheel body's origin in the defective car's frame


def _build() -> list:
    cars = []
    for i, name in enumerate(CAR_NAMES):
        y = L.SLOTS_Y[i]
        cars.append(Body.build(name, pos=(L.CAR_X, y, TOP), mass=L.BODY_MASS + (3 if i == 0 else 4) * L.WHEEL_MASS,
                               shapes=_car_shapes(i == 0), color=BEECH, friction=L.CAR_FRICTION, solver_iterations=32,
                               com=(0.0, 0.0, 0.020), contact_offset=0.002))
    bx, by = L.CAR_X + WHEEL_C[0], L.SLOTS_Y[0] + WHEEL_C[1]
    wheel = Body.build("wheel", pos=(bx, by, TOP + WHEEL_C[2]), mass=L.WHEEL_MASS,
                       shapes=_wheel_shapes(L.AXLE_X, 1.0, origin=WHEEL_C), color=BIRCH, friction=(0.8, 0.7),
                       solver_iterations=32, contact_offset=0.001, ignore=("car_bad",))
    bodykit.slider("loose_peg", "car_bad", "wheel", anchor=(bx, by, TOP + WHEEL_C[2]), axis="Y",
                   limits=(0.0, L.LOOSE_STOP), stiffness=0.0, damping=L.LOOSE_DAMPING, max_force=L.LOOSE_FRICTION_N)
    basket = Body.build("basket", pos=(L.BASKET_XY[0], L.BASKET_XY[1], TOP), mass=L.BASKET_MASS,
                        shapes=_container_shapes(L.BASKET_L, L.BASKET_W, L.BASKET_H, BASKET_C), color=BASKET_C,
                        friction=(0.6, 0.5), solver_iterations=16)
    box = Body.build("repair_box", pos=(L.BOX_XY[0], L.BOX_XY[1], TOP), mass=L.BOX_MASS,
                     shapes=_container_shapes(L.BOX_L, L.BOX_W, L.BOX_H, BOX_C) + _sign_shapes(), color=BOX_C, friction=(0.6, 0.5),
                     solver_iterations=16)
    return cars + [wheel, basket, box]


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]
SPEC = {o.name: o for o in OBJECTS}


# -- the hidden fact: which slot holds the defective car, and whether it is turned round -------------------------------
def variant(seed: int) -> tuple[int, bool]:
    """Slot of the defective car (0 left, 1 middle, 2 right) and whether it points -x (its loose wheel then at the rear
    right instead of the front left). Six layouts; the episode seed picks one."""
    v = int(seed) % 6
    return v // 2, bool(v % 2)


def loose_corner(slot: int, turned: bool) -> str:
    return ("rear right" if turned else "front left") + f" of car in slot {slot + 1}"


# -- reading the scene ------------------------------------------------------------------------------------------------
def wheel_travel(st) -> float:
    """How far (m) the loose wheel's peg has slid out along its axle (0 = fully in)."""
    loc = st["car_bad"].to_local(st["wheel"].pos)
    return float(loc[1] - WHEEL_C[1])


def _car_corners(st, name: str) -> list[np.ndarray]:
    hx, hy = L.BODY_L / 2, L.CAR_W / 2
    return [st[name].pos + st[name].rotation() @ np.array([sx * hx, sy * hy, z])
            for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, L.CAR_H)]


def inside(st, name: str, cont: str) -> bool:
    """Is the whole car (its 100 x 72.6 x 46 mm box) within the container's inner walls, seen from above, with its base
    below the rim?"""
    if cont == "basket":
        hx, hy, h = L.BASKET_W / 2 - L.WALL, L.BASKET_L / 2 - L.WALL, L.BASKET_H
    else:
        hx, hy, h = L.BOX_W / 2 - L.WALL, L.BOX_L / 2 - L.WALL, L.BOX_H
    c = st[cont]
    for p in _car_corners(st, name):
        loc = c.to_local(p)
        if abs(loc[0]) > hx or abs(loc[1]) > hy:
            return False
    return float(c.to_local(st[name].pos)[2]) < h


def wheel_in_box(st) -> bool:
    loc = st["repair_box"].to_local(st["wheel"].pos)
    return (abs(loc[0]) <= L.BOX_W / 2 - L.WALL and abs(loc[1]) <= L.BOX_L / 2 - L.WALL and loc[2] < L.BOX_H)


def where(st, name: str) -> str:
    if inside(st, name, "basket"):
        return "basket"
    if inside(st, name, "repair_box"):
        return "repair box"
    return "elsewhere"


def finger_points(p, r, half: float) -> list[np.ndarray]:
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


def _hand_points(base) -> list:
    from bodykit import as_numpy
    from yam_arena import ik, station

    out = []
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        out.extend(finger_points(p, r, half))
    if not all(np.all(np.isfinite(p)) for p in out):
        raise ValueError("a gripper position is not finite")
    return out


def _shape_boxes(name: str) -> list[tuple[np.ndarray, np.ndarray]]:
    return [(np.asarray(sh.centre, float), sh.half_extent()) for sh in SPEC[name].shapes if getattr(sh, "collide", True)]


_SHAPES = {n: _shape_boxes(n) for n in CAR_NAMES + ("wheel",)}


def clearance(st, points) -> float:
    """Smallest distance (m) from the finger points to any part of any car or the loose wheel (each part's box)."""
    best = float("inf")
    for name, boxes in _SHAPES.items():
        for p in points:
            loc = st[name].to_local(p)
            for c, h in boxes:
                d = np.maximum(0.0, np.abs(loc - c) - h)
                best = min(best, float(np.linalg.norm(d)))
    return best


# -- episode state, the score -----------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["TOY_DEBUG"], "w") if os.environ.get("TOY_DEBUG") else None
HANDS_CLEAR = 0.03  # every finger point at least 30 mm from every car ...
CLEAR_STEPS = 20  # ... for the last 1 s of the trial


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, clear_hist=[], st=None, clear=float("inf"), max_travel=0.0, report={})


_blank()


@bodykit.on_reset
def _reset() -> None:
    keep = {k: _EP.get(k) for k in ("variant", "start")}
    _blank()
    _EP.update(keep)


def evaluate(st, clear_hist) -> dict:
    places = {n: where(st, n) for n in CAR_NAMES}
    clear = len(clear_hist) >= CLEAR_STEPS and min(clear_hist[-CLEAR_STEPS:]) >= HANDS_CLEAR
    bad_boxed = places["car_bad"] == "repair box" and wheel_in_box(st)
    # the proof that the fault was found by the test, not guessed: the loose wheel stands pulled out at the end (a dry
    # peg stays where it was pulled). NaN fails it. (The simulator stops the peg 12 mm out, so it cannot come off here.)
    out = wheel_travel(st)
    found = bool(np.isfinite(out)) and out * 1000.0 >= L.FOUND_MM
    good_boxed = any(places[n] == "repair box" for n in ("car_a", "car_b"))
    in_basket = sum(places[n] == "basket" for n in ("car_a", "car_b"))
    score = 0.0
    if bad_boxed and found and not good_boxed and clear:
        score = {2: 1.0, 1: 0.5, 0: 0.25}[in_basket]
    return dict(score=score, defective=places["car_bad"], good=[places["car_a"], places["car_b"]],
                wheel_in_box=wheel_in_box(st), found=found, wheel_out_mm=round(wheel_travel(st) * 1000, 1),
                max_wheel_out_mm=round(_EP.get("max_travel", 0.0) * 1000, 1), hands_clear=clear,
                min_clear_mm=round(min(clear_hist[-CLEAR_STEPS:]) * 1000, 1) if clear_hist else None,
                variant=_EP.get("variant"))


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["st"] = st
    try:
        clear = clearance(st, _hand_points(env.unwrapped))
    except Exception as err:  # noqa: BLE001  (fails closed: counts as a hand on a car)
        clear = 0.0
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    _EP["clear"] = clear
    _EP["clear_hist"].append(clear)
    if len(_EP["clear_hist"]) > CLEAR_STEPS:
        _EP["clear_hist"].pop(0)
    _EP["max_travel"] = max(_EP["max_travel"], wheel_travel(st))
    _EP["step"] += 1
    if _DBG is not None and _EP["step"] % 4 == 0:
        _debug(st)


def report(objects=None) -> dict:
    st = _EP.get("st")
    if st is None:
        return dict(score=0.0)
    return evaluate(st, _EP["clear_hist"])


def score(objects) -> float:
    """Read at the end of the trial (see task.json "scoring"). 0 unless the defective car (and its wheel) lies within
    the repair box's walls, below its rim, its loose wheel stands at least 4 mm out along its axle (the proof the fault
    was found by a pull, not guessed), no sound car is in the repair box, and every finger point has been at least
    30 mm from every car for the last 1 s. Then 1.0 with both sound cars within the basket's walls below its rim, 0.5
    with one, 0.25 with none."""
    rep = report(objects)
    _EP["report"] = rep
    if _DBG is not None:
        _DBG.write(f"score report: {rep}\n")
        _DBG.flush()
    return float(rep["score"])


def distance(objects) -> float:
    st = _EP.get("st")
    if st is None:
        return 1.0
    p = np.asarray(st["car_bad"].pos, float)
    return float(np.linalg.norm(p[:2] - np.asarray(L.BOX_SPOT)))


def _debug(st) -> None:
    cars = " ".join(f"{n}=({st[n].pos[0]:.3f},{st[n].pos[1]:.3f},{st[n].pos[2] - TOP:.3f}) t{st[n].tilt():.0f}"
                    for n in CAR_NAMES)
    _DBG.write(f"{_EP['step']:4d} wheel_out={wheel_travel(st) * 1000:5.2f}mm {cars} clear={_EP['clear']:.3f} "
               f"basket=({st['basket'].pos[0]:.3f},{st['basket'].pos[1]:.3f}) "
               f"box=({st['repair_box'].pos[0]:.3f},{st['repair_box'].pos[1]:.3f})\n")
    _DBG.flush()


# -- the start: everything fixed except which slot holds the defective car and its heading ------------------------------
def _yaw_q(turned: bool):
    return (0.0, 0.0, 1.0, 0.0) if turned else (0.0, 0.0, 0.0, 1.0)


def poses(seed: int) -> dict:
    slot, turned = variant(seed)
    others = [i for i in range(3) if i != slot]
    out = {}
    z = TOP + 0.0005
    out["car_bad"] = (np.array([L.CAR_X, L.SLOTS_Y[slot], z]), _yaw_q(turned))
    s = -1.0 if turned else 1.0
    out["wheel"] = (np.array([L.CAR_X + s * WHEEL_C[0], L.SLOTS_Y[slot] + s * WHEEL_C[1], z + WHEEL_C[2]]),
                    _yaw_q(turned))
    for name, i in zip(("car_a", "car_b"), others):
        out[name] = (np.array([L.CAR_X, L.SLOTS_Y[i], z]), _yaw_q(False))
    ident = (0.0, 0.0, 0.0, 1.0)
    out["basket"] = (np.array([L.BASKET_XY[0], L.BASKET_XY[1], z]), ident)
    out["repair_box"] = (np.array([L.BOX_XY[0], L.BOX_XY[1], z]), ident)
    return out


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put every body at its place for this seed, let them settle, return the first observation."""
    import torch
    from yam_arena import inspect_embodiment as ie
    from yam_arena import scene as _scene

    seed = int(os.environ.get("TOY_VARIANT_SEED", seed if seed is not None else 0))
    base = emb._base
    _blank()
    for name, (pos, quat) in poses(seed).items():
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(np.asarray(pos, float), dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(getattr(ie, "SETTLE_S", 0.5) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    _blank()
    slot, turned = variant(seed)
    _EP["variant"] = f"seed {seed}: loose wheel at the {loose_corner(slot, turned)}"
    st = _scene.object_states(emb._env, NAMES)
    _EP["st"] = st
    if _DBG is not None:
        _DBG.write(_EP["variant"] + "\n")
        _debug(st)
    print(f"toydemo: {_EP['variant']}", flush=True)
    return emb._observation()


# -- the scripted demo (reads object poses and the tugged wheel's travel from the simulator to plan; moves the parts
#    only through the grippers, by contact) ---------------------------------------------------------------------------
VARIANT = os.environ.get("DEMO_VARIANT", "")  # "" (full), allbasket, allbox, fixedguess, nothing, onearm (README)
WHEEL_OPEN, WHEEL_SQUEEZE = -(L.WHEEL_R + 0.007), -(L.WHEEL_R - 0.003)  # 3 mm past contact on the 31.75 mm wheel
BODY_OPEN, BODY_SQUEEZE = -0.031, -(L.BODY_W / 2 - 0.003)  # 3 mm past contact on the 38 mm body
PICK_PITCH = math.radians(35.0)
TILT = math.radians(L.TUG_TILT_DEG)


def ready_pose(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, seconds: float, finger: float = FINGER_OPEN, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return Waypoint(seconds, tuple(p.tolist()), finger, label, r)


def rest(arm: str, seconds: float) -> Waypoint:
    """The end pose: the ready pose drawn back 12 cm and out 4 cm."""
    p, r = ready_pose(arm)
    side = 1.0 if arm == "left" else -1.0
    return Waypoint(seconds, tuple((p + np.array([-0.12, 0.04 * side, 0.0])).tolist()), FINGER_OPEN,
                    "back to the ready pose", r)


def demo():
    """Both arms wait at the ready pose; the plan is made stage by stage from the first control step (toydemo.py)."""
    return {arm: [park(arm, 0.5, label="look")] for arm in ("left", "right")}


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) / np.linalg.norm(close)
    x = x - z * float(x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _wp(sec, pos, finger, label, rot) -> Waypoint:
    return Waypoint(float(sec), tuple(np.asarray(pos, float).tolist()), float(finger), label, np.asarray(rot, float))


def _hold(w: Waypoint, sec: float, label: str | None = None) -> Waypoint:
    return Waypoint(float(sec), w.pos, w.finger, label or w.label, w.yaw)


def tug_moves(arm: str, car_p: np.ndarray, axle_x_world: float, first: bool) -> list[Waypoint]:
    """Pinch this arm's wheel on the axle at world x ``axle_x_world`` across its tread (front and back), pull it 10 mm
    outward along the axle, let go and back up."""
    s = 1.0 if arm == "left" else -1.0
    a = np.array([0.0, -s * math.sin(TILT), -math.cos(TILT)])  # down, leaning in from outside
    close = (-1.0, 0.0, 0.0) if arm == "left" else (1.0, 0.0, 0.0)  # the sign the IK reaches (dev/signs.py)
    r = _frame(a, close)
    c = np.array([axle_x_world, car_p[1] + s * (L.WHEEL_Y + 0.002), TOP + L.WHEEL_R])
    tcp = c - a * L.TCP_ABOVE_AXLE
    above = tcp - a * 0.06
    out = tcp + np.array([0.0, s * L.TUG_PULL, 0.0])
    return [
        _wp(3.0 if first else 1.4, above, WHEEL_OPEN, "over the wheel", r),
        _wp(1.0, tcp, WHEEL_OPEN, "fingers either side of the wheel", r),
        _wp(0.6, tcp, WHEEL_SQUEEZE, "pinch the wheel", r),
        _wp(1.0, out, WHEEL_SQUEEZE, "pull the wheel", r),
        _wp(0.4, out, WHEEL_SQUEEZE, "pull the wheel", r),
        _wp(0.4, out, WHEEL_OPEN, "let go of the wheel", r),
        _wp(0.6, out - a * 0.06, WHEEL_OPEN, "back off", r),
    ]


def _pick_rot() -> np.ndarray:
    a = np.array([math.sin(PICK_PITCH), 0.0, -math.cos(PICK_PITCH)])
    return _frame(a, (0.0, -1.0, 0.0))  # the sign the IK reaches (dev/signs.py)


def place_moves(arm: str, car_p: np.ndarray, spot, drop_extra: float = 0.0) -> list[Waypoint]:
    """Grip the car's body across its width from above (hand leaning forward 35 degrees), lift it over the walls,
    lower it into the box at ``spot`` and let go."""
    r = _pick_rot()
    a = r[:, 2]
    grip = np.array([car_p[0], car_p[1], TOP + L.PICK_TCP_Z])
    over_car = np.array([car_p[0], car_p[1], TOP + L.LIFT_Z])
    over = np.array([spot[0], spot[1], TOP + L.LIFT_Z])
    down = np.array([spot[0], spot[1], TOP + L.FLOOR + L.DROP_Z + drop_extra])
    return [
        _wp(2.2, grip - a * 0.07, BODY_OPEN, "over the car", r),
        _wp(1.0, grip, BODY_OPEN, "fingers either side of the car", r),
        _wp(0.6, grip, BODY_SQUEEZE, "hold the car", r),
        _wp(1.2, over_car, BODY_SQUEEZE, "lift the car", r),
        _wp(2.0, over, BODY_SQUEEZE, "over the box", r),
        _wp(1.2, down, BODY_SQUEEZE, "lower into the box", r),
        _wp(0.5, down, BODY_OPEN, "let go of the car", r),
        _wp(0.8, down - a * 0.08 + np.array([0.0, 0.0, 0.04]), BODY_OPEN, "back off up", r),
    ]


class Planner:
    """The demo's stages. Each call to next_stage returns the next moves for both hands, or None when done."""

    def __init__(self):
        self.stage = 0
        self.queue: list = []
        self.found: dict[str, bool] = {}
        self.basket_spots = list(L.BASKET_SPOTS)
        self.started = False
        self.last_tug: tuple | None = None

    def _car_at(self, st, slot: int) -> str:
        y = L.SLOTS_Y[slot]
        return min(CAR_NAMES, key=lambda n: abs(float(st[n].pos[1]) - y) + abs(float(st[n].pos[0]) - L.CAR_X))

    def _travel(self, st, car: str) -> float:
        """The tugged wheels' travel (read from the simulator): only the loose peg can move; a glued wheel reads 0."""
        return wheel_travel(st) if car == "car_bad" else 0.0

    def build(self, st) -> None:
        if VARIANT == "nothing":
            self.queue = [("wait",)]
            return
        cars = [self._car_at(st, i) for i in range(3)]
        if VARIANT == "allbasket":
            spots = [(0.52, -0.235), (0.52, -0.125), (0.52, -0.015)]
            self.queue = [("place", "right", c, spots[i]) for i, c in enumerate(cars)]
        elif VARIANT == "allbox":
            spots = [(0.52, 0.185, 0.0), (0.52, 0.255, 0.0), (0.52, 0.22, 0.06)]
            self.queue = [("place", "left", c, spots[i]) for i, c in enumerate(cars)]
        elif VARIANT == "fixedguess":  # rejects the left car untested, the other two to the basket
            self.queue = [("place", "left", cars[0], L.BOX_SPOT), ("place", "right", cars[1], L.BASKET_SPOTS[1]),
                          ("place", "right", cars[2], L.BASKET_SPOTS[0])]
        else:  # full, onearm: test each car, then place it
            for c in cars:
                self.queue += [("tug", c, +1), ("tug", c, -1), ("decide", c)]
        self.queue.append(("rest",))

    def next_stage(self, st):
        if not self.started:
            self.build(st)
            self.started = True
        while self.queue:
            item = self.queue.pop(0)
            kind = item[0]
            if kind == "wait":
                return {arm: [park(arm, 30.0, label="wait")] for arm in ("left", "right")}
            if kind == "rest":
                return {arm: [rest(arm, 2.0), rest(arm, 1.2)] for arm in ("left", "right")}
            if kind == "tug":
                car, front = item[1], item[2]
                if self.found.get(car):
                    continue  # this car already failed: no more tugs on it
                p = np.asarray(st[car].pos, float)
                ax = p[0] + front * L.AXLE_X
                first = self.last_tug is None or self.last_tug[0] != car
                arms = ("right",) if VARIANT == "onearm" else ("left", "right")
                plan = {arm: tug_moves(arm, p, ax, first) for arm in arms}
                if VARIANT == "onearm":
                    plan["left"] = [park("left", sum(w.duration for w in plan["right"]), label="wait")]
                self.last_tug = (car, front)
                self.queue.insert(0, ("check", car))
                return plan
            if kind == "check":
                car = item[1]
                d = self._travel(st, car)
                print(f"toydemo: {car} wheel travel after the tug {d * 1000:.1f} mm", flush=True)
                if d * 1000 >= L.SLID_MM:
                    self.found[car] = True
                continue
            if kind == "decide":
                car = item[1]
                p = np.asarray(st[car].pos, float)
                self.last_tug = None
                if self.found.get(car):
                    arm, spot = "left", L.BOX_SPOT
                else:
                    arm, spot = "right", self.basket_spots.pop(0) if self.basket_spots else L.BASKET_SPOTS[1]
                print(f"toydemo: {car} -> {'repair box' if arm == 'left' else 'basket'}", flush=True)
                return self._place(arm, p, spot, 0.0)
            if kind == "place":
                arm, car, spot = item[1], item[2], item[3]
                extra = spot[2] if len(spot) > 2 else 0.0
                return self._place(arm, np.asarray(st[car].pos, float), spot[:2], extra)
        return None

    def _place(self, arm, p, spot, extra):
        moves = place_moves(arm, p, spot, extra)
        other = "right" if arm == "left" else "left"
        t = sum(w.duration for w in moves)
        return {arm: moves, other: [rest(other, min(2.0, t)), _hold(rest(other, 0.1), max(0.1, t - 2.0), "wait")]}


PLANNER = Planner()


@bodykit.on_reset
def _new_planner() -> None:
    global PLANNER
    PLANNER = Planner()


def next_stage(st):
    return PLANNER.next_stage(st)


import toydemo  # noqa: E402  (the demo's control hooks for yam_scripted)

toydemo.install()
toydemo_task = True
