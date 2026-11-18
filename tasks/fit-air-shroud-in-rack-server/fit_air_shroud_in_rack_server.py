"""Fit the air shroud in a rack server: tilt it so both front tabs go into the fan cage's slots, under the cage's top
surface, then lower it until it is seated.

A Dell PowerEdge R760 2U chassis (434 mm wide, 722.7 mm from its front face to its rear wall, 86.8 mm high) lies on
the bench between the arms with its system cover off, front panel toward the robot. Inside, from the front: the drive
cage under its closed top, the fan cage, then the open bay over the two processors' heat sinks and four memory rows,
where the air shroud belongs, then the riser cages. The shroud (one rigid black plastic duct, 426 x 165 x 79 mm) rests
on top of the drive cage. Metres; x forward from the arm bases, y left, z up.
"""
import math
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None
DIAG = False  # GPU diagnostic copies print SHROUD_DIAG lines


def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_TIMERS = {}
_EVER = {}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)
        tcp = ik.tcp_pose(q[arm_ids], station.BASES[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def held_for(key, cond):
    """Elapsed time over consecutive observed control steps; repeat calls are harmless."""
    _episode_memory()
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    previous_step, start = _TIMERS.get(key, (None, None))
    if not cond:
        _TIMERS[key] = (step, None)
        return 0.0
    if start is None or step not in (previous_step, previous_step + 1) or t < start:
        start = t
    _TIMERS[key] = (step, start)
    return float(t - start)


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None:
        return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_LIMIT_STEPS = 2400  # 120 s at 20 Hz
DEMO_PACE = 1.0  # scales every demo duration except the fingers' close and open
DEMO_FLAT = False  # the wrong way, for a check run: lower the shroud level straight onto its place, no tuck

MM = 0.001
SHROUD_BLACK, STEEL, CAGE_GREY, BOARD_GREEN = (0.06, 0.06, 0.07), (0.78, 0.79, 0.80), (0.55, 0.56, 0.58), (0.10, 0.30, 0.18)
HEATSINK, DIMM_GREEN, FAN_BLACK, RISER_GREY = (0.74, 0.75, 0.77), (0.12, 0.42, 0.22), (0.16, 0.16, 0.17), (0.66, 0.67, 0.69)
TOUCH_BLUE = (0.36, 0.62, 0.86)

# ---------------------------------------------------------------- the chassis (fixed: it lies on the bench)
CHASSIS_W, CHASSIS_H, CHASSIS_LEN = 0.434, 0.0868, 0.7227  # Dell: 434 wide (no ears), 86.8 high, 22.0 + 700.7 long
SHEET = 0.001  # floor pan and side walls, 1 mm steel
FLOOR = TOP + SHEET  # the floor pan's top: the shroud's end walls stand on it
WALL_TOP = CHASSIS_H - SHEET  # side walls' top above the bench with the system cover off
INNER_Y = CHASSIS_W / 2 - SHEET  # 0.216: the side walls' inner faces
X_FRONT = -0.150  # the front face, 150 mm behind the arm bases' line (the body passes between the bases)
X_REAR = X_FRONT + CHASSIS_LEN  # 0.5727
X_EARS = X_FRONT + 0.022  # rack ears' plane
FRONT_TOP_END = X_FRONT + 0.135  # the drive cage's closed top (front panel to the backplane's cable space)

# The shroud's seated pose: its frame's origin is the middle of its footprint on the floor pan.
X0 = 0.228
SEATED = np.array([X0, 0.0, FLOOR])

# Fan cage (75 mm deep): rear wall 1.5 mm steel facing the shroud, 1 mm from the seated shroud's front strip; its top
# plate's underside 80.5 mm above the floor pan; two slots in the rear wall under the top plate take the shroud's tabs.
CAGE_FACE = X0 - 0.0835  # the rear wall's face toward the shroud (x)
CAGE_WALL_T = 0.0015
CAGE_FRONT = CAGE_FACE - 0.075
LIP_UNDER = 0.0805  # top plate's underside above the floor pan (the slots' top edge)
LIP_T = 0.0015
SLOT_LOW = 0.074  # slots' bottom edge above the floor pan
SLOT_Y, SLOT_W = 0.107, 0.022  # slot centres at y +-0.107, 22 mm wide
FAN_BACK = CAGE_FACE - 0.012  # the fans' back faces (pockets behind the slots)

RISERS_X = X0 + 0.0825 + 0.035  # riser cages' front faces, 35 mm behind the seated shroud's back edge
RISERS_H = 0.080
BOARD_T = 0.007  # system board top above the floor pan (standoffs + board)
BOARD_Y = 0.209

# Board components under the shroud, in the seated shroud's frame (x, y about X0, 0): heat sinks and memory rows.
HS_X, HS_Y, HS_TOP = (-0.050, 0.050), (0.0765, 0.1375), 0.071  # 2U heat sinks, 61 x 100 mm, tops 71 mm up
HS_BASE, HS_FIN_T = 0.015, 0.0008  # the sink's base block (cold plate + heat pipes) up to 15 mm; fins above it
DIMM_X, DIMM_TOP, DIMM_T = (-0.0665, 0.0665), 0.046, 0.005  # DDR5 RDIMMs (133 mm long) standing in their sockets
DIMM_ROWS = ((0.1485, 0.207), (0.0075, 0.0655))  # outer and inner rows of eight, each side


def _b(x0, x1, y0, y1, z0, z1, color):
    """A box from world x, y ranges and heights above the bench (TOP)."""
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)


def _chassis_shapes():
    s = SHEET
    z_floor = SHEET
    shapes = [
        _b(X_FRONT, X_REAR, -CHASSIS_W / 2, CHASSIS_W / 2, 0.0, s, STEEL),  # floor pan
        _b(X_FRONT, X_REAR, INNER_Y, CHASSIS_W / 2, 0.0, WALL_TOP, STEEL),  # left side wall
        _b(X_FRONT, X_REAR, -CHASSIS_W / 2, -INNER_Y, 0.0, WALL_TOP, STEEL),  # right side wall
        _b(X_REAR - s, X_REAR, -INNER_Y, INNER_Y, 0.0, WALL_TOP, STEEL),  # rear wall
        _b(X_EARS - 0.002, X_EARS, CHASSIS_W / 2, 0.241, 0.0, CHASSIS_H, STEEL),  # rack ears
        _b(X_EARS - 0.002, X_EARS, -0.241, -CHASSIS_W / 2, 0.0, CHASSIS_H, STEEL),
        _b(X_FRONT, FRONT_TOP_END, -INNER_Y, INNER_Y, z_floor, CHASSIS_H, STEEL),  # drive cage under its closed top
        # fan cage: six fan modules side by side, top plate, rear wall below the slots, and around the two slots
        _b(CAGE_FRONT, CAGE_FACE, -INNER_Y, INNER_Y, z_floor + LIP_UNDER, z_floor + LIP_UNDER + LIP_T, CAGE_GREY),
        _b(CAGE_FACE - CAGE_WALL_T, CAGE_FACE, -INNER_Y, INNER_Y, z_floor, z_floor + SLOT_LOW, CAGE_GREY),
    ]
    y_cuts = [-INNER_Y, -SLOT_Y - SLOT_W / 2, -SLOT_Y + SLOT_W / 2, SLOT_Y - SLOT_W / 2, SLOT_Y + SLOT_W / 2, INNER_Y]
    for y0, y1 in zip(y_cuts[0::2], y_cuts[1::2]):
        shapes.append(_b(CAGE_FACE - CAGE_WALL_T, CAGE_FACE, y0, y1, z_floor + SLOT_LOW, z_floor + LIP_UNDER, CAGE_GREY))
    # system board, heat sinks, memory rows, riser cages
    shapes.append(_b(CAGE_FACE, RISERS_X, -BOARD_Y, BOARD_Y, z_floor, z_floor + BOARD_T, BOARD_GREEN))
    fan_w = 2 * INNER_Y / 6
    for k in range(6):  # each module 70 mm wide with a 2 mm gap to the next
        y0 = -INNER_Y + k * fan_w
        shapes.append(_b(CAGE_FRONT, FAN_BACK, y0 + 0.001, y0 + fan_w - 0.001, z_floor, z_floor + LIP_UNDER, FAN_BLACK))
    for sgn in (1, -1):
        ys = sorted((sgn * HS_Y[0], sgn * HS_Y[1]))
        shapes.append(_b(X0 + HS_X[0], X0 + HS_X[1], ys[0], ys[1], z_floor + BOARD_T, z_floor + HS_BASE, HEATSINK))
        n_fin = 16  # fins along the airflow, 0.8 mm thick, on a 4.0 mm pitch across the 61 mm sink
        pitch = (ys[1] - ys[0] - HS_FIN_T) / (n_fin - 1)
        for k in range(n_fin):
            y0 = ys[0] + k * pitch
            shapes.append(_b(X0 + HS_X[0], X0 + HS_X[1], y0, y0 + HS_FIN_T, z_floor + HS_BASE, z_floor + HS_TOP,
                             HEATSINK))
        for row in DIMM_ROWS:  # eight modules a row, 5 mm thick (heat spreaders), sockets on a 7.36 mm pitch
            ys = sorted((sgn * row[0], sgn * row[1]))
            pitch = (ys[1] - ys[0] - DIMM_T) / 7
            for k in range(8):
                y0 = ys[0] + k * pitch
                shapes.append(_b(X0 + DIMM_X[0], X0 + DIMM_X[1], y0, y0 + DIMM_T, z_floor + BOARD_T,
                                 z_floor + DIMM_TOP, DIMM_GREEN))
    shapes.append(_b(RISERS_X, X_REAR - s, -INNER_Y + 0.001, INNER_Y - 0.001, z_floor, z_floor + RISERS_H, RISER_GREY))
    return shapes


CHASSIS = Part("chassis", (0.0, 0.0, TOP), _chassis_shapes(), mass=None, color=STEEL, friction=(0.35, 0.30),
               contact_offset=0.001)

# ---------------------------------------------------------------- the air shroud (shroud frame, seated: SEATED)
SHROUD_MASS = 0.30  # estimate (must be measured): 426 x 165 x 79 mm moulded plastic duct, ~2 mm walls
HALF_X, HALF_Y, HEIGHT = 0.0825, 0.213, 0.079
TOP_T, WALL_T = 0.002, 0.002
DUCT_A, DUCT_B = 0.0705, 0.1435  # duct walls' centre lines (y), either side of each heat sink
DUCT_BOTTOM = 0.010  # duct walls stop 3 mm above the board
WING_FLOOR = 0.050  # each wing (over the outer memory row) is a trough, its floor 4 mm above the modules
WING_Y = (0.1445, 0.211)
GRIP_WALL_T = 0.0025  # the wing's back wall (the robot's side away: toward the risers), gripped across its thickness
TAB_Y, TAB_W, TAB_L = 0.107, 0.016, 0.007  # two front tabs, 16 mm wide, 7 mm long, on the top's front edge


def _sb(x0, x1, y0, y1, z0, z1, color=SHROUD_BLACK):
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)


def _shroud_shapes():
    hx, h = HALF_X, HEIGHT
    sh = [
        _sb(-hx, hx, -WING_Y[0], WING_Y[0], h - TOP_T, h),  # top over the heat sinks and inner memory
        _sb(-hx, -hx + WALL_T, -WING_Y[0], WING_Y[0], h - 0.010, h - TOP_T),  # front strip
        _sb(hx - WALL_T, hx, -WING_Y[0], WING_Y[0], h - 0.010, h - TOP_T),  # back strip
    ]
    for s in (1, -1):
        sh += [
            _sb(-hx, hx, *sorted((s * (DUCT_A - WALL_T / 2), s * (DUCT_A + WALL_T / 2))), DUCT_BOTTOM, h - TOP_T),
            _sb(-hx, hx, *sorted((s * (DUCT_B - WALL_T / 2), s * (DUCT_B + WALL_T / 2))), DUCT_BOTTOM, h),
            _sb(-hx, hx, *sorted((s * (HALF_Y - WALL_T), s * HALF_Y)), 0.0, h),  # end wall: stands on the floor pan
            _sb(-hx, hx - GRIP_WALL_T, *sorted((s * WING_Y[0], s * WING_Y[1])), WING_FLOOR, WING_FLOOR + TOP_T),
            _sb(hx - GRIP_WALL_T, hx, *sorted((s * WING_Y[0], s * WING_Y[1])), WING_FLOOR + TOP_T, h),  # grip wall
            _sb(-hx, -hx + WALL_T, *sorted((s * WING_Y[0], s * WING_Y[1])), WING_FLOOR + TOP_T, h),
            _sb(-hx - TAB_L, -hx, *sorted((s * (TAB_Y - TAB_W / 2), s * (TAB_Y + TAB_W / 2))), h - TOP_T, h),  # tab
        ]
    return sh


START = np.array([X_FRONT + 0.105, 0.0, TOP + CHASSIS_H])  # on the drive cage's closed top, end walls down
SHROUD_FRICTION = (0.45, 0.35)  # moulded plastic on steel (estimate)
SHROUD = Part("shroud", tuple(START), _shroud_shapes(), mass=SHROUD_MASS, color=SHROUD_BLACK,
              friction=SHROUD_FRICTION, contact_offset=0.001)
OBJECTS = [CHASSIS, SHROUD]

# Shroud-frame reference points
TAB_TIPS = np.array([(-HALF_X - TAB_L, s * TAB_Y, HEIGHT - TOP_T / 2) for s in (1, -1)])  # left, right
TAB_TOPS = np.array([[(-HALF_X - TAB_L, s * TAB_Y, HEIGHT), (-HALF_X, s * TAB_Y, HEIGHT)] for s in (1, -1)])
FEET = np.array([(x, s * (HALF_Y - WALL_T / 2), 0.0) for x in (-HALF_X, HALF_X) for s in (1, -1)])  # end walls' corners
REF_POINTS = np.array([(-HALF_X, HALF_Y, HEIGHT), (-HALF_X, -HALF_Y, HEIGHT), (HALF_X, 0.0, HEIGHT)])

# ---------------------------------------------------------------- scoring
TAB_PAST = 0.003  # a tab is in its slot when its tip is >= 3 mm past the cage's rear face ...
TAB_SIDE = SLOT_W / 2  # ... between the slot's sides ...
TAB_UNDER = 0.0005  # ... and its top no higher than the cage top's underside (+0.5 mm contact tolerance)
SEAT_HIGH, SEAT_LOW = 0.002, 0.002  # every end-wall corner within 2 mm of the floor pan
STILL_WINDOW, STILL_MOVE = 0.5, 0.0010
LINEAR_SPEED, ANGULAR_SPEED = 0.010, 0.5
DROP_SPEED = 0.60


def pose(sh):
    return np.asarray(sh.origin, float), np.asarray(sh.rotation(), float)


def world(sh, local):
    o, rot = pose(sh)
    return o + np.asarray(local, float) @ rot.T


def tab_terms(sh):
    tips = world(sh, TAB_TIPS)
    out = []
    for i, s in enumerate((1, -1)):
        tops = world(sh, TAB_TOPS[i])
        past = float(CAGE_FACE - tips[i][0])
        side = float(tips[i][1] - s * SLOT_Y)
        above = float(max(tops[:, 2]) - (FLOOR + LIP_UNDER))
        ok = past >= TAB_PAST and abs(side) <= TAB_SIDE and above <= TAB_UNDER
        out.append({"in": bool(ok), "past": past, "side": side, "above": above})
    return out


def seat_terms(sh):
    gaps = world(sh, FEET)[:, 2] - FLOOR
    return {"seated": bool(np.all((gaps <= SEAT_HIGH) & (gaps >= -SEAT_LOW))), "gaps": gaps}


def released_shroud(sh, clear=RELEASE_CLEAR):
    """Both grippers' centre points at least `clear` outside the shroud's own box (in its frame)."""
    hands = grippers()
    if hands is None:
        return True
    o, rot = pose(sh)
    low = np.array([-HALF_X - TAB_L, -HALF_Y, 0.0])
    high = np.array([HALF_X, HALF_Y, HEIGHT])
    for tcp, _ in hands.values():
        p = rot.T @ (np.asarray(tcp, float) - o)
        if float(np.linalg.norm(np.maximum(np.maximum(low - p, p - high), 0.0))) < clear:
            return False
    return True


_WINDOW = {"episode": None, "hist": deque()}


def at_rest(sh):
    t = _RUN["t"]
    if t is None:
        return bool(np.linalg.norm(sh.lin_vel) < LINEAR_SPEED and np.linalg.norm(sh.ang_vel) < ANGULAR_SPEED)
    hist = _WINDOW["hist"]
    if _WINDOW["episode"] != _RUN["episode"] or (hist and t < hist[-1][0]):
        hist.clear()
        _WINDOW["episode"] = _RUN["episode"]
    ref = world(sh, REF_POINTS)
    if not hist or t > hist[-1][0] + 1e-9:
        hist.append((t, ref))
    while len(hist) > 1 and hist[1][0] <= t - STILL_WINDOW + 1e-9:
        hist.popleft()
    if t - hist[0][0] < STILL_WINDOW - 0.051:
        return False
    return all(float(np.max(np.linalg.norm(r - ref, axis=1))) <= STILL_MOVE for _, r in hist)


def off_chassis(sh):
    o = np.asarray(sh.origin, float)
    return bool(abs(o[1]) > CHASSIS_W / 2 + 0.05 or not X_FRONT - 0.05 <= o[0] <= X_REAR or o[2] < TOP - 0.01)


def score_terms(objects):
    sh = objects["shroud"]
    tabs = tab_terms(sh)
    seat = seat_terms(sh)
    both = all(t["in"] for t in tabs)
    dropped = ever("shroud dropped (fell faster than 0.6 m/s)", float(-sh.lin_vel[2]) > DROP_SPEED)
    fell = ever("shroud off the chassis", off_chassis(sh))
    free = released_shroud(sh)
    rest = at_rest(sh)
    full_s = held_for("both tabs in, seated, released, at rest", both and seat["seated"] and free and rest)
    part_s = held_for("both tabs in, released, at rest", both and free and rest)
    if DIAG:
        _diag(sh, tabs, seat, free, rest)
    return {"tabs": tabs, "seat": seat, "both": both, "dropped": dropped, "fell": fell, "released": free,
            "at_rest": rest, "full_s": full_s, "part_s": part_s}


def score(objects):
    t = score_terms(objects)
    if t["dropped"] or t["fell"]:
        return 0.0
    if t["both"] and t["seat"]["seated"] and t["full_s"] + 1e-10 >= STILL_S:
        return 1.0
    if t["both"] and t["seat"]["seated"]:
        return 0.5
    if t["both"] and t["part_s"] + 1e-10 >= STILL_S:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- diagnostics (GPU copies only)
_DIAG = {"step": -1}


def _rpy(rot):
    pitch = math.degrees(math.asin(float(np.clip(-rot[2, 0], -1, 1))))
    roll = math.degrees(math.atan2(rot[2, 1], rot[2, 2]))
    yaw = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
    return roll, pitch, yaw


def _diag(sh, tabs, seat, free, rest):
    step = _RUN["step"]
    if step == _DIAG["step"] or step % 5:
        return
    _DIAG["step"] = step
    o, rot = pose(sh)
    r, p, y = _rpy(rot)
    hands = grippers() or {}
    tcp = " ".join(f"{a[0]}=({','.join(f'{v * 1000:.1f}' for v in hands[a][0])};{hands[a][1]:.2f})" for a in hands)
    tb = " ".join(f"tab{i}(in={int(t['in'])},past={t['past'] * 1000:.1f},side={t['side'] * 1000:.1f},"
                  f"above={t['above'] * 1000:.1f})" for i, t in enumerate(tabs))
    print(f"SHROUD_DIAG t={_RUN['t']:.2f} o=({','.join(f'{v * 1000:.1f}' for v in (o - SEATED))}) "
          f"rpy=({r:.2f},{p:.2f},{y:.2f}) v={np.linalg.norm(sh.lin_vel) * 1000:.0f} {tb} "
          f"feet=({','.join(f'{g * 1000:.1f}' for g in seat['gaps'])}) free={int(free)} rest={int(rest)} {tcp} "
          f"F={_hand_forces()}", flush=True)


def _hand_forces():
    """Each hand's force on what it holds (N, world x, y, z), from the joints' PD error (the arms run PD plus their own
    gravity compensation, so the error carries the load): J^T F = K (q_target - q). Diagnostic only."""
    env = _RUN["env"]
    if env is None:
        return "-"
    try:
        from yam_arena import ik, station
        from yam_arena.constants import ARM_JOINTS, SHOULDER, WRIST
        k = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
        out = []
        for arm in ("left", "right"):
            art = env.unwrapped.scene[arm + "_robot"]
            ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = art.data.joint_pos[0][ids].detach().cpu().numpy().astype(float)
            qt = art.data.joint_pos_target[0][ids].detach().cpu().numpy().astype(float)
            tau = k * (qt - q)
            p0, r0 = ik.tcp_pose(q, station.BASES[arm])
            jac = np.zeros((6, 6))
            for j in range(6):
                dq = q.copy()
                dq[j] += 1e-5
                p1, r1 = ik.tcp_pose(dq, station.BASES[arm])
                ax, a = _axis_angle(r1 @ r0.T)
                jac[:3, j] = (p1 - p0) / 1e-5
                jac[3:, j] = ax * a / 1e-5
            w = np.linalg.lstsq(jac.T, tau, rcond=None)[0]
            out.append(f"{arm[0]}({w[0]:.1f},{w[1]:.1f},{w[2]:.1f})")
        return " ".join(out)
    except Exception as e:  # noqa: BLE001
        return f"err:{type(e).__name__}"


# ---------------------------------------------------------------- demo
# Both hands at once, one pinch on each wing's back wall (the edge away from the robot), fingers closing across the
# wall along x, so the two hands carry the shroud like a tray by its back edge. Lift it off the drive cage, carry it
# back over the bay while tilting its front (tab) edge down 8 degrees, lower it tilted into the bay 12 mm short of
# the fan cage, draw it toward the cage so both tabs go into their slots under the cage top, then swing the back
# edge down about the tabs until the shroud lies level on its seats; let go and lift clear.
GRIP_Y = (WING_Y[0] + WING_Y[1]) / 2  # 177.75 mm: the middle of each wing
GRIP_LOCAL = {"left": np.array([HALF_X - GRIP_WALL_T / 2, GRIP_Y, WING_FLOOR + 0.019]),
              "right": np.array([HALF_X - GRIP_WALL_T / 2, -GRIP_Y, WING_FLOOR + 0.019])}  # pad tips 4 mm off the floor
YAWS = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along x, across the wall
GRIP_OPEN = -0.0110  # pads 22 mm apart around the 2.5 mm wall (both hands fit between the risers and the trough)
TILT_DEG = 8.0
BACK_OFF = 0.012  # lowered 12 mm short of the cage, then drawn forward
LIFT = 0.025
PARK = {"left": np.array([0.150, 0.300, TOP + 0.200]), "right": np.array([0.150, -0.300, TOP + 0.200])}
CORRECTION_MAX, CORRECTION_MAX_DEG = 0.040, 10.0  # a read corrects at most this much (a bad read cannot fling it)
GRASP_MAX, GRASP_MAX_DEG = 0.025, 8.0  # the hands follow the shroud's start pose by at most this much
FINAL_HOLD_S = 3.0
_READS = {"episode": None, "values": {}}
PIVOT = np.array([-HALF_X - TAB_L, 0.0, HEIGHT - TOP_T / 2])  # the tabs' tips line (shroud frame)


def _rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def tilted(origin_of_pivot, deg):
    """(origin, rotation) of the shroud tipped `deg` front-edge-down about its tabs' tips line, that line's middle at
    the given world point (positive deg lowers the front, toward the robot)."""
    rot = _rot_y(-deg)  # about +y by -deg: points at -x (the front) go down
    origin = np.asarray(origin_of_pivot, float) - rot @ PIVOT
    return origin, rot


def _shroud_now():
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        st = arena_scene.object_states(env, ["shroud"])["shroud"]
        return np.asarray(st.origin, float), np.asarray(st.rotation(), float)
    except Exception as e:  # noqa: BLE001
        print(f"SHROUD_READ failed: {e!r}", flush=True)
        return None


def _axis_angle(rot):
    c = (np.trace(rot) - 1.0) / 2.0
    a = math.acos(float(np.clip(c, -1.0, 1.0)))
    if a < 1e-9:
        return np.array([0.0, 0.0, 1.0]), 0.0
    axis = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]) / (2 * math.sin(a))
    return axis, a


def _aa_matrix(axis, a):
    x, y, z = axis
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(a) * k + (1.0 - math.cos(a)) * (k @ k)


def _grasp(planned):
    """Before the grasp: the hands follow the shroud to where it actually rests (C = T_actual T_planned^-1), at most
    25 mm and 8 degrees, so the pads straddle its walls however it settled."""
    if _READS["episode"] != _RUN["episode"]:
        _READS["episode"], _READS["values"] = _RUN["episode"], {}
    values = _READS["values"]
    if "grasp" not in values:
        now = _shroud_now()
        if now is None:
            values["grasp"] = (np.eye(3), np.zeros(3))
        else:
            (po, pr), (ao, ar) = planned, now
            dR = ar @ pr.T
            axis, a = _axis_angle(dR)
            dR = _aa_matrix(axis, min(a, math.radians(GRASP_MAX_DEG)))
            move = ao - po
            n = float(np.linalg.norm(move))
            if n > GRASP_MAX:
                move *= GRASP_MAX / n
            values["grasp"] = (dR, po + move - dR @ po)  # turns about the planned origin, then shifts
            print(f"SHROUD_READ grasp t={_RUN['t']:.2f} start_off_mm={tuple(round(float(v) * 1000, 2) for v in ao - po)} "
                  f"rot_deg={math.degrees(a):.2f} rpy_act={tuple(round(v, 2) for v in _rpy(ar))}", flush=True)
    return values["grasp"]


def _correction(key, planned, prior):
    """World transform C (R, t) applied to every later hand target: C_new = T_planned T_actual^-1 C_old (the part
    held where it was planned, assuming the grip holds), the new part clamped to 40 mm and 10 degrees."""
    if key == "grasp":
        return _grasp(planned)
    if _READS["episode"] != _RUN["episode"]:
        _READS["episode"], _READS["values"] = _RUN["episode"], {}
    values = _READS["values"]
    if key not in values:
        Rc, tc = prior() if prior else (np.eye(3), np.zeros(3))
        now = _shroud_now()
        if now is None:
            values[key] = (Rc, tc)
        else:
            (po, pr), (ao, ar) = planned, now
            dR = pr @ ar.T
            axis, a = _axis_angle(dR)
            a = min(a, math.radians(CORRECTION_MAX_DEG))
            dR = _aa_matrix(axis, a)
            # pivot the correction about the tab line's middle so the rotation does not swing the tabs
            pa = ao + ar @ PIVOT
            pp = po + pr @ PIVOT
            dt = pp - dR @ pa
            n = float(np.linalg.norm(pp - pa))
            if n > CORRECTION_MAX:
                dt = dt - (pp - pa) + (pp - pa) * CORRECTION_MAX / n
            values[key] = (dR @ Rc, dR @ tc + dt)
            print(f"SHROUD_READ {key} t={_RUN['t']:.2f} tabline_err_mm={tuple(round(float(v) * 1000, 2) for v in pp - pa)} "
                  f"rot_err_deg={math.degrees(_axis_angle(pr @ ar.T)[1]):.2f} rpy_act={tuple(round(v, 2) for v in _rpy(ar))}",
                  flush=True)
    return values[key]


class Grip(float):
    def __new__(cls, yaw, matrix=None):
        value = super().__new__(cls, yaw)
        value.grip_matrix = None if matrix is None else np.array(matrix, float)
        return value


def _install_grip_matrix():
    from yam_arena import demo as arena_demo
    original = arena_demo.grip_rotation
    if getattr(original, "_grip_matrix", False):
        return

    def grip_rotation(yaw=0.0):
        matrix = getattr(yaw, "grip_matrix", None)
        return matrix.copy() if matrix is not None else original(float(yaw))

    grip_rotation._grip_matrix = True
    grip_rotation.__wrapped__ = original
    arena_demo.grip_rotation = grip_rotation


_install_grip_matrix()


class ShroudWaypoint(Waypoint):
    """A hand waypoint worked out when played: the planned shroud pose, the grip, and the read corrections."""

    def __init__(self, duration, arm, origin, rot, finger, label, key, prior_keys, keep=False, lift=0.0):
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.arm, self.origin, self.rot, self.finger, self.label = arm, np.asarray(origin, float), rot, finger, label
        self.key, self.lift = key, lift
        g0 = __import__("yam_arena.demo", fromlist=["grip_rotation"]).grip_rotation(YAWS[arm])
        self._g0 = g0

    def _corr(self):
        return _correction_chain(self.key)

    @property
    def pos(self):
        Rc, tc = self._corr()
        p = self.origin + self.rot @ GRIP_LOCAL[self.arm] + np.array([0.0, 0.0, self.lift])
        return tuple(float(v) for v in Rc @ p + tc)

    @property
    def yaw(self):
        Rc, _ = self._corr()
        return Grip(YAWS[self.arm], Rc @ self.rot @ self._g0)


_PLANNED = {}


def _correction_chain(key):
    if key is None:
        return np.eye(3), np.zeros(3)
    order = list(_PLANNED)
    i = order.index(key)
    prior = (lambda k=order[i - 1]: _correction_chain(k)) if i > 0 else None
    return _correction(key, _PLANNED[key], prior)


def demo_poses():
    """[(label, seconds, shroud origin, shroud rotation, finger, read key, keep, hand lift)]."""
    O, C = GRIP_OPEN, FINGER_CLOSED
    I = np.eye(3)
    start = START.copy()
    lifted = start + np.array([0.0, 0.0, LIFT])
    tab_seat = SEATED + PIVOT  # the tab line's middle when seated (world)
    pre = tab_seat + np.array([BACK_OFF, 0.0, 0.0])
    if DEMO_FLAT:
        above = SEATED + np.array([0.0, 0.0, 0.095])
        carry_o = np.array([above[0], 0.0, max(lifted[2], above[2])])
        return [
            ("hands over the wings' back walls", 2.5, start, I, O, "grasp", False, 0.032),
            ("pads down beside the walls", 1.5, start, I, O, "grasp", False, 0.0),
            ("pinch both walls", 1.2, start, I, C, "grasp", True, 0.0),
            ("settle the pinch", 0.4, start, I, C, "grasp", False, 0.0),
            ("lift off the drive cage", 1.5, lifted, I, C, "grasp", False, 0.0),
            ("hold lifted", 0.6, lifted, I, C, "grasp", False, 0.0),
            ("carry over the bay, level", 3.0, carry_o, I, C, "lifted", False, 0.0),
            ("lower above the bay", 1.5, above, I, C, "lifted", False, 0.0),
            ("hold above the bay", 0.6, above, I, C, "lifted", False, 0.0),
            ("lower level onto its place", 3.0, SEATED + np.array([0.0, 0.0, 0.0005]), I, C, "above", False, 0.0),
            ("hold", 0.6, SEATED + np.array([0.0, 0.0, 0.0005]), I, C, "above", False, 0.0),
            ("let go", 1.0, SEATED, I, O, "above", True, 0.0),
            ("rise clear", 1.5, SEATED, I, O, "above", False, 0.060),
        ]
    o_hi, r_t = tilted(pre + np.array([0.0, 0.0, 0.090]), TILT_DEG)  # end walls' front corners above the side walls
    o_mid, _ = tilted(pre + np.array([0.0, 0.0, 0.025]), TILT_DEG)  # tabs 25 mm above slot height (above the cage top)
    o_pre, _ = tilted(pre + np.array([0.0, 0.0, -0.0005]), TILT_DEG)  # tab tips mid-slot, 6 mm short of the cage
    o_in, _ = tilted(tab_seat + np.array([0.0, 0.0, -0.0005]), TILT_DEG)  # tab tips 6 mm through the cage wall
    levels = [tilted(tab_seat + np.array([0.0, 0.0, -0.0005 + 0.001 * f]), TILT_DEG * (1 - f)) for f in (1 / 3, 2 / 3, 1.0)]
    return [
        ("hands over the wings' back walls", 2.5, start, I, O, "grasp", False, 0.032),
        ("pads down beside the walls", 1.5, start, I, O, "grasp", False, 0.0),
        ("pinch both walls", 1.2, start, I, C, "grasp", True, 0.0),
        ("settle the pinch", 0.4, start, I, C, "grasp", False, 0.0),
        ("lift off the drive cage", 1.5, lifted, I, C, "grasp", False, 0.0),
        ("hold lifted", 0.6, lifted, I, C, "grasp", False, 0.0),
        ("carry back over the bay, tipping the front down", 4.0, o_hi, r_t, C, "lifted", False, 0.0),
        ("hold over the bay", 0.8, o_hi, r_t, C, "lifted", False, 0.0),
        ("lower tilted into the bay", 3.0, o_mid, r_t, C, "over", False, 0.0),
        ("hold, tabs 25 mm above the slots", 0.6, o_mid, r_t, C, "over", False, 0.0),
        ("lower tilted, tabs to slot height", 2.0, o_pre, r_t, C, "mid", False, 0.0),
        ("hold short of the cage", 0.6, o_pre, r_t, C, "mid", False, 0.0),
        ("draw forward: tabs into the slots", 2.0, o_in, r_t, C, "pre", False, 0.0),
        ("swing the back down about the tabs (1)", 1.2, levels[0][0], levels[0][1], C, "pre", False, 0.0),
        ("swing the back down about the tabs (2)", 1.2, levels[1][0], levels[1][1], C, "pre", False, 0.0),
        ("swing the back down about the tabs (3)", 1.2, levels[2][0], levels[2][1], C, "pre", False, 0.0),
        ("hold seated", 0.6, levels[2][0], levels[2][1], C, "pre", False, 0.0),
        ("let go", 1.0, levels[2][0], levels[2][1], O, "pre", True, 0.0),
        ("rise clear", 1.5, levels[2][0], levels[2][1], O, "pre", False, 0.060),
    ]


def demo():
    plan = demo_poses()
    _PLANNED.clear()
    for i, (label, secs, o, r, f, key, keep, lift) in enumerate(plan):
        if key and key not in _PLANNED:  # read when this waypoint starts: the shroud is at the previous one
            j = max(i - 1, 0)  # (the grasp read, at the first waypoint, is against the start)
            _PLANNED[key] = (np.asarray(plan[j][2], float), np.asarray(plan[j][3], float))
    out = {}
    for arm in ("left", "right"):
        wps = [ShroudWaypoint(secs, arm, o, r, f, label, key, None, keep, lift)
               for label, secs, o, r, f, key, keep, lift in plan]
        wps.append(Waypoint(2.0 * DEMO_PACE, tuple(float(v) for v in PARK[arm]), GRIP_OPEN, "park", YAWS[arm]))
        wps.append(Waypoint(FINAL_HOLD_S, tuple(float(v) for v in PARK[arm]), GRIP_OPEN, "hold clear", YAWS[arm]))
        out[arm] = wps
    return out


# ---------------------------------------------------------------- offline test layouts (states.yaml)
def _quat(rot):
    from yam_arena.ik import matrix_to_quat_xyzw
    return [float(v) for v in matrix_to_quat_xyzw(np.asarray(rot, float))]


def tipped(deg):
    """Seated with both tabs in, the back raised by turning `deg` front-down about the tabs' tips line."""
    o, rot = tilted(SEATED + PIVOT, deg)
    return [float(v) for v in o], _quat(rot)


def on_lip():
    """Lowered level onto its place the wrong way: both tabs resting on the fan cage's top, back feet on the floor."""
    foot = np.array([HALF_X, 0.0, 0.0])
    tab_bottom = np.array([-HALF_X - TAB_L / 2, 0.0, HEIGHT - TOP_T])
    rel = tab_bottom - foot
    lip_top = LIP_UNDER + LIP_T
    best = None
    for k in range(20001):  # front-up angle so the tab's underside sits on the cage top
        ang = math.radians(k * 0.0005)
        rot = _rot_y(math.degrees(ang))
        z = (rot @ rel)[2]
        if best is None or abs(z - lip_top) < best[0]:
            best = (abs(z - lip_top), rot)
    rot = best[1]
    o = SEATED + foot - rot @ foot
    return [float(v) for v in o], _quat(rot)


def racked(deg):
    """Turned `deg` about a vertical line through the left tab's tip: the right tab backs out of its slot."""
    a = math.radians(deg)
    rot = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
    piv = SEATED + TAB_TIPS[0] * np.array([1.0, 1.0, 0.0])
    o = piv + rot @ (SEATED - piv)
    return [float(v) for v in o], _quat(rot)
