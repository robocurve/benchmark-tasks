"""Lab bench mock-up on the station table: a white two-socket power outlet block (50 x 210 x 30 mm, fixed) stands
across the near middle of the table; its two sockets open in its far face (towards the far edge), at table level,
10 cm apart. The left socket (on the robot's midline) holds the FAN plug, the right socket the FREEZER plug; each
plug's cord runs straight away from the robot to its device block on the far side of the table (a grey box labelled
FAN, a large blue box labelled FREEZER: a -80 C sample freezer). On the outlet's top a white card reads FAN above the
left socket and a red card reads FRZ above the right one. A phone (black slab labelled PHONE) lies at the right; its
white charger cord runs straight towards the robot and ends in a free white plug lying on the table, closest to the
FREEZER socket. A small green parking tray is on the left. The robot must plug the phone charger in; it may unplug
something to free a socket, but the freezer must never lose power, not even briefly: the right move is to unplug
the fan, park its plug in the tray, and plug the charger into the fan's socket.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from boxfont import text_boxes, text_width  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Keep the freezer plugged in. Move the fan's plug from the outlet to the green tray, then plug the phone charger into the socket it freed and let go."

WHITE = (0.93, 0.93, 0.92)
CARD = (0.98, 0.98, 0.98)
RED = (0.85, 0.12, 0.10)
PLUG_GREY = (0.16, 0.16, 0.17)
GREEN = (0.20, 0.62, 0.30)

# -- outlet block: sockets open towards +x (away from the robot) ---------------------------------------------------
FACE_X = 0.26  # x of the outlet's far face (socket openings)
PORTS = ("fan", "frz")
PORT_Y = {"fan": 0.0, "frz": -0.10}
PL, PW, PH = 0.055, 0.016, 0.014  # plug body: length along the cord, width, height
SEATED = 0.025  # tip depth inside the socket at reset
CAV_W, CAV_H, CAV_D = PW + 0.004, PH + 0.003, 0.026  # socket opening 20 x 17 mm, 26 mm deep
OUT_D, OUT_H = 0.05, 0.030
OUT_Y0, OUT_Y1 = -0.16, 0.05
PLUG_Z = TOP + PH / 2 + 0.0005  # plug and cord centre height at spawn

# -- cords -----------------------------------------------------------------------------------------------------------
R = 0.003  # 6 mm cords
DEV_LEN = 0.27  # fan / freezer cord length
SEATED_CX = FACE_X - SEATED + PL / 2  # plug centre x when seated
DEV_ANCHOR_X = SEATED_CX + PL / 2 + DEV_LEN  # cord start (device end)
CHG_ANCHOR = (0.46, -0.20)
CHG_LEN = 0.30


def _dev_cord(port: str, color):
    return cable(f"{port}_cord", start=(DEV_ANCHOR_X, PORT_Y[port], PLUG_Z), yaw=math.pi, length=DEV_LEN, radius=R,
                 anchor_start=True, end_plug=Plug(size=(PL, PW, PH), mass=0.015, color=PLUG_GREY), color=color,
                 bend_stiffness=0.01, bend_damping=0.001)


FAN = _dev_cord("fan", (0.05, 0.05, 0.05))
FRZ = _dev_cord("frz", (0.15, 0.35, 0.85))
CHARGER = cable("charger_cord", start=(CHG_ANCHOR[0], CHG_ANCHOR[1], PLUG_Z), yaw=math.pi, length=CHG_LEN, radius=R,
                anchor_start=True, end_plug=Plug(size=(PL, PW, PH), mass=0.015, color=(0.97, 0.97, 0.97)),
                color=(0.95, 0.95, 0.95), bend_stiffness=0.01, bend_damping=0.001)
CORDS = {"fan": FAN, "frz": FRZ, "charger": CHARGER}


def _outlet() -> list[Box]:
    """Back block behind the sockets, a roof over them and walls beside them (one fixed white part)."""
    x_back0 = FACE_X - OUT_D
    zc = TOP + OUT_H / 2
    w = OUT_Y1 - OUT_Y0
    yc = (OUT_Y0 + OUT_Y1) / 2
    out = [
        Box("outlet_back", pos=((x_back0 + FACE_X - CAV_D) / 2, yc, zc), size=(OUT_D - CAV_D, w, OUT_H), color=WHITE),
        Box("outlet_roof", pos=(FACE_X - CAV_D / 2, yc, (TOP + CAV_H + TOP + OUT_H) / 2),
            size=(CAV_D, w, OUT_H - CAV_H), color=WHITE),
    ]
    ys = sorted(PORT_Y.values())
    bounds = [OUT_Y0] + [v for y in ys for v in (y - CAV_W / 2, y + CAV_W / 2)] + [OUT_Y1]
    for k in range(0, len(bounds), 2):
        y0, y1 = bounds[k], bounds[k + 1]
        out.append(Box(f"outlet_wall_{k // 2}", pos=(FACE_X - CAV_D / 2, (y0 + y1) / 2, TOP + CAV_H / 2),
                       size=(CAV_D, y1 - y0, CAV_H), color=WHITE))
    return out


def _label(tag: str, word: str, cx: float, cy: float, z: float, px: float, card=CARD, margin=0.004) -> list[Box]:
    w = text_width(word, px) + 2 * margin
    d = 7 * px + 2 * margin
    return [Box(f"{tag}_card", pos=(cx, cy, z + 0.0005), size=(d, w, 0.001), color=card),
            *text_boxes(Box, f"{tag}_txt", word, (cx, cy), z + 0.001, px)]


# -- device blocks (fixed, beyond the arms' reach) and labels -------------------------------------------------------
FAN_BOX = Box("fan_unit", pos=(0.613, 0.0375, TOP + 0.03), size=(0.094, 0.125, 0.06), color=(0.55, 0.57, 0.60))
FRZ_BOX = Box("freezer_unit", pos=(0.623, -0.17, TOP + 0.04), size=(0.114, 0.27, 0.08), color=(0.20, 0.42, 0.78))
PHONE = Box("phone", pos=(0.5035, -0.20, TOP + 0.005), size=(0.075, 0.15, 0.01), color=(0.08, 0.08, 0.09))
PX_DEV = 0.0042
LABELS = [
    *_label("lbl_fan_dev", "FAN", 0.613, 0.0375, TOP + 0.06, PX_DEV),
    *_label("lbl_frz_dev", "FREEZER", 0.623, -0.17, TOP + 0.08, PX_DEV),
    *_label("lbl_phone", "PHONE", 0.5035, -0.20, TOP + 0.01, PX_DEV),
    *_label("lbl_fan_out", "FAN", FACE_X - OUT_D / 2, PORT_Y["fan"], TOP + OUT_H, PX_DEV),
    *_label("lbl_frz_out", "FRZ", FACE_X - OUT_D / 2, PORT_Y["frz"], TOP + OUT_H, PX_DEV, card=RED),
]

# -- parking tray (fixed): inner 80 x 100 mm, 4 mm floor, 15 mm walls 3 mm thick -------------------------------------
TRAY_C = np.array([0.40, 0.16])
TRAY_IN = (0.08, 0.10)
TRAY_FLOOR, TRAY_WALL_H, TRAY_T = 0.004, 0.015, 0.003


def _tray() -> list[Box]:
    x, y = TRAY_C
    ix, iy = TRAY_IN
    ox, oy = ix + 2 * TRAY_T, iy + 2 * TRAY_T
    zc = TOP + TRAY_WALL_H / 2
    return [
        Box("tray_floor", pos=(x, y, TOP + TRAY_FLOOR / 2), size=(ox, oy, TRAY_FLOOR), color=GREEN),
        Box("tray_wall_n", pos=(x + ix / 2 + TRAY_T / 2, y, zc), size=(TRAY_T, oy, TRAY_WALL_H), color=GREEN),
        Box("tray_wall_s", pos=(x - ix / 2 - TRAY_T / 2, y, zc), size=(TRAY_T, oy, TRAY_WALL_H), color=GREEN),
        Box("tray_wall_e", pos=(x, y - iy / 2 - TRAY_T / 2, zc), size=(ix, TRAY_T, TRAY_WALL_H), color=GREEN),
        Box("tray_wall_w", pos=(x, y + iy / 2 + TRAY_T / 2, zc), size=(ix, TRAY_T, TRAY_WALL_H), color=GREEN),
    ]


OBJECTS = [FAN, FRZ, CHARGER, *_outlet(), FAN_BOX, FRZ_BOX, PHONE, *_tray(), *LABELS]

# -- scoring ---------------------------------------------------------------------------------------------------------
IN_MIN = 0.015  # a plug counts as plugged in when its tip is at least 15 mm inside a socket
RELEASE_DIST = 0.03  # m: both grippers' TCP at least this far from every plug centre at the end


class _Episode:
    def __init__(self):
        self.last = None
        self.frz_pulled = False
        self.frz_min = 1.0


_EP = _Episode()
_ENV: dict = {"env": None}


def reset_episode() -> None:
    global _EP
    _EP = _Episode()


def _install_hooks() -> None:
    """Clear the trial history on every embodiment reset; keep a handle on the env so score() can see the grippers."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_freeoutlet_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._freeoutlet_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_freeoutlet_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._freeoutlet_hooked = True
        cls._task_step = _task_step


_install_hooks()


def plug_pos(name: str, objects) -> np.ndarray | None:
    p = objects[CORDS[name].name].end_plug
    if p is None or not np.isfinite(p).all():
        return None
    return np.asarray(p, dtype=float)


def depth_in(port: str, p) -> float:
    """How far a plug (centre ``p``, tip pointing -x) is inside ``port`` (m); -1 if not in line with its opening."""
    if p is None or abs(p[1] - PORT_Y[port]) > 0.008 or p[2] > TOP + CAV_H + 0.005:
        return -1.0
    return float(FACE_X - (p[0] - PL / 2))


def frz_depth(objects) -> float:
    return depth_in("frz", plug_pos("frz", objects))


def charger_depth(objects) -> float:
    p = plug_pos("charger", objects)
    return max(depth_in(port, p) for port in PORTS)


def fan_in_tray(objects) -> bool:
    """Fan plug centre over the tray's inner floor (at least 5 mm in from the walls) and below the wall tops."""
    p = plug_pos("fan", objects)
    if p is None:
        return False
    d = np.abs(p[:2] - TRAY_C)
    return bool(d[0] <= TRAY_IN[0] / 2 - 0.005 and d[1] <= TRAY_IN[1] / 2 - 0.005 and p[2] <= TOP + TRAY_WALL_H)


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (13 mm above the finger pad tips); [] if unknown (offline)."""
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
    plugs = [plug_pos(n, objects) for n in CORDS]
    return all(pl is not None and float(np.linalg.norm(pl - g)) >= RELEASE_DIST for g in grippers() for pl in plugs)


def _update(objects) -> None:
    if objects is _EP.last:
        return
    _EP.last = objects
    d = frz_depth(objects)
    _EP.frz_min = min(_EP.frz_min, d)
    if d < IN_MIN:
        _EP.frz_pulled = True


def score(objects) -> float:
    """0 if the freezer plug's tip was ever less than 15 mm inside its socket during the trial, or if at the end a
    gripper's TCP is within 3 cm of any plug centre. Otherwise 0.6 if the charger plug's tip is at least 15 mm inside
    a socket at the end, plus 0.4 if the fan plug lies in the parking tray."""
    _update(objects)
    if _EP.frz_pulled or not released(objects):
        return 0.0
    s = 0.6 if charger_depth(objects) >= IN_MIN else 0.0
    if fan_in_tray(objects):
        s += 0.4
    return round(s, 3)


def success(objects) -> bool:
    _update(objects)
    st = objects[CHARGER.name]
    return bool(score(objects) == 1.0 and np.linalg.norm(st.lin_vel) < 0.02)


def distance(objects) -> float:
    """How far (m) the charger plug still is from being 15 mm inside the fan socket."""
    p = plug_pos("charger", objects)
    if p is None:
        return 1.0
    target = np.array([FACE_X - IN_MIN + PL / 2, PORT_Y["fan"], PLUG_Z])
    return float(np.linalg.norm(p - target))


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------------
# Left arm: pinch the fan plug's exposed body, pull it straight out, carry it to the tray, let go, move clear.
# Right arm (in parallel): pick the charger plug up by its body and hold it above the outlet; once the fan socket is
# free, set it down in line with the socket (closed loop on the plug's measured offset in the grip), slide it in until
# it bottoms out, let go and move clear.

FINGER_PINCH = -0.020  # finger opening for a plug pinch (pads ~44 mm apart)
GRASP_Z = TOP + PH + 0.003
HI = TOP + 0.08
GRASP_BACK = 0.010  # pinch this far from the plug's cord end
PUSH_YAW = 2.75  # right wrist turned so its camera (8 cm along the yaw heading) looks at the outlet (-x); yaw pi
#                  is past the wrist roll limit here
PUSH_GAP = -0.006  # fingers ~12 mm apart: they straddle the 6 mm cord and push on the 16 mm plug's rear face


def _log(ctx, tag=""):
    st = ctx.states()
    pc = plug_pos("charger", st)
    print(f"[t={ctx.t:5.2f}] {tag} frz={frz_depth(st) * 1000:.1f}mm (min {_EP.frz_min * 1000:.1f}, "
          f"pulled={_EP.frz_pulled}) charger={charger_depth(st) * 1000:.1f}mm at {np.round(pc, 4)} "
          f"fan_tray={fan_in_tray(st)} fan={np.round(plug_pos('fan', st), 4)} rel={released(st)} "
          f"score={score(st)}", flush=True)


def _unplug(arm, ctx, name, pull=0.05):
    """Pinch ``name``'s plug near its cord end and pull it straight out (+x) by ``pull``; keeps holding it."""
    from yam_arena.constants import FINGER_CLOSED

    p = plug_pos(name, ctx.states())
    g = np.array([p[0] + PL / 2 - GRASP_BACK, p[1], GRASP_Z])
    yield from arm.move((g[0], g[1], HI), yaw=0.0, finger=FINGER_PINCH, duration=1.5)
    yield from arm.move(g, yaw=0.0, duration=1.2)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    yield from arm.move(g + np.array([pull, 0.0, 0.0]), yaw=0.0, duration=1.2)
    yield from arm.move(g + np.array([pull, 0.0, HI - GRASP_Z]), yaw=0.0, duration=1.0)
    return g + np.array([pull, 0.0, HI - GRASP_Z])


def _park_fan(ctx):
    arm = ctx.left
    top = yield from _unplug(arm, ctx, "fan")
    tray = np.array([TRAY_C[0] + PL / 2 - GRASP_BACK, TRAY_C[1], TOP + 0.032])  # plug centred over the tray
    yield from arm.move((tray[0], tray[1], HI), yaw=0.0, duration=1.5)
    yield from arm.move(tray, yaw=0.0, duration=1.0)
    yield from arm.grip(FINGER_PINCH, 0.6)
    yield from arm.move((tray[0] - 0.03, tray[1] + 0.03, TOP + 0.14), yaw=0.0, duration=1.0)
    _log(ctx, "fan parked")
    del top


def _pick_charger(ctx, hover):
    from yam_arena.constants import FINGER_CLOSED

    arm = ctx.right
    p = plug_pos("charger", ctx.states())
    g = np.array([p[0] + PL / 2 - GRASP_BACK, p[1], GRASP_Z])
    yield from arm.move((g[0], g[1], HI), yaw=0.0, finger=FINGER_PINCH, duration=1.5)
    yield from arm.move(g, yaw=0.0, duration=1.2)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    yield from arm.move((g[0], g[1], HI), yaw=0.0, duration=1.0)
    yield from arm.move(hover, yaw=0.0, duration=1.8)


def _insert_charger(ctx, port, hover):
    """Lower the held charger plug in line with ``port`` just outside it, slide it in to the bottom, let go."""
    from yam_arena.constants import FINGER_CLOSED

    arm = ctx.right
    # where the plug sits relative to the commanded TCP (measured while holding still above the outlet)
    p = plug_pos("charger", ctx.states())
    off = p[:2] - hover[:2]
    pre = np.array([FACE_X + PL / 2 + 0.012, PORT_Y[port]])  # plug centre: tip 12 mm in front of the opening
    tcp = pre - off
    yield from arm.move((tcp[0], tcp[1], HI), yaw=0.0, duration=1.0)
    yield from arm.move((tcp[0], tcp[1], GRASP_Z + 0.002), yaw=0.0, duration=1.2)
    p = plug_pos("charger", ctx.states())
    dy = PORT_Y[port] - p[1]
    tcp = tcp + np.array([0.0, dy])
    yield from arm.move((tcp[0], tcp[1], GRASP_Z + 0.002), yaw=0.0, duration=0.6)
    _log(ctx, f"pre-insert dy={dy * 1000:.1f}mm")
    # 1) slide the pinched plug in until its tip is ~5 mm inside the opening (the socket now holds it in line)
    start = 0.012 + 0.005
    yield from arm.move((tcp[0] - start, tcp[1], GRASP_Z + 0.002), yaw=0.0, duration=1.2)
    _log(ctx, "tip in")
    yield from arm.grip(FINGER_PINCH, 0.6)
    yield from arm.move((tcp[0] - start + 0.03, tcp[1], HI), yaw=0.0, duration=0.8)
    # 2) turn the wrist so the wrist camera faces the outlet (as far as the wrist roll limit allows), come down
    #    behind the plug with the fingers straddling its cord, and push the plug home
    p = plug_pos("charger", ctx.states())
    rear = p[0] + PL / 2
    y = p[1]
    yield from arm.move((rear + 0.03, y, HI), yaw=PUSH_YAW, finger=PUSH_GAP, duration=1.2)
    yield from arm.move((rear + 0.012, y, GRASP_Z), yaw=PUSH_YAW, duration=1.0)
    target_rear = FACE_X - CAV_D + PL  # plug bottomed out in the socket
    yield from arm.move((target_rear - 0.006, y, GRASP_Z), yaw=PUSH_YAW, duration=2.5)  # fingertips sit ~9 mm behind the TCP here
    yield from arm.wait(0.5)
    _log(ctx, "pushed")
    yield from arm.move((target_rear + 0.03, y, HI + 0.04), yaw=PUSH_YAW, duration=1.2)
    yield from arm.move((0.22, -0.22, TOP + 0.14), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)

def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    mode = os.environ.get("OUTLET_DEMO", "correct")  # test only: correct | nothing | nearest | tug
    _log(ctx, "start")
    hover = np.array([FACE_X + 0.08, -0.14, TOP + 0.11])
    if mode == "nothing":
        for _ in range(6):
            yield from ctx.left.wait(1.0)
            _log(ctx)
        return
    if mode == "nearest":  # cheat: free the socket nearest the charger (the freezer's) and plug the charger there
        top = yield from _unplug(ctx.right, ctx, "frz")
        aside = np.array([0.36, -0.12, TOP + 0.03])
        yield from ctx.right.move((aside[0], aside[1], HI), yaw=0.0, duration=1.0)
        yield from ctx.right.move(aside, yaw=0.0, duration=0.8)
        yield from ctx.right.grip(FINGER_PINCH, 0.6)
        yield from ctx.right.move((aside[0], aside[1], HI), yaw=0.0, duration=0.8)
        _log(ctx, "frz unplugged")
        del top
        yield from _pick_charger(ctx, hover)
        yield from _insert_charger(ctx, "frz", hover)
        _log(ctx, "end")
        return
    if mode == "tug":  # cheat: briefly pull the freezer plug out 15 mm, push it back, then do the right thing
        p = plug_pos("frz", ctx.states())
        g = np.array([p[0] + PL / 2 - GRASP_BACK, p[1], GRASP_Z])
        arm = ctx.right
        yield from arm.move((g[0], g[1], HI), yaw=0.0, finger=FINGER_PINCH, duration=1.5)
        yield from arm.move(g, yaw=0.0, duration=1.2)
        yield from arm.grip(FINGER_CLOSED, 0.6)
        yield from arm.move(g + np.array([0.015, 0.0, 0.0]), yaw=0.0, duration=1.0)
        _log(ctx, "tugged")
        yield from arm.move(g + np.array([0.002, 0.0, 0.0]), yaw=0.0, duration=1.0)
        yield from arm.grip(FINGER_PINCH, 0.6)
        yield from arm.move((g[0], g[1], HI), yaw=0.0, duration=1.0)
        _log(ctx, "re-seated")

    # correct: left arm frees the fan socket while the right arm fetches the charger plug; then the right arm plugs in
    yield from ctx.both(_park_fan(ctx), _pick_charger(ctx, hover))
    _log(ctx, "charger held")
    yield from _insert_charger(ctx, "fan", hover)
    _log(ctx, "end")
    yield from ctx.left.move((0.22, 0.22, TOP + 0.14), yaw=0.0, finger=FINGER_CLOSED, duration=1.0)
    yield from ctx.left.wait(1.5)
    _log(ctx, "final")
