"""On the station table: a network patch panel and a small switch, patched according to a printed port map.

The PATCH PANEL (dark grey block, 48 cm wide, 6 cm deep, 3 cm tall, fixed) stands across the far side of the table.
Six ports (rectangular openings 20 mm wide x 16.5 mm tall, 26 mm deep, at table level) face the robot, 8 cm apart;
a white label above each port on the panel's top face carries its number in black block digits, 1 to 6 from the
robot's left to its right. A grey SWITCH (7 x 10 x 4 cm, fixed) stands in the middle of the table. Three 6 mm patch
cords, 28 cm long, leave the switch's side faces and lie straight across the table: BLUE from its left face towards
the robot's left, YELLOW and WHITE (behind it) from its right face towards the robot's right; each ends in a plug of
its own colour (55 x 16 x 14 mm) lying free on the table. A white PORT MAP card lies on the table in front of the
switch, printed in black block capitals:  BLUE 2 / YELLOW 5 / WHITE 3.
Each plug must end up pushed into the port the card gives for its colour; no plug may be in any other port.

Layout is a derangement: the port nearest each plug (blue -> 1, yellow and white -> 6) is never its mapped port and
the map is not "in order", so position-only patching scores 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var PATCH_DEMO): correct (default) | nothing | nearest | inorder | hold.
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

INSTRUCTION = ("Patch the three cords from the switch into the patch panel exactly as the port map card says: push "
               "each plug fully into the numbered port listed for its colour. Leave all other ports empty, then let "
               "go of the plugs.")

PORT_MAP = {"blue": 2, "yellow": 5, "white": 3}  # what the card says

# -- patch panel: six ports facing the robot (-x), numbered 1..6 from the robot's left (+y) to its right ------------
FACE_X = 0.40  # port faces
PORT_Y = {1: 0.20, 2: 0.12, 3: 0.04, 4: -0.04, 5: -0.12, 6: -0.20}
PL, PW, PH = 0.055, 0.016, 0.014  # plug body: length along the cord, width, height
CAV_W, CAV_H, CAV_D = PW + 0.004, PH + 0.0025, 0.026  # port opening (y, z) and depth (x): 20 x 16.5 x 26 mm
PANEL_D, PANEL_W, PANEL_H = 0.060, 0.48, 0.030
PANEL_GREY = (0.16, 0.17, 0.18)
WHITE_CARD = (0.97, 0.97, 0.97)


def _panel() -> list[Box]:
    xc = FACE_X + CAV_D / 2
    out = [
        Box("panel_back", pos=(FACE_X + CAV_D + (PANEL_D - CAV_D) / 2, 0.0, TOP + PANEL_H / 2),
            size=(PANEL_D - CAV_D, PANEL_W, PANEL_H), color=PANEL_GREY),
        Box("panel_roof", pos=(xc, 0.0, TOP + (CAV_H + PANEL_H) / 2), size=(CAV_D, PANEL_W, PANEL_H - CAV_H),
            color=PANEL_GREY),
    ]
    # dividers between the port openings (and at both ends)
    edges = [PANEL_W / 2] + [e for k in sorted(PORT_Y) for e in (PORT_Y[k] + CAV_W / 2, PORT_Y[k] - CAV_W / 2)] \
        + [-PANEL_W / 2]
    for i in range(0, len(edges), 2):
        y0, y1 = edges[i], edges[i + 1]
        out.append(Box(f"panel_div{i // 2}", pos=(xc, (y0 + y1) / 2, TOP + CAV_H / 2), size=(CAV_D, y0 - y1, CAV_H),
                       color=PANEL_GREY))
    # number labels on the panel's top face, above each port
    px = 0.0040
    lx = FACE_X + 0.032
    for k, y in PORT_Y.items():
        out.append(Box(f"port{k}_label", pos=(lx, y, TOP + PANEL_H + 0.0005), size=(7 * px + 0.008, 5 * px + 0.010,
                       0.001), color=WHITE_CARD))
        out += text_boxes(Box, f"port{k}_num", str(k), (lx, y), TOP + PANEL_H + 0.001, px)
    return out


# -- switch and cords -----------------------------------------------------------------------------------------------
SW_X0, SW_X1, SW_W, SW_H = 0.20, 0.27, 0.10, 0.040
SWITCH = Box("switch", pos=((SW_X0 + SW_X1) / 2, 0.0, TOP + SW_H / 2), size=(SW_X1 - SW_X0, SW_W, SW_H),
             color=(0.42, 0.44, 0.47))
SWITCH_TOP = Box("switch_top", pos=((SW_X0 + SW_X1) / 2, 0.0, TOP + SW_H + 0.0005), size=(0.05, 0.08, 0.001),
                 color=(0.30, 0.32, 0.35))

R = 0.003  # 6 mm patch cord
CORD_LEN = 0.28
PLUG_Z = TOP + PH / 2 + 0.0005  # cord and plug centre height at spawn (plugs rest on the table)
COLORS = {"blue": (0.10, 0.30, 0.85), "yellow": (0.95, 0.80, 0.08), "white": (0.95, 0.95, 0.95)}
PLUG_COLORS = {"blue": (0.07, 0.22, 0.70), "yellow": (0.88, 0.72, 0.05), "white": (0.88, 0.88, 0.88)}
# (exit x on the switch, side: +1 left face / -1 right face)
EXITS = {"blue": (0.215, +1), "yellow": (0.215, -1), "white": (0.255, -1)}


def _cord(c: str):
    x, s = EXITS[c]
    return cable(f"cord_{c}", start=(x, s * (SW_W / 2 + 0.006), PLUG_Z), yaw=s * math.pi / 2, length=CORD_LEN,
                 radius=R, anchor_start=True, end_plug=Plug(size=(PL, PW, PH), mass=0.015, color=PLUG_COLORS[c]),
                 color=COLORS[c], bend_stiffness=0.01, bend_damping=0.001)


CORDS = {c: _cord(c) for c in ("blue", "yellow", "white")}

# -- port map card (white, black block capitals, read from the robot side) -------------------------------------------
CARD_PX = 0.0040
CARD_LINES = [f"{c.upper():<7}{PORT_MAP[c]}" for c in ("blue", "yellow", "white")]  # "BLUE   2", ...
CARD_X = 0.11
CARD_LINE_GAP = 0.010
CARD_MARGIN = 0.008


def _card() -> list[Box]:
    w = max(text_width(t, CARD_PX) for t in CARD_LINES)
    lh = 7 * CARD_PX
    d = len(CARD_LINES) * lh + (len(CARD_LINES) - 1) * CARD_LINE_GAP + 2 * CARD_MARGIN
    out = [Box("map_card", pos=(CARD_X, 0.0, TOP + 0.0005), size=(d, w + 2 * CARD_MARGIN, 0.001), color=WHITE_CARD)]
    for i, t in enumerate(CARD_LINES):
        x = CARD_X + d / 2 - CARD_MARGIN - lh / 2 - i * (lh + CARD_LINE_GAP)
        out += text_boxes(Box, f"map_l{i}", t, (x, 0.0), TOP + 0.001, CARD_PX)
    return out


OBJECTS = [*_panel(), SWITCH, SWITCH_TOP, *_card(), *CORDS.values()]

# -- scoring ---------------------------------------------------------------------------------------------------------
ENTER = 0.005  # m: a plug is IN a port once its tip is at least 5 mm inside the opening
SEATED_MIN = 0.015  # m: credit when the tip is at least 15 mm inside its mapped port
RELEASE_DIST = 0.03  # m: every gripper TCP at least this far from every plug centre
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline use)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_patch_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._patch_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads), [] if unknown."""
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


def plug(objects, c: str) -> tuple[np.ndarray, np.ndarray]:
    """(centre, unit axis from the cord through the plug tip) of cord ``c``'s plug."""
    st = objects[f"cord_{c}"]
    p = np.asarray(st.end_plug, dtype=float)
    a = p - np.asarray(st.points[-1], dtype=float)
    return p, a / (np.linalg.norm(a) + 1e-12)


def port_of(objects, c: str) -> tuple[int | None, float]:
    """(port the plug's tip is in, tip depth past the port face in m); (None, depth) if not in any port."""
    p, a = plug(objects, c)
    tip = p + a * PL / 2
    if not np.isfinite(tip).all():
        return None, -1.0
    depth = float(tip[0] - FACE_X)
    if depth < ENTER or tip[2] > TOP + CAV_H + 0.003:
        return None, depth
    for k, y in PORT_Y.items():
        if abs(tip[1] - y) <= CAV_W / 2 + 0.002:
            return k, depth
    return None, depth


def released(objects) -> bool:
    return all(float(np.linalg.norm(plug(objects, c)[0] - g)) >= RELEASE_DIST for g in grippers() for c in CORDS)


def wrong_patch(objects) -> bool:
    """Any plug in a port other than the one the card gives for its colour."""
    for c in CORDS:
        k, _ = port_of(objects, c)
        if k is not None and k != PORT_MAP[c]:
            return True
    return False


def score(objects) -> float:
    """0 if any plug is in a wrong port (tip >= 5 mm inside), or a gripper is within 3 cm of any plug at the end.
    Otherwise 1/3 per plug whose tip is at least 15 mm inside the port the card gives for its colour."""
    if wrong_patch(objects) or not released(objects):
        return 0.0
    good = 0
    for c in CORDS:
        k, d = port_of(objects, c)
        good += int(k == PORT_MAP[c] and d >= SEATED_MIN)
    return good / 3.0


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and all(np.linalg.norm(objects[f"cord_{c}"].lin_vel) < 0.02 for c in CORDS))


def distance(objects) -> float:
    """Sum over the plugs of how far (m) each tip is from 15 mm inside its mapped port."""
    tot = 0.0
    for c in CORDS:
        p, a = plug(objects, c)
        tip = p + a * PL / 2
        tgt = np.array([FACE_X + SEATED_MIN, PORT_Y[PORT_MAP[c]], TOP + PH / 2])
        tot += float(np.linalg.norm(tip - tgt)) if np.isfinite(tip).all() else 1.0
    return tot


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------------
# Left arm serves ports 1-3, right arm ports 4-6. A plug is carried pinched at its middle (pinched near one end it
# swings upright) and turned on the way so it points at the panel; it is set down flat on the table in front of its
# port, squared up, re-pinched 10 mm from its cord end and pushed in along the table (closed loop on the plug pose).
# The right arm turns the WHITE plug and sets it down in the middle of the table (hand-over spot) while the left arm
# lifts BLUE and holds it clear; then the left arm patches BLUE into port 2 while the right arm patches YELLOW into
# port 5; finally the left arm takes WHITE from the hand-over spot and patches it into port 3.

FINGER_PINCH = -0.020  # finger opening for the plug pinch (pads ~44 mm apart)
GRASP_BACK = 0.010  # push grip: pinch this far from the plug's cord end
GRASP_AHEAD = PL - GRASP_BACK  # push-grip TCP -> plug tip along the plug axis
SEAT_TO = 0.0235  # target tip depth (port is 26 mm deep)
GZ = TOP + PH + 0.003  # TCP height when pinching a plug on the table
CARRY = 0.003  # carry the plug only this far above the table (lifted higher, it swings about the pinch axis)
HI = TOP + 0.08
RELAY = (0.335, 0.0)  # plug centre at the hand-over spot (pointing at the panel)
PARK = {"left": (0.20, 0.22, TOP + 0.14), "right": (0.20, -0.22, TOP + 0.14)}


def _log(ctx, tag=""):
    st = ctx.states()
    pp = {c: port_of(st, c) for c in CORDS}
    pl = {c: np.round(plug(st, c)[0], 3).tolist() for c in CORDS}
    ang = {c: round(math.degrees(math.atan2(plug(st, c)[1][1], plug(st, c)[1][0])), 1) for c in CORDS}
    print(f"[t={ctx.t:5.2f}] {tag} cmd={[round(math.degrees(a._yaw), 1) for a in (ctx.left, ctx.right)]} "
          f"ports={ {c: (k, round(d * 1000, 1)) for c, (k, d) in pp.items()} } plugs={pl} "
          f"ang={ang} rel={released(st)} score={score(st):.3f} "
          f"tcp={[np.round(g, 3).tolist() for g in grippers()]}", flush=True)


def _dbg(ctx, tag):
    if os.environ.get("PATCH_DBG"):
        _log(ctx, tag)


def _norm_yaw(y: float) -> float:
    while y > math.pi / 2:
        y -= math.pi
    while y < -math.pi / 2:
        y += math.pi
    return y


def _pinch(arm, ctx, c, back=0.0, lift=CARRY):
    """Pinch plug ``c`` (fingers across it) ``back`` m behind its centre and lift it ``lift``. Returns the yaw."""
    from yam_arena.constants import FINGER_CLOSED

    p, a = plug(ctx.states(), c)
    g = p - a * back
    th = math.atan2(a[1], a[0])
    # grip yaw = the plug's heading itself (not mod pi), so that turning the grip to yaw 0 turns the plug to +x
    yaw = float(np.clip(th, -math.pi / 2, math.pi / 2)) if abs(th) < math.pi / 2 + 0.2 else _norm_yaw(th)
    yield from arm.move((g[0], g[1], HI), yaw=yaw, finger=FINGER_PINCH, duration=1.5)
    yield from arm.move((g[0], g[1], GZ), yaw=yaw, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    if lift:
        yield from arm.move((g[0], g[1], GZ + lift), yaw=yaw, duration=0.6)
    return yaw


def _turn_to(arm, x, y, z, duration=2.5, yaw=0.0, n=5):
    """Move the TCP to (x, y, z) while turning the grip to ``yaw`` in small steps."""
    p0, y0 = arm.tcp(), arm._yaw
    for k in range(1, n + 1):
        f = k / n
        yield from arm.move(tuple(p0 + f * (np.array([x, y, z]) - p0)), yaw=y0 + f * (yaw - y0),
                            duration=duration / n)


def _set_down(arm, ctx, c, x, y, release=True):
    """Carry the plug (held at its middle) to centre (x, y) pointing +x, lower it flat and square it up."""
    _dbg(ctx, f"{c} picked")
    yield from _turn_to(arm, x, y, GZ + CARRY)
    _dbg(ctx, f"{c} turned")
    yield from arm.move((x, y, GZ), yaw=0.0, duration=0.6)
    yaw = 0.0
    for _ in range(3):  # turn the grip until the plug (now flat on the table) points along +x
        _, a = plug(ctx.states(), c)
        err = math.atan2(a[1], a[0])
        if abs(err) < 0.015:
            break
        yaw = float(np.clip(yaw - err, -math.pi / 2, math.pi / 2))
        yield from arm.move((x, y, GZ), yaw=yaw, duration=0.5)
        yield from arm.wait(0.2)
    _dbg(ctx, f"{c} set down")
    if release:
        yield from arm.grip(FINGER_PINCH, 0.5)
        yield from arm.move((x, y, GZ + 0.03), yaw=yaw, duration=0.5)


def _insert(arm, ctx, c, port):
    """Set plug ``c`` down in front of ``port``, re-pinch it near its cord end, line it up and push it SEAT_TO deep
    (closed loop on the plug pose). Returns the final TCP (x, y, yaw)."""
    py = PORT_Y[port]
    yield from _set_down(arm, ctx, c, FACE_X - 0.012 - PL / 2, py)
    yaw = yield from _pinch(arm, ctx, c, back=PL / 2 - GRASP_BACK, lift=0.0)
    _dbg(ctx, f"{c} re-pinched")
    gx, gy = (float(v) for v in arm.tcp()[:2])
    for _ in range(3):  # the released cord may have turned the plug: square it up again, held flat on the table
        _, a = plug(ctx.states(), c)
        err = math.atan2(a[1], a[0])
        if abs(err) < 0.015:
            break
        yaw = float(np.clip(yaw - err, -math.pi / 2, math.pi / 2))
        yield from arm.move((gx, gy, GZ), yaw=yaw, duration=0.5)
        yield from arm.wait(0.2)
    for _ in range(3):  # line the plug tip up with the port axis (the arm sags a few mm under load)
        p, a = plug(ctx.states(), c)
        err = py - (p + a * PL / 2)[1]
        if abs(err) < 0.0007:
            break
        gy += float(np.clip(err, -0.01, 0.01))
        yield from arm.move((gx, gy, GZ), yaw=yaw, duration=0.5)
        yield from arm.wait(0.2)
    _dbg(ctx, f"{c} aligned")
    for i in range(3):  # push; top up if the plug slipped back in the fingers
        k, d = port_of(ctx.states(), c)
        if k == port and d >= SEAT_TO - 0.002:
            break
        p, a = plug(ctx.states(), c)
        gx += (FACE_X + SEAT_TO) - (p + a * PL / 2)[0]
        yield from arm.move((gx, gy, GZ), yaw=yaw, duration=2.0 if i == 0 else 1.0)
        yield from arm.wait(0.2)
    _dbg(ctx, f"{c} pushed")
    return gx, gy, yaw


def _release(arm, at, park=True):
    gx, gy, yaw = at
    yield from arm.grip(FINGER_PINCH, 0.6)
    yield from arm.move((gx - 0.004, gy, HI), yaw=yaw, duration=1.0)
    if park:
        yield from arm.move(PARK[arm.name], yaw=yaw, duration=1.2)


def _patch(arm, ctx, c, port, park=True):
    """Pick plug ``c`` up, patch it into ``port``, let go."""
    yield from _pinch(arm, ctx, c)
    at = yield from _insert(arm, ctx, c, port)
    _log(ctx, f"{c} -> port {port}")
    yield from _release(arm, at, park)


def _to_relay(arm, ctx, c):
    """Pick plug ``c``, turn it to point at the panel and set it down at the hand-over spot."""
    yield from _pinch(arm, ctx, c)
    yield from _set_down(arm, ctx, c, *RELAY)
    yield from arm.move(PARK[arm.name], yaw=0.0, duration=1.2)


def _hold_clear(arm, ctx, c):
    """Pick plug ``c``, turn it towards the panel and hold it clear of the hand-over spot."""
    yield from _pinch(arm, ctx, c)
    yield from _turn_to(arm, 0.27, 0.25, GZ + CARRY, duration=2.0)


def _wait(ctx, s):
    yield from ctx.left.wait(s)


def demo_script(ctx):
    mode = os.environ.get("PATCH_DEMO", "correct")  # test only
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":  # cheat: do nothing
        for _ in range(6):
            yield from _wait(ctx, 1.0)
            _log(ctx)
        return
    if mode == "nearest":  # cheat: patch each plug into the free port nearest to it (blue 1, yellow 6, white 5)
        yield from ctx.both(_patch(L, ctx, "blue", 1), _patch(Rt, ctx, "yellow", 6))
        yield from _patch(Rt, ctx, "white", 5)
        yield from _wait(ctx, 1.5)
        _log(ctx, "end")
        return
    if mode == "inorder":  # cheat: ports in card order 1, 2, 3 (blue 1 wrong, white 3 right; yellow left alone)
        yield from ctx.both(_patch(L, ctx, "blue", 1), _to_relay(Rt, ctx, "white"))
        yield from _patch(L, ctx, "white", 3)
        yield from _wait(ctx, 1.5)
        _log(ctx, "end")
        return
    # correct: white to the hand-over spot while blue is held clear; blue -> 2 and yellow -> 5 together; white -> 3
    yield from ctx.both(_hold_clear(L, ctx, "blue"), _to_relay(Rt, ctx, "white"))
    _log(ctx, "white at hand-over")

    def left_blue():
        at = yield from _insert(L, ctx, "blue", PORT_MAP["blue"])
        _log(ctx, "blue seated")
        if mode == "hold":
            return at
        yield from _release(L, at, park=False)
        return at

    def right_yellow():
        yield from _pinch(Rt, ctx, "yellow")
        at = yield from _insert(Rt, ctx, "yellow", PORT_MAP["yellow"])
        _log(ctx, "yellow seated")
        if mode == "hold":  # cheat: patch correctly but keep holding the yellow plug
            yield from _wait(ctx, 30.0)
            return
        yield from _release(Rt, at)

    if mode == "hold":
        yield from ctx.both(left_blue(), right_yellow())
        return
    yield from ctx.both(left_blue(), right_yellow())
    yield from _patch(L, ctx, "white", PORT_MAP["white"])
    yield from _wait(ctx, 1.5)
    _log(ctx, "end")
