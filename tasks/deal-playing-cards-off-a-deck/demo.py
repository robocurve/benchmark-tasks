"""Demo for the card-dealing task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 1000] [--no-record] [--mode full|one|skip|order|double|heap|still]

Each card is dealt the same way. A hand, its fingers closed and pointing down (leaning 20 degrees forward), comes
over the deck and lowers the rear edge of its fingertips onto the back of the top card, 1 cm in from the card's
near edge. Pressing lightly, it moves straight away from the robot: the card comes with it by friction and runs out
over the deck's far edge, held flat by the fingertips on its tail. As the fingertips pass the edge the card tips
down onto the board; the hand slows and follows it down, and the card drops flat when its tail leaves the deck. The
hand lifts a little, sets its fingertips down again on the middle of the card, and slides it across the board to
its place, where it lifts off. A card for the near place is slid sideways first and then drawn back toward the
robot, so that it passes clear of the deck. The left hand deals to the left place, the right hand to the near and
right places, in turn: left, near, right.

Controls: ``--mode one`` deals the first card only. ``--mode skip`` deals to the left place and then to the right
place, leaving out the near place. ``--mode order`` deals to the near place first, then left, then right.
``--mode double`` sets the fingertips against the deck's near edge, 4 mm below its top, and pushes: several cards
come off together. ``--mode heap`` does the same 1 cm down and pushes much of the deck away. ``--mode still`` keeps both
arms still.

The script is open loop. Every target comes from the layout constants in the task file and the sizes in
``cardkit.py``; the height of the deck's top is worked out from how many cards have been dealt. The only feedback is
each hand's own measured position (the robot's proprioception), used to correct for the arm's sag. How hard a hand
presses is set by how far below the surface it is commanded, so its sag has to be known where it presses: before
dealing, each hand hovers for half a second over every place it will bring a card to and notes its sag there. The
demo never reads the state of a card.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
FULL_OPEN = 0.0939                 # m between the pads at grip command 1.0


def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def gap(width: float) -> float:
    """Grip command (1 open .. 0 closed) for a gap between the pads in metres."""
    return min(1.0, max(0.0, width / FULL_OPEN))


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Hand:
    """One arm: smooth moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = self.worst_deg = 0.0

    def _solve(self, pos, rot) -> None:
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, float(np.degrees(re)))
        self.cmd = np.r_[q, self.grip]

    def set_grip(self, grip: float, duration: float = 0.5):
        """Generator: open or close to ``grip`` in place."""
        from yam_arena import ik

        g0 = self.grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.grip = g0 + ik.smoothstep(k / n) * (grip - g0)
            self.cmd = np.r_[self.q, self.grip]
            yield

    def move_joints(self, q_goal, duration: float, grip: float | None = None):
        """Generator: swing to joint angles ``q_goal``, every joint moving together."""
        from yam_arena import ik

        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q, self.grip = q0 + s * (np.asarray(q_goal, float) - q0), g0 + s * (g1 - g0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base)
            self.cmd = np.r_[self.q, self.grip]
            yield


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(*gens):
    """Generator: run several moves side by side until all have finished."""
    live = list(gens)
    while live:
        for g in list(live):
            try:
                next(g)
            except StopIteration:
                live.remove(g)
        if live:
            yield


def best_ik(hand, pos, rot):
    """Joint angles for ``pos``/``rot`` from the best of a few seeds: the one that leaves the joints most room."""
    from yam_arena import ik

    seeds = [hand.q.copy()]
    for q6 in (-1.5, -0.7, 0.7, 1.5):
        s = hand.q.copy()
        s[5] = q6
        seeds.append(s)
    for q1, q5 in ((0.6, 0.8), (-0.6, -0.8), (1.0, 1.2), (-1.0, -1.2)):
        s = hand.q.copy()
        s[0], s[4] = q1, q5
        seeds.append(s)
    best = None
    for seed in seeds:
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), seed, base_pos=hand.base, iters=400)
        room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
        cost = (pe > 5e-4 or re > 5e-3, -room)
        if best is None or cost < best[1]:
            best = (q, cost)
    return best[0]



TOOL: dict = {}                          # where each hand's tool point actually is (updated every control step)
TOOLROT: dict = {}                       # ... and which way the hand actually points
PHASE: dict = {"name": "start"}
SAGS: dict = {}                          # each hand's sag corrector, for the trace

UP = np.array([0.0, 0.0, 1.0])
TIP = 0.013                              # the fingertips' ends are this far past the tool point
TIP_THICK = 0.004                        # from the middle of the closed fingertips to their outer face, at the end
LEAN = 20.0                              # the dealing hand leans this far forward from straight down: leaning, the arm
#                                          gives more under a push, so the same error in height changes its press less
PRESS = float(os.environ.get("CARDS_PRESS", "0.002"))   # on the board, the fingertips are commanded this far below the card's back
PRESS_DECK = float(os.environ.get("CARDS_PRESS_DECK", "0.002"))   # ... and on the deck, this far
TAIL = float(os.environ.get("CARDS_TAIL", "0.010"))      # ... and set down this far in from the card's rear edge
HOVER = 0.030                            # the fingertips' height above the deck's top on the way in
CARRY = 0.080                            # ... and above the table when a hand waits or travels
RAMP = 0.030                             # (the double control) past the deck the fingertips come down over this distance
PAST = float(os.environ.get("CARDS_PAST", "0.006"))      # the fingertips go on this far after the card's tail should have left the deck
EXTRA = float(os.environ.get("CARDS_EXTRA", "0.001"))    # while the card tips over the edge the fingertips are commanded this much lower
SPEED_TIP = float(os.environ.get("CARDS_SPEED_TIP", "0.035"))  # m/s while the card tips over the deck's edge
SPEED_OFF = float(os.environ.get("CARDS_SPEED_OFF", "0.07"))   # m/s while a card is slid off the deck
SPEED = float(os.environ.get("CARDS_SPEED", "0.12"))           # ... and across the table
PARK = np.array([-0.070, 0.070])         # a waiting hand hovers this far back and out from its own place
GAIN, BIAS_MAX = 0.25, 0.025             # how fast, and how far, a hand corrects for its own sag
TURN_GAIN, TURN_MAX = 0.15, math.radians(8.0)   # ... and for pointing off the way it is told to


def _rotvec(rot) -> np.ndarray:
    """The rotation vector of a rotation matrix."""
    a = math.acos(float(np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0)))
    if a < 1e-9:
        return np.zeros(3)
    return np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]) * (a / (2.0 * math.sin(a)))


def _turn(vec) -> np.ndarray:
    """The rotation matrix of a rotation vector."""
    a = float(np.linalg.norm(vec))
    if a < 1e-12:
        return np.eye(3)
    k = np.asarray(vec, float) / a
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * (kx @ kx)


def _square(rot) -> np.ndarray:
    """The nearest rotation whose axes are the coordinate axes (the fixed turn between two ways of naming a frame)."""
    out = np.zeros((3, 3))
    m = np.abs(rot).copy()
    for _ in range(3):
        i, j = np.unravel_index(int(np.argmax(m)), m.shape)
        out[i, j] = math.copysign(1.0, rot[i, j])
        m[i, :] = -1.0
        m[:, j] = -1.0
    return out



class Sag:
    """Slowly offsets a hand's command by its measured error, so that its tool point ends up where it is wanted, and
    turns the command by the hand's measured error in direction. While ``press`` is set the correction in height is
    frozen: the hand is then commanded a fixed distance into what it presses on, which sets how hard it presses."""

    def __init__(self, hand):
        self.hand, self.bias, self.want = hand, np.zeros(3), None
        self.live, self.trim, self.want_rot, self.frame, self.off = True, np.zeros(3), None, None, 0.0
        self.press = False

    def measured(self):
        rot = TOOLROT.get(self.hand.name)
        if rot is None:
            return None
        if self.frame is None:                       # found once, while the arm rests at its ready pose
            self.frame = _square(rot.T @ self.hand.rot)
        return rot @ self.frame

    def to(self, pos, rot):
        if self.want is not None and self.hand.name in TOOL:
            step = GAIN * (self.want - TOOL[self.hand.name])
            if self.press:
                step[2] = 0.0
            self.bias = np.clip(self.bias + step, -BIAS_MAX, BIAS_MAX)
        got = self.measured()
        if self.want_rot is not None and got is not None:
            err = _rotvec(self.want_rot @ got.T)
            self.off = math.degrees(float(np.linalg.norm(err)))
            if self.live and not self.press:
                trim = self.trim + TURN_GAIN * err
                size = float(np.linalg.norm(trim))
                self.trim = trim if size <= TURN_MAX else trim * (TURN_MAX / size)
        self.want, self.want_rot = np.asarray(pos, float), rot
        self.hand._solve(self.want + self.bias, _turn(self.trim) @ rot)


def glide(sag, target, seconds: float, rot):
    """Generator: take the hand's tool point in a straight line to ``target``, correcting for sag on the way."""
    from yam_arena import ik

    start = sag.want if sag.want is not None else sag.hand.pos.copy()
    n = max(1, round(seconds * CONTROL_HZ))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        sag.to(start + s * (np.asarray(target, float) - start), rot)
        yield


def go(sag, pos, rot, seconds: float):
    """Generator: swing the arm to a pose it is not yet near, by its joints, then settle on it."""
    yield from sag.hand.move_joints(best_ik(sag.hand, pos, rot), seconds, grip=0.0)
    sag.want = None
    yield from glide(sag, pos, 0.4, rot)


# -- where things are, from the layout ------------------------------------------------------------------------------
def hand_rot(name: str) -> np.ndarray:
    """A dealing hand: fingers closed, leaning ``LEAN`` degrees forward, closing along the line away from the robot."""
    a = math.radians(LEAN)
    return frame((math.sin(a), 0.0, -math.cos(a)), (1.0 if name == "right" else -1.0, 0.0, 0.0))


def tool_for(name: str, touch) -> np.ndarray:
    """The tool point that puts the lowest edge of the hand's closed fingertips (their rear edge) at ``touch``."""
    a = math.radians(LEAN)
    rot = hand_rot(name)
    rear = np.array([-math.cos(a), 0.0, -math.sin(a)]) * TIP_THICK          # from the middle of the fingertips' end
    return np.asarray(touch, float) - rear - TIP * rot[:, 2]


def deck_top(T, dealt: int) -> float:
    """The height of the deck's top (world z) once ``dealt`` cards have gone."""
    return T.TOP + (T.COUNT - dealt) * T.K.CARD_T


def deck_edge(T, u) -> float:
    """How far it is from the middle of the deck to its edge, going along the unit direction ``u`` (world xy)."""
    loc = T.K._rz(T.DECK_YAW)[:2, :2].T @ np.asarray(u, float)[:2]
    half = np.array([T.K.CARD_W / 2, T.K.CARD_L / 2])
    return float(min(half[i] / abs(loc[i]) for i in range(2) if abs(loc[i]) > 1e-9))


def spot(T, place: int, round_: int = 0) -> np.ndarray:
    """Where the middle of a place's card is put: on the place's mark."""
    return np.array(T.PLACES[place], float)


AWAY = np.array([1.0, 0.0])              # every card leaves the deck straight away from the robot, over its far edge


def follow(T, dealt: int, edge: float, back: float, s: float) -> float:
    """The height (world z) of the back of the card being dealt, under the fingertips, once the card has gone ``s``
    out from the deck: flat on the deck until the fingertips reach the deck's edge; after that, tipped over the edge
    with its front on the board."""
    card = T.K.CARD_T
    top = deck_top(T, dealt)
    if s <= edge + back:
        return top
    pivot = top - card - T.TOP                        # the edge it tips over: the top of the card under it
    return T.TOP + card + pivot * (edge + back) / s


CAL: dict = {}                           # (hand, place, round) -> the hand's sag correction in height over that spot
SKIM = 0.012                             # the fingertips' height above the board when a hand hovers over a spot


def survey(T, sag, spots):
    """Generator: hover over each of ``spots`` (key -> xy) in turn and note the hand's sag correction there."""
    name = sag.hand.name
    rot = hand_rot(name)
    for key, xy in spots:
        yield from glide(sag, tool_for(name, (xy[0], xy[1], T.TOP + SKIM)), 0.8, rot)
        yield from glide(sag, tool_for(name, (xy[0], xy[1], T.TOP + SKIM)), 0.6, rot)
        CAL[(name, *key)] = float(sag.bias[2])


def deal(T, sag, target_xy, dealt: int, press: float = None, low: float = 0.0, sag_end: float = None):
    """Generator: slide the top card off the deck and on to ``target_xy`` (where its middle is to end up).
    ``dealt``: how many cards have left the deck already. ``low``: set the fingertips this far below the deck's top,
    against its rear edge, instead of on it, and push (the ``double`` and ``heap`` controls)."""
    from yam_arena import ik

    name = sag.hand.name
    rot = hand_rot(name)
    press = PRESS if press is None else press
    press_deck = min(press, PRESS_DECK)
    deck = np.array(T.DECK_XY, float)
    tgt = np.asarray(target_xy, float)
    u = AWAY
    top, edge = deck_top(T, dealt), deck_edge(T, u)
    table = T.TOP + T.K.CARD_T
    back = edge - TAIL                                # the fingertips' place on the card, behind its middle
    start = deck - ((edge + 0.010) if low else back) * u

    def at(xy, z: float) -> np.ndarray:
        return tool_for(name, (xy[0], xy[1], z))

    def run(p0, p1, speed: float, height):
        dist = float(np.linalg.norm(p1 - p0))
        n = max(2, round(dist / speed * CONTROL_HZ))
        for k in range(1, n + 1):
            f = ik.smoothstep(k / n)
            sag.to(at(p0 + f * (p1 - p0), height(f * dist)), rot)
            yield

    PHASE["name"] = f"deal {dealt + 1} in"
    z_in = (top - low) if low else top + 0.004
    yield from glide(sag, at(start, top + HOVER), 0.5, rot)
    yield from glide(sag, at(start, z_in), 0.6, rot)
    yield from glide(sag, at(start, z_in), 0.3, rot)
    sag.press = True
    if low:
        PHASE["name"] = f"deal {dealt + 1} push"
        clear = 2 * edge + 0.010                      # where the fingertips pass the deck's far edge
        through = start + (clear + RAMP) * u

        def height(s: float) -> float:
            drop = float(np.clip((s - clear) / RAMP, 0.0, 1.0))
            return z_in + (table - press - z_in) * (drop * drop * (3 - 2 * drop))

        yield from run(start, through, SPEED_OFF, height)
        end = through + (tgt - (deck + (edge + RAMP) * u))
        yield from run(through, end, SPEED, lambda s: table - press)
    else:
        yield from glide(sag, at(start, top - press_deck), 0.5, rot)
        yield from glide(sag, at(start, top - press_deck), 0.2, rot)
        PHASE["name"] = f"deal {dealt + 1} off deck"
        brink = start + (edge + back) * u             # the fingertips at the deck's edge
        out = start + (2 * edge + PAST) * u
        yield from run(start, brink, SPEED_OFF, lambda s: top - press_deck)
        PHASE["name"] = f"deal {dealt + 1} tip"
        yield from run(brink, out, SPEED_TIP, lambda s: follow(T, dealt, edge, back, edge + back + s) - press_deck - EXTRA)
        PHASE["name"] = f"deal {dealt + 1} regrip"
        mid = deck + (2 * edge + 0.002) * u            # the middle of the card where it should now lie
        yield from glide(sag, at(out, table + SKIM), 0.25, rot)
        sag.press = False                             # in the air again: the hand finds its sag here afresh
        yield from glide(sag, at(mid, table + SKIM), 0.3, rot)
        yield from glide(sag, at(mid, table + SKIM), 0.6, rot)
        sag.press = True
        sag_mid = float(sag.bias[2])
        sag_end = sag_mid if sag_end is None else sag_end
        yield from glide(sag, at(mid, table + 0.003), 0.25, rot)
        yield from glide(sag, at(mid, table - press), 0.5, rot)                # the last few millimetres slowly
        yield from glide(sag, at(mid, table - press), 0.2, rot)
        PHASE["name"] = f"deal {dealt + 1} slide"
        end = tgt
        # A place nearer the robot than the card lies is reached sideways first, then back, clear of the deck.
        path = [mid, np.array([mid[0], end[1]]), end] if end[0] < mid[0] - 0.02 else [mid, end]
        whole = sum(float(np.linalg.norm(b - a)) for a, b in zip(path, path[1:]))
        gone = 0.0
        for a, b in zip(path, path[1:]):
            dist = float(np.linalg.norm(b - a))
            n = max(2, round(dist / SPEED * CONTROL_HZ))
            for k in range(1, n + 1):
                f = ik.smoothstep(k / n)
                g = (gone + f * dist) / whole
                sag.bias[2] = sag_mid + g * (sag_end - sag_mid)  # the sag noted over the spot the card is going to
                sag.to(at(a + f * (b - a), table - press), rot)
                yield
            gone += dist
        yield from glide(sag, at(end, table - press), 0.2, rot)
    PHASE["name"] = f"deal {dealt + 1} lift"
    sag.press = False
    yield from glide(sag, at(end, T.TOP + 0.030), 0.4, rot)


def hover(T, name: str, xy) -> np.ndarray:
    return tool_for(name, (xy[0], xy[1], T.TOP + CARRY))


def script(T, left, right, mode: str = "full", idle_s: float = 60.0):
    """The demo as a generator: one yield per control step."""
    if mode == "still":
        yield from wait(idle_s)
        return
    ls, rs = Sag(left), Sag(right)
    SAGS.update(left=ls, right=rs)
    lr, rr = hand_rot("left"), hand_rot("right")
    deck = np.array(T.DECK_XY, float)
    park = {"left": hover(T, "left", np.array(T.PLACES[0]) + PARK), "right": hover(T, "right", np.array(T.PLACES[2]) + PARK * [1, -1])}
    over = {"left": hover(T, "left", deck), "right": hover(T, "right", deck)}
    PHASE["name"] = "ready"
    left.grip = right.grip = 1.0
    yield from both(go(ls, park["left"], lr, 1.8), go(rs, park["right"], rr, 1.8))

    def home(sag):
        yield from glide(sag, park[sag.hand.name], 1.2, lr if sag is ls else rr)

    def one(sag, place, round_, dealt, **kw):
        rot = lr if sag is ls else rr
        yield from glide(sag, over[sag.hand.name], 1.0, rot)
        yield from deal(T, sag, spot(T, place, round_), dealt, sag_end=CAL.get((sag.hand.name, place, round_)), **kw)

    if mode == "double":
        yield from one(rs, 2, 0, 0, low=float(os.environ.get("CARDS_LOW", "0.004")))
        yield from home(rs)
    elif mode == "heap":
        yield from one(rs, 2, 0, 0, low=0.010)
        yield from home(rs)
    else:
        plan = {"one": [0],                                    # the first card only
                "skip": [0, 2],                                # the near place left out
                "order": [1, 0, 2],                            # the near place first
                }.get(mode) or list(range(len(T.PLACES)))
        PHASE["name"] = "survey"

        def tour(sag, places):
            yield from survey(T, sag, [((p, 0), spot(T, p)) for p in places if p in plan])
            yield from home(sag)

        yield from both(tour(ls, (0,)), tour(rs, (2, 1)))    # right place first: the left hand is then out of the way
        for dealt, place in enumerate(plan):
            sag, other = (ls, rs) if place == 0 else (rs, ls)
            yield from both(one(sag, place, 0, dealt), home(other))
        yield from both(home(ls), home(rs))
    yield from wait(2.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 1000, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "full", every: int = 5, cam: tuple = (320, 176)) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam[0],
                               cam_height=cam[1], headless=not window)
    T = emb.task.module
    obs = emb.reset(Scene(id="cards-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode, idle_s=max_steps / CONTROL_HZ)
    frames: dict[str, list] = {}
    trace = []

    def snap_frames(o) -> None:
        if record:
            for name, img in o.images.items():
                frames.setdefault(name, []).append(np.asarray(img))

    def box() -> dict:
        from yam_arena.scene import object_states   # importable only once the simulator has started

        st = object_states(emb._env, list(T.CARDS))
        top = T.TOP
        out_ = {}
        # the six cards that are dealt, and the one under them: place (mm), turn (deg) and whether the back is up
        for n in T.CARDS[-7:]:
            p, q = st[n].pos, st[n].quat_xyzw
            r = T.K._rotation(q)
            out_[n[-2:]] = [round(1000 * float(p[0]), 1), round(1000 * float(p[1]), 1), round(1000 * float(p[2] - top), 2),
                            round(math.degrees(math.atan2(r[1, 1], r[0, 1])), 1), round(float(r[2, 2]), 2)]
        # the pile: how far any card under the top seven has moved sideways, and how far any card is from the height
        # it would have in an undisturbed pile (- : pressed into the cards under it)
        rest = T.CARDS[:-7]
        if rest:
            d = [float(np.hypot(st[n].pos[0] - T.DECK_XY[0], st[n].pos[1] - T.DECK_XY[1])) for n in rest]
            dz = [float(st[n].pos[2] - T.card_start(k)[0][2]) for k, n in enumerate(rest)]
            out_["pile"] = [round(1000 * max(d), 2), round(1000 * min(dz), 3), round(1000 * max(dz), 3)]
        return out_

    def measure() -> None:
        from cardkit import finger_frames, finger_points

        TOOL["left"], TOOL["right"] = (np.asarray(p, float) for p in finger_points(emb._env))
        TOOLROT["left"], TOOLROT["right"] = finger_frames(emb._env)

    def hands() -> dict:
        from cardkit import _np

        out_ = {}
        for h in (left, right):
            robot = emb._env.unwrapped.scene[f"{h.name}_robot"]
            q, jn = _np(robot.data.joint_pos)[0], list(robot.joint_names)
            out_[h.name] = {"tool_mm": [round(1000 * float(v), 1) for v in TOOL[h.name] - [0.0, 0.0, T.TOP]],
                            "want_mm": [round(1000 * float(v), 1) for v in h.pos - [0.0, 0.0, T.TOP]],
                            "gap_mm": round(1000 * sum(abs(float(q[jn.index(j)])) for j in ("joint7", "joint8")), 1),
                            "off_deg": round(SAGS[h.name].off, 2) if h.name in SAGS else None}
        return out_

    snap_frames(obs)
    measure()
    start = {"task": T.report(None), "box": box()}
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(Action(data=pack(left.cmd, right.cmd)))
        steps += 1
        measure()
        info = res.info
        snap_frames(res.observation)
        if steps % every == 0:
            trace.append({"step": steps, "phase": PHASE["name"], "score": info.get("score"), "task": T.report(None),
                          "hands": hands(), "box": box()})
        if steps % 20 == 0 and trace:
            t = trace[-1]
            print(f"step {steps} {t['phase']} score {t['score']} {t['task']} cards {t['box']} L {t['hands']['left']} R {t['hands']['right']}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    result = {"lot": T.LOT, "mode": mode, "press_mm": 1000 * PRESS, "tail_mm": 1000 * TAIL,
              "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "start": start, "end": {"task": T.report(None), "box": box()}, "trace": trace}
    if record:
        import imageio

        for name, fs in frames.items():
            _write_mp4(fs, out / f"demo_{name}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2, default=lambda o: o.item()))
    print(json.dumps({k: v for k, v in result.items() if k != "trace"}), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1000)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="full", choices=("full", "one", "skip", "order", "double", "heap", "still"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    ap.add_argument("--cam", default="320x176", help="recorded frame size, e.g. 640x352")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every,
        cam=tuple(int(v) for v in a.cam.split("x")))
