#!/usr/bin/env python3
"""Reference runs of the garlic press task.

    scripts/rta demo <id> mech/tasks/garlic_press [run,run,...] [--quick]

The right hand picks the clove up with a light pinch across its width (0.5 mm per finger past touching: about
2.5 N), carries it over the open press's chamber, turned lengthways, and lets it go with its fingertips 2 mm over the
rim. Then it pinches the open upper handle lightly across its width 45 mm from its end, carries it along its arc
from its stop (120 degrees) to 74 degrees, past where its weight turns it shut (84), and lets go: it falls onto the
clove. Meanwhile the left hand fetches the knife and holds it up. The right hand, closed, pushes the end of the lower handle until the
press stands 51 mm past the board's edge, its chamber over the saucer; sets its closed fingertips flat on the upper
handle's top face 15 mm from its end and presses, tilted half the handle's angle from straight down towards its grip
end (so that the fingertips do not slide on the face and the press does not skate on the board), following the
handle as it goes down (30 N asked at first, 10 N more every second up to 75 N, which the arm gives as about 66 N),
until the handle is closed. (A pinch across the handle presses only through the pads' friction: it slid down the
handle at 48 N, short of the clove's yield: run b3g8.) Then it pinches the closed press across both handles there
and lifts it 6 cm. The left hand turns the knife 65 degrees onto its edge, pointing along the press from its
tip, slides it in under the press, pushes that edge lightly up against the sieve and runs it across from one side of
the chamber to the other (the fingers then hold the knife by its handle's flat sides: a blade lying flat and pushed
up turned in them, trial b3g14). It takes the knife out, lays it flat with its underside, where the garlic is, across
the saucer's near rim and draws it back along its length, twice, the rim wiping the garlic into the saucer (laid on
the well instead, the blade, longer than the well, met the rim: b3g14). The right hand carries the lifted press
aside; the left lays the knife on the board and goes back to its ready pose; the right hand sets the press down near
where it was pressed, beside the knife, and goes back to its ready pose.

What the script reads from the simulator: where the clove, the press (both parts), the knife and the saucer are,
to put the hands on them (a person sees all that); the press's place on the board, to stop pushing it; how far the
handle is closed, to stop pressing (a person sees the handles meet); during the scrape, every 0.2 s, where the knife
and the press are, to keep the knife's edge along the sieve (a person watches the blade against it). Not the garlic's
amounts, not any force, not the score.

Runs. ``full``: 1.0. ``pressed``: as full, no scraping: the knife is not used (0.5). ``half``: the blade run only
part of the way across (0.75). ``in_chamber``: the clove in, the handle onto it, nothing pressed (0.25). ``across``:
the clove let go across the chamber, then as full. ``held``: as full, the right hand stays on the press (0.75).

``--quick``: every run's plan against the arms' reach and joint limits, without the simulator.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

RUNS = ("full", "pressed", "half", "in_chamber", "across", "held")
TIP = 0.0129  # m the fingertips' ends reach past the grasp point
UP = 0.06  # m over a grasp before and after
OPENING = 0.0939  # m between the pads fully open (grip 1)
PINCH = 0.0005  # m per finger past touching for the clove: about 2.5 N per pad (5,000 N/m)
PUSH_FIRST, PUSH_MORE, PUSH_MOST = 30.0, 1.0, 75.0  # N asked at first, more every LOOK_S, at most (the arm then gives about 66 N: trial b3g4)
PRESS_U = 0.140  # where the right hand presses the upper handle and pinches the closed press to lift it: 15 mm from its end
HANDLE_TOP_W = 0.023  # the upper handle's top face (closed coordinates)
PAD_HALF = 0.0135  # m: half the pads' width, which lies along the handle when the fingers close across it
HALF_PART = 0.35  # the run ``half``: the edge run 35 % of the way across, past two of the five columns of holes (12 strands):
# about 49 % on the saucer, 0.75 (planned, not recorded: one full-length recording per task; 0.75 is shown by the rule
# test)
LIFT_W = 0.014  # the grasp point over the press's underside when it is pinched to be lifted: the fingertips 1 mm over it
LOOK_S = 0.1  # s between two looks while sliding and pressing
CLOSED_DEG = 2.0  # the handle this near closed: pressed through
SLIDE_STEP = 0.005  # m a look while pushing the press to the board's edge
SCRAPE_UP_N = 2.0  # N asked to push the knife's edge up against the sieve: the task takes a strand only with 0.5 N or more on it,
# and the arm's joints hold back part of a small push (1 N asked gave 0.5 to 0.9 N: b3g15); 2 N about 9 cm from the right
# hand's grip turns the press there less than its own weight does the other way
SCRAPE_SIDE = 0.0115  # m: the edge run across from this far on one side of the sieve's middle to as far on the other (holes to 9.75 mm, walls at 12)
SCRAPE_S = 2.5  # s for the run across
SCRAPE_OUT = 0.10  # m the knife is slid in along its length under the press (its tip first clear of the press's tip)
SCRAPE_LOOK_S = 0.2  # s between two looks while the knife is lined up with the sieve and run across it
SCRAPE_GAIN = 0.8  # the share of the knife's error against the sieve taken out at each look
SCRAPE_TURN_MOST, SCRAPE_MOVE_MOST = math.radians(1.0), 0.003  # at most this much of a correction at one look
SCRAPE_ROLL_DEG = 65.0  # the knife turned this far onto its +y edge for the scrape: the fingers' pads then hold most of the turn
# a push gives it, which their friction alone did not (b3g14); 75 leaves the elbow no room under the press, 90 is past the wrist's stop (dry runs)
WIPE_DOWN_N = 1.5  # N asked to press the blade on the saucer's rim (more than about 3 N on its rim tips the saucer)
WIPE_MARGIN = 0.004  # m the rim starts short of the garlic's band along the blade and ends past it
WIPE_S = 2.5  # s for each draw back over the rim
WIPE_PASSES = 2
SHOW_BACK = 0.06  # m the knife is drawn back after the wipe, clear of the saucer, for the top camera's look at the garlic on it
KNIFE_IN = 0.25  # m the knife is slid in along its length to its place on the board
PRESS_ASIDE = np.array([0.10, 0.0, 0.04])  # m the lifted press is carried aside while the knife goes onto the board
KNIFE_DOWN_X = 0.225  # m: the used knife's middle line where it is laid on the board, along its near strip (the press lies from x 0.267)
LIFT = 0.06  # m the closed press is lifted over the saucer
DRY_PHI_ON_CLOVE = math.radians(34.5)  # the dry run's handle on the clove (test of the arrival)
HANDLE_U, HANDLE_W = 0.110, 0.017  # where the right hand pinches the open upper handle (its middle, 45 mm from its end)
OVER_DEG = (120.0, 112.0, 104.0, 96.0, 90.0, 84.0, 78.0, 74.0)  # the handle carried over: its weight turns it shut below 84
HANDLE_WIDTH = 0.026


def partial_turn(rotation: np.ndarray, share: float, most: float = math.pi) -> np.ndarray:
    """``share`` of a rotation about its own axis, at most ``most`` rad (for the small turns of a correction)."""
    angle = math.acos(max(-1.0, min(1.0, (float(np.trace(rotation)) - 1.0) / 2.0)))
    if angle < 1e-9:
        return np.eye(3)
    axis = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]])
    import demo_kit as kit

    return kit.about(axis, min(share * angle, most))


class Plan:
    def __init__(self, m, see=None) -> None:
        import demo_kit as kit

        self.m, self.see, self.kit, self.Move = m, see, kit, kit.Move
        self.dry = {"shift": 0.0, "lift": 0.0, "phi": m.OPEN_MOST, "knife": None}  # the dry run's world: what the plan has done
        self.held_knife = None  # (offset, rotation) of the knife in the left hand's frame, once it holds it

    # --- where things are: from the simulator, or in the dry run from what the plan has done ---
    def look(self) -> dict | None:
        return self.see() if self.see is not None else None

    def lower(self) -> tuple[np.ndarray, np.ndarray]:
        seen = self.look()
        if seen is not None:
            return seen["press"]
        return self.m.START_HINGE + np.array([0.0, self.dry["shift"], self.dry["lift"]]), np.eye(3)

    def phi(self) -> float:
        seen = self.look()
        return float(seen["hinge"]) if seen is not None else self.dry["phi"]

    def on_lower(self, u: float, v: float, w: float) -> np.ndarray:
        position, rotation = self.lower()
        return position + rotation @ np.array(self.m.uvw(u, v, w))

    def on_upper(self, u: float, v: float, w: float) -> np.ndarray:
        seen = self.look()
        if seen is not None:
            position, rotation = seen["upper"]
        else:
            position, rotation = self.m.mk.joint_pose(self.m.mk.MECH.joints["hinge"], self.phi(), self.lower())
        return position + rotation @ np.array(self.m.uvw(u, v, w))

    def clove(self) -> tuple[np.ndarray, np.ndarray]:
        seen = self.look()
        if seen is not None:
            return seen["clove"]
        k, side, yaw, x, y = self.m.STATE["lie"]
        position, quat = self.m.clove_pose(k, side, yaw, x, y)
        return position, self.m.mk.quat_to_matrix(quat)

    def knife(self) -> tuple[np.ndarray, np.ndarray]:
        seen = self.look()
        if seen is not None:
            return seen["knife"]
        if self.dry["knife"] is not None:
            return self.dry["knife"]
        return np.array([*self.m.KNIFE_AT, self.m.TOP + self.m.KNIFE_HANDLE[2] / 2]), np.eye(3)

    def saucer(self) -> np.ndarray:
        seen = self.look()
        return seen["saucer"][0] if seen is not None else np.array([*self.m.SAUCER_XY, self.m.TOP])

    def hand(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        seen = self.look()
        return seen["hands"][side] if seen is not None else (None, None)

    # --- the steps ---
    def pick_clove(self):
        m, Move, down = self.m, self.Move, self.kit.down
        k = m.CLOVES.index(m.STATE["clove"])
        _, width, thick, _ = m.CLOVE_SIZES[k]
        side = m.STATE["lie"][1]
        across = thick if side else width  # what the fingers close across
        position, rotation = self.clove()
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])  # its length's direction; the fingers close across it
        grasp = np.array([position[0], position[1], m.TOP + m.BOARD_T + TIP + 0.001])
        up = np.array([0.0, 0.0, UP])
        self.pinch = (across - 2 * PINCH) / OPENING
        self.wide = min(1.0, (across + 0.024) / OPENING)
        yield {"right": [Move(2.0, pos=grasp + up, rot=down(yaw), grip=self.wide, label="over the clove"), Move(1.2, pos=grasp, label="down to it"),
                         Move(0.8, grip=self.pinch, label="pinch"), Move(0.8, pos=grasp + up, label="lift it")]}  # fmt: skip

    def drop_clove(self, across: bool = False):
        """Over the chamber, lengthways (or across it), let go with the fingertips 2 mm over the rim."""
        m, Move, down = self.m, self.Move, self.kit.down
        m_mid = (m.CHAMBER_U[0] + m.CHAMBER_U[1]) / 2
        _, rotation = self.lower()
        along = math.atan2(-rotation[1, 1], -rotation[0, 1])  # the press's line, tip to grip end: its parts' -y
        yaw = along + (math.pi / 2 if across else 0.0)
        _, held = self.clove()
        now = math.atan2(held[1, 0], held[0, 0])  # the clove's (and the hand's) turn now
        yaw = min((yaw, yaw + math.pi, yaw - math.pi), key=lambda a: abs(math.remainder(a - now, 2 * math.pi)))  # the nearer way round
        target = self.on_lower(m_mid, 0.0, m.RIM_W + 0.002 + TIP)
        correction = np.zeros(3)
        hand_at, hand_rot = self.hand("right")
        if hand_at is not None:  # where the clove sits in the fingers, carried over to the new turn of the hand
            clove_at, _ = self.clove()
            in_hand = hand_rot.T @ (clove_at - hand_at)
            new_rot = down(yaw)
            correction = new_rot @ in_hand
            correction[2] = 0.0
        place = target - correction
        up = np.array([0.0, 0.0, UP])
        yield {"right": [Move(2.5, pos=place + up, rot=down(yaw), label="over the chamber"), Move(1.2, pos=place, label="down to the rim"),
                         Move(0.6, grip=self.wide, label="let it go"), Move(0.8, pos=place + up, label="up")]}  # fmt: skip

    def handle_over(self):
        """Right hand, pointing down: a light pinch across the open upper handle (0.5 mm per finger past touching) at
        HANDLE_U, carried along the handle's arc through OVER_DEG and let go; the handle falls onto the clove. The
        handle turns between the pads as it goes. The hand comes to it level, from 40 mm on the grip end's side (from
        above it cannot reach), and leaves 30 mm up. (From the saucer's side the left hand could not: reach4, reach6.)"""
        m, Move, down = self.m, self.Move, self.kit.down
        lower, rotation = self.lower()
        yaw = math.atan2(rotation[1, 0], rotation[0, 0]) + math.pi / 2  # the fingers close across the handle (along the press's v)
        arc = []
        for deg in OVER_DEG:
            u, w = m.opened(HANDLE_U, HANDLE_W, math.radians(deg))
            arc.append(lower + rotation @ np.array(m.uvw(u, 0.0, w)))
        gripward = rotation @ np.array([0.0, -1.0, 0.0])  # along the press, towards its grip end
        wide, light = (HANDLE_WIDTH + 0.024) / OPENING, (HANDLE_WIDTH - 2 * PINCH) / OPENING
        moves = [Move(1.0, rot=down(yaw), grip=wide, label="turn across"), Move(1.8, pos=arc[0] + 0.040 * gripward, label="beside the open handle"),
                 Move(1.0, pos=arc[0], label="round it"), Move(0.6, grip=light, label="pinch it lightly")]  # fmt: skip
        moves += [Move(0.5, pos=p, label="bring it over") for p in arc[1:]]
        moves += [Move(0.5, grip=wide, label="let it go"), Move(0.8, pos=arc[-1] + np.array([0.0, 0.0, 0.030]), label="up"),
                  Move(1.5, pos=np.array([0.20, -0.25, m.TOP + 0.16]), rot=down(0.0), label="away")]  # fmt: skip
        yield {"right": moves}
        self.dry["phi"] = DRY_PHI_ON_CLOVE

    @staticmethod
    def together(*plans) -> dict:
        """One step out of one-step plans for different hands, each run to its end."""
        step = {}
        for plan in plans:
            for one in plan:
                step.update(one)
        return step

    def fetch_knife(self):
        m, Move, down = self.m, self.Move, self.kit.down
        position, rotation = self.knife()
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])  # the knife's length; the fingers close across its handle
        grasp = np.array([position[0], position[1], m.TOP + TIP + 0.001])
        up = np.array([0.0, 0.0, UP])
        yield {"left": [Move(2.0, pos=grasp + up, rot=down(yaw), grip=0.45, label="over the knife"), Move(1.2, pos=grasp, label="down to it"),
                        Move(0.8, grip=0.0, label="close"), Move(1.0, pos=grasp + np.array([0.0, 0.0, 0.10]), label="lift it")]}  # fmt: skip
        self.dry["knife"] = (grasp + np.array([0.0, 0.0, 0.10 - TIP - 0.001 + m.KNIFE_HANDLE[2] / 2]), np.eye(3))

    def grip_knife(self) -> None:
        """Note how the knife sits in the left hand (the simulator's poses; in the dry run as grasped)."""
        hand_at, hand_rot = self.hand("left")
        position, rotation = self.knife()
        if hand_at is None:
            hand_rot = self.kit.down(0.0)
            hand_at = position + np.array([0.0, 0.0, TIP + 0.001 - self.m.KNIFE_HANDLE[2] / 2])
        self.held_knife = (hand_rot.T @ (position - hand_at), hand_rot.T @ rotation)

    def hand_for_knife(self, position: np.ndarray, rotation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """The left hand's grasp point and turn that put the held knife at ``position`` turned ``rotation``."""
        offset, turn = self.held_knife
        hand_rot = rotation @ turn.T
        return position - hand_rot @ offset, hand_rot

    def slide(self):
        """Right hand, closed: the end of the lower handle pushed until the press stands PRESSING_OVERHANG past the
        board's edge."""
        m, Move, down = self.m, self.Move, self.kit.down
        end = self.on_lower(0.155, 0.0, 0.003)
        _, rotation = self.lower()
        towards = rotation @ np.array([0.0, 1.0, 0.0])  # along the press, towards its tip
        behind = end - towards * (0.0064 + 0.002) + np.array([0.0, 0.0, TIP])
        yield {"right": [Move(2.0, pos=behind + np.array([0.0, 0.0, UP]), rot=down(0.0), grip=0.0, label="over the press's end"), Move(1.2, pos=behind, label="down behind it")]}
        pushed = 0.0
        while pushed < 0.080:
            seen = self.look()
            now = seen["overhang"] if seen is not None else m.START_TIP_Y + self.dry["shift"] - m.BOARD_Y[1]
            gap = m.PRESSING_OVERHANG - now
            if gap <= 0.0005:
                break
            step = min(SLIDE_STEP, gap + 0.0005)
            pushed += step
            self.dry["shift"] += step
            yield {"right": [Move(0.25, pos=behind + towards * pushed, label="push the press")]}
        yield {"right": [Move(0.6, pos=behind + towards * (pushed - 0.010), label="off it"), Move(0.8, pos=behind + towards * (pushed - 0.010) + np.array([0.0, 0.0, UP]), label="up")]}

    def press_pose(self, asked: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """(grasp point, turn, push, touching point from the grasp point in the gripper's frame) for the closed
        fingertips on the upper handle's top face: the hand pointing straight down, the fingers closing across the
        handle; the fingertips' ends level, their edge towards the grip end on the face at PRESS_U (the face rises
        that way, so the rest of the ends stand clear of it). The push ``asked`` N, tilted half the handle's angle from
        straight down towards its grip end. (Pointing into the face, the wrist would go past its stop under the
        push.)"""
        m, down = self.m, self.kit.down
        _, rotation = self.lower()
        yaw = math.atan2(rotation[1, 0], rotation[0, 0]) + math.pi / 2  # the fingers close across the handle
        rot = down(yaw)
        seen = self.look()
        if seen is not None:
            position, turn = seen["upper"]
        else:
            position, turn = m.mk.joint_pose(m.mk.MECH.joints["hinge"], self.phi(), self.lower())
        along = turn @ np.array([0.0, -1.0, 0.0])  # along the handle, towards its grip end
        flat = np.array([along[0], along[1], 0.0]) / max(1e-9, math.hypot(along[0], along[1]))
        touch = position + turn @ np.array(m.uvw(PRESS_U, 0.0, HANDLE_TOP_W))
        grasp = touch - PAD_HALF * flat + np.array([0.0, 0.0, TIP])
        tilt = 0.5 * self.phi()
        push = asked * (math.sin(tilt) * flat - math.cos(tilt) * np.array([0.0, 0.0, 1.0]))
        return grasp, rot, push, rot.T @ (touch - grasp)

    def press(self):
        """Right hand, fingers closed: the fingertips set on the upper handle's top face 15 mm from its end and
        pressed until the handle is closed (see the module's text)."""
        Move = self.Move
        up = np.array([0.0, 0.0, 1.0])
        grasp, rot, _, at = self.press_pose(0.0)
        yield {"right": [Move(2.0, pos=grasp + 0.04 * up, rot=rot, grip=0.0, label="over the handle"), Move(1.2, pos=grasp + 0.002 * up, label="down to it"),
                         Move(0.4, pos=grasp, label="on it")]}  # fmt: skip
        asked, t = PUSH_FIRST, 0.0
        while self.phi() > math.radians(CLOSED_DEG) and t < 30.0:
            asked = min(PUSH_MOST, asked + PUSH_MORE)
            t += LOOK_S
            if self.see is None:
                self.dry["phi"] = max(0.0, self.dry["phi"] - math.radians(1.5))
            grasp, rot, push, at = self.press_pose(asked)
            yield {"right": [Move(LOOK_S, pos=grasp, rot=rot, push=push, push_at=at, label="press")]}
        grasp, rot, push, at = self.press_pose(asked)
        yield {"right": [Move(1.0, pos=grasp, rot=rot, push=push, push_at=at, label="press home"), Move(0.5, push=(0.0, 0.0, 0.0), label="ease"),
                         Move(0.8, pos=grasp + 0.04 * up, label="off the handle")]}  # fmt: skip
        self.dry["phi"] = 0.0

    def lift(self):
        """Right hand: a pinch across both handles of the closed press 15 mm from their ends, and up 6 cm."""
        Move, down = self.Move, self.kit.down
        _, rotation = self.lower()
        yaw = math.atan2(rotation[1, 0], rotation[0, 0]) + math.pi / 2  # the fingers close across the handles
        grasp = self.on_lower(PRESS_U, 0.0, LIFT_W)
        wide = (HANDLE_WIDTH + 0.024) / OPENING
        yield {"right": [Move(1.5, pos=grasp + np.array([0.0, 0.0, UP]), rot=down(yaw), grip=wide, label="over the handles"), Move(1.2, pos=grasp, label="down beside them"),
                         Move(0.6, grip=0.0, label="close on both"), Move(1.5, pos=grasp + np.array([0.0, 0.0, LIFT]), label="lift the press")]}  # fmt: skip
        self.dry["lift"] = LIFT

    def sieve_at(self) -> float:
        """Where along the knife (its frame's x) the sieve's middle comes: the blade's middle."""
        return (self.m.BLADE_X[0] + self.m.BLADE_X[1]) / 2

    def scrape(self, part: float = 1.0):
        """Left hand: the knife turned SCRAPE_ROLL_DEG onto its +y edge, pointing along the press from its tip; slid in
        under the press 15 mm below the sieve and lined up with it; the top corner of that edge pushed up against the
        sieve's underside and run across from one side of the chamber to the other (``part`` of the way); then down and
        out. Every SCRAPE_LOOK_S the knife's and the press's poses are read and the hand's aim corrected, so that the
        knife lies along the sieve as the press now stands, that corner on it (aimed once, the wrist's give and the
        press's turn in the other hand left it 2 degrees off, touching with its tip only: b3g15). The underside, which
        faces the way the knife runs, takes the garlic."""
        m, Move = self.m, self.Move
        self.grip_knife()
        _, rotation = self.lower()
        a = math.radians(SCRAPE_ROLL_DEG)
        in_press = np.column_stack([(0.0, -1.0, 0.0), (math.cos(a), 0.0, math.sin(a)), (-math.sin(a), 0.0, math.cos(a))])  # the knife in the press's frame
        knife_rot = rotation @ in_press
        up, along = rotation @ np.array([0.0, 0.0, 1.0]), rotation @ np.array([0.0, -1.0, 0.0])
        corner = np.array([self.sieve_at(), m.KNIFE_BLADE[1] / 2, m.BLADE_Z[1]])  # the top face's corner on the +y edge, under the sieve's middle (knife frame)
        mid_u = (m.CHAMBER_U[0] + m.CHAMBER_U[1]) / 2

        def on_path(s: float, w: float) -> np.ndarray:  # that corner's place in the press's frame: ``s`` of the way across, ``w`` over the sieve's underside
            return np.array(m.uvw(mid_u, -SCRAPE_SIDE + 2 * SCRAPE_SIDE * part * s, w))

        def world(point: np.ndarray) -> np.ndarray:
            at, turn = self.lower()
            return at + turn @ point

        start = world(on_path(0.0, -0.015))
        hand_out, rot = self.hand_for_knife(start - SCRAPE_OUT * along - knife_rot @ corner, knife_rot)
        hand_in, _ = self.hand_for_knife(start - knife_rot @ corner, knife_rot)
        push_at = self.held_knife[1] @ corner + self.held_knife[0]  # where it pushes: that corner, in the hand's frame as held
        yield {"left": [Move(2.5, pos=hand_out, rot=rot, label="onto its edge"), Move(1.2, pos=hand_in, label="in under the press")]}
        aim = [hand_in, rot]

        def correct(target: np.ndarray, full: bool) -> None:
            """The aim turned about the knife's corner by SCRAPE_GAIN of the turn the knife lacks against the press as it
            now stands, and moved by SCRAPE_GAIN of the corner's miss of ``target`` (press frame): all of it (``full``),
            or only the part square to the sieve, and then only towards the sieve (a corner on the sieve is not pushed
            further in: the push does that)."""
            seen = self.look()
            if seen is None:
                return
            (press_at, press_rot), (knife_at, knife_now) = seen["press"], seen["knife"]
            turn = partial_turn(press_rot @ in_press @ knife_now.T, SCRAPE_GAIN, SCRAPE_TURN_MOST)
            at = knife_at + knife_now @ corner
            miss = press_at + press_rot @ target - at
            if not full:
                normal = press_rot @ np.array([0.0, 0.0, 1.0])
                miss = max(0.0, float(miss @ normal)) * normal
            move = SCRAPE_GAIN * miss
            if np.linalg.norm(move) > SCRAPE_MOVE_MOST:
                move *= SCRAPE_MOVE_MOST / np.linalg.norm(move)
            aim[0] = at + turn @ (aim[0] - at) + move
            aim[1] = turn @ aim[1]

        for _ in range(3):
            correct(on_path(0.0, -0.015), full=True)
            yield {"left": [Move(SCRAPE_LOOK_S, pos=aim[0], rot=aim[1], label="line up")]}
        correct(on_path(0.0, -0.015), full=True)
        _, press_rot = self.lower()
        normal = press_rot @ np.array([0.0, 0.0, 1.0])
        aim[0] = aim[0] + 0.0155 * normal  # from 15 mm under the sieve to 0.5 mm into it
        yield {"left": [Move(0.8, pos=aim[0], rot=aim[1], push=tuple(SCRAPE_UP_N * normal), push_at=push_at, label="against it")]}
        n = max(1, int(round(SCRAPE_S * part / SCRAPE_LOOK_S)))
        for k in range(n):
            correct(on_path(k / n, 0.0), full=False)
            _, press_rot = self.lower()
            aim[0] = aim[0] + press_rot @ (on_path((k + 1) / n, 0.0) - on_path(k / n, 0.0))
            yield {"left": [Move(SCRAPE_LOOK_S, pos=aim[0], rot=aim[1], label="across")]}
        _, press_rot = self.lower()
        normal = press_rot @ np.array([0.0, 0.0, 1.0])
        yield {"left": [Move(0.4, push=(0.0, 0.0, 0.0), label="ease"), Move(0.8, pos=aim[0] - 0.020 * normal, label="down off it"),
                        Move(1.2, pos=aim[0] - 0.020 * normal - SCRAPE_OUT * along, label="out")]}  # fmt: skip
        end = world(on_path(1.0, -0.020)) - SCRAPE_OUT * along
        self.dry["knife"] = (end - knife_rot @ corner, knife_rot)

    def wipe(self):
        """Left hand: the knife laid flat, pointing the way it scraped, its underside (where the garlic is, along its +y
        edge) across the saucer's near rim short of the garlic's band; pressed and drawn back along its length until
        the rim has passed the whole band, the rim wiping the garlic towards the tip, into the saucer; up, over and
        again (WIPE_PASSES)."""
        m, Move = self.m, self.Move
        _, held = self.knife()
        along = held[:, 0] - held[2, 0] * np.array([0.0, 0.0, 1.0])  # the way the knife points, level
        along /= np.linalg.norm(along)
        turn = self.kit.about((0.0, 0.0, 1.0), math.atan2(along[1], along[0]))  # flat, pointing that way
        rim = self.saucer() + np.array([0.0, 0.0, m.SAUCER_H]) - m.RIM_OUTER * along  # the near rim's top edge, under the knife
        band = (self.sieve_at() - 0.0145 - m.BAND, self.sieve_at() + 0.0145 + m.BAND)  # where the garlic rides along the blade (the holes' rows)
        lo, hi = band[0] - WIPE_MARGIN, band[1] + WIPE_MARGIN

        def hand_at(x: float, lift: float) -> tuple[np.ndarray, np.ndarray]:  # the underside at ``x`` along the blade ``lift`` over the rim
            return self.hand_for_knife(rim - turn @ np.array([x, 0.0, m.BLADE_Z[0]]) + np.array([0.0, 0.0, lift]), turn)

        hand_over, rot = hand_at(lo, 0.015)
        hand_on, _ = hand_at(lo, -0.0005)
        hand_end, _ = hand_at(hi, -0.0005)
        hand_up, _ = hand_at(hi, 0.015)
        hand_high, _ = hand_at(hi, 0.070)  # at the end 7 cm over the rim: the knife then goes over the press, set down meanwhile, to the board
        push_at = self.held_knife[1] @ np.array([(lo + hi) / 2, 0.0, m.BLADE_Z[0]]) + self.held_knife[0]
        moves = [Move(2.5, pos=hand_over, rot=rot, label="flat over the rim")]
        for k in range(WIPE_PASSES):
            again = " again" if k else ""
            if k:
                moves.append(Move(1.2, pos=hand_over, label="over" + again))
            moves += [Move(0.6, pos=hand_on, label="on the rim" + again), Move(0.5, push=(0.0, 0.0, -WIPE_DOWN_N), push_at=push_at, label="press" + again),
                      Move(WIPE_S, pos=hand_end, label="draw it back" + again), Move(0.4, push=(0.0, 0.0, 0.0), label="ease" + again),
                      Move(0.6, pos=hand_up if k < WIPE_PASSES - 1 else hand_high, label="up" + again)]  # fmt: skip
        clear, _ = self.hand_for_knife(rim - turn @ np.array([hi, 0.0, m.BLADE_Z[0]]) - SHOW_BACK * along + np.array([0.0, 0.0, 0.070]), turn)
        moves += [Move(0.8, pos=clear, label="clear of the saucer"), Move(0.5, label="the saucer seen")]  # the top camera sees the garlic on it
        yield {"left": moves}
        self.dry["knife"] = (rim - turn @ np.array([hi, 0.0, m.BLADE_Z[0]]) - SHOW_BACK * along + np.array([0.0, 0.0, 0.070]), turn)

    def lay_knife(self):
        """Left hand: the used knife laid flat on the board (not on the bare bench), along its near strip where the press
        is set down after it, pointing right, its handle at the board's left end: brought first to its line over the
        saucer's side, then slid in along its length; let go, up and back to the ready pose. The right hand holds the
        lifted press aside meanwhile (``press_aside``), so the score cannot reach 1.0 while this arm crosses the top
        camera's view (laying the knife there, it covers saucer and press)."""
        m, Move = self.m, self.Move
        place = np.array([KNIFE_DOWN_X, m.BOARD_Y[1] - 0.003 - m.KNIFE_HANDLE[0] / 2, m.TOP + m.BOARD_T + m.KNIFE_HANDLE[2] / 2 + 0.002])
        turn = self.kit.about((0.0, 0.0, 1.0), -math.pi / 2)
        hand, rot = self.hand_for_knife(place, turn)
        yield {"left": [Move(2.0, pos=hand + np.array([0.0, KNIFE_IN, 0.06]), rot=rot, label="beside the board"), Move(1.2, pos=hand + np.array([0.0, 0.0, 0.06]), label="over the board"),
                        Move(1.2, pos=hand, label="down"), Move(0.6, grip=0.45, label="let it go"), Move(0.3, pos=hand + np.array([0.0, 0.0, 0.015]), label="up"),
                        Move(2.0, pos=self.ready("left")[0], rot=self.ready("left")[1], label="ready pose")]}  # fmt: skip
        self.dry["knife"] = (place, turn)

    def set_down(self, then: str = "ready"):
        """Right hand: the closed press set down where it was pressed; then (``then``) let go, up and back to its ready
        pose ("ready"), or the hand stays on it ("stay")."""
        m, Move = self.m, self.Move
        grasp = self.on_lower(PRESS_U, 0.0, LIFT_W)
        down_at = grasp - np.array([0.0, 0.0, LIFT - 0.001])
        self.dry["lift"] = 0.0
        moves = [Move(1.5, pos=down_at, label="set it down"), Move(0.4, label="down")]
        if then == "stay":
            moves.append(Move(3.0, label="hand stays"))
        else:
            ready_at, ready_turn = self.ready("right")
            moves += [Move(0.6, grip=0.6, label="let go"), Move(1.0, pos=down_at + np.array([0.0, 0.0, 0.08]), label="up"), Move(1.5, pos=ready_at, rot=ready_turn, label="ready pose")]
        yield {"right": moves}

    def press_aside(self):
        """Right hand: the lifted press carried PRESS_ASIDE (towards the far side and up), clear of the left arm while it
        lays the knife (holding it where it was lifted, or waiting by it, the arms met: b3g21)."""
        self.lifted_grasp = self.on_lower(PRESS_U, 0.0, LIFT_W)
        yield {"right": [self.Move(1.2, pos=self.lifted_grasp + PRESS_ASIDE, label="the press aside")]}

    def set_down_last(self):
        """Right hand, last: the press back over where it was pressed and set down there, beside the knife; let go
        (fingers opened just enough, the knife 15 mm off), up and back to the ready pose. The score reaches 1.0 as the
        hand passes 3 cm from the press and the trial ends a second later; this arm's way back crosses none of the top
        camera's lines to saucer, press and knife (projected), and the left arm is already in its ready pose."""
        m, Move = self.m, self.Move
        down_at = self.lifted_grasp - np.array([0.0, 0.0, LIFT - 0.001])
        self.dry["lift"] = 0.0
        ready_at, ready_turn = self.ready("right")
        yield {"right": [Move(1.2, pos=self.lifted_grasp, label="back over its place"), Move(1.5, pos=down_at, label="set it down"), Move(0.4, label="down"),
                         Move(0.6, grip=0.4, label="let go"), Move(0.6, pos=down_at + np.array([0.0, 0.0, 0.05]), label="up"),
                         Move(1.4, pos=ready_at, rot=ready_turn, label="ready pose")]}  # fmt: skip

    def ready(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        """The grasp point and turn of ``side``'s hand in its ready pose (the pose a trial starts from)."""
        from yam_arena import ik, station
        from yam_arena.constants import READY

        return ik.tcp_pose(np.array(READY, dtype=float), np.asarray(station.BASES[side], dtype=float))

    def steps(self, name: str):
        yield from self.pick_clove()
        yield from self.drop_clove(across=name == "across")
        if name in ("pressed", "in_chamber"):
            yield from self.handle_over()
        else:
            yield self.together(self.handle_over(), self.fetch_knife())  # the left hand fetches the knife meanwhile
        if name == "in_chamber":
            yield {"left": [self.Move(1.5, pos=np.array([0.20, 0.25, self.m.TOP + 0.16]), label="away")]}
            return
        yield from self.slide()
        yield from self.press()
        yield from self.lift()
        if name == "pressed":
            yield from self.set_down()
            return
        yield from self.scrape(part=HALF_PART if name == "half" else 1.0)
        yield from self.wipe()
        if name == "held":
            yield from self.set_down(then="stay")
            yield from self.lay_knife()
            return
        yield from self.press_aside()  # the right hand holds the lifted press aside while the knife goes onto the board
        yield from self.lay_knife()
        yield from self.set_down_last()  # then the press down beside it, and the right hand goes home last

    def whole(self, name: str) -> dict:
        """The plan as one list of moves per arm (dry run): each step's arms kept in step with each other."""
        out = {"left": [], "right": []}
        for step in self.steps(name):
            longest = max(sum(move.seconds for move in moves) for moves in step.values())
            for side in ("left", "right"):
                moves = list(step.get(side, []))
                rest = longest - sum(move.seconds for move in moves)
                if rest > 1e-9:
                    moves.append(self.Move(rest, label="waits"))
                out[side].extend(moves)
        return out


def seer(m):
    """What the script reads from the simulator (see the module's text): a function giving the poses of the clove, the
    press's two parts, the knife, the saucer and the hands, how far the handle is open and the press's place."""
    mk = m.mk

    def hand_turn(side: str) -> np.ndarray:
        """The hand's turn as it stands: its gripper link's (the grasp point's frame), read as the kit reads the grasp point."""
        robot = mk.MECH.environment.unwrapped.scene[f"{side}_robot"]
        link = robot.data.body_link_pose_w.torch[0, robot.find_bodies("gripper")[0][0]].detach().cpu().numpy().astype(float)
        return mk.quat_to_matrix(link[3:7])

    def see() -> dict:
        hands = {side: (mk.gripper(side)[0], hand_turn(side)) for side in ("left", "right")}
        return {"press": mk.pose("press"), "upper": mk.pose("upper"), "hinge": mk.joint("hinge"), "clove": mk.pose(m.STATE["clove"]), "knife": mk.pose("knife"),
                "saucer": mk.pose("saucer"), "overhang": m.overhang(), "hands": hands}  # fmt: skip

    return see


def main() -> int:
    import demo_kit as kit

    quick = "--quick" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--quick"]
    out = Path(args[0])
    runs = args[1].split(",") if len(args) > 1 else ["full"]
    if any(run not in RUNS for run in runs):
        raise SystemExit(f"a run is one of {sorted(RUNS)}; got {runs}")
    if quick:
        from yam_arena import station, task_file

        m = task_file.load(str(HERE / "task.py")).module
        for run in runs:
            m.begin(0)
            print(f"DRY {run}: {json.dumps(kit.dry_run(Plan(m).whole(run), station.BASES, limit_s=m.RUN_LIMIT_STEPS / 20.0), default=str)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    failed = []
    for run in runs:
        try:

            plan = Plan(m, seer(m))
            script = plan.steps(run)
            state = {"over": False}

            def director(t: float, arms: dict, pictures, script=script, state=state, plan=plan) -> None:
                if pictures is None or mk.MECH.environment is None or not mk.MECH.poses:
                    return
                while not state["over"] and t >= max(arm.seconds for arm in arms.values()) - 1e-6:  # the next step is planned when both arms are through
                    step = next(script, None)
                    if step is None:
                        state["over"] = True
                        break
                    longest = max(sum(move.seconds for move in moves) for moves in step.values())
                    for side in ("left", "right"):
                        moves = list(step.get(side, []))
                        rest = longest - sum(move.seconds for move in moves)
                        if rest > 1e-9:
                            moves.append(plan.Move(rest, label="waits"))
                        arms[side].moves.extend(moves)

            def extra() -> dict:
                a = m.amounts()
                return {"hinge (deg)": round(math.degrees(mk.joint("hinge")), 2), "face (mm)": round(1000 * m.face_height(mk.joint("hinge")), 2), "amounts (g)": a,
                        "overhang (mm)": round(1000 * m.overhang(), 1), "touched": m.STATE["touched"], "level": m.level(), "at rest": m.at_rest(),
                        "hands": {side: [round(float(v), 4) for v in mk.gripper(side)[0]] for side in ("left", "right")},
                        "clove (m)": [round(float(v), 4) for v in mk.pose(m.STATE["clove"])[0]]}  # fmt: skip

            folder = out / run
            result = kit.run(embodiment, {"left": [], "right": []}, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=1.5, sheet_frames=16)
            rows = result["rows"]
            scores = [row["info"].get("score", 0.0) for row in rows]
            firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 0.75, 1.0)}
            readings = {"score": scores[-1], "first reached": firsts, "seconds": rows[-1]["t"], "ended": result.get("termination_reason"), "report": result["report"], "worst IK": result["worst_ik"],
                        "events": m.STATE["events"]}  # fmt: skip
            (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
            print(f"DEMO {run}: score {scores[-1]:.2f} after {result['seconds']:.1f} s ({result['steps']} steps of {m.RUN_LIMIT_STEPS}); first reached {firsts}; report {json.dumps(result['report'], default=str)}", flush=True)
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"RUN {run} FAILED: {error!r}", flush=True)
            failed.append(run)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
