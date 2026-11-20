#!/usr/bin/env python3
"""Reference run of the inhaler task: the safe way, scripted.

    scripts/rta demo <id> mech/tasks/inhaler_coin [--check] [--quick]

The right hand takes the inhaler lying on its side by the barrel from above, 33 mm along the barrel from the
mouthpiece's axis, its fingers closing across the barrel; lifts it 12 cm over the bench and turns it 30 degrees about
the vertical, so that the mouthpiece points at the arms. The left hand's wrist camera looks into the
mouthpiece from 5 cm in front of it and 3 cm over it: the coin in it. The left hand goes back to its ready pose; the
right hand turns the mouthpiece down 70 degrees about the barrel and waits, and shakes it up and down 15 mm until the
coin has fallen out (at most four times); turns it back level; the left wrist camera looks into the mouthpiece again:
empty. The left hand takes the cap from the bench (it lies open side down) from above by its narrow sides, lifts it
and turns it a quarter turn on the way so that its opening faces the mouth, lines it up with the mouthpiece 5 mm in
front of the mouth, and slides it on, 1 cm a second, until it is seated; lets go, backs off and goes to its ready
pose. The right hand brings the capped inhaler over the tray, turned a quarter turn from how it held it (the barrel along
the tray), lets it go 2 mm over the rim, so that it drops onto the felt, and goes to its ready pose. The run waits (the coin looked at, the mouthpiece held down, the empty mouthpiece looked at) so that
the six evenly spaced frames of its recording fall on what the spec names; the episode ends 2 s after the score
reaches 1.0.

What the script reads from the simulator, as a person sees it: where the inhaler, the cap and the tray lie when a
hand goes for them; how the cap sits in the hand and where the mouthpiece is, to line the two up; whether the coin
has left the mouthpiece; whether the cap is seated. Not the score.

``--check``: the run without videos (a try of the plan, not the recording). ``--quick``: the plan against the arms'
reach and joint limits, without the simulator (the poses as built; one shake, no corrections).
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

SIDES = ("left", "right")
GRASP_Z = 0.033  # m along the barrel from the mouthpiece's axis: where the right hand takes it (the dry checks)
GRASP_IN_I = np.column_stack([[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]])  # the right hand in the inhaler's frame I: closing across the barrel, pointing down onto it as it lies
OPEN_BARREL, SQUEEZE = 0.55, 0.15  # the right hand's opening over the barrel (52 mm for 30 mm) and closed on it (pressing)
HOLD_AT = (0.33, -0.06, 0.12)  # m: the right hand's grasp point while it holds the inhaler up (x, y, height over the bench): the left hand's wrist keeps 9 degrees or more from its stop as it puts the cap on
HOLD_ROT = np.column_stack([[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])  # the inhaler's frame I held up: the mouthpiece at the arms (-x), the opening's long axis up, the barrel along +y
TIP = math.radians(70.0)  # the mouthpiece turned down this far about the barrel
SHAKE, SHAKES = 0.015, 4  # m up and down; at most this many
LOOK = (0.05, 0.04, 0.03)  # m from the mouth's end: back (along the mouthpiece), to the side (+y), up: the left wrist camera's look (coin 42 px across)
CAP_GRASP = 0.005  # m from the cap's middle towards its end: the left hand's grasp point (its fingertips 2 mm over the bench)
OPEN_CAP, PINCH_CAP = 0.60, 0.20  # the left hand's opening over the cap (56 mm for 26 mm) and closed on it: 19 mm asked, about 18 N a finger (5000 N/m;
# at 0.12, about 36 N, a pad went into the cap's 1 mm wall as it was lifted, run w9i22)
CAP_CLOSING = 1.0  # the left hand's fingers close along the cap's short axis this way (+1 or -1: the hand turned half a turn)
PRE_CAP = 0.005  # m: the cap lined up with its rim this far in front of the mouth's end
BACK_OFF = 0.03  # m further back along the mouthpiece's line (and 1 cm more, 2 cm higher, where it is turned) that the cap comes from
LINED_UP, SQUARE, SETTLE = 0.00035, math.radians(1.0), 0.6  # m off the line and rad off its direction to start the slide; s a correction settles
CORRECT, LOOKS, WAIT = 2, 6, 0.2  # corrections at most, looks at most, s between looks after them, before each slide: both arms wander up to
# 1.5 mm in half a second while they hold still (runs w9i30, w9i31: the cap came to the mouth 1.0 to 1.3 mm off and jammed)
ATTEMPTS, STUCK, STUCK_S = 4, 0.0005, 0.4  # slides at most; a slide is stuck when the cap's end has come less than STUCK nearer the mouth in the last
# STUCK_S while the hand went on (the cap lags the hand by about 3 mm as it slides on: runs w9i24, w9i33)
SLOW, SLIDE, STEP_S, PAST = 0.005, 0.010, 0.1, 0.002  # m/s the cap slides until its rim is past the lead-in (8 mm), then on; s a step; m asked beyond the
# seat at most. At 2 cm/s the cap went 1.5 mm off the line in its first 5 mm and stuck on the mouth's end (run w9i32)
TRAY_DROP = 0.002  # m the barrel's underside over the tray's rim when the hand lets go: the fingertips stay over the walls (it drops 27 mm
# onto the felt)
TRAY_TURN = -0.5 * math.pi  # rad about the vertical from how it is held: the barrel along the tray's length, the mouthpiece across it, the
# only way the capped inhaler (80 x 64 mm) fits the 127 x 76 mm inside
UP = 0.08  # m over a place, before and after
LOOK_UNTIL, DOWN_UNTIL, EMPTY_UNTIL = 15.0, 28.0, 40.0  # s from the start: the run (about 65 s) is paced so that the six evenly spaced frames of
# the recording show what the spec names (the coin seen in the mouthpiece, the mouthpiece down, the mouthpiece seen empty, the cap going on)


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def frame(closing, approach) -> np.ndarray:
    """A hand's rotation from its closing axis and its approach axis (columns: closing, the pads' width, approach)."""
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def turn_between(r_from: np.ndarray, r_to: np.ndarray) -> float:
    return float(math.acos(max(-1.0, min(1.0, 0.5 * (np.trace(r_to @ r_from.T) - 1.0)))))


def pose(p, r) -> np.ndarray:
    """A rigid pose as a 4x4 matrix."""
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = np.asarray(r, dtype=float), np.asarray(p, dtype=float)
    return out


def inverse(t: np.ndarray) -> np.ndarray:
    return pose(-t[:3, :3].T @ t[:3, 3], t[:3, :3].T)


class Plan:
    """The run as steps, each a dict of moves for both hands (the same length for both), planned when it is asked
    for: a step that depends on what is seen is planned after the steps before it have been played."""

    def __init__(self, m, live: bool) -> None:
        import demo_kit as kit
        from yam_arena import ik, station
        from yam_arena.constants import READY

        self.m, self.mk, self.kit, self.Move, self.live = m, m.mk, kit, kit.Move, live
        self.ready = {side: ik.tcp_pose(np.array(READY, dtype=float), station.BASES[side]) for side in SIDES}
        self.shakes, self.corrections, self.slid, self.clock, self.jams = 0, [], 0.0, 0.0, 0
        self.held = None  # the inhaler's frame I as planned while the right hand holds it up: (position, rotation)
        self.cap_in_hand = None  # the cap's pose in the left hand's frame: (position, rotation)

    # --- what is seen ---
    def inhaler(self) -> tuple[np.ndarray, np.ndarray]:
        if self.live:
            return self.m.F.i_pose()
        return (self.held if self.held is not None else (np.array(self.m.INHALER_POS, dtype=float), np.array(self.m.INHALER_ROT, dtype=float)))

    def body(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        if self.live:
            p, r = self.mk.pose(name)
            return np.array(p, dtype=float), np.array(r, dtype=float)
        spec = self.mk.MECH.bodies[name]
        return np.array(spec.pos, dtype=float), spec.rotation()

    def coin_out(self) -> bool:
        return (not self.m.coin_in()) if self.live else True

    def seated(self) -> bool:
        return self.m.seated() if self.live else False

    def hand(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        """A hand's grasp point and turn as read (live)."""
        return np.array(self.mk.gripper(side)[0], dtype=float), np.array(self.mk.gripper_turn(side), dtype=float)

    # --- poses ---
    def right_on(self, i_pos, i_rot) -> tuple[np.ndarray, np.ndarray]:
        """The right hand's grasp point and turn that hold the inhaler with its frame I at (i_pos, i_rot)."""
        return np.asarray(i_pos) + np.asarray(i_rot) @ np.array([0.0, 0.0, GRASP_Z]), np.asarray(i_rot) @ GRASP_IN_I

    def hold_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """The inhaler's frame I while held up: the grasp point at HOLD_AT, the mouthpiece at the arms."""
        r = HOLD_ROT
        grasp = np.array([HOLD_AT[0], HOLD_AT[1], self.m.TOP + HOLD_AT[2]])
        return grasp - r @ np.array([0.0, 0.0, GRASP_Z]), r

    def look(self, i_pos, i_rot) -> tuple[np.ndarray, np.ndarray]:
        """The left hand's pose whose wrist camera looks into the mouth of the inhaler at (i_pos, i_rot)."""
        mouth = i_pos + i_rot @ np.array([self.m.P.MOUTH_X, 0.0, 0.0])
        out = i_rot[:, 0]  # out of the mouth: level when the inhaler is level
        side = np.cross([0.0, 0.0, 1.0], out)
        at = mouth + LOOK[0] * out - LOOK[1] * side + np.array([0.0, 0.0, LOOK[2]])
        a = (mouth - at) / np.linalg.norm(mouth - at)
        return at, frame(np.cross(a, [0.0, 0.0, 1.0]), a)

    def cap_target(self, gap: float, i_pos, i_rot) -> tuple[np.ndarray, np.ndarray]:
        """The left hand's pose that puts the cap (as it sits in the hand) on the mouthpiece's line, its end ``gap``
        from the mouth's end, its opening towards the mouth."""
        P = self.m.P
        cap_pos = i_pos + i_rot @ np.array([P.MOUTH_X + gap + P.CAP_END - 0.5 * P.CAP_DEPTH, 0.0, 0.0])
        cap_rot = i_rot @ np.diag([-1.0, 1.0, -1.0])
        p_in, r_in = self.cap_in_hand
        hand_rot = cap_rot @ r_in.T
        return cap_pos - hand_rot @ p_in, hand_rot

    def both(self, left: list, right: list, idle: tuple[str, str] = ("waits", "waits")) -> dict:
        """Moves for both hands, the shorter list padded with a wait (labelled ``idle``) so that both end together;
        the plan's clock moves on by the step's length."""
        Move = self.Move
        tl, tr = sum(m.seconds for m in left), sum(m.seconds for m in right)
        if tl < tr - 1e-9:
            left = [*left, Move(tr - tl, label=idle[0])]
        elif tr < tl - 1e-9:
            right = [*right, Move(tl - tr, label=idle[1])]
        self.clock += max(tl, tr)
        return {"left": left, "right": right}

    def until(self, when: float, before: float = 0.0, least: float = 0.2) -> float:
        """Seconds of a wait that ends at ``when`` on the plan's clock, after ``before`` seconds of this step."""
        return max(least, when - self.clock - before)

    # --- the run ---
    def steps(self):
        m, Move = self.m, self.Move
        P = m.P
        up = np.array([0.0, 0.0, UP])
        ready_l, ready_r = self.ready["left"], self.ready["right"]
        # the right hand takes the inhaler where it lies
        i_pos, i_rot = self.inhaler()
        grasp, hand = self.right_on(i_pos, i_rot)
        yield self.both([], [Move(2.0, pos=grasp + up, rot=hand, grip=OPEN_BARREL, label="over the barrel"), Move(1.5, pos=grasp, label="down"),
                             Move(0.6, grip=SQUEEZE, label="takes it"), Move(0.3, label="holds it")], ("ready", ""))  # fmt: skip
        # lifts it straight up, and turns it on the way to where it holds it, the mouthpiece to the arms
        self.held = self.hold_pose()
        hold_grasp, hold_hand = self.right_on(*self.held)
        yield self.both([], [Move(0.8, pos=grasp + np.array([0.0, 0.0, 0.06]), label="lifts it"), Move(1.8, pos=hold_grasp, rot=hold_hand, label="turns it")], ("ready", ""))
        # the left wrist camera looks into the mouthpiece: the coin in it
        at, turn = self.look(*self.inhaler())
        yield self.both([Move(2.5, pos=at, rot=turn, grip=0.3, label="looks in"), Move(self.until(LOOK_UNTIL, 2.5), label="sees the coin")], [], ("", "holds it up"))
        yield self.both([Move(1.8, pos=ready_l[0], rot=ready_l[1], label="back")], [], ("", "holds it up"))
        # the mouthpiece down, and shaken until the coin is out
        tipped = ry(-TIP) @ self.held[1]
        down_grasp, down_hand = self.right_on(self.held[0] + self.held[1] @ np.array([0.0, 0.0, GRASP_Z]) - tipped @ np.array([0.0, 0.0, GRASP_Z]), tipped)
        yield self.both([], [Move(1.8, pos=down_grasp, rot=down_hand, label="mouthpiece down"), Move(1.2, label="waits")], ("ready", ""))
        while not self.coin_out() or (not self.live and self.shakes == 0):
            if self.shakes >= SHAKES:
                raise RuntimeError(f"the coin is still in the mouthpiece after {SHAKES} shakes")
            self.shakes += 1
            yield self.both([], [Move(0.2, pos=down_grasp + np.array([0.0, 0.0, SHAKE]), label="shakes"), Move(0.25, pos=down_grasp - np.array([0.0, 0.0, SHAKE]), label="shakes"),
                                 Move(0.2, pos=down_grasp, label="shakes"), Move(0.6, label="waits")], ("ready", ""))  # fmt: skip
        yield self.both([], [Move(self.until(DOWN_UNTIL), label="holds it down")], ("ready", ""))
        yield self.both([], [Move(1.5, pos=hold_grasp, rot=hold_hand, label="level again"), Move(0.4, label="holds it up")], ("ready", ""))
        # the left wrist camera looks again: empty
        at, turn = self.look(*self.inhaler())
        yield self.both([Move(2.0, pos=at, rot=turn, grip=0.3, label="looks again"), Move(self.until(EMPTY_UNTIL, 2.0), label="sees it empty")], [], ("", "holds it up"))
        # the left hand takes the cap from the bench, from above, by its narrow sides
        c_pos, c_rot = self.body("cap")
        pick = c_pos + c_rot @ np.array([-CAP_GRASP, 0.0, 0.0])
        pick_hand = frame(CAP_CLOSING * c_rot[:, 2], [0.0, 0.0, -1.0])
        self.cap_in_hand = (pick_hand.T @ (c_pos - pick), pick_hand.T @ c_rot)
        yield self.both([Move(2.0, pos=pick + up, rot=pick_hand, grip=OPEN_CAP, label="over the cap"), Move(1.3, pos=pick, label="down"),
                         Move(0.6, grip=PINCH_CAP, label="takes the cap"), Move(0.3, label="holds it")], [], ("", "holds it up"))  # fmt: skip
        if self.live:  # how the cap sits in the hand, as seen
            h_pos, h_rot = self.hand("left")
            c_pos, c_rot = self.body("cap")
            self.cap_in_hand = (h_rot.T @ (c_pos - h_pos), h_rot.T @ c_rot)
        # lifts it, turns it on the way, and brings it in front of the mouth on the mouthpiece's line
        i_pos, i_rot = self.inhaler()
        pre_gap = P.CAP_DEPTH - P.CAP_END + PRE_CAP  # the cap's end this far from the mouth's end: its rim PRE_CAP in front of it
        target, target_rot = self.cap_target(pre_gap, i_pos, i_rot)
        along = -i_rot[:, 0]  # the way the cap goes on: into the mouth
        come = target - (BACK_OFF + 0.01) * along + np.array([0.0, 0.0, 0.02])
        yield self.both([Move(1.0, pos=pick + np.array([0.0, 0.0, 0.10]), label="lifts the cap"), Move(2.5, pos=come, rot=target_rot, label="turns it"),
                         Move(0.8, pos=target - BACK_OFF * along, label="in line"), Move(1.0, pos=target, label="in front of the mouth"), Move(0.4, label="holds it")], [],
                        ("", "holds it up"))  # fmt: skip
        # lines the cap up with the mouthpiece, as seen (the hand is asked where the cap would come right if the cap stays
        # off the wanted place as it is off now: the arm does not follow its targets exactly), and slides it on until
        # it is seated. Both arms wander up to 1.5 mm in half a second while they hold still: a slide whose cap stops
        # following the hand (its rim on the mouth's end) is backed off, looked at again and tried again
        aim, aim_rot = target, target_rot
        most = pre_gap + PAST
        for attempt in range(ATTEMPTS if self.live else 1):
            for look in range(LOOKS if self.live else 0):
                i_pos, i_rot = self.inhaler()
                c_pos, c_rot = self.body("cap")
                want = pose(i_pos + i_rot @ np.array([P.MOUTH_X + pre_gap + P.CAP_END - 0.5 * P.CAP_DEPTH, 0.0, 0.0]), i_rot @ np.diag([-1.0, 1.0, -1.0]))
                off = c_pos - want[:3, 3]
                off_line = float(np.linalg.norm(off - np.dot(off, along) * along))
                turned = turn_between(c_rot, want[:3, :3])
                self.corrections.append({"try": attempt + 1, "off line mm": round(1000 * off_line, 3), "along mm": round(1000 * float(np.dot(off, along)), 3),
                                         "turned deg": round(math.degrees(turned), 3)})  # fmt: skip
                if off_line <= LINED_UP and turned <= SQUARE:
                    break
                if look >= CORRECT:
                    yield self.both([Move(WAIT, label="waits for it in line")], [], ("", "holds it up"))
                    continue
                in_hand = pose(*self.cap_in_hand)
                asked = pose(aim, aim_rot) @ in_hand  # where the cap would be if the hand stood where it was asked
                new = asked @ inverse(pose(c_pos, c_rot)) @ want @ inverse(in_hand)
                aim, aim_rot = new[:3, 3], new[:3, :3]
                yield self.both([Move(0.5, pos=aim, rot=aim_rot, label="lines it up"), Move(SETTLE, label="lines it up")], [], ("", "holds it up"))
            self.slid, jammed, gaps = 0.0, False, []
            back = int(round(STUCK_S / STEP_S))
            while self.slid < most - 1e-9 and not self.seated():
                self.slid = min(most, self.slid + (SLOW if self.slid < PRE_CAP + P.LEAD_LEN else SLIDE) * STEP_S)
                yield self.both([Move(STEP_S, pos=aim + self.slid * along, label="slides the cap on")], [], ("", "holds it up"))
                if self.live:
                    gaps.append(self.m.gap())
                    if len(gaps) > back and gaps[-1 - back] - gaps[-1] < STUCK and gaps[-1] > P.LEAD_LEN:  # stuck short of the seat
                        jammed = True
                        break
            if not jammed:
                break
            self.jams += 1
            yield self.both([Move(0.5, pos=aim, label="backs off to try again"), Move(0.4, label="backs off to try again")], [], ("", "holds it up"))
        left_pos = aim + self.slid * along
        yield self.both([Move(0.3, label="holds it seated"), Move(0.6, grip=OPEN_CAP, label="lets go"), Move(0.8, pos=left_pos - 0.03 * along, label="backs off")], [],
                        ("", "holds it up"))  # fmt: skip
        # the left hand goes to its ready pose; the right hand lays the capped inhaler in the tray, lying as it is held
        # (from how it sits in the hand, as seen), and goes to its ready pose
        t_pos, t_rot = self.body("tray")
        rim = float(t_pos[2] + P.TRAY_H)
        r_lay = rz(TRAY_TURN) @ self.held[1]
        middle = np.array([0.017, 0.0, 0.025])  # the middle of the capped inhaler's outline in I (lying)
        lay_i = np.array([t_pos[0], t_pos[1], rim + P.BARREL_R + TRAY_DROP]) - np.array([*(r_lay @ middle)[:2], 0.0])
        lay, lay_hand = self.right_on(lay_i, r_lay)
        if self.live:
            h_pos, h_rot = self.hand("right")
            i_pos, i_rot = self.inhaler()
            in_hand = pose(h_rot.T @ (i_pos - h_pos), h_rot.T @ i_rot)  # the inhaler's frame in the hand
            hand_t = pose(lay_i, r_lay) @ inverse(in_hand)
            lay, lay_hand = hand_t[:3, 3], hand_t[:3, :3]
        yield self.both([Move(2.0, pos=ready_l[0], rot=ready_l[1], grip=1.0, label="ready")],
                        [Move(2.5, pos=lay + up, rot=lay_hand, label="over the tray"), Move(1.5, pos=lay, label="over its inside"), Move(0.3, label="holds it"),
                         Move(0.6, grip=OPEN_BARREL, label="lets go"), Move(0.6, pos=lay + np.array([0.0, 0.0, 0.05]), label="up"),
                         Move(1.2, pos=ready_r[0], rot=ready_r[1], grip=1.0, label="ready")], ("ready", ""))  # fmt: skip
        yield self.both([Move(3.0, label="ready")], [Move(3.0, label="ready")])

    def whole(self) -> dict:
        out: dict = {side: [] for side in SIDES}
        for step in self.steps():
            for side, moves in step.items():
                out[side].extend(moves)
        return out


def main() -> int:
    import demo_kit as kit

    quick, check = "--quick" in sys.argv, "--check" in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = Path(args[0])
    if quick:
        from yam_arena import station, task_file

        m = task_file.load(str(HERE / "task.py")).module
        plan = Plan(m, live=False)
        print(f"DRY full: {json.dumps(kit.dry_run(plan.whole(), station.BASES, limit_s=m.RUN_LIMIT_STEPS / 20.0), default=str)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    plan = Plan(m, live=True)
    script = plan.steps()
    state = {"over": False, "error": None, "since": {}, "saved": set()}

    def snap(name: str, camera: str, picture) -> None:
        """A camera's picture as rendered and at the size a policy gets it."""
        import cv2

        frame = np.asarray(picture)
        cv2.imwrite(str(out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(out / f"{name}_policy.png"), cv2.cvtColor(kit.policy_picture(frame), cv2.COLOR_RGB2BGR))
        state["saved"].add(name)

    def director(t: float, arms: dict, pictures) -> None:
        if pictures is None or mk.MECH.environment is None or not mk.MECH.poses:
            return
        for side in SIDES:  # pictures for the README: what the left wrist camera sees in the mouthpiece; the cap seated; laid in the tray
            label = arms[side].target(t)[3]
            since = state["since"].setdefault((side, label), t)
            for want, after, name, camera in (("sees the coin", 2.0, "look_coin_left_wrist", "left_cam"), ("sees it empty", 2.0, "look_empty_left_wrist", "left_cam"),
                                              ("holds it seated", 0.25, "seated_left_wrist", "left_cam"), ("lets go", 0.5, "laid_top", "top_cam")):  # fmt: skip
                if label == want and t - since >= after and name not in state["saved"]:
                    snap(name, camera, pictures[camera])
        while not state["over"] and all(t >= arms[side].seconds - 1e-6 for side in SIDES):  # the next step is planned when the last is through
            try:
                step = next(script, None)
            except RuntimeError as error:
                state["error"] = repr(error)
                step = None
            if step is None:
                state["over"] = True
                break
            for side, moves in step.items():
                arms[side].moves.extend(moves)

    def extra() -> dict:
        p, r = m.F.i_pose()
        c = m.rel("cap")
        load = m.L.STATE["load_in_i"]
        return {"coins in": m.coins_in(), "seated": m.seated(), "left opening": round(mk.gripper("left")[1], 3), "coin held": m.L.lodged(),
                "load angle (deg)": None if load is None or not m.L.lodged() else round(float(np.degrees(np.arctan2(np.hypot(load[0], load[2]), -load[1]))), 2), "gap (mm)": round(1000 * m.gap(), 2), "cap off line (mm)": round(1000 * float(np.hypot(c[0][1], c[0][2])), 3),
                "coin in I (mm)": (1000 * m.rel("coin")[0]).round(1).tolist(), "inhaler (mm)": (1000 * p).round(1).tolist(), "mouthpiece points": r[:, 0].round(3).tolist(),
                "in the tray": m.in_tray(), "at rest": m.at_rest(), "let go": m.let_go(), "shakes": plan.shakes, "slid (mm)": round(1000 * plan.slid, 1)}  # fmt: skip

    result = kit.run(embodiment, {side: [] for side in SIDES}, out, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=1.0,
                     sheet_frames=12, record=not check)  # fmt: skip
    rows = result["rows"]
    scores = [row["info"].get("score", 0.0) for row in rows]
    firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 1.0)}
    seen = {"first coin out (s)": next((row["t"] for row in rows if not row["extra"]["coins in"]), None),
            "first seated (s)": next((row["t"] for row in rows if row["extra"]["seated"]), None)}  # fmt: skip
    held = [(row["t"], row["right"], row["extra"]["load angle (deg)"]) for row in rows if row["extra"]["load angle (deg)"] is not None]
    seen["coin let go"] = m.L.STATE["released"]
    seen["while the coin was held: most load angle (deg), most jolt (g)"] = [round(float(np.degrees(m.L.STATE["most_angle"])), 2), round(m.L.STATE["most_jolt"], 3)]
    seen["most load angle by the right hand's step while the coin was held (deg)"] = {label: max(a for _, lab, a in held if lab == label) for label in dict.fromkeys(lab for _, lab, _ in held)}
    readings = {"score": scores[-1], "first reached": firsts, **seen, "seconds": rows[-1]["t"], "steps": len(rows), "ended": result.get("termination_reason"),
                "shakes": plan.shakes, "slides jammed and tried again": plan.jams, "line-up readings": plan.corrections, "slid (mm)": round(1000 * plan.slid, 1), "plan error": state["error"],
                "end": rows[-1]["extra"], "report": result["report"], "worst IK": result["worst_ik"]}  # fmt: skip
    (out / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
    print(f"DEMO: score {scores[-1]:.2f} ended {result.get('termination_reason')} after {result['seconds']:.1f} s ({result['steps']} steps of {m.RUN_LIMIT_STEPS})", flush=True)
    print(f"READINGS {json.dumps(readings, default=str)}", flush=True)
    return 0 if scores[-1] == 1.0 else 1


if __name__ == "__main__":
    sys.exit(main())
