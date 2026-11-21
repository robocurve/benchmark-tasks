"""Full-score demonstration: the left stick plays the hi-hat, the right stick the snare and, in bar 4, the tom, at
75 bpm; then both sticks are held still, clear of the kit.

Each arm plays with its gripper pointing sideways toward the robot's centre line, the finger pads on the stick's
upper and lower sides (``poses``). A stroke moves joint 6 alone, the wrist roll, whose axis passes through the grip
point: the stick waits with its tip lifted, and one control step before its count joint 6 is commanded past the
angle at which the tip meets the pad (a throw: the YAM's wrist is overdamped, so the tip meets the pad at about
lever x overshoot / 0.15 s); at the first control step after the stick touches a pad, joint 6 is commanded back up,
so the hand does not press on through the pad. The overshoot sets the loudness (hi-hat numbers loud, ands soft); the
ratio of lift to overshoot sets when within the control step the tip lands (4 to 12 ms before the count's step).
The right arm glides between the snare's and the tom's poses in joint space.

The demonstration reads simulator state, as a scripted demonstration may: the trigger's record of when each stick
last touched a pad. It moves nothing itself.

``RDB_DRUM_DEMO`` (or ``variant``) selects a control (README, "Controls"): flat, tempo82, no_fill, flam_bar3,
one_arm, air, press, rest, flail, no_stop.
"""

import math
import os
import random

from inspect_robots import Action, ActionChunk
import numpy as np

from . import fixed_scene as F
from . import layout as L
from .poses import contact_poses

DT = 1. / F.CONTROL_HZ
LEAD = 24                                   # control steps before the first count (1.2 s)
# Joint 6 past contact (rad), per arm and dynamic: the tip meets the pad at about 1.8 (left) and 1.65 (right) m/s per
# rad of overshoot (prototype runs: the wrist behaves as a first-order lag of 0.14 and 0.155 s).
OVER = {"left": {"loud": .90, "soft": .45, "flat": .66}, "right": {"loud": .97, "medium": .80, "soft": .49}}
# Joint 6 above contact (rad) where each stroke waits. With the runner's delta limit (no joint target more than 1 rad
# from the last), a loud stroke is thrown in two control steps: PREP (rad above contact) one step before the throw,
# then the full overshoot. Both set so that the tip lands about 40 ms into the throw's step, 10 ms before the count.
LIFT = {"left": {"loud": .387, "soft": .225, "flat": .297}, "right": {"loud": .398, "medium": .262, "soft": .147}}
PREP = {"left": {"loud": .08}, "right": {"loud": .01, "medium": .18}}
MAX_DELTA = 1.0
CLEAR_LIFT = .100                           # after the last pair, both tips lifted this far (along the pad's normal)
GLIDE_END = 4                               # a glide between pads ends this many steps before the next count
PRESS_SPEED = .20                           # control "press": the tip brought down at 0.2 m/s ...
PRESS_HOLD = 6                              # ... and held on the pad 0.3 s
REST_STEPS = 12                             # control "rest": the stick left on the snare 0.6 s after each snare note
AIR_GAP = .010                              # control "air": each stroke stops 10 mm above the pad
VARIANTS = ("full", "flat", "tempo82", "no_fill", "flam_bar3", "one_arm", "air", "press", "rest", "flail", "no_stop",
            "gripper")
GRIPPER_TOUCH = (.362, -.140, .095)         # control "gripper": the snare's rim nearest the robot (tabletop frame)


def schedule(variant):
    """{arm: [(control step of the count, pad, dynamic)]} for a variant."""
    bpm = 82. if variant == "tempo82" else L.BPM
    notes = list(L.NOTES)
    if variant == "no_fill":
        notes = [n for n in notes if n[0] < 15.]
    if variant == "no_stop":                       # one more bar after the closing pair
        for k in range(1, 8):
            notes.append((16. + k / 2, "hihat", "loud" if k % 2 == 0 else "soft"))
        notes += [(17., "snare", "loud"), (19., "snare", "loud"), (20., "hihat", "loud")]
    out = {"left": [], "right": []}
    for b, pad, dyn in sorted(notes):
        if variant == "flat" and pad == "hihat":
            dyn = "flat"
        k = LEAD + int(round(b * 60. / bpm * F.CONTROL_HZ))
        if variant == "flam_bar3" and pad == "snare" and b in (9., 11.):
            k += 1                                  # thrown a step late: lands about 45 ms after the hi-hat
        arm = F.PLAYS[pad]
        if variant == "one_arm" and arm == "right":
            continue
        out[arm].append((k, pad, dyn))
    if variant == "flail":                          # about 6 strokes a second for 13 s, at random
        rng = random.Random(0)
        out = {"left": [], "right": []}
        for arm, pads in (("left", ("hihat",)), ("right", ("snare", "snare", "tom"))):
            k = LEAD
            while k < LEAD + 260:
                k += rng.choice((3, 4, 5, 6))
                out[arm].append((k, rng.choice(pads), rng.choice(("loud", "soft"))))
    return out


class DrumReference:
    def __init__(self, variant=None):
        self.variant = variant or os.environ.get("RDB_DRUM_DEMO", "full")
        if self.variant not in VARIANTS:
            raise ValueError(f"unknown variant {self.variant!r}; one of {VARIANTS}")
        poses = contact_poses()
        self.stroke = {pad: poses[(F.PLAYS[pad], pad)][0] for pad in F.PADS}
        self.plan = schedule(self.variant)
        self.joints = None
        self.state = None
        self.log = {"events": []}
        self.phases = []
        self.done = False

    def observe(self, joints, state):
        self.joints = np.asarray(joints, dtype=float)
        self.state = state

    def touched_since(self, arm, k):
        """Whether ``arm``'s stick has touched a pad since physics step ``k`` (the trigger's record)."""
        if self.state is None:
            return False
        last = self.state["last_touch"].get(F.STICKS[arm])
        return last is not None and last > k

    def physics_step(self):
        return self.state["k"] if self.state is not None else 0

    def lift_for(self, arm, dyn):
        return LIFT[arm][dyn]

    def arm_steps(self, arm):
        """Yields joint targets (6) for ``arm``, one per control step, forever."""
        if self.variant == "press":
            yield from self._press_steps(arm)
            return
        plan = self.plan[arm]
        pad0 = plan[0][1] if plan else ("hihat" if arm == "left" else "snare")
        cur = self.stroke[pad0]
        q_hold = cur.q_at(cur.phi_for(F.START_TIP_LIFT))
        step = 0
        for idx, (k_note, pad, dyn) in enumerate(plan):
            st = self.stroke[pad]
            lift, over = self.lift_for(arm, dyn), OVER[arm][dyn]
            k_throw = k_note - 1
            if st is not cur:                       # glide to the other pad's pose
                q_from = q_hold
                q_to = st.q_at(lift)
                s0, s1 = step, max(step + 1, k_throw - GLIDE_END)
                while step < s1:
                    u = (step - s0 + 1) / (s1 - s0)
                    u = u * u * (3 - 2 * u)
                    step += 1
                    yield q_from + u * (q_to - q_from)
                cur = st
            prep = PREP[arm].get(dyn) if self.variant not in ("air",) else None
            while step < k_throw - (1 if prep is not None else 0):     # wait at the lift
                step += 1
                yield st.q_at(lift)
            if prep is not None:                    # the first half of a loud throw (the delta limit)
                step += 1
                yield st.q_at(prep)
            if self.variant == "air":
                target = st.q_at(st.phi_for(AIR_GAP))
                for _ in range(2):
                    step += 1
                    yield target
                q_hold = st.q_at(lift)
                continue
            k0 = self.physics_step()
            step += 1
            self.log["events"].append((step, arm, pad, dyn, "throw"))
            yield st.q_at(-over)                     # the throw
            for _ in range(2):                       # lift at the first step after contact (at most two steps late)
                if self.touched_since(arm, k0):
                    break
                step += 1
                yield st.q_at(-over)
            nxt = plan[idx + 1] if idx + 1 < len(plan) else None
            if self.variant == "rest" and pad == "snare":
                for _ in range(REST_STEPS):
                    step += 1
                    yield st.q_at(-.06)              # left on the head
            if nxt is not None and self.stroke[nxt[1]] is st:
                q_hold = st.q_at(self.lift_for(arm, nxt[2]))
            else:
                q_hold = st.q_at(lift)
            step += 1
            yield q_hold
        # the end: lift clear and hold still
        q_end = cur.q_at(cur.phi_for(CLEAR_LIFT))
        if self.variant == "gripper" and arm == "right":
            yield from self._gripper_touch(q_hold, q_end)
        self.done_steps = step
        while True:
            yield q_end

    def _gripper_touch(self, q_from, q_end):
        """Control "gripper": after the closing pair the right hand, its stick turned up out of the way, comes down
        onto the snare's near rim with its fingers, presses, and goes back up."""
        import math as m

        from . import poses

        st = self.stroke["snare"]
        s = poses.stick_dir(st.heading, -m.radians(55.))
        R = poses.grip_rotation(s, poses.GAMMA["right"])
        x, y, z = GRIPPER_TOUCH
        targets = []
        for dz in (.07, .02):                       # the fingers reach about 11 mm into the rim; the stick stays clear
            q, err, _, _ = poses.solve(np.array((x, y, z + dz)), R, st.base, [st.q_c] + poses.SEEDS)
            targets.append(q)
        high, low = targets
        for path, n in (((q_from, high), 6), ((high, low), 6)):      # down within 0.7 s of the last pair, before
            # the end can have been clean for 1 s (the trial would end there at 1.0)
            a, b = path
            for k in range(1, n + 1):
                u = k / n
                yield a + (u * u * (3 - 2 * u)) * (b - a)
        for _ in range(6):
            yield low
        for k in range(1, 11):
            u = k / 10
            yield low + (u * u * (3 - 2 * u)) * (q_end - low)

    def _press_steps(self, arm):
        """Control "press": at each count the tip is brought down onto the right pad at about 0.2 m/s, held there
        0.3 s, and raised; joint 6's command never moves down faster than that."""
        plan = self.plan[arm]
        if not plan:
            st = self.stroke["hihat" if arm == "left" else "snare"]
            while True:
                yield st.q_at(st.phi_for(CLEAR_LIFT))
        up = .12                                     # about 30 mm over the pad between presses
        step, phi, cur = 0, None, None
        for idx, (k_note, pad, _) in enumerate(plan):
            st = self.stroke[pad]
            if st is not cur:
                if cur is not None:                  # move over to the other pad, lifted
                    q_from, q_to = cur.q_at(up), st.q_at(up)
                    s0, s1 = step, max(step + 1, k_note - 8)
                    while step < s1:
                        u = (step - s0 + 1) / (s1 - s0)
                        step += 1
                        yield q_from + (u * u * (3 - 2 * u)) * (q_to - q_from)
                cur, phi = st, up if phi is None else up
            rate = PRESS_SPEED / st.lever * DT
            while step < k_note + PRESS_HOLD:
                step += 1
                target = -.02 if step >= k_note - int(math.ceil((up + .02) / rate)) else up
                phi = max(phi - rate, target) if target < phi else min(phi + 3 * rate, target)
                yield st.q_at(phi)
            nxt = plan[idx + 1][0] if idx + 1 < len(plan) else None
            while nxt is not None and step < nxt - int(math.ceil((up + .02) / rate)) - 1 and phi < up:
                step += 1
                phi = min(phi + 3 * rate, up)
                yield st.q_at(phi)
        st = cur
        while True:
            yield st.q_at(st.phi_for(CLEAR_LIFT))

    def sequence(self):
        left, right = self.arm_steps("left"), self.arm_steps("right")
        last = max((p[-1][0] for p in self.plan.values() if p), default=LEAD)
        tick = 0
        while True:
            tick += 1
            q_l, q_r = next(left), next(right)
            if tick > last + 4:
                self.done = True
            data = np.concatenate([q_l, [0.], q_r, [0.]])
            yield ActionChunk(actions=[Action(data=data, meta={"phase": "play" if not self.done else "hold"})],
                              control_hz=F.CONTROL_HZ)
