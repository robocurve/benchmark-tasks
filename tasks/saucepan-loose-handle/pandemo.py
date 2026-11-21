"""The scripted demo for saucepan_loose_handle.py: stage-by-stage planning and a sag correction for the soft arms.

Whenever the ``yam_scripted`` policy's plan runs out, it asks the task file for the next stage, planned from the pans'
poses and, after a rock test, from how far the rocked handle rose against its pan, both read from the simulator. It moves
the pans only through the grippers, by contact. The simulated arms are soft and sag under a carried pan, so the policy
adds to each joint target a bounded running sum of the joint's error, except while a hand squeezes, rocks, presses or
lets go (so no push is made stronger than planned).

DEMO_VARIANT picks the plan: "" (full: rock every handle, sound pans to the burners with handles turned in, the loose one
to the tray), "untested" (the first two pans to the burners, no rocking), "fixedguess" (no rocking; the right-hand pan
to the tray, the other two to the burners), "handlesout" (full, but the handles are left pointing at the robot),
"nothing" (the hands stay at the ready pose), "probe" (rock all three handles, move nothing).
"""

from __future__ import annotations

import math
import os

import numpy as np

import layout as L

VARIANT = os.environ.get("DEMO_VARIANT", "")
_PLAN_LOG = open(os.environ["PAN_DEBUG"] + ".plan", "w") if os.environ.get("PAN_DEBUG") else None
SAG_GAIN = float(os.environ.get("PAN_SAG_GAIN", "0.2"))
SAG_MAX = float(os.environ.get("PAN_SAG_MAX", "0.25"))  # rad per joint
FREEZE = ("squeeze", "rock", "press", "let go")  # no correction while the hand pushes or pulls on a part
UNWIND = ("back off", "ready")

# finger joint targets (m from closed; each pad moves this far from the middle)
RIM_OPEN, RIM_SQUEEZE = -0.013, 0.0  # the 5 mm rolled rim: 2.5 mm past contact on each side (the fingers' limit)
GRIP_OPEN, GRIP_SQUEEZE = -0.021, -0.010  # the 26 mm phenolic grip: 3 mm past contact on each side
LEAN_DEG = 30.0  # carries: the hand leans forward 30 degrees, along the wall it pinches
LIFT = 0.11  # carries: the pan's base goes 11 cm up (the burner plates are 7.6 cm high)
ROCK_UP = 0.016  # commanded lift of the handle grip; a loose grip rises 6 mm at that point before it binds
GRIP_D = 0.07  # the hands take the grip 70 mm out from the screw
TURN_TO_DEG = 80.0  # the handle is swung from pointing at the robot to 80 degrees outward
_C8, _S8 = math.cos(math.radians(L.GRIP_PITCH_DEG)), math.sin(math.radians(L.GRIP_PITCH_DEG))
ARMS = ("left", "right")


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * float(np.dot(close, z))
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _lean(direction, deg: float) -> np.ndarray:
    t = math.radians(deg)
    d = np.asarray(direction, float)
    d = d / max(1e-9, np.linalg.norm(d[:2]))
    return np.array([d[0] * math.sin(t), d[1] * math.sin(t), -math.cos(t)])


def _ready(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def _pick_flip(arm: str, pos, rot, last=None) -> np.ndarray:
    """Of the two grips that differ by a half turn about the approach (a parallel gripper grasps the same either way),
    the one the arm reaches with the smaller IK error from its ready pose."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    if last is None:
        last = _ready(arm)[1]
    cands = []
    for f in (1.0, -1.0):
        r = np.asarray(rot, float) @ np.diag([f, f, 1.0])
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(r), np.array(READY, float),
                             base_pos=station.BASES[arm], iters=150)
        cands.append((pe * 1000 + math.degrees(re), _rot_angle(r, last), r))
    ok = [c for c in cands if c[0] < 3.0]
    return min(ok, key=lambda c: c[1])[2] if ok else min(cands, key=lambda c: c[0])[2]


def _rot_angle(a, b) -> float:
    c = np.clip((np.trace(np.asarray(a).T @ np.asarray(b)) - 1.0) / 2.0, -1.0, 1.0)
    return math.acos(c)


def _wp(sec, pos, finger, label, rot):
    from yam_arena.demo import Waypoint

    return Waypoint(float(sec), tuple(np.asarray(pos, float).tolist()), float(finger), label, np.asarray(rot, float))


def ready(arm: str, sec: float, label: str = "back to the ready pose", finger: float = -0.02):
    p, r = _ready(arm)
    return _wp(sec, p, finger, label, r)


def rest(arm: str, sec: float):
    p, r = _ready(arm)
    s = 1.0 if arm == "left" else -1.0
    return _wp(sec, p + np.array([-0.10, 0.05 * s, 0.0]), -0.02, "back to the ready pose", r)


def demo():
    """Both hands wait at the ready pose; the plan is made stage by stage from the first control step."""
    from bodykit import allow_matrix_grips

    allow_matrix_grips()
    return {arm: [ready(arm, 0.5, "look")] for arm in ARMS}


def _dur(ws) -> float:
    return sum(w.duration for w in ws)


class Planner:
    """The demo's stages. next_stage(st) returns the next moves for both hands, or None when the demo is done."""

    def __init__(self, task):
        self.t = task
        self.queue: list = []
        self.started = False
        self.burners_used = 0
        self.slot_pan: dict[int, str] = {}
        self.rise: dict[int, float] = {}  # slot -> how far its handle rose against its pan when rocked (degrees)
        self.last: dict = {}  # each hand's last planned orientation

    # -- poses -------------------------------------------------------------------------------------------------------
    def _pan_at(self, st, slot: int) -> str:
        y = L.SLOTS_Y[slot]
        return min(self.t.PAN_NAMES, key=lambda n: math.hypot(st[n].pos[0] - L.PAN_X, st[n].pos[1] - y))

    @staticmethod
    def _rim_point(pan_pos, side: float) -> np.ndarray:
        return np.array([pan_pos[0], pan_pos[1] + side * (L.PAN_R - L.WALL_T / 2),
                         pan_pos[2] + L.PAN_H - self_bead_half()])

    def _handle_point(self, st, pan: str) -> tuple[np.ndarray, np.ndarray]:
        """The grip point 70 mm out from the screw, and the handle's horizontal direction."""
        R = st[pan].rotation()
        local = np.array([L.HINGE_X - GRIP_D * _C8, 0.0, L.HINGE_Z + GRIP_D * _S8])
        d = R @ np.array([-1.0, 0.0, 0.0])
        d[2] = 0.0
        return st[pan].pos + R @ local, d / np.linalg.norm(d)

    # -- moves -------------------------------------------------------------------------------------------------------
    def rock_moves(self, st, pan: str, rocker: str):
        holder = "right" if rocker == "left" else "left"
        hp, d = self._handle_point(st, pan)
        rr = _pick_flip(rocker, hp, _frame((0, 0, -1), (-d[1], d[0], 0.0)), self.last.get(rocker))
        up = hp + np.array([0.0, 0.0, ROCK_UP])
        # both hands start each test from the ready pose: going straight from one test to the next, their paths cross
        r_moves = [
            ready(rocker, 1.4, "ready pose between tests"),
            _wp(1.6, hp + [0, 0, 0.06], GRIP_OPEN, "over the handle", rr),
            _wp(0.8, hp, GRIP_OPEN, "fingers either side of the handle", rr),
            _wp(0.4, hp, GRIP_SQUEEZE, "squeeze the handle", rr),
            _wp(0.6, up, GRIP_SQUEEZE, "rock the handle up", rr),
            _wp(0.3, up, GRIP_SQUEEZE, "rock the handle up", rr),
            _wp(0.5, hp, GRIP_SQUEEZE, "rock the handle down", rr),
            _wp(0.6, up, GRIP_SQUEEZE, "rock the handle up", rr),
            _wp(0.3, up, GRIP_SQUEEZE, "rock the handle up", rr),
            _wp(0.5, hp, GRIP_SQUEEZE, "rock the handle down", rr),
            _wp(0.3, hp, GRIP_OPEN, "let go of the handle", rr),
            _wp(0.6, hp + [0, 0, 0.06], GRIP_OPEN, "back off up", rr),
        ]
        s = 1.0 if holder == "left" else -1.0
        g = self._rim_point(st[pan].pos, s)
        rh = _pick_flip(holder, g, _frame((0, 0, -1), (0, 1, 0)), self.last.get(holder))
        hold_t = _dur(r_moves[4:10])
        h_moves = [
            ready(holder, 1.4, "ready pose between tests"),
            _wp(1.6, g + [0, 0, 0.06], RIM_OPEN, "over the rim", rh),
            _wp(0.8, g, RIM_OPEN, "fingers either side of the rim", rh),
            _wp(0.4, g, RIM_SQUEEZE, "squeeze the rim", rh),
            _wp(hold_t, g, RIM_SQUEEZE, "squeeze the rim (hold the pan)", rh),
            _wp(0.3, g, RIM_OPEN, "let go of the rim", rh),
            _wp(0.6, g + [0, 0, 0.06], RIM_OPEN, "back off up", rh),
        ]
        return {rocker: r_moves, holder: h_moves}

    def carry_moves(self, st, pan: str, arm: str, side: float, dest_xy, dest_z: float):
        a = _lean((1.0, 0.0), LEAN_DEG)
        g = self._rim_point(st[pan].pos, side)
        dz = g[2] - st[pan].pos[2]
        dg = np.array([dest_xy[0], dest_xy[1] + side * (L.PAN_R - L.WALL_T / 2), dest_z + dz])
        r = _pick_flip(arm, g, _frame(a, (0, 1, 0)), self.last.get(arm))
        lift = np.array([0.0, 0.0, LIFT])
        return [
            _wp(1.8, g - a * 0.06, RIM_OPEN, "over the rim", r),
            _wp(0.8, g, RIM_OPEN, "fingers either side of the rim", r),
            _wp(0.5, g, RIM_SQUEEZE, "squeeze the rim", r),
            _wp(1.2, g + lift, RIM_SQUEEZE, "lift the pan", r),
            _wp(2.0, dg + lift, RIM_SQUEEZE, "carry the pan", r),
            _wp(1.2, dg + [0, 0, 0.006], RIM_SQUEEZE, "lower the pan", r),
            _wp(0.5, dg - [0, 0, 0.004], RIM_SQUEEZE, "press the pan down", r),
            _wp(0.4, dg - [0, 0, 0.004], RIM_OPEN, "let go of the rim", r),
            _wp(0.7, dg - a * 0.07 + [0, 0, 0.03], RIM_OPEN, "back off up", r),
        ]

    def turn_moves(self, st, pan: str, arm: str, outward: float):
        """Take the handle and swing it, and the pan with it, from pointing at the robot to TURN_TO_DEG outward."""
        hp, d0 = self._handle_point(st, pan)
        centre = st[pan].pos
        rg = math.hypot(hp[0] - centre[0], hp[1] - centre[1])
        phi0 = math.atan2(d0[1], d0[0])
        target = math.atan2(outward * math.sin(math.radians(TURN_TO_DEG)), -math.cos(math.radians(TURN_TO_DEG)))
        dphi = (target - phi0 + math.pi) % (2 * math.pi) - math.pi

        def pose(phi):
            d = np.array([math.cos(phi), math.sin(phi), 0.0])
            p = np.array([centre[0], centre[1], hp[2]]) + d * rg
            a = _lean(-d, LEAN_DEG)
            return p, a, _frame(a, (-d[1], d[0], 0.0))

        p0, a0, r0 = pose(phi0)
        flip = _pick_flip(arm, p0, r0, self.last.get(arm))
        f = np.diag([1.0, 1.0, 1.0]) if np.allclose(flip, r0) else np.diag([-1.0, -1.0, 1.0])
        moves = [
            _wp(1.6, p0 - a0 * 0.06, GRIP_OPEN, "over the handle", r0 @ f),
            _wp(0.8, p0, GRIP_OPEN, "fingers either side of the handle", r0 @ f),
            _wp(0.4, p0, GRIP_SQUEEZE, "squeeze the handle", r0 @ f),
        ]
        n = 4
        for k in range(1, n + 1):
            p, a, r = pose(phi0 + dphi * k / n)
            moves.append(_wp(0.8, p, GRIP_SQUEEZE, "turn the handle in", r @ f))
        moves += [
            _wp(0.3, p, GRIP_OPEN, "let go of the handle", r @ f),
            _wp(0.7, p - a * 0.06, GRIP_OPEN, "back off up", r @ f),
        ]
        return moves

    # -- stages ------------------------------------------------------------------------------------------------------
    def build(self, st) -> None:
        self.slot_pan = {i: self._pan_at(st, i) for i in range(3)}
        if VARIANT == "nothing":
            self.queue = [("wait",)]
            return
        if VARIANT == "untested":
            self.queue = [("burner", 0), ("burner", 1)]
        elif VARIANT == "fixedguess":
            self.queue = [("burner", 0), ("burner", 1), ("tray", 2)]
        elif VARIANT == "probe":
            self.queue = [("test", i) for i in range(3)]
        else:  # rock every handle first, before any pan goes on the hot plate; then place them
            for i in range(3):
                self.queue += [("test", i), ("note", i)]
            self.queue.append(("place",))
        self.queue.append(("rest",))

    def _pad(self, plan: dict) -> dict:
        t = max(_dur(v) for v in plan.values())
        for arm in ARMS:
            if arm not in plan:
                plan[arm] = [ready(arm, min(2.0, t)), ready(arm, max(0.1, t - 2.0), "wait")]
        return plan

    def next_stage(self, st):
        plan = self._next(st)
        for arm, ws in (plan or {}).items():
            if ws:
                self.last[arm] = np.asarray(ws[-1].yaw, float)
        return plan

    def _next(self, st):
        if not self.started:
            self.build(st)
            self.started = True
        while self.queue:
            item = self.queue.pop(0)
            kind = item[0]
            if kind == "wait":
                return {arm: [ready(arm, 5.0, "wait")] for arm in ARMS}
            if kind == "rest":
                return {arm: [rest(arm, 2.0), rest(arm, 1.0)] for arm in ARMS}
            if kind == "place":
                loose = [i for i in range(3) if self.rise.get(i, 0.0) >= L.LOOSE_DEG]
                tray = loose[0] if len(loose) == 1 else None
                print(f"pandemo: loose handle found in slot {tray + 1 if tray is not None else 'none'}", flush=True)
                self.queue[0:0] = ([("tray", tray)] if tray is not None else []) + [
                    ("burner", i) for i in range(3) if i != tray][:2]
                continue
            pan = self.slot_pan[item[1]]
            if kind == "note":
                rise = float(self.t._EP["angle_max"]) if pan == "pan_loose" else 0.0  # a sound grip cannot turn
                self.rise[item[1]] = rise
                print(f"pandemo: slot {item[1] + 1} handle rose {rise:.2f} deg when rocked", flush=True)
                continue
            if kind == "test":
                rocker = "right" if item[1] == 2 else "left"
                self.t._EP["angle_max"] = 0.0  # the rise of the rocked handle against its pan, from now on
                print(f"pandemo: rock the handle of the pan in slot {item[1] + 1}", flush=True)
                return self.rock_moves(st, pan, rocker)
            if kind == "tray":
                return self._pad({"right": self.carry_moves(st, pan, "right", -1.0, L.TRAY_C, L.TOP + L.TRAY_T)})
            if kind == "burner":
                b = self.burners_used
                self.burners_used += 1
                arm, side = ("left", 1.0) if b == 0 else ("right", -1.0)
                bx, by, _ = L.BURNERS[b]
                if VARIANT != "handlesout":
                    self.queue.insert(0, ("turn", item[1], arm, side))
                return self._pad({arm: self.carry_moves(st, pan, arm, side, (bx, by), L.TOP + L.BURNER_TOP)})
            if kind == "turn":
                return self._pad({item[2]: self.turn_moves(st, pan, item[2], item[3])})
        return None


def self_bead_half() -> float:
    return 0.004  # the hands pinch the middle of the 8 mm rolled rim


def install() -> None:
    """Hook yam_scripted (if it is importable): plan stage by stage, and correct each arm's sag."""
    try:
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import STATE_KEY
        from yam_arena import scripted
        from yam_arena.demo import Waypoint
    except Exception:  # the task file also loads where yam_scripted is not installed (the checker)
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_pandemo", False):
        return
    plain_reset, plain_act = cls.reset, cls.act

    def mine(self) -> bool:
        return self._task is not None and hasattr(self._task.module, "pandemo_task")

    def reset(self, scene):
        plain_reset(self, scene)
        self._pd_done = not mine(self)
        self._pd_off = np.zeros(14)

    def extend(self, plan: dict, t: float) -> None:
        start = max(t + self._dt, max(p.duration for p in self._plans.values()))
        for arm, have in self._plans.items():
            gap = start - have.duration
            last = have.waypoints[-1]
            if gap > 1e-6:
                have.waypoints.append(Waypoint(gap, last.pos, last.finger, last.label, last.yaw))
            have.waypoints.extend(plan.get(arm, []))
        self._end = max(p.duration for p in self._plans.values())

    def act(self, observation):
        if not mine(self):
            return plain_act(self, observation)
        m = self._task.module
        t = float(observation.state_time)
        if not self._pd_done and t + self._dt >= self._end - 0.05:
            st = m._EP.get("st")
            if st is None:
                raise RuntimeError("pandemo: no poses recorded")
            plan = m.next_stage(st)
            if plan is None:
                self._pd_done = True
            else:
                extend(self, plan, t)
        labels = {arm: p.target(t + self._dt)[3] for arm, p in self._plans.items()}
        chunk = plain_act(self, observation)
        planned = np.asarray(chunk.actions[0].data, float).copy()
        measured = np.asarray(observation.state[STATE_KEY], float)
        fields = [f"{int(round(t * 20)):4d}"]
        for arm, idx in (("left", np.r_[0:6]), ("right", np.r_[7:13])):
            lab = labels.get(arm, "")
            if any(lab.startswith(u) for u in UNWIND):
                self._pd_off[idx] *= 0.85
            elif not any(lab.startswith(fz) for fz in FREEZE):
                self._pd_off[idx] = np.clip(self._pd_off[idx] + SAG_GAIN * (planned[idx] - measured[idx]),
                                            -SAG_MAX, SAG_MAX)
            if _PLAN_LOG is not None:
                err = np.degrees(np.abs(planned[idx] - measured[idx])).max()
                fields.append(f"{arm}: err={err:.1f}deg off={np.linalg.norm(self._pd_off[idx]):.3f} "
                              f"grip={measured[idx[-1] + 1]:.3f} [{lab}]")
        if _PLAN_LOG is not None:
            _PLAN_LOG.write(" ".join(fields) + "\n")
            _PLAN_LOG.flush()
        out = planned + self._pd_off
        return ActionChunk(actions=[Action(data=out, meta=chunk.actions[0].meta)], control_hz=chunk.control_hz)

    cls.reset, cls.act = reset, act
    cls._pandemo = True
