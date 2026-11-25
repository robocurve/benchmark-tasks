"""The scripted demo: one generator script per station (yam_pair CONTRACT §5), each in its own station frame,
reading object poses from the simulator (privileged). The two stations share no state: each runs on its own clock and
on what it reads of the light and the screws. They follow the same plan, so their moves of the light agree.

Each station's two hands pinch the rubber trim on the lower edges of the light's two long walls, 80 mm in from its end
(approach 35 degrees up, toward the light's middle; fingers across the wall). The hands hold their end on a closed
loop: each hand's height is corrected by the light's measured height at its grip (the arms give under the load), and
each station's two hands together correct their end's place along and across the light. The plan:

1. lift the light 12 mm off its stands; 2. carry it 5 mm toward A, so the keyholes' round centre holes come under the
screw heads; 3. raise it to 2 mm under the ceiling (the heads go up through the centre holes) once this end's keyholes
are lined up, and wait until both ends are up; 4. slide it 10 mm on toward A, into its outline: the screws go into the
narrow slots; 5. lower it onto the heads and let go. Then each arm takes the screwdriver beside it, brings it up under
the screw on its side, pushes the bit into the head (the motor runs while pushed) and holds the push until the clutch
slips, then lowers the driver and parks it, still in hand.

Variants (CLP_VARIANT) are the scorer checks of the README.
"""

from __future__ import annotations

import math
import os

import numpy as np
from yam_arena import ik
from yam_pair.demo import together, wait

import geometry as g

CEIL = g.CEIL
DEBUG = bool(os.environ.get("CLP_DEBUG"))
ELEV = 35.0                    # the pinch approach's elevation, degrees
PINCH_Z = -g.HOUSING_D + 0.008  # the tool point 8 mm above the walls' lower edge (the pads hold its lowest 21 mm)
REST_Z = -g.START_DROP          # the back's height (relative to the ceiling) on the stands
LIFT_Z = -0.018                 # lifted: the back 18 mm under the ceiling, 6 mm under the heads' bottoms
UP_Z = -0.002                   # raised: the back 2 mm under the ceiling (its underside 6 mm above the heads' tops)
HUNG_Z = -(g.GAP - g.SHEET_T)   # the back's height when it hangs on the heads (8 mm under the ceiling)
LOWER_Z = HUNG_Z - 0.003        # the hands go 3 mm lower than that, so the heads take the light
CENTRED_DX = g.KEY_LOCK         # the light's offset (toward B) with the centre holes under the heads
PARTIAL_TURNS = 2.0             # the checks that leave screws short: 2 of their 4.7 turns
DRV_ELEV = 0.0                  # the hand holds the upright screwdriver with its approach level
DRV_YAW = 0.0                   # and, under a screw, pointing straight ahead
DRIVE_SEED = [-1.31, 1.22, 1.54, -0.32, -1.31, 0.0]       # left arm, the driver below its screw
FOREARM_TOUCH = [-1.38, 1.44, 2.44, -1.07, -1.34, 0.03]   # check "forearm": A's left link 4 grazes the light (offline)


def rotm(approach, finger) -> np.ndarray:
    a = np.asarray(approach, float)
    a = a / np.linalg.norm(a)
    f = np.asarray(finger, float)
    f = f - a * (f @ a)
    f = f / np.linalg.norm(f)
    return np.column_stack([f, np.cross(a, f), a])


def elevated(elev_deg: float, yaw_deg: float = 0.0) -> np.ndarray:
    e, y = math.radians(elev_deg), math.radians(yaw_deg)
    return np.array([math.cos(e) * math.cos(y), math.cos(e) * math.sin(y), math.sin(e)])


def _rot_q(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


# seeds for the pinch pose (from dev/hold_torque.py): left arm; the right arm's are mirrored
PINCH_SEEDS = ([-1.83, 1.51, 2.10, 1.34, -0.91, 2.0], [-1.70, 1.45, 2.02, 1.18, -0.95, 1.79],
               [-1.50, 1.37, 1.96, 0.88, -0.96, 1.44])


def mirror(q) -> np.ndarray:
    q = np.asarray(q, float)
    return np.array([-q[0], q[1], q[2], q[3], -q[4], -q[5]])


class Station:
    """What one station's script knows: its own frame, its two arms, the object poses it reads."""

    def __init__(self, ctx, s: str, variant: str):
        self.ctx, self.s, self.variant = ctx, s, variant
        self.m = -1.0 if s == "a" else 1.0          # this end's side of the light (housing x)
        self.arm = {"left": ctx.left, "right": ctx.right}
        L, W, WT = g.HOUSING_L, g.HOUSING_W, g.WALL_T
        # each hand's grip spot in the housing frame: on the long wall on its own side, 80 mm in from this end
        self.spot = {}
        for side in ("left", "right"):
            ys = 1.0 if (s == "a") == (side == "left") else -1.0
            self.spot[side] = np.array([self.m * (L / 2 - g.GRIP_IN), ys * (W / 2 - WT / 2), PINCH_Z])
        self.dx, self.zb = g.START_SHIFT, REST_Z    # the light's planned offset toward B and back height
        self.corr_xy = np.zeros(2)                  # this station's correction of its end's place (world xy)
        self.corr_z = {"left": 0.0, "right": 0.0}
        self.freeze_z = False
        self.active = {"left": False, "right": False}
        self.R = {}
        self.phase = {"left": "start", "right": "start"}
        self.stage = "start"
        self.done = {}

    # -- reading the scene ---------------------------------------------------------------------------------------
    def housing(self):
        h = self.ctx.states()["housing"]
        return np.asarray(h.pos, float), _rot_q(h.quat_xyzw)

    def screw_pz(self, name: str) -> float:
        return float(self.ctx.states()["screws"].joints[f"{name}_pz"])

    def screw_spin(self, name: str) -> float:
        return float(self.ctx.states()["screws"].joints[f"{name}_spin"])

    def head_bottom(self, name: str) -> np.ndarray:
        hx, hy = g.SCREWS[name]
        return np.array([hx, hy, CEIL + self.screw_pz(name) - g.SCREW_L - g.HEAD_H])

    def key_error(self) -> np.ndarray:
        """This end's two keyholes' centre holes minus their screws (world xy), averaged; and the worst distance."""
        p, R = self.housing()
        errs = []
        for name in g.STATION_SCREWS[self.s]:
            kx, ky = g.KEYS[g.SCREW_KEY[name]]
            c = p + R @ np.array([kx, ky, 0.0])
            errs.append(c[:2] - np.array(g.SCREWS[name]))
        return np.mean(errs, axis=0), max(float(np.linalg.norm(e)) for e in errs)

    def end_heights(self):
        """The back's top at each end, relative to the ceiling."""
        p, R = self.housing()
        return [float((p + R @ np.array([sx * g.HOUSING_L / 2, 0.0, 0.0]))[2] - CEIL) for sx in (-1.0, 1.0)]

    def slot_along(self) -> list[float]:
        """For this end's screws: the shank's distance from its keyhole's centre along the slot (mm, signed)."""
        p, R = self.housing()
        out = []
        for name in g.STATION_SCREWS[self.s]:
            kx, ky = g.KEYS[g.SCREW_KEY[name]]
            loc = R.T @ (np.array([*g.SCREWS[name], CEIL]) - p)
            out.append(float(loc[0] - kx))
        return out

    def L(self, p) -> np.ndarray:
        return self.ctx.local(np.asarray(p, float))

    def measured_tcp(self, side: str) -> np.ndarray:
        st = self.ctx.state
        q = st[0:6] if side == "left" else st[7:13]
        return ik.tcp_pose(np.asarray(q, float), self.arm[side].base)[0]

    def log(self, side, text):
        if self.phase[side] != text:
            self.phase[side] = text
            if DEBUG:
                print(f"[scr] {self.s} {side} t {self.ctx.t:.2f} phase {text}", flush=True)

    def say(self, text):
        self.stage = text
        if DEBUG:
            print(f"[scr] {self.s} t {self.ctx.t:.2f} stage {text}", flush=True)

    # -- the hold ------------------------------------------------------------------------------------------------------
    def grip_target(self, side: str) -> np.ndarray:
        """Where this hand's tool point should be (world): its grip spot with the light at the planned pose, plus the
        corrections."""
        sp = self.spot[side]
        w = np.array([self.dx + sp[0], sp[1], CEIL + self.zb + sp[2]])
        w[:2] += self.corr_xy
        w[2] += self.corr_z[side]
        return w

    def pinch_rot(self, side: str) -> np.ndarray:
        if side not in self.R:
            arm = self.arm[side]
            p = self.L(self.grip_target(side))
            best = None
            for f in ((0, -1, 0),):                 # fingers this way round: the other way the wrist meets the wall
                R = rotm(elevated(ELEV), f)
                for q0 in [*PINCH_SEEDS, *(mirror(q) for q in PINCH_SEEDS)]:
                    q, err, rerr = ik.solve(p, ik.matrix_to_quat_xyzw(R), q_init=np.array(q0, float),
                                            base_pos=arm.base, iters=250)
                    key = err + 0.05 * rerr + 0.02 * abs(q[0] - (-1.7 if side == "left" else 1.7))
                    if best is None or key < best[0]:
                        best = (key, R, q)
            self.R[side], self.q_pinch = best[1], getattr(self, "q_pinch", {})
            self.q_pinch[side] = best[2]
        return self.R[side]

    def hold(self, side: str):
        """Closed loop while active: this hand at its grip spot on the light at the planned pose."""
        arm = self.arm[side]
        while self.active[side]:
            p, R = self.housing()
            meas = p + R @ self.spot[side]
            want = np.array([self.dx + self.spot[side][0], self.spot[side][1], CEIL + self.zb + self.spot[side][2]])
            err = want - meas
            if not self.freeze_z:
                self.corr_z[side] = float(np.clip(self.corr_z[side] + 0.25 * err[2], -0.02, 0.06))
            if side == "left":                       # one update per step for the station's xy correction
                e_xy = np.zeros(2)
                for sd in ("left", "right"):
                    e_xy += (np.array([self.dx + self.spot[sd][0], self.spot[sd][1]]) - (p + R @ self.spot[sd])[:2]) / 2
                self.corr_xy = np.clip(self.corr_xy + np.array([0.06, 0.12]) * e_xy, -0.03, 0.03)
            arm.aim(self.L(self.grip_target(side)), self.pinch_rot(side))
            yield

    def ramp(self, attr: str, to: float, seconds: float):
        a0 = getattr(self, attr)
        n = max(1, round(seconds * 20))
        for k in range(1, n + 1):
            setattr(self, attr, a0 + smooth(k / n) * (to - a0))
            yield

    # -- the script ----------------------------------------------------------------------------------------------------
    def run(self):
        v = self.variant
        if v == "nothing" or (v == "alone" and self.s == "b"):
            yield from wait(5.0)
            return
        if v == "alone":
            yield from self.alone()
            return
        if v in ("fetch_only", "drive_test"):      # development: the screwdriving alone
            yield from wait(1.0)
            yield from together(self.drive("left"), self.drive("right"))
            yield from wait(2.0)
            return
        yield from self.take_hold()
        for side in ("left", "right"):
            self.active[side] = True
        yield from together(self.hold("left"), self.hold("right"), self.conductor())
        if v in ("drop", "unslid", "set_down", "let_go_b", "one_end_hung") or self.done.get("stop"):
            yield from wait(4.0)
            return
        if v == "still_held" and self.s == "a":
            return
        yield from together(self.drive("left"), self.drive("right"))
        if v == "forearm" and self.s == "a":
            yield from self.touch_with("forearm")
        if v == "driver_touch" and self.s == "a":
            yield from self.touch_with("driver")
        yield from wait(4.0)

    def take_hold(self):
        """Both hands from below to the trim on the long walls, open, then up and closed."""
        self.say("reach")
        for side in ("left", "right"):
            self.log(side, "reach")
            self.pinch_rot(side)
        pre = {sd: self.L(self.grip_target(sd)) - 0.05 * self.R[sd][:, 2] - np.array([0.0, 0.0, 0.03])
               for sd in ("left", "right")}
        low = {sd: pre[sd] - np.array([0.0, 0.0, 0.12]) for sd in ("left", "right")}       # first well below the light
        q_low = {sd: self.solve(sd, low[sd], self.R[sd], [self.q_pinch[sd] if sd == "left" else mirror(self.q_pinch[sd]),
                                                          *PINCH_SEEDS]) for sd in ("left", "right")}
        yield from together(*(self.joint_move(sd, q_low[sd], low[sd], self.R[sd], 5.0, grip=1.0) for sd in ("left", "right")))
        yield from together(*(self.arm[sd].move(pre[sd], self.R[sd], 1.5) for sd in ("left", "right")))
        yield from together(*(self.arm[sd].move(self.L(self.grip_target(sd)), self.R[sd], 1.5) for sd in ("left", "right")))
        yield from together(*(self.arm[sd].set_grip(0.0, 0.8) for sd in ("left", "right")))
        yield from wait(0.3)

    def conductor(self):
        """The plan, on top of the running holds."""
        v = self.variant
        while self.ctx.t < 6.5:                     # both stations start lifting at the same time
            yield
        self.say("lift")
        yield from self.ramp("zb", LIFT_Z, 1.5)
        yield from wait(0.5)
        if v == "set_down":                         # lower it back onto its stands by hand, then let go
            self.say("set down")
            yield from self.ramp("zb", REST_Z - 0.004, 2.0)
            yield from wait(1.0)
            yield from self.let_go()
            return
        if v == "let_go_b" and self.s == "b":       # B opens its hands with the light lifted, and keeps them there
            yield from wait(1.0)
            yield from self.let_go(quick=True)
            return
        self.say("carry toward A")
        yield from self.ramp("dx", CENTRED_DX, 3.0)
        self.say("line up")
        t0, ok = self.ctx.t, 0
        while self.ctx.t - t0 < 4.0 and ok < 10:
            ok = ok + 1 if self.key_error()[1] < 0.0004 else 0
            yield
        if v == "drop":                             # open all hands with the light 14 mm over its stands, below the heads
            yield from self.ramp("zb", -0.016, 1.0)
            yield from wait(0.5)
            yield from self.let_go(quick=True)
            return
        low_b = v == "one_end_hung" and self.s == "b"     # B keeps its end low: its heads never go through
        self.say("raise")
        yield from self.ramp("zb", LIFT_Z if low_b else UP_Z, 1.5)
        t0 = self.ctx.t
        while min(self.end_heights()) < -0.0055 and self.ctx.t - t0 < 6.0:     # both ends up over the heads
            yield
        yield from wait(0.3)
        if v == "unslid":                           # open all hands with the light up, the heads in the centre holes
            yield from self.let_go()
            return
        self.say("slide toward A")
        yield from self.ramp("dx", 0.0, 2.0)
        t0 = self.ctx.t
        while min(abs(a) for a in self.slot_along()) < 0.0085 and self.ctx.t - t0 < 3.0:
            yield
        yield from wait(0.3)
        self.say("lower onto the heads")
        yield from self.ramp("zb", LIFT_Z if low_b else LOWER_Z, 1.5)
        self.freeze_z = True
        yield from wait(0.5)
        if (v == "still_held" and self.s == "a") or low_b:     # these hands keep their hold to the end
            while True:
                yield
        yield from self.let_go()

    def let_go(self, quick: bool = False):
        """Open both hands, then take them away (back along the approach and down). quick (check "drop"): open them
        fast and keep them still for 1.5 s first."""
        self.say("let go")
        self.active = {"left": False, "right": False}
        yield
        yield from together(*(self.arm[sd].set_grip(1.0, 0.25 if quick else 0.6) for sd in ("left", "right")))
        if quick:
            yield from wait(1.5)
        yield from together(*(self.arm[sd].move(self.arm[sd].pos - 0.04 * self.R[sd][:, 2] - [0.0, 0.0, 0.05],
                                                self.R[sd], 1.0) for sd in ("left", "right")))
        self.say("hands off")

    # -- screwdriving ------------------------------------------------------------------------------------------------
    def driver_rot(self, arm, target_local) -> np.ndarray:
        """The driver upright between the fingers, the approach level and pointing from the arm's base to the target."""
        d = np.asarray(target_local, float) - arm.base
        yaw = math.atan2(d[1], d[0])
        return rotm(elevated(0.0, math.degrees(yaw)), (math.sin(yaw), -math.cos(yaw), 0.0))

    def servo_to(self, arm, side, pos_local, R, seconds: float):
        """Move the hand there, then correct by its measured tool point (FK of the measured joints)."""
        off = np.zeros(3)
        for _ in range(max(1, round(seconds * 20))):
            off = np.clip(off + 0.3 * (pos_local - self.measured_tcp(side)), -0.03, 0.03)
            arm.aim(pos_local + off, R)
            yield

    def solve(self, side, pos, R, seeds) -> np.ndarray:
        """IK for this hand, from the seeds given (the right arm's mirrored), the exact solution nearest the first."""
        arm = self.arm[side]
        best = None
        for q0 in seeds:
            q0 = np.asarray(q0, float) if side == "left" else mirror(q0)
            q, err, rerr = ik.solve(pos, ik.matrix_to_quat_xyzw(R), q_init=q0, base_pos=arm.base, iters=300)
            key = (err > 0.001 or rerr > 0.01, err + 0.05 * rerr)
            if best is None or key < best[0]:
                best = (key, q)
        return best[1]

    def joint_move(self, side, q1, pos, R, seconds, grip=None):
        """Straight in joint space to q1 (an IK solution of pos, R)."""
        arm = self.arm[side]
        q0, g0 = arm.q.copy(), arm.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(seconds * 20))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            arm.q, arm.grip = q0 + s * (np.asarray(q1) - q0), g0 + s * (g1 - g0)
            yield
        arm.pos, arm.rot = np.asarray(pos, float), np.asarray(R, float)

    def drive(self, side: str):
        v, arm = self.variant, self.arm[side]
        name = f"{self.s}_{side}"
        if (v == "untightened" or (v == "one_end" and self.s == "b") or (v in ("one_screw", "forearm") and name == "a_left")):
            yield from arm.move(arm.pos - [0.0, 0.0, 0.10], arm.rot, 1.5)
            yield from arm.go_home(2.5)
            return
        sgn = 1.0 if side == "left" else -1.0
        # -- down from the light, then over the screwdriver beside this arm, down around it, and take it off its holder
        self.log(side, "fetch driver")
        yield from arm.move(arm.pos - [0.0, 0.0, 0.10], arm.rot, 1.2)
        hx, hy = g.HOLDER_LOCAL[side]
        yaw = math.radians(g.HOLDER_YAW[side])
        grip = np.array([hx, hy, g.TOP + g.HOLDER_FLOOR + g.DRV_GRIP])
        Rh = rotm(elevated(DRV_ELEV, g.HOLDER_YAW[side]), (math.sin(yaw), -math.cos(yaw), 0.0))
        pre = grip + [0.0, 0.0, 0.20]                             # the fingers above the bit
        q_pre = self.solve(side, pre, Rh, ([-0.06, 0.37, 0.86, -0.49, -1.1, 0.0], [-0.06, 0.22, 0.65, -0.42, -1.1, 0.0]))
        yield from self.joint_move(side, q_pre, pre, Rh, 4.0, grip=1.0)
        yield from arm.move(grip + [0.0, 0.0, 0.02], Rh, 2.0)
        yield from self.servo_to(arm, side, grip, Rh, 0.8)
        loose = v == "slip" and name == "b_right"
        yield from arm.set_grip(0.355 if loose else 0.0, 0.8)    # slip: closes only to 33.3 mm on the 36 mm grip
        yield from arm.move(arm.pos + [0.0, 0.0, 0.06], Rh, 0.8)
        if DEBUG:
            self.report_driver(side, "lifted")
        if not self.driver_in_hand(side):                 # missed it: don't chase a dropped driver
            self.log(side, "no driver in hand")
            yield from arm.set_grip(1.0, 0.5)
            yield from arm.move(arm.pos + [0.0, 0.0, 0.05], arm.rot, 1.0)
            return
        # -- toward the screw on this side, below the walls; then under it (approach straight ahead), the bit 90 mm below
        # the head, and up to 30 mm below it
        head = self.L(self.head_bottom(name))
        bit_up = g.DRV_TIP - g.DRV_GRIP
        yield from arm.move(np.array([0.20, sgn * 0.26, g.TOP + 0.48]), Rh, 1.5)
        dyaw = math.radians(-sgn * DRV_YAW)               # the wrist stays outside the long wall, clear of the end wall
        Rs = rotm(elevated(DRV_ELEV, math.degrees(dyaw)), (math.sin(dyaw), -math.cos(dyaw), 0.0))
        low = head - np.array([0.0, 0.0, 0.060 + bit_up])          # the bit just below the light's lower edge
        q_low = self.solve(side, low, Rs, (DRIVE_SEED,))
        self.log(side, "to the screw")
        yield from self.joint_move(side, q_low, low, Rs, 3.0)
        yield from wait(0.3)
        yield from arm.move(low + [0.0, 0.0, 0.03], Rs, 1.0)
        if DEBUG:
            self.report_driver(side, "under the screw")
        yield from self.push_and_turn(side, name, Rs, partial=(v == "one_tight_b" and self.s == "b"))
        # -- down and away, still holding it
        self.log(side, "park the driver")
        # down 100 mm, interpolated in joint space: at the screw the elbow is folded nearly shut, where a straight-line
        # move's IK can jump several degrees in one step (a one-step torque spike)
        yield from arm.move_joints(arm.pos - [0.0, 0.0, 0.10], arm.rot, 2.0)
        yield from arm.move(np.array([0.18, sgn * 0.22, g.TOP + 0.50]), arm.rot, 3.0)  # out from under the light
        yield from arm.move(np.array([0.18, sgn * 0.22, g.TOP + 0.58]), arm.rot, 2.0)  # beside it: a light pose to hold

    def driver_in_hand(self, side) -> bool:
        """The driver's bottom within 15 mm of where it sits in a good grip, and within 10 degrees of upright."""
        d = self.ctx.states()[f"driver_{self.s}_{side}"]
        Rd = self.ctx.layout.rot_to_local(self.s, _rot_q(d.quat_xyzw))
        rel = self.L(np.asarray(d.pos, float)) - self.measured_tcp(side)
        return bool(np.linalg.norm(rel - [0.0, 0.0, -g.DRV_GRIP]) < 0.015 and Rd[2, 2] > math.cos(math.radians(10.0)))

    def report_driver(self, side, when):
        """Debug: where the driver sits in the hand (station frame): its bottom relative to the measured tool point."""
        d = self.ctx.states()[f"driver_{self.s}_{side}"]
        Rd = self.ctx.layout.rot_to_local(self.s, _rot_q(d.quat_xyzw))
        rel = self.L(np.asarray(d.pos, float)) - self.measured_tcp(side)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(Rd[2, 2])))))
        print(f"[fetch] {self.s} {side} t {self.ctx.t:.2f} {when}: driver bottom - tool point "
              f"{np.round(rel * 1000, 1).tolist()} mm, tilt {tilt:.1f} deg, grip {self.ctx.state[6 if side == 'left' else 13]:.2f}",
              flush=True)

    def push_and_turn(self, side, name, Rs, partial=False):
        """Bring the bit up under the head and push it in, servoing the bit tip read from the driver's pose:
        - centre it 20 mm below the head (lateral error under 1 mm for 0.25 s);
        - rise to 3 mm below the head, still centring (back down if it drifts 5 mm off);
        - push it 1 mm into the head (about 10 N: the motor runs) and follow the head up while the screw turns;
        - once the screw has turned once more after its head met the light's back, stop when it has stopped turning for
          0.25 s (the clutch slips) or when the bit starts to walk off the head (0.8 mm off its axis); or after 16 s."""
        arm = self.arm[side]
        bit_up = g.DRV_TIP - g.DRV_GRIP
        off = np.zeros(3)
        t0 = self.ctx.t
        spin_ref, spin_t, spin0 = self.screw_spin(name), self.ctx.t, self.screw_spin(name)
        spin_u, raw_prev = 0.0, spin0
        tilt_R = np.eye(3)
        mode, ok_n, first, rise, seat_spin = "centre", 0, True, -0.020, None
        cmd = arm.pos.copy()
        self.log(side, "up to the head")
        lay = self.ctx.layout
        while self.ctx.t - t0 < 16.0:
            if not self.driver_in_hand(side):
                self.log(side, "driver slipped in the hand")
                break
            d = self.ctx.states()[f"driver_{self.s}_{side}"]
            Rd = lay.rot_to_local(self.s, _rot_q(d.quat_xyzw))         # all in this station's frame
            bit = self.L(np.asarray(d.pos, float)) + Rd[:, 2] * g.DRV_TIP
            head = self.L(self.head_bottom(name))
            e_xy = head[:2] - bit[:2]
            lat = float(np.linalg.norm(e_xy))
            if first:                                # the driver's place in the hand: take it out at once
                off[:2] = e_xy
                first = False
            if mode == "centre":
                ok_n = ok_n + 1 if lat < 0.001 else 0
                goal_z = head[2] - 0.020
                if ok_n >= 5:
                    mode, rise = "rise", -0.020
            elif mode == "rise":
                rise = min(rise + 0.0005, -0.003)    # 10 mm/s up to 3 mm below the head
                goal_z = head[2] + rise
                if lat > 0.005:
                    mode, ok_n = "centre", 0
                elif rise >= -0.003 and lat < 0.0008:
                    mode = "push"
                    self.log(side, "push and turn")
            else:
                goal_z = head[2] + 0.001                # the bit 1 mm into the head: about 10 N
                if lat > 0.003:
                    mode, rise = "rise", -0.006
            k_xy, k_z, step = (0.04, 0.06, 0.001) if mode == "push" else (0.08, 0.1, 0.002)
            want = np.r_[head[:2], goal_z - bit_up] + off
            if np.all(np.abs(want - cmd) < step):   # integrate only while the command keeps up (no wind-up)
                off += np.r_[k_xy * e_xy, k_z * (goal_z - bit[2])]
                off = np.clip(off, -0.06, 0.06)
                want = np.r_[head[:2], goal_z - bit_up] + off
            cmd = cmd + np.clip(want - cmd, -step, step)                 # at most 40 mm/s (20 mm/s pushing)
            axis = Rd[:, 2]                          # turn the hand to bring the driver upright
            tv = np.cross(axis, [0.0, 0.0, 1.0])
            sn = float(np.linalg.norm(tv))
            if sn > 1e-6:
                ang = math.asin(min(1.0, sn))
                k = tv / sn
                K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                tilt_R = (np.eye(3) + math.sin(0.1 * ang) * K + (1 - math.cos(0.1 * ang)) * K @ K) @ tilt_R
            arm.aim(cmd, tilt_R @ Rs)
            if DEBUG and (round(self.ctx.t * 20) % 10 == 0 or os.environ.get("CLP_DRV_TRACE") == f"{self.s}_{side}"):
                print(f"[drv] {self.s} {side} t {self.ctx.t:.2f} {mode} lat {lat * 1000:.2f} dz {(bit[2] - head[2]) * 1000:+.2f} "
                      f"tilt {math.degrees(math.acos(max(-1.0, min(1.0, axis[2])))):.1f} off {np.round(off * 1000, 1).tolist()}",
                      flush=True)
            yield
            raw = self.screw_spin(name)
            spin_u += math.remainder(raw - raw_prev, 2 * math.pi)
            raw_prev = raw
            if partial and spin_u >= PARTIAL_TURNS * 2 * math.pi:
                break
            if seat_spin is None and self.screw_pz(name) >= g.TRAVEL - 0.0003:
                seat_spin = spin_u                   # the head has met the light's back
            past = seat_spin is not None and spin_u - seat_spin > 2 * math.pi and mode == "push"
            if past and lat >= 0.0008:
                break                                # a turn past seating and the bit starts to walk off the head: stop
            if abs(spin_u - spin_ref) > 0.3:
                spin_ref, spin_t = spin_u, self.ctx.t
            elif past and lat < 0.001 and self.ctx.t - spin_t > 0.25:
                break                                # a turn past seating, the bit in the head, no turning: the clutch slips
        # let the push off slowly: the bit back down 4 mm below the head over about 2 s, still on the closed loop
        for _ in range(40):
            d = self.ctx.states()[f"driver_{self.s}_{side}"]
            Rd = lay.rot_to_local(self.s, _rot_q(d.quat_xyzw))
            bit = self.L(np.asarray(d.pos, float)) + Rd[:, 2] * g.DRV_TIP
            head = self.L(self.head_bottom(name))
            off[2] = float(np.clip(off[2] + 0.1 * (head[2] - 0.004 - bit[2]), -0.06, 0.06))
            want = np.r_[head[:2], head[2] - 0.004 - bit_up] + off
            cmd = cmd + np.clip(want - cmd, -0.0002, 0.0002)
            arm.aim(cmd, arm.rot)
            yield
        if DEBUG:
            print(f"[scr] {self.s} {side} t {self.ctx.t:.2f} screw {name} turned {spin_u / (2 * math.pi):.2f}", flush=True)

    # -- the checks' extra moves -----------------------------------------------------------------------------------------
    def touch_with(self, what: str):
        """A's left arm leaves its link 4 (a wrist link, not the gripper or fingers) or its held screwdriver against the
        light, to the end of the trial."""
        arm = self.arm["left"]
        self.log("left", f"{what} against the light")
        p, R = self.housing()
        if what == "driver":                       # the held driver's collar against the long wall's outer face
            yield from arm.move(np.array([arm.pos[0], g.HOUSING_W / 2 + g.DRV_COLLAR_R - 0.003, arm.pos[2] + 0.01]),
                                arm.rot, 2.5)
        else:                                      # link 4 (a wrist link) up against the light's long
            q_touch = np.array(FOREARM_TOUCH)      # wall from below; the gripper and fingers stay 10 mm or more clear
            q_pre = q_touch + np.array([0.0, -0.15, -0.20, 0.0, 0.0, 0.0])
            pos, rot = ik.tcp_pose(q_touch, arm.base)
            yield from self.joint_move("left", q_pre, pos, rot, 3.0, grip=1.0)
            yield from self.joint_move("left", q_touch, pos, rot, 2.0)
        while True:
            yield

    def alone(self):
        """A alone tries its best: both hands pinch the long walls as far toward the light's middle (its centre of mass)
        as they reach, and it runs the demo's lift, carry, raise and slide."""
        self.say("alone: reach toward the middle")
        self.q_pinch = {}
        for side in ("left", "right"):
            arm = self.arm[side]
            for xi in np.arange(0.60, 0.10, -0.01):           # the farthest spot along the wall this hand reaches
                self.spot[side][0] = -g.HOUSING_L / 2 + (xi - g.END_GAP - g.START_SHIFT)   # station x -> housing x
                p = self.L(self.grip_target(side))
                Rr = rotm(elevated(ELEV), (0, -1, 0))
                q = self.solve(side, p, Rr, [*PINCH_SEEDS, [0.0, 1.6, 1.6, 0.5, 0.0, 0.0], [-0.5, 1.8, 1.8, 0.3, -0.8, 1.5]])
                tcp, Rq = ik.tcp_pose(q, arm.base)
                if np.linalg.norm(tcp - p) < 0.001:
                    self.R[side], self.q_pinch[side] = Rr, q
                    break
            if DEBUG:
                print(f"[scr] a alone {side}: grip spot {np.round(self.spot[side] * 1000).tolist()} mm (housing frame), "
                      f"station x {xi:.2f}", flush=True)
        yield from self.take_hold()
        for side in ("left", "right"):
            self.active[side] = True
        yield from together(self.hold("left"), self.hold("right"), self.conductor())


def station_script(s: str, variant: str = ""):
    def script(ctx):
        st = Station(ctx, s, variant)
        ctx.flags["station"] = st
        yield from st.run()

    return script
