"""The scripted demo: one closed-loop controller per robot (CONTRACT §5.2), and the check variants.

Each controller reads only its own robot's 14-D joint state, the clock and the server's pose (privileged sim state),
and writes only its own 14-D action. The two share no object, message or barrier: they start together, follow the same
time table, and each corrects its own two grasps from the server pose it reads (the same pose the other one reads).

Robot A holds the server's -x side, robot B its +x side, each with its front arm (nearer the stand, -y) 45 mm behind
the front panel and its rear arm at the mirror point about the centre of mass. Fingers close vertically over the edge:
the lower finger carries the load. Each arm's joint targets are pre-loaded by K^-1 J^T F for its quarter of the weight.
"""

from __future__ import annotations

import math

import numpy as np
from yam_arena import ik
from yam_arena.constants import READY, SHOULDER, WRIST
from yam_arena.station import BASES

import armload
import geometry as g

HZ = 20.0
K = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
INSET = 0.012                      # tool point inside the side edge
GRIP_HOLD = (g.BODY_H - 0.008) / 0.094   # commanded opening while holding: 4 mm inside each face
G_FRONT_Y = g.PANEL_Y[0] + 0.045
G_REAR_Y = 2 * g.COM_Y - G_FRONT_Y
PUSH_HANDOVER = 0.43
PUSH_ROLL = {"front": 0.0, "rear": 0.0}               # front arms push to here, rear arms from here to home


def smooth(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


GRIP = {"front": (0.0, 10.0, 0.0), "rear": (15.0, 10.0, math.pi)}   # (yaw toward the rack, pitch down, roll)


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def grip_frame(inward: float, yaw_deg: float = 0.0, pitch_deg: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """World rotation of a grip approaching the side edge along x (``inward`` = +1 for robot A, -1 for B), turned
    ``yaw`` toward the rack and tipped ``pitch`` down. The YAM's fingers close along the gripper's x: vertical here."""
    z = _rz(math.radians(yaw_deg) * inward) @ np.array([inward, 0.0, 0.0])
    x = np.array([0.0, 0.0, 1.0])
    R = np.column_stack([x, np.cross(z, x), z])
    return R @ _ry(-math.radians(pitch_deg)) @ _rz(roll)


def push_frame(inward: float, theta_deg: float, roll: float = 0.0) -> np.ndarray:
    """Closed fingers pointing mostly +y (toward the rack), turned ``theta`` from the inward x direction; fingers
    (the gripper's x) vertical."""
    th = math.radians(theta_deg)
    z = np.array([inward * math.cos(th), math.sin(th), 0.0])
    x = np.array([0.0, 0.0, 1.0])
    return np.column_stack([x, np.cross(z, x), z]) @ _rz(roll)


# The shared time table (s). Both robots follow it from the same start.
T = dict(approach=2.5, reach_in=1.0, close=0.8, preload=0.5, lift=2.0, carry=2.0, settle=0.6, stab=1.6,
         rear_open=0.8, rear_out=0.8, deep=1.5, guide=0.4, front_open=0.8, front_out=0.8, to_push=1.6, push=3.4,
         seat=0.8, front_away=1.0, home=2.5)
E_ALIGN, E_STAB, E_DEEP, E_GUIDE = -0.005, 0.080, 0.170, 0.170


def schedule():
    t, out = 0.0, {}
    for k, v in T.items():
        out[k] = (t, t + v)
        t += v
    out["end"] = (t, t)
    return out


SCHED = schedule()


class ArmCmd:
    """Cartesian target -> joint target for one arm (station frame), with a pre-load and warm-started IK."""

    def __init__(self, side: str):
        self.side = side
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.worst_mm = 0.0

    SEEDS = ([0.8, 1.4, 1.2, 0.0, 0.5, 0.0], [-0.8, 1.4, 1.2, 0.0, -0.5, 0.0], list(READY))

    def solve(self, pos, rot):
        quat = ik.matrix_to_quat_xyzw(rot)
        q, err, rerr = ik.solve(np.asarray(pos, float), quat, q_init=self.q, base_pos=BASES[self.side], iters=80)
        if err > 0.002 or rerr > math.radians(2):
            for seed in self.SEEDS:
                q2, e2, r2 = ik.solve(np.asarray(pos, float), quat, q_init=np.array(seed, float), base_pos=BASES[self.side],
                                      iters=200)
                if e2 + 0.05 * r2 < err + 0.05 * rerr and np.abs(q2 - self.q).max() < 1.2:
                    q, err, rerr = q2, e2, r2
        self.q = q
        self.worst_mm = max(self.worst_mm, err * 1000)
        return q

    def tcp(self):
        return ik.tcp_pose(self.q, BASES[self.side])


class Racker:
    """One robot's controller. ``station``: "a" (holds the -x side) or "b" (+x side)."""

    def __init__(self, station: str, layout, variant: str = ""):
        self.s, self.L, self.variant = station, layout, variant
        self.sx = -1.0 if station == "a" else 1.0                 # which side of the server this robot holds
        self.inward = -self.sx
        self.role_side = {"front": "right" if station == "a" else "left", "rear": "left" if station == "a" else "right"}
        self.arm = {r: ArmCmd(self.role_side[r]) for r in ("front", "rear")}
        self.tcp_off = {r: np.zeros(3) for r in ("front", "rear")}  # free-motion tool-point correction (world)
        self.common = np.zeros(3)                                    # loaded: shared correction of both grasps (world)
        self.diff = 0.0                                              # loaded: front up / rear down (m)
        self.diffx = 0.0                                             # loaded: front / rear across (yaw), m
        self.q_cmd = {r: np.array(READY, float) for r in ("front", "rear")}
        self.start_tcp, self.anchor, self.last_tgt, self.release_pt = {}, {}, {}, {}
        self.finished = False
        self.phase = "start"
        self.y0 = None
        self.loads = {"front": 0.0, "rear": 0.0}
        self.peak_load = {"front": 0.0, "rear": 0.0}
        self.ik_by_phase = {}
        self.mirror = 0.0 if station == "a" else math.pi    # robot B's wrist turns the other way (joint limits)
        self.idle = variant == "nothing" or (variant == "a_alone" and station == "b") or (
            variant == "b_alone" and station == "a")

    # -- helpers ------------------------------------------------------------------------------------------------------
    def grasp_local(self, role):
        return np.array([self.sx * (g.BODY_W / 2 - INSET), G_FRONT_Y if role == "front" else G_REAR_Y, g.BODY_H / 2])

    def q_meas(self, state, role):
        side = self.role_side[role]
        return np.asarray(state[0:6] if side == "left" else state[7:13], float)

    def tcp_meas(self, state, role):
        _, _, p, _ = armload.fk(self.q_meas(state, role), self.role_side[role])
        return self.L.to_world(self.s, p)

    def measure_loads(self, state):
        for role in ("front", "rear"):
            q = self.q_meas(state, role)
            tau = np.clip(K * (self.q_cmd[role] - q), -np.array(g.CONT), np.array(g.CONT))
            self.loads[role] = armload.support_force(q, tau, self.role_side[role]) / 9.81

    def command(self, role, p_world, R_world, force_up=0.0):
        a = self.arm[role]
        before = a.worst_mm
        a.worst_mm = 0.0
        q = a.solve(self.L.to_local(self.s, p_world), self.L.rot_to_local(self.s, R_world))
        key = f"{role}:{self.phase}"
        self.ik_by_phase[key] = max(self.ik_by_phase.get(key, 0.0), a.worst_mm)
        a.worst_mm = max(before, a.worst_mm)
        if force_up:
            q = q + armload.preload(q, force_up, a.side, K)
        self.q_cmd[role] = q
        self.last_cmd_w = {**getattr(self, "last_cmd_w", {}), role: np.round(1000 * np.asarray(p_world), 1).tolist()}
        return np.r_[q, a.grip]

    def free_move(self, role, desired, R_world, state, gain=0.4):
        """Move the tool point to ``desired`` (world), correcting for the arm's tracking error from its own joints."""
        err = desired - self.tcp_meas(state, role)
        self.tcp_off[role] = np.clip(self.tcp_off[role] + np.clip(gain * err, -0.002, 0.002), -0.03, 0.03)
        return self.command(role, desired + self.tcp_off[role], R_world)

    def role_of(self, side):
        return "front" if self.role_side["front"] == side else "rear"

    def name(self, t):
        for k, (a, b) in SCHED.items():
            if a <= t < b:
                return k
        return "end"

    def release_t(self, role):
        """When this arm starts to open: the rear arms once the members are 60 mm in, the front arms at 170 mm."""
        if self.variant == "letgo":
            return SCHED["settle"][1]
        if self.variant in ("hold", "setdown", "b_high_hold"):
            return float("inf")
        return SCHED["rear_open"][0] if role == "rear" else SCHED["front_open"][0]

    # -- the controller -------------------------------------------------------------------------------------------------
    def act(self, view):
        if self.idle:
            self.finished = True
            self.phase = "ready"
            return np.r_[READY, 1.0, READY, 1.0]
        t, state = view.t, np.asarray(view.state, float)
        srv = view.objects()["server"]
        p, R = np.asarray(srv.pos, float), _rot(srv.quat_xyzw)
        if self.y0 is None:
            self.y0 = p[1]
            for role in ("front", "rear"):
                pos, rot = self.arm[role].tcp()
                self.start_tcp[role] = (self.L.to_world(self.s, pos), self.L.rot_to_world(self.s, rot))
        self.phase = self.name(t)
        self.measure_loads(state)
        out = self.alone(t, p, R, state) if self.variant == "a_alone" else self.team(t, p, R, state)
        return np.r_[out[self.role_of("left")], out[self.role_of("right")]]

    def desired_pose(self, t):
        """The server origin's desired position at time t, the same for both robots."""
        S, v = SCHED, self.variant
        e_start = self.y0 + g.MEMBER_Y[1] - g.Y_FRONT
        z_stand = g.STAND_TOP
        z_d = z_stand + smooth((t - S["lift"][0]) / T["lift"]) * (g.Z_RACKED - z_stand)
        e_d = e_start + smooth((t - S["carry"][0]) / T["carry"]) * (E_ALIGN - e_start)
        for name, e0, e1 in (("stab", E_ALIGN, E_STAB), ("deep", E_STAB, E_DEEP), ("guide", E_DEEP, E_GUIDE)):
            if t >= S[name][0]:
                e_d = e0 + smooth((t - S[name][0]) / T[name]) * (e1 - e0)
        if v == "b_high_hold":
            e_d = min(e_d, E_STAB)
        if v == "slide":
            z_d = z_stand
            e_d = e_start + smooth((t - S["lift"][0]) / (T["lift"] + T["carry"])) * 0.20
        if v in ("hold", "setdown") and t >= S["carry"][0]:
            e_d = e_start
        if v == "setdown" and t >= S["carry"][0]:
            z_d = max(g.Z_RACKED - (t - S["carry"][0]) * 0.150, z_stand - 0.002)
        return np.array([0.0, g.Y_FRONT + e_d - g.MEMBER_Y[1], z_d])

    def team(self, t, p, R, state):
        S, v = SCHED, self.variant
        mg4 = g.MASS * 9.81 / 4
        P_d = self.desired_pose(t)
        out = {}
        gw_d = {r: P_d + self.grasp_local(r) for r in ("front", "rear")}
        gw_m = {r: p + R @ self.grasp_local(r) for r in ("front", "rear")}
        bump = np.zeros(3)
        if v in ("b_high", "b_high_hold") and self.s == "b" and t >= S["carry"][0]:
            bump[2] = 0.022 * smooth((t - S["carry"][0]) / T["carry"])
        holding = [r for r in ("front", "rear") if S["preload"][0] <= t < self.release_t(r)]
        if t >= S["lift"][0] and holding:
            err = {r: gw_d[r] + bump - gw_m[r] for r in holding}
            mean = sum(err.values()) / len(err)
            self.common = np.clip(self.common + np.clip(0.3 * mean, -0.0015, 0.0015), -0.06, 0.06)
            if len(holding) == 2:
                pitch = (err["front"][2] - err["rear"][2]) / 2
                balance = 0.0006 * (self.loads["rear"] - self.loads["front"])
                self.diff = float(np.clip(self.diff + np.clip(0.2 * pitch + balance, -0.001, 0.001), -0.03, 0.03))
                if t < S["deep"][0]:                             # yaw: the members must enter square
                    yaw = (err["front"][0] - err["rear"][0]) / 2
                    self.diffx = float(np.clip(self.diffx + np.clip(0.3 * yaw, -0.001, 0.001), -0.03, 0.03))
            for r in holding:
                self.peak_load[r] = max(self.peak_load[r], self.loads[r])
        for role in ("front", "rear"):
            a = self.arm[role]
            yaw_g, pitch_g, roll_g = GRIP[role]
            Rg = grip_frame(self.inward, yaw_g, pitch_g, roll_g + self.mirror)
            out_dir = np.array([-self.inward, 0.0, 0.0])
            rel0 = self.release_t(role)
            if t < S["approach"][1]:
                s = smooth((t - S["approach"][0]) / T["approach"])
                pre = gw_m[role] + 0.06 * out_dir
                p0, R0 = self.start_tcp[role]
                self.anchor[role] = gw_m[role].copy()
                a.grip = 1.0
                out[role] = self.command(role, p0 + s * (pre - p0), ik.slerp_matrix(R0, Rg, s))
                continue
            if t < S["reach_in"][1]:
                s = smooth((t - S["reach_in"][0]) / T["reach_in"])
                out[role] = self.free_move(role, self.anchor[role] + (1 - s) * 0.06 * out_dir, Rg, state)
                continue
            if t < S["preload"][0]:
                a.grip = 1.0 + smooth((t - S["close"][0]) / T["close"]) * (GRIP_HOLD - 1.0)
                out[role] = self.free_move(role, self.anchor[role], Rg, state)
                continue
            if t < rel0:
                force = mg4 * smooth((t - S["preload"][0]) / T["preload"])
                if v == "slide":
                    force = 0.0
                if v == "setdown" and P_d[2] <= g.STAND_TOP:
                    force = 0.0
                    if t > S["carry"][0] + 2.0:
                        a.grip = 1.0
                sign = 1.0 if role == "front" else -1.0
                tgt = gw_d[role] + bump + self.anchor_off(role) + self.common + np.array([sign * self.diffx, 0,
                                                                                          sign * self.diff])
                if t >= S["deep"][0] and v != "b_high_hold":     # in the rails: let them steer across, push along
                    tgt[0] = gw_m[role][0] + self.anchor_off(role)[0]
                if role == "front" and t >= S["rear_open"][0] - 0.3 and v not in ("hold", "setdown", "slide", "b_high_hold"):
                    w = smooth((t - S["rear_open"][0] + 0.3) / 1.0)      # the rails take the weight: follow in z
                    force *= 1.0 - w
                    tgt[2] = (1.0 - w) * tgt[2] + w * self.tcp_meas(state, role)[2]
                if v == "slide" and t >= S["carry"][1]:
                    a.grip = 1.0
                self.last_tgt[role] = (tgt, Rg)
                out[role] = self.command(role, tgt, Rg, force)
                continue
            if t < rel0 + 0.8:
                a.grip = GRIP_HOLD + smooth((t - rel0) / 0.5) * (1.0 - GRIP_HOLD)
                tgt, Rg_ = self.last_tgt[role]
                fade = mg4 * (1.0 - smooth((t - rel0) / 0.5)) if role == "front" else 0.0
                out[role] = self.command(role, tgt, Rg_, fade)
                continue
            out[role] = self.after_release(t, role, p, R, state)
        return out

    def anchor_off(self, role):
        """The free-motion correction found at the grasp, kept while holding (so the hand does not jump)."""
        return self.tcp_off[role]

    def panel_point(self, p, R, role):
        x_push = self.sx * (0.17 if role == "front" else 0.137)
        return p + R @ np.array([x_push, g.PANEL_Y[0], g.BODY_H / 2])

    def after_release(self, t, role, p, R, state):
        """Back the hand out, then push the front panel home: front arms to PUSH_HANDOVER, rear arms the rest.
        Transfers are joint-space moves (no Cartesian path to fail); pushes are Cartesian along +y."""
        S, a, v = SCHED, self.arm[role], self.variant
        out_dir = np.array([-self.inward, 0.0, 0.0])
        prev_tgt, prev_R = self.last_tgt[role]
        out0 = self.release_t(role) + 0.8
        if t < out0 + 0.8:                                   # straight out from the edge
            s = smooth((t - out0) / 0.8)
            a.grip = 1.0
            return self.command(role, prev_tgt + s * 0.06 * out_dir, prev_R)
        if v in ("stop", "letgo", "slide"):                  # no push: both arms to their rest poses
            yaw_g, pitch_g, roll_g = GRIP[role]
            rest = np.array([self.sx * (0.36 if role == "front" else 0.33), -0.10 if role == "front" else 0.20, 1.08])
            R_rest = (push_frame(self.inward, 70.0, PUSH_ROLL[role] + self.mirror) if role == "front" else
                      grip_frame(self.inward, 0.0, 10.0, roll_g + self.mirror))
            if t >= out0 + 0.8 + 1.8:
                self.finished = True
            return self.joint_move(role, t, out0 + 0.8, 1.8, rest, R_rest, grip=1.0)
        theta = 70.0 if role == "front" else 40.0
        Rp = push_frame(self.inward, theta, PUSH_ROLL[role] + self.mirror)
        a_dir = Rp[:, 2]
        face = self.panel_point(p, R, role)

        def contact_tcp(y_face):
            return np.array([face[0], y_face, face[2]]) - 0.013 * a_dir   # pad tips on the face

        if role == "front":
            t_move, t_push = S["to_push"][0], S["push"][0]
            if t < t_push:                                   # joint-space move to 30 mm short of the face
                if t < t_move:
                    a.grip = 1.0
                    return self.hold_q(role)
                return self.joint_move(role, t, t_move, T["to_push"], contact_tcp(face[1] - 0.03), Rp, grip=0.0)
            if t < S["seat"][1]:                             # push it home; the ears press the posts
                self.push_last = self.push(role, t, S["push"], face, contact_tcp, Rp, state, g.MEMBER_Y[1] + 0.004)
                return self.push_last
            if v == "stay":
                self.finished = True
                return self.push_last
            backed = self.push_last_pos + np.array([0.0, -0.04, 0.0])
            if t < S["front_away"][0] + 0.4:                 # straight back off the face first
                s = smooth((t - S["front_away"][0]) / 0.4)
                return self.command(role, self.push_last_pos + s * (backed - self.push_last_pos), Rp)
            s = smooth((t - S["front_away"][0] - 0.4) / 1.6)     # then out and up to a rest pose clear of the rack
            rest = np.array([self.sx * 0.36, -0.10, 1.08])
            if t >= S["home"][1]:
                self.finished = True
            return self.command(role, backed + s * (rest - backed), Rp)
        rest = np.array([self.sx * 0.33, 0.20, 1.08])               # the rear arms rest up and outside, clear of it
        yaw_g, pitch_g, roll_g = GRIP[role]
        out = self.joint_move(role, t, out0 + 0.8, 1.8, rest, grip_frame(self.inward, 0.0, 10.0, roll_g + self.mirror),
                              grip=1.0)
        if t >= S["home"][1]:
            self.finished = True
        return out

    def push(self, role, t, window, face, contact_tcp, Rp, state, e_goal):
        """Push the panel along +y toward ``e_goal`` (member tips past the posts), the pads leading the face by up to
        15 mm; the tool point's own tracking error is corrected across and along, not up and down."""
        a = self.arm[role]
        s = smooth((t - window[0]) / (window[1] - window[0]))
        y_goal = g.Y_FRONT + e_goal - g.MEMBER_Y[1] + g.PANEL_Y[0]
        y_start = getattr(self, "push_y0", {}).get(role)
        if y_start is None:
            self.push_y0 = {**getattr(self, "push_y0", {}), role: face[1]}
            y_start = face[1]
        y_t = min(y_start + s * (y_goal - y_start), face[1] + 0.04)
        des = contact_tcp(y_t)
        err = des - self.tcp_meas(state, role)
        err[2] = 0.0
        self.tcp_off[role] = np.clip(self.tcp_off[role] + np.clip(0.3 * err, -0.002, 0.002), -0.03, 0.03)
        a.grip = 0.0
        self.push_last_pos = des - np.array([0.0, max(0.0, des[1] - face[1] - 0.005), 0.0])
        return self.command(role, des + self.tcp_off[role] * np.array([1.0, 1.0, 0.0]), Rp)

    def joint_move(self, role, t, t0, dur, pos_w, R_w, grip):
        """Joint-space move from where the arm was at ``t0`` to the IK solution of a world pose."""
        a = self.arm[role]
        key = (role, t0)
        if not hasattr(self, "_jm"):
            self._jm = {}
        if key not in self._jm:
            self.tcp_off[role] = np.zeros(3)
            q1 = self.solve_far(role, pos_w, R_w)
            self._jm[key] = (a.q.copy(), q1)
        q0, q1 = self._jm[key]
        s = smooth((t - t0) / dur)
        a.grip = grip
        a.q = q0 + s * (q1 - q0)
        self.q_cmd[role] = a.q.copy()
        return np.r_[a.q, a.grip]

    def joint_ready(self, role, t, t0, dur):
        a = self.arm[role]
        key = (role, "ready", t0)
        if not hasattr(self, "_jm"):
            self._jm = {}
        if key not in self._jm:
            self._jm[key] = (a.q.copy(), np.array(READY, float))
        q0, q1 = self._jm[key]
        s = smooth((t - t0) / dur)
        a.grip = 1.0
        a.q = q0 + s * (q1 - q0)
        self.q_cmd[role] = a.q.copy()
        return np.r_[a.q, a.grip]

    def hold_q(self, role):
        a = self.arm[role]
        return np.r_[a.q, a.grip]

    def solve_far(self, role, pos_w, R_w):
        a = self.arm[role]
        pos, quat = self.L.to_local(self.s, pos_w), ik.matrix_to_quat_xyzw(self.L.rot_to_local(self.s, R_w))
        best = None
        for q0 in [a.q] + [np.array(q, float) for q in ArmCmd.SEEDS] + [np.array([0.0, 1.6, 1.4, 0.2, 0.0, 0.0])]:
            q, err, rerr = ik.solve(pos, quat, q_init=q0, base_pos=BASES[a.side], iters=250)
            key = err * 1000 + math.degrees(rerr) + 0.05 * float(np.abs(q - a.q).sum())
            if best is None or key < best[0]:
                best = (key, q, err)
        key = f"{role}:{self.phase}"
        self.ik_by_phase[key] = max(self.ik_by_phase.get(key, 0.0), best[2] * 1000)
        return best[1]

    def go_ready(self, t, role, t0):
        a = self.arm[role]
        if not hasattr(self, "_ready_from"):
            self._ready_from = {}
        if role not in self._ready_from:
            self._ready_from[role] = a.q.copy()
        s = smooth((t - t0) / T["home"])
        a.grip = 1.0
        a.q = self._ready_from[role] + s * (np.array(READY) - self._ready_from[role])
        self.q_cmd[role] = a.q.copy()
        if s >= 1.0 and (role == "rear" or self.variant in ("stop", "letgo", "slide")):
            self.finished = True
        return np.r_[a.q, a.grip]

    # -- check: robot A alone holds the front and rear faces by their middles and tries to rack it ---------------------
    def alone(self, t, p, R, state):
        S = SCHED
        out = {}
        mg2 = g.MASS * 9.81 / 2
        P_d = self.desired_pose(t)
        e_start = self.y0 + g.MEMBER_Y[1] - g.Y_FRONT
        if t >= S["carry"][0]:
            e_d = e_start + smooth((t - S["carry"][0]) / T["carry"]) * (E_STAB - e_start)
            P_d[1] = g.Y_FRONT + e_d - g.MEMBER_Y[1]
        for role in ("front", "rear"):
            a = self.arm[role]
            G = np.array([0.0, g.PANEL_Y[0] + 0.012, g.BODY_H / 2]) if role == "front" else np.array(
                [0.0, g.BODY_Y[1] - 0.012, g.BODY_H / 2])
            z = np.array([0.0, 1.0 if role == "front" else -1.0, 0.0])
            x = np.array([0.0, 0.0, 1.0])
            Rg = np.column_stack([x, np.cross(z, x), z]) @ _ry(-math.radians(10))
            gw_d, gw_m = P_d + G, p + R @ G
            if t < S["approach"][1]:
                s = smooth(t / T["approach"])
                p0, R0 = self.start_tcp[role]
                self.anchor[role] = gw_m.copy()
                a.grip = 1.0
                out[role] = self.command(role, p0 + s * (gw_m - 0.06 * z - p0), ik.slerp_matrix(R0, Rg, s))
            elif t < S["reach_in"][1]:
                s = smooth((t - S["reach_in"][0]) / T["reach_in"])
                out[role] = self.free_move(role, self.anchor[role] - (1 - s) * 0.06 * z, Rg, state)
            elif t < S["preload"][0]:
                a.grip = 1.0 + smooth((t - S["close"][0]) / T["close"]) * (GRIP_HOLD - 1.0)
                out[role] = self.free_move(role, self.anchor[role], Rg, state)
            else:
                force = mg2 * smooth((t - S["preload"][0]) / T["preload"])
                if t >= S["lift"][0]:
                    self.common = np.clip(self.common + np.clip(0.15 * (gw_d - gw_m), -0.0015, 0.0015), -0.08, 0.08)
                out[role] = self.command(role, gw_d + self.common, Rg, force)
            if t > S["guide"][1]:
                self.finished = True
        self.phase = "alone:" + self.name(t)
        return out


def controllers(layout, variant: str = ""):
    return {"a": Racker("a", layout, variant), "b": Racker("b", layout, variant)}
