"""Scripted demonstration for the humming top: the pumping controller.

The arm is driven by joint position targets at 20 Hz. Each joint follows its target like a damped spring
(stiffness and damping are the actuators' published gains), so a fast stroke commanded as a plain path lags,
swings sideways by more than a centimetre and tips the hand by several degrees, which drags the top's point
about. This controller plans the hand's motion (a straight vertical line through the top's point) and sends,
for every control step, the target that makes the lagging joints follow that plan: the planned joint angle
plus its speed times damping/stiffness plus its acceleration times inertia/stiffness plus the angle the
expected push bends the joint by. Between strokes it reads the top (spin, point, rod) and the hand and
re-plans; before letting go it stands the top upright over its own point.
"""
import math
import numpy as np
from yam_arena import ik

DT = .05
KQ = np.array([80., 80., 80., 10., 10., 10.])        # N m / rad, actuator position gains
CQ = np.array([5., 5., 5., 1.5, 1.5, 1.5])           # N m s / rad, actuator damping gains
IQ = np.array([.25, .25, .25, .015, .015, .015])     # kg m2, effective joint inertias fitted to free-air strokes


def smooth(s):
    s = min(max(s, 0.), 1.)
    return s * s * s * (10 - 15 * s + 6 * s * s)      # zero speed and acceleration at both ends


def jacobian(q, base):
    p, r = ik.tcp_pose(q, base)
    J = np.zeros((6, 6))
    eps = 1e-6
    for i in range(6):
        dq = np.array(q, float)
        dq[i] += eps
        p2, r2 = ik.tcp_pose(dq, base)
        J[:3, i] = (p2 - p) / eps
        J[3:, i] = ik._rot_error(r2, r) / eps
    return J


class Lead:
    """Joint targets that make the lagging joints follow a planned TCP path."""
    def __init__(self, base, q0):
        self.base = np.asarray(base, float)
        self.q = np.array(q0, float)

    def joints(self, pos, R):
        q, _, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(R), self.q, base_pos=self.base, iters=40)
        return q

    def command(self, p0, p1, p2, load_up=0., gain=1.):
        """p0, p1, p2: planned (pos, R) now, half a step on, one step on. Returns the (pos, R) to send."""
        q0 = self.joints(*p0)
        self.q = q0
        q1 = self.joints(*p1)
        q2 = self.joints(*p2)
        if np.linalg.norm(np.asarray(p2[0]) - np.asarray(p0[0])) < 1e-4:
            gain = 0.                # standing still or only turning: send the pose itself
        qd = (q2 - q0) / DT
        qdd = 4 * (q2 - 2 * q1 + q0) / (DT * DT)
        qc = q1 + gain * (CQ / KQ * qd + IQ / KQ * qdd)
        if load_up:
            J = jacobian(q1, self.base)
            qc = qc - J[:3].T @ np.array([0., 0., load_up]) / KQ     # the push bends the joints; lean against it
        qc = np.clip(qc, ik.LOWER + 1e-3, ik.UPPER - 1e-3)
        self.q = q0
        return ik.tcp_pose(qc, self.base)


class Stroke:
    """Planned rod travel s(t) for one down stroke: speed up, stay just ahead of the spinning body, brake."""
    def __init__(self, travel, spin, K, alpha, margin=10., a_up=7., a_brake=9., u_end=.10, u_cap=.62):
        self.travel = travel
        u0 = min(u_cap, (spin + margin) / K)
        a_follow = alpha / K
        t = 0.
        s = 0.
        u = 0.
        self.pts = [(0., 0., 0.)]
        self.load = []                 # (t0, t1) while the screw is expected to push back
        h = .002
        t_eng = None
        while s < travel and t < 2.:
            brake_dist = max(0., (u * u - u_end * u_end) / (2 * a_brake))
            if travel - s <= brake_dist + 1e-4 and u > u_end:
                if t_eng is not None and len(self.load) == 0:
                    self.load.append((t_eng, t))
                u = max(u_end, u - a_brake * h)
            elif u < u0 and t_eng is None:
                u = min(u0, u + a_up * h)
            else:
                if t_eng is None:
                    t_eng = t
                u = min(u_cap, u + a_follow * h)
            s += u * h
            t += h
            self.pts.append((t, s, u))
        if t_eng is not None and not self.load:
            self.load.append((t_eng, t))
        self.T = t
        self.ts = np.array([p[0] for p in self.pts])
        self.ss = np.array([p[1] for p in self.pts])

    def s(self, t):
        return float(np.interp(t, self.ts, self.ss))

    def loaded(self, t):
        return any(a <= t <= b for a, b in self.load)


def expm(r):
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(r, float) / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * Kx + (1 - math.cos(th)) * Kx @ Kx


class Obs:
    """What the controller reads at a control step (from the scene; a camera-based policy would estimate it)."""
    def __init__(self, rod, spin, tip_xy, axis, hand_pos, hand_R):
        self.rod = float(rod)
        self.spin = float(spin)
        self.tip_xy = np.asarray(tip_xy, float)
        self.axis = np.asarray(axis, float)
        self.hand_pos = np.asarray(hand_pos, float)
        self.hand_R = np.asarray(hand_R, float)


class Pumper:
    """Pumps the knob N times, stands the top upright over its point, lets go and moves away."""
    def __init__(self, base, q_arm, start_pos, R_down, K, alpha, push, strokes=6, rod_top=.065, pinch=-.0105,
                 open_g=-.026, watch=(.09, .17, .205), grip_above_point=.145, log=print):
        self.lead = Lead(base, q_arm)
        self.R0 = np.array(R_down, float)
        self.K, self.alpha, self.push = K, alpha, push
        self.N = strokes
        self.rod_top = rod_top
        self.pinch, self.open_g = pinch, open_g
        self.watch = np.asarray(watch, float)
        self.L = grip_above_point
        self.log = log
        self.xy = np.array(start_pos[:2], float)      # the hand's line (the top's point)
        self.zb = float(start_pos[2])                 # hand height with the knob bottomed
        self.bias = np.zeros(3)
        self.segs = []
        self.end = 0.
        self.state = 'start'
        self.stroke_i = 0
        self.spin_nominal = 0.
        self.centre = None
        self.cmd = {}
        self.release_step = None
        self.hold = (np.array(start_pos, float), self.R0, pinch)
        self.land = .004                              # plan each stroke to end this far above the stop
        self.stroke_kw = {}
        self.t_top, self.t_ret, self.t_bottom = .25, .55, .10
        self.t_open, self.t_lift, self.lift = .30, .45, .064   # straight up until 5 cm clear of the knob
        self.follow = 0.                              # (off) share of the knob-over-point error taken out per step
        self.Rc = np.eye(3)                           # slow turn of the hand that keeps the top's axis upright
        self.straighten = .2                          # share of the top's lean taken out per step between strokes
        self.enough = 1e9                             # stop pumping once the spin read at the bottom passes this
        self.N_nominal = strokes
        self.follow_cap = .0015                       # m per control step
        self.shift = np.zeros(2)                      # slow sideways shift of the whole plan

    # ---- timeline ----
    def add(self, T, f, tag):
        self.segs.append((self.end, T, f, tag))
        self.end += T

    def plan(self, t):
        if not self.segs:
            return (*self.hold, 0.)
        for t0, T, f, tag in self.segs:
            if t < t0 + T - 1e-7:      # a time on a boundary belongs to the segment that starts there
                return f(max(0., t - t0))
        t0, T, f, tag = self.segs[-1]
        return f(T)

    def tag(self, t):
        for t0, T, f, tag in self.segs:
            if t < t0 + T - 1e-7:
                return tag
        return self.segs[-1][3] if self.segs else 'start'

    def _line(self, z, finger=None):
        return np.array([self.xy[0], self.xy[1], z]), self.R0, self.pinch if finger is None else finger

    def _move(self, T, z0, z1, xy0=None, tag='move'):
        xy0 = self.xy.copy() if xy0 is None else np.array(xy0, float)
        xy1 = self.xy.copy()

        def f(tau):
            s = smooth(tau / T)
            xy = xy0 + s * (xy1 - xy0)
            return np.array([xy[0], xy[1], z0 + s * (z1 - z0)]), self.R0, self.pinch, 0.
        self.add(T, f, tag)

    def _top(self):
        """How far the knob is raised: the full travel with the hand straight down, less when the hand is turned
        (near the top of the stroke the arm has only a few degrees of wrist reach left)."""
        ang = math.degrees(math.acos(max(-1., min(1., (np.trace(self.Rc) - 1) / 2))))
        return self.rod_top - .013 * min(1., max(0., (ang - 1.5) / 2.))

    # ---- events ----
    def _advance(self, t, obs):
        """Called when the plan runs out within the next step: append what comes next."""
        rod = obs.rod if obs is not None else (self._top() if self.state in ('top',) else 0.)
        if obs is not None and self.state in ('start', 'ready', 'top'):
            # where the hand is when the knob is bottomed, and the hand's steady offset from its plan
            want = self.plan(t)[0] + np.array([self.shift[0], self.shift[1], 0.])
            step = np.clip(.7 * (obs.hand_pos - want), -.004, .004)
            if rod < .005:
                step[2] = 0.                      # the knob is on its stop: the height error is not the arm's
            self.bias = np.clip(self.bias + step, -.008, .008)
            self.zb = float(want[2] - rod + step[2])
        if self.state == 'start':
            self._pause(.4, self.zb + rod, 'settle')
            self.state = 'ready'
        elif self.state == 'ready':
            self._move(.8, self.zb + rod, self.zb + self._top(), tag='raise the knob')
            self._pause(self.t_top, self.zb + self._top(), 'top')
            self.state = 'top'
        elif self.state == 'top':
            self.stroke_i += 1
            spin = obs.spin if obs is not None else self.spin_nominal
            st = Stroke(max(.01, rod - self.land), max(spin, 0.), self.K, self.alpha, **self.stroke_kw)
            z0 = self.zb + rod
            # start so that the screw's push ends exactly on a control step (commands are held for 50 ms)
            if st.load:
                wait = (-(self.end + st.load[0][1])) % DT
                if wait > 1e-6:
                    self._pause(wait, z0, 'top')

            def f(tau, st=st, z0=z0, xy=self.xy.copy()):
                return (np.array([xy[0], xy[1], z0 - st.s(tau)]), self.R0, self.pinch,
                        self.push if st.loaded(tau) else 0.)
            self.add(st.T, f, f'pump {self.stroke_i}')
            eng = sum(b - a for a, b in st.load)
            self.spin_nominal = max(spin, 0.) + (self.alpha) * eng * .85 - 2.
            self.log(f'PUMP_PLAN stroke {self.stroke_i} spin {spin:.1f} rod {rod * 1000:.1f} mm T {st.T:.3f} s '
                     f'loaded {eng:.3f} s line {np.round(self.xy + self.shift, 4).tolist()} bias_mm {np.round(self.bias * 1000, 1).tolist()}')
            self._move(.15, z0 - st.s(st.T), self.zb - .0015, tag='bottom')
            self._pause(self.t_bottom, self.zb - .0015, 'bottom')
            self.state = 'bottom'
        elif self.state == 'bottom':
            more = self.stroke_i < (self.N if obs is not None else self.N_nominal)
            if more and (obs is None or obs.spin < self.enough):
                self._move(self.t_ret, self.zb - .0015, self.zb + self._top(), tag=f'return {self.stroke_i}')
                self._pause(self.t_top, self.zb + self._top(), 'top')
                self.state = 'top'
            else:
                self._pause(.30, self.zb - .0015, 'settle')
                self.state = 'centre'
                self.centre = dict(t0=self.end, xy=np.zeros(2), R=np.eye(3), ok=0, n=0)
                self._pause(1.2, self.zb - .0015, 'stand it upright')
        elif self.state == 'centre':
            self._release(t)

    def _pause(self, T, z, tag):
        def f(tau, z=z, xy=self.xy.copy()):
            return np.array([xy[0], xy[1], z]), self.R0, self.pinch, 0.
        self.add(T, f, tag)

    def _release(self, t):
        c = self.centre
        p0 = np.array([self.xy[0] + self.shift[0] + c['xy'][0], self.xy[1] + self.shift[1] + c['xy'][1], self.zb - .001])
        R = c['R'] @ self.Rc @ self.R0
        # cut the plan here (the upright phase may end early)
        self.segs = [s for s in self.segs if s[0] < t]
        if self.segs:
            t0, T, f, tag = self.segs[-1]
            self.segs[-1] = (t0, t - t0, f, tag)
        self.end = t

        def opening(tau):
            return p0, R, self.pinch + smooth(tau / self.t_open) * (self.open_g - self.pinch), 0.

        def lift(tau):
            return p0 + np.array([0, 0, self.lift * smooth(tau / self.t_lift)]), R, self.open_g, 0.
        p1 = p0 + np.array([0, 0, self.lift])

        def away(tau):
            s = smooth(tau / .9)
            return p1 + s * (self.watch - p1), ik.slerp_matrix(R, self.R0, s), self.open_g, 0.
        self.add(self.t_open, opening, 'open on the knob')
        self.add(self.t_lift, lift, 'lift off the knob')
        self.add(.90, away, 'move away')
        self.add(60., lambda tau: (self.watch, self.R0, self.open_g, 0.), 'watch it spin')
        self.state = 'done'
        self.log(f'PUMP_PLAN release at {t:.2f} s offset_mm {np.round(c["xy"] * 1000, 1).tolist()}')

    # ---- one control step ----
    def step(self, k, obs=None, planned=False):
        """Target (pos, R, finger) for control step k (time k * DT after the pinch). planned: the planned pose
        itself, without the lead for the joints' lag."""
        if k in self.cmd:
            return self.cmd[k]
        t = k * DT
        guard = 0
        while self.state != 'done' and t + DT > self.end - 1e-9 and guard < 4:
            self._advance(min(t, self.end), obs)
            guard += 1
        if self.state == 'centre' and self.centre is not None and t >= self.centre['t0'] + .30 - 1e-9:
            out = self._centre_step(t, obs)
        else:
            p0, p1, p2 = self.plan(t), self.plan(t + DT / 2), self.plan(t + DT)
            if planned:
                self.cmd[k] = (p2[0], p2[1], p2[2])
                return self.cmd[k]
            tag = self.tag(t)
            still = tag in ('bottom', 'settle')
            if obs is not None and self.state != 'done' and still:
                # Between strokes, turn the hand a little so the top's axis comes upright (the pinch carries the top).
                a = obs.axis / np.linalg.norm(obs.axis)
                Rc = expm(self.straighten * np.cross(a, np.array([0., 0., 1.]))) @ self.Rc
                if math.degrees(math.acos(max(-1., min(1., (np.trace(Rc) - 1) / 2)))) < 4.5:
                    self.Rc = Rc
                if self.follow:
                    step = np.clip(self.follow * (obs.tip_xy - obs.hand_pos[:2]), -self.follow_cap, self.follow_cap)
                    self.shift = np.clip(self.shift + step, -.03, .03)
            b = self.bias - np.array([self.shift[0], self.shift[1], 0.]) * (self.state != 'done')
            Rc = self.Rc if self.state != 'done' else np.eye(3)
            pos, R = self.lead.command((p0[0] - b, Rc @ p0[1]), (p1[0] - b, Rc @ p1[1]), (p2[0] - b, Rc @ p2[1]),
                                       load_up=p1[3])
            out = (pos, R, p2[2])
        self.cmd[k] = out
        return out

    def _centre_step(self, t, obs):
        """Closed loop: bring the knob over the point and the axis to vertical; then let go."""
        c = self.centre
        z = self.zb - .001
        if obs is not None:
            a = obs.axis / np.linalg.norm(obs.axis)
            tilt = math.degrees(math.acos(max(-1., min(1., a[2]))))
            # hand turned by what brings the top's axis upright (the pinch carries the top; the point follows)
            r = np.cross(a, np.array([0., 0., 1.]))
            c['R'] = expm(.30 * r) @ c['R']
            c['ok'] = c['ok'] + 1 if tilt < .7 else 0
            c['n'] += 1
            if c['n'] % 4 == 0:
                self.log(f'PUMP_UPRIGHT t={t:.2f} tilt={tilt:.2f} offset_mm={np.round(c["xy"] * 1000, 1).tolist()}')
        pos = np.array([self.xy[0] + self.shift[0] + c['xy'][0], self.xy[1] + self.shift[1] + c['xy'][1], z])
        R = c['R'] @ self.Rc @ self.R0
        if c['ok'] >= 3 and c['n'] >= 6:
            self._release(round((t + DT) / DT) * DT)
        b = self.bias
        pos, R = self.lead.command((pos - b, R), (pos - b, R), (pos - b, R))
        return pos, R, self.pinch
