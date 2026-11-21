"""A kinematic stand-in for the simulator, for dry runs of the scripted demonstration through the real scorer.

Not physics. The arms follow their commands with a first-order lag per joint (shoulder and elbow 0.07 s, wrist
0.15 s, jaws 0.10 s: the lags seen in Isaac); the converter follows the rules its real geometry imposes:

* a hand is at its handle when its fingertip centre is within the paddle's reach of the grasp point; closing the
  jaws there lifts that side's paddle by (49 mm stack - jaw gap), up to its 16 mm travel, and the jaws then hold;
* a pin is out of its notch only when its paddle is up at least 14 mm (its 10 mm engagement plus clearance);
  a released pin drops into a notch only when the arm angle is within the slot's side clearance of that notch,
  otherwise it rides on the teeth and drops into the next notch it meets;
* while any pin is in a notch the top cannot move; with both out, the top goes where the holding hands carry it
  along its arc (the mean of the two hands if both hold); with both out and no hand holding, it sinks under the
  net weight;
* the monitor and keyboard ride on the top unchanged.
"""

import math

import numpy as np
from yam_arena import ik
from yam_arena.constants import READY

from rdb_desk_converter import layout as L
from rdb_desk_converter.fixed_scene import ARMS, BASES, CONTROL_HZ, START_THETA

DT = 1. / CONTROL_HZ
LAG = np.array((.07, .07, .07, .15, .15, .15, .10))
FULL_GAP = .0939
STACK = -L.PADDLE_TOP_REST + 2 * L.PADDLE_HALF[2]          # 49 mm: board + gap + paddle, released
PIN_OUT = L.PIN_R[1] - L.NOTCH_R[0]                        # 14 mm: the pin's bottom clears the teeth
SLOT_TOL = L.SEAT_SETTINGS                                 # settings: a pin drops in within the slot's play
NEAR = np.array((.020, .012, .008))                        # a fingertip centre this close (x, y, z) is at its handle
SINK = .05                                                 # rad/s the free, unheld top sinks or rises at


class KinematicWorld:
    def __init__(self):
        self.q = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]]).astype(float)
        self.theta = START_THETA
        self.latch = dict.fromkeys(ARMS, 0.)
        self.grip_offset = dict.fromkeys(ARMS, None)
        self.tick = 0

    def tcp(self, arm):
        i = 7 * ARMS.index(arm)
        return ik.tcp_pose(self.q[i:i + 6], BASES[arm])[0]

    def gap(self, arm):
        return float(self.q[7 * ARMS.index(arm) + 6]) * FULL_GAP

    def at_handle(self, arm):
        return bool(np.all(np.abs(self.tcp(arm) - L.grasp_point(arm, self.theta)) <= NEAR))

    def state(self):
        return {"theta": self.theta, "latch": dict(self.latch)}

    def touching(self):
        return any(self.at_handle(a) and self.gap(a) < .080 for a in ARMS)

    def step(self, command):
        command = np.asarray(command, dtype=float)
        alpha = 1. - np.exp(-DT / np.r_[LAG, LAG])
        self.q = self.q + alpha * (command - self.q)
        # jaws closed on a handle stop at the stack
        push = {}
        for a in ARMS:
            i = 7 * ARMS.index(a) + 6
            if self.at_handle(a):
                stack_min = (STACK - L.PADDLE_TRAVEL) / FULL_GAP
                self.q[i] = max(self.q[i], stack_min)
                push[a] = float(np.clip(STACK - self.gap(a), 0., L.PADDLE_TRAVEL))
            else:
                push[a] = 0.
        holding = {a: self.at_handle(a) and push[a] >= L.PADDLE_TRAVEL - 1e-6 for a in ARMS}
        setting = L.setting_of(self.theta)
        aligned = abs(setting - round(setting)) <= SLOT_TOL + 1e-6
        in_slot = {}
        for a in ARMS:
            was_in = self.latch[a] < PIN_OUT - 1e-9
            if push[a] >= PIN_OUT or was_in or aligned:
                self.latch[a] = push[a]                            # out, staying in its notch, or dropping in
            else:
                self.latch[a] = PIN_OUT                            # riding on the teeth
            in_slot[a] = self.latch[a] < PIN_OUT - 1e-9
        if any(in_slot.values()):
            if aligned:                                            # the loaded top rests on the pins' lower contact
                self.theta = L.seated(round(setting))
        else:
            wanted = []
            for a in ARMS:
                if holding[a]:
                    if self.grip_offset[a] is None:
                        self.grip_offset[a] = self.tcp(a) - L.grasp_point(a, self.theta)
                    wanted.append(self._angle_for(a, self.tcp(a) - self.grip_offset[a]))
            if wanted:
                self.theta = float(np.clip(np.mean(wanted), *L.LIFT_LIMITS))
            else:
                net = L.net_push(self.theta)
                if abs(net) > 1.:
                    self.theta -= math.copysign(SINK * DT, net)
            # a pin riding on the teeth drops into a notch it meets
            setting = L.setting_of(self.theta)
            if abs(setting - round(setting)) <= SLOT_TOL + 1e-6:
                for a in ARMS:
                    if push[a] < PIN_OUT:
                        self.latch[a] = push[a]
                if any(push[a] < PIN_OUT for a in ARMS):
                    self.theta = L.seated(round(setting))
        for a in ARMS:
            if not holding[a]:
                self.grip_offset[a] = None
        self.tick += 1

    def _angle_for(self, arm, point):
        """Arm angle whose grasp point is nearest ``point``."""
        grid = self.theta + np.radians(np.linspace(-3., 3., 241))
        d = [np.linalg.norm(L.grasp_point(arm, t) - point) for t in grid]
        return float(grid[int(np.argmin(d))])

    def poses(self):
        top = L.top_origin(self.theta)
        eye = np.eye(3)
        monitor = (top + (L.MONITOR_AT[0], L.MONITOR_AT[1], 0.), eye)
        keyboard = (top + (L.KEYBOARD_AT[0], L.KEYBOARD_AT[1], -L.TRAY_DROP), eye)
        return top, eye, monitor, keyboard
