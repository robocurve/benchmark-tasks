"""Kinematics of the bar and the key for use ONLY where there is no simulator: tools/offline/check_task.py (whose own
carry proxy cannot slide a bar through a bore, turn a key or tighten a screw) and the tests. The simulator never takes
a result from here: there the bar and the key are free bodies, the screws a rule (setscrew.py, run by task.py) and the
clamp a weld.

What this file assumes is what the design claims, so the offline prediction is the design's, not a measurement. It
moves the parts from the arms' COMMANDED pad-tip points and openings (rigstate's offline feed), and the key's turn from
the right arm's commanded wrist yaw (read from a shadow of the demo plan, because rigstate carries no wrist angle):

- a gripper holds the bar once its pad tips have straddled the bar open (within 6 mm of its axis, over its length,
  within 8 mm under to 1 mm over its equator) and then close below the 12 mm shank; it lets go once open wider than
  13.5 mm. Held, the bar moves with the gripper (its heading kept);
- a gripper holds the key once its pad tips have straddled the crossbar open (within 5 mm of the shaft, on the
  crossbar) and then close below 8 mm; it lets go above 9.5 mm. Held, the key moves with the gripper and turns with
  its wrist yaw about the vertical through the gripper's axis;
- a closed gripper (pads one block, opening under 4 mm) behind the bar's cutting end, its pad tips within the end
  face's height and width, pushes the bar along its axis: the bar's cutting end never stays past the block's face;
- a released bar not clamped drops onto the rack or the holder (axis at the lathe's height) or the bench; a released
  key drops into the stand's hole if its tip is within 2 mm of it, else lies on the bench;
- the screw model (setscrew.ScrewModel) then runs on the result; its weld freezes the bar where it is.
"""

from __future__ import annotations

import math

import numpy as np

import layout as L
import rigstate
import setscrew

BAR_W, KEY_W = 12.0 * L.MM, 8.0 * L.MM
GRASP_BAR, RELEASE_BAR = BAR_W - 0.0001, BAR_W + 0.0015
PUSH_FACE = 0.004  # a closed block's face on the bar's end from the TCP along y (measured on the GPU)
PUSH_GAP = 4.0 * L.MM  # "closed" for a pusher
GRASP_KEY, RELEASE_KEY = KEY_W - 0.0001, KEY_W + 0.0015


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class OfflineBoring:
    def __init__(self, bar_spec, key_spec, shadow=None, key_error=None) -> None:
        self.bar_spec, self.key_spec = bar_spec, key_spec
        self.key_error = key_error  # callable(tcp) -> (dx, dy) metres: where the GPU arm lands off its command (the demo's trims undo it)
        self.shadow = shadow  # callable(side, t) -> 3x3 TCP rotation or None
        self.model = setscrew.ScrewModel()
        self.reset()

    def reset(self) -> None:
        import mechkit as mk

        self.model.reset()
        bo, bq = self.bar_spec.spawn_pose()
        ko, kq = self.key_spec.spawn_pose()
        self.bar_o, self.bar_r = np.array(bo, dtype=float), mk.quat_to_matrix(bq)
        self.key_o, self.key_r = np.array(ko, dtype=float), mk.quat_to_matrix(kq)
        self.held = {"bar": None, "key": None}
        self.rel = {}
        self.yaw0 = {}
        self.around = {(o, s): False for o in ("bar", "key") for s in rigstate.ARMS}
        self.welded = False
        self.events: list = []
        self.t = 0.0

    # -- helpers ----------------------------------------------------------------------------------------------------
    def _near_bar(self, tips) -> bool:
        p = self.bar_r.T @ (np.asarray(tips) - self.bar_o)
        dz = tips[2] - self.bar_o[2]
        return (math.hypot(p[1], 0.0) <= 0.006 and abs(p[0]) <= 0.070 and -0.008 <= dz <= 0.001
                and abs(tips[0] - (self.bar_o + self.bar_r[:, 0] * p[0])[0]) <= 0.006)

    def _near_key(self, tips) -> bool:
        d = math.hypot(tips[0] - self.key_o[0], tips[1] - self.key_o[1])
        dz = tips[2] - self.key_o[2]
        return d <= 0.005 and (L.KEY_BAR_Z - 8.0) * L.MM <= dz <= (L.KEY_BAR_Z + 1.0) * L.MM

    def _yaw(self, side: str) -> float | None:
        if self.shadow is None:
            return None
        r = self.shadow(side, self.t)
        return None if r is None else math.atan2(float(r[1, 0]), float(r[0, 0]))

    def _drop_bar(self) -> None:
        y = self.bar_o[1]
        over_rack = L.RACK_Y[0] * L.MM <= y <= L.RACK_Y[1] * L.MM
        over_holder = self.bar_o[1] - 0.075 < L.Y_FACE * L.MM and self.bar_o[1] + 0.075 > L.Y_BACK * L.MM
        z = L.TOP + (L.Z_AX if (over_rack or over_holder) else L.BAR_R) * L.MM
        self.bar_o = np.array([self.bar_o[0], self.bar_o[1], z])

    def _drop_key(self) -> None:
        d = math.hypot(self.key_o[0] - L.KEY_XY[0] * L.MM, self.key_o[1] - L.KEY_XY[1] * L.MM)
        if d <= 0.002:
            self.key_o = np.array([L.KEY_XY[0] * L.MM, L.KEY_XY[1] * L.MM, L.TOP + L.STAND_FLOOR * L.MM])
            self.key_r = np.eye(3)
        else:
            self.key_r = ry(math.pi / 2) @ rz(math.atan2(self.key_r[1, 0], self.key_r[0, 0]))
            self.key_o = np.array([self.key_o[0], self.key_o[1], L.TOP + L.KEY_BAR_R * L.MM])

    def _push(self, tips) -> None:
        """A closed pad block behind the cutting end (heading +y) moves the bar along y, never pulls it."""
        ax = self.bar_r[:, 0]
        if ax[1] < 0.9:
            return
        tail = self.bar_o[1] + 0.075 * ax[1]
        face = tips[1] - PUSH_FACE + L.GPU_PUSH_LAG * L.MM
        dz = tips[2] - self.bar_o[2]
        if (abs(tips[0] - self.bar_o[0]) <= 0.0135 + 0.004 and -0.005 <= dz <= 0.0055
                and face < tail):
            self.bar_o = np.array([self.bar_o[0], self.bar_o[1] - (tail - face), self.bar_o[2]])

    # -- one control step -------------------------------------------------------------------------------------------
    def step(self, arms, t: float) -> None:
        self.t = t
        read = rigstate.read_arms(arms)
        if read is None:
            return
        grips = {}
        for side, (tcp, tips, opening) in zip(rigstate.ARMS, read):
            grips[side] = (np.asarray(tcp, dtype=float), np.asarray(tips, dtype=float),
                           float(np.clip(opening, 0.0, 1.0)) * rigstate.FULL_GAP)
        for name, near_fn, grasp, release in (("bar", self._near_bar, GRASP_BAR, RELEASE_BAR),
                                              ("key", self._near_key, GRASP_KEY, RELEASE_KEY)):
            for side, (tcp, tips, gap) in grips.items():
                if self.held[name] == side:
                    if gap > release:
                        self.held[name] = None
                        self.events.append(("release", name, side))
                        if name == "key":
                            self._drop_key()
                        elif not self.welded:
                            self._drop_bar()
                    continue
                near = near_fn(tips)
                if (near and self.around[(name, side)] and gap < grasp and self.held[name] is None
                        and not (name == "bar" and self.welded)):
                    self.held[name] = side
                    o = self.bar_o if name == "bar" else self.key_o
                    self.rel[name] = o - tcp
                    self.yaw0[name] = self._yaw(side)
                    if name == "key":
                        self.key_r0 = self.key_r.copy()
                    self.events.append(("grasp", name, side))
                if not near:
                    self.around[(name, side)] = False
                elif gap > release + 0.002:
                    self.around[(name, side)] = True
        if self.held["bar"] is None and not self.welded:
            for side, (tcp, tips, gap) in grips.items():
                if gap < PUSH_GAP:
                    self._push(tips)
        for name in ("bar", "key"):
            side = self.held[name]
            if side is None or (name == "bar" and self.welded):
                continue
            tcp = grips[side][0]
            rel = self.rel[name]
            if name == "key":
                y1, y0 = self._yaw(side), self.yaw0.get(name)
                turn = 0.0 if (y1 is None or y0 is None) else y1 - y0
                self.key_o = tcp + rz(turn) @ rel
                if self.key_error is not None:
                    ex, ey = self.key_error(tcp)
                    self.key_o = self.key_o + np.array([ex, ey, 0.0])
                self.key_r = rz(turn) @ self.key_r0
            else:
                self.bar_o = tcp + rel
        out = self.model.update(self.key_o, self.key_r, self.bar_o, self.bar_r)
        if out.fire:
            self.welded = True
            self.events.append(("weld", out.record))
        if out.release:
            self.welded = False
            self.events.append(("unweld",))

    # -- the states the scorer sees ---------------------------------------------------------------------------------
    def states(self, objects: dict, bar_name: str, key_name: str) -> dict:
        import mechkit as mk

        out = dict(objects)
        out[bar_name] = mk.state_from_poses(self.bar_spec, (*self.bar_o, *mk.matrix_to_quat(self.bar_r)), np.zeros(6),
                                            source="offline")
        out[key_name] = mk.state_from_poses(self.key_spec, (*self.key_o, *mk.matrix_to_quat(self.key_r)), np.zeros(6),
                                            source="offline")
        return out
