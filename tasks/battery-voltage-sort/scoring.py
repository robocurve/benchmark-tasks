"""Electrical contact, measurement history and final-state sorting score. No simulator imports."""

from dataclasses import dataclass
import math

import numpy as np

import spec as S


@dataclass
class Body:
    pos: np.ndarray
    quat_xyzw: np.ndarray
    lin_vel: np.ndarray
    ang_vel: np.ndarray
    low: np.ndarray
    high: np.ndarray

    @property
    def rotation(self):
        return S.rotation(self.quat_xyzw)


def tip_position(probe):
    return probe.pos + probe.rotation @ S.TIP_LOCAL


def terminal_contact(probe, cell, positive, force):
    """A sphere tip touches the exposed end face, with a reported physical contact."""
    local = cell.rotation.T @ (tip_position(probe) - cell.pos)
    sign = 1 if positive else -1
    gap = sign * (local[1] - sign * S.CELL_HALF)
    radius = S.POSITIVE_RADIUS if positive else S.NEGATIVE_RADIUS
    return bool(
        math.isfinite(float(force)) and force >= S.MIN_CONTACT_FORCE
        and S.TIP_RADIUS-S.CONTACT_TOLERANCE <= gap <= S.TIP_RADIUS + S.CONTACT_TOLERANCE
        and np.linalg.norm(local[[0, 2]]) <= radius + S.TIP_RADIUS
    )


def envelope_distance(a_low, a_high, b_low, b_high):
    return float(np.linalg.norm(np.maximum(np.maximum(a_low-b_high, b_low-a_high), 0)))


class Trial:
    def __init__(self, voltages_cv=S.VOLTAGES_CV, threshold_cv=S.THRESHOLD_CV):
        self.voltages = dict(zip(S.CELL_NAMES, voltages_cv, strict=True))
        self.threshold = int(threshold_cv)
        self.measured = {}
        self.events = []
        self.reading_cv = None
        self.connected_cell = None
        self.contact_time = 0.0
        self.invalid_reason = None
        self.time = 0.0
        self.completed_hold = 0.0
        self.score = 0.0
        self.details = {}

    def _connection(self, bodies, forces):
        red, black = (bodies[name] for name in S.PROBE_NAMES)
        contacts = []
        for name in S.CELL_NAMES:
            cell = bodies[name]
            rf = forces.get("red_probe", {}).get(name, 0.0)
            bf = forces.get("black_probe", {}).get(name, 0.0)
            if terminal_contact(red, cell, True, rf) and terminal_contact(black, cell, False, bf):
                contacts.append((name, self.voltages[name]))
            elif terminal_contact(red, cell, False, rf) and terminal_contact(black, cell, True, bf):
                contacts.append((name, -self.voltages[name]))
        return contacts[0] if len(contacts) == 1 else (None, None)

    def update(self, dt, bodies, forces, robot_boxes=()):
        if dt <= 0 or not math.isfinite(dt):
            raise ValueError("The scoring time step must be positive and finite")
        self.time += dt
        for name in (*S.CELL_NAMES, *S.PROBE_NAMES):
            body = bodies[name]
            arrays = (body.pos, body.quat_xyzw, body.low, body.high, body.lin_vel, body.ang_vel)
            if any(not np.isfinite(array).all() for array in arrays):
                self.invalid_reason = "nonfinite object state"
                break
            if np.any(body.low < S.TABLE_LOW) or np.any(body.high > S.TABLE_HIGH):
                self.invalid_reason = f"{name} left the table workspace"

        name, reading = self._connection(bodies, forces) if not self.invalid_reason else (None, None)
        self.contact_time = self.contact_time + dt if name is not None and name == self.connected_cell else (dt if name else 0.0)
        self.connected_cell, self.reading_cv = name, reading
        if name is not None and self.contact_time + 1e-8 >= S.CONTACT_HOLD and name not in self.measured:
            self.measured[name] = abs(reading)
            self.events.append({"time_s": round(self.time, 4), "battery": name, "reading_v": reading / 100})

        details = {}
        for name in S.CELL_NAMES:
            body = bodies[name]
            target = "alive" if self.voltages[name] >= self.threshold else "dead"
            center = S.BIN_CENTERS[target]
            inside = bool(
                np.all(body.low[:2] >= center[:2] - S.BIN_INNER_HALF-S.BIN_TOLERANCE)
                and np.all(body.high[:2] <= center[:2] + S.BIN_INNER_HALF+S.BIN_TOLERANCE)
                and body.low[2] >= S.BIN_FLOOR - S.BIN_TOLERANCE
                and body.high[2] <= S.BIN_RIM + S.BIN_TOLERANCE
            )
            still = bool(np.linalg.norm(body.lin_vel) <= S.STILL_SPEED and np.linalg.norm(body.ang_vel) <= S.STILL_SPIN)
            clearance = min((envelope_distance(body.low, body.high, lo, hi) for lo, hi in robot_boxes), default=float("inf"))
            released = clearance >= S.ROBOT_CLEARANCE
            credited = name in self.measured and inside and still and released
            details[name] = {"measured": name in self.measured, "target": target, "inside": inside,
                             "still": still, "released": released, "credited": bool(credited)}
        self.details = details
        self.score = 0.0 if self.invalid_reason else sum(d["credited"] for d in details.values()) / len(S.CELL_NAMES)
        self.completed_hold = self.completed_hold + dt if self.score == 1.0 else 0.0
        return self.score

    @property
    def success(self):
        return self.invalid_reason is None and self.completed_hold + 1e-8 >= S.SUCCESS_HOLD

    def report(self):
        return {"time_s": round(self.time, 4), "score": self.score, "success": self.success,
                "display_v": None if self.reading_cv is None else self.reading_cv / 100,
                "measured": {name: value/100 for name, value in self.measured.items()},
                "invalid_reason": self.invalid_reason, "batteries": self.details, "measurements": self.events}
        
