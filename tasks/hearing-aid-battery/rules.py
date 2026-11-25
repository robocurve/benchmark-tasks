"""The scorer: a tracker fed once per control step with what the overhead video and a look at the door would show.
Pure NumPy, so it is tested offline (test_rules.py). task.json's `scoring`, line by line:

- 1.0, held 1 s: old cell in the cup; tab off the new cell and in the cup; new cell seated in the door's cradle, + side to
  the door's + mark (axis within 20 deg, centre within 1.5 mm); door clicked shut (under 2 deg and latched); the aid let go,
  every gripper part and the magnet at least 3 cm from the aid and its door.
- 0.5: the same, but the door shut to within 20 deg, not clicked.
- 0.25: old cell and tab in the cup, new cell seated + side up with its tab off, door open 20 deg or more.
- 0.1: the door has been opened (past 20 deg) and the old cell is in the cup.
- 0, always: a finger touched a cell; a cell was dropped (touched the mat or table outside the cup); the new cell upside
  down in the door; the new cell in the door with its tab on; the aid off the mat; the aid held or touched at the end
  (the hands-clear check is part of every tier above 0).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

MM = 0.001
SEAT_XY_TOL = 1.5 * MM
SEAT_Z_TOL = 1.0 * MM
TILT_TOL_DEG = 20.0
DOOR_SHUT_DEG = 2.0
DOOR_NEAR_DEG = 20.0
CLEAR = 0.030
TOUCH = 0.5 * MM  # a finger box this close to a cell's surface touches it
DROP_Z = 0.6 * MM  # a cell's lowest point this close to the mat's top lies on it
HOLD_S = 1.0
TAB_OFF = 1.5 * MM  # the tab moved this far from where its bond holds it: off


@dataclass(frozen=True)
class Geometry:
    z0: float  # the mat's top
    top: float  # the table's top
    mat_c: tuple
    mat_size: tuple
    cup_xy: tuple
    cup_r: float
    cup_h: float
    cradle_rel: tuple  # the cradle's axis point in the door's frame (x, y, seated z of a cell's origin)
    cell_r: float
    cell_body_h: float
    cap_h: float
    tab_rel: tuple  # the tab's origin in the new cell's frame while bonded
    aid_pts: np.ndarray  # points on the aid (aid frame) for the hands-clear check
    door_pts: np.ndarray  # points on the door (door frame)
    stick_pts: np.ndarray  # points along the magnet (its frame)


@dataclass
class Pose:
    p: np.ndarray
    R: np.ndarray

    def world(self, pts) -> np.ndarray:
        return np.asarray(pts, float) @ self.R.T + self.p

    def local(self, pts) -> np.ndarray:
        return (np.asarray(pts, float) - self.p) @ self.R


@dataclass
class Box:
    """A gripper box: its TCP frame (R, t) and bounds lo, hi in that frame."""

    R: np.ndarray
    t: np.ndarray
    lo: np.ndarray
    hi: np.ndarray
    finger: bool  # a finger link's box (tip_left / tip_right), else the gripper's body

    def distance(self, pts) -> np.ndarray:
        q = (np.asarray(pts, float) - self.t) @ self.R
        d = np.maximum(np.maximum(self.lo - q, q - self.hi), 0.0)
        return np.linalg.norm(d, axis=1)


@dataclass
class Snapshot:
    t: float
    pose: dict  # name -> Pose: aid, door, old_cell, new_cell, tab (or None), magnet
    door_deg: float
    latched: bool
    boxes: list  # Box, both grippers


def cell_points(g: Geometry, pose: Pose, n: int = 16) -> np.ndarray:
    """Points on a cell's surface: rims of both faces of the can, the face centres and the cap's face."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.c_[np.cos(a), np.sin(a), np.zeros(n)] * g.cell_r
    h = g.cell_body_h / 2
    pts = np.r_[ring + [0, 0, h], ring + [0, 0, -h], ring, [[0, 0, h], [0, 0, -h - g.cap_h]]]
    return pose.world(pts)


@dataclass
class Judge:
    g: Geometry
    touched: list = field(default_factory=list)
    dropped: list = field(default_factory=list)
    aid_off: bool = False
    opened: bool = False
    tab_off: bool = False
    clear_since: float | None = None
    max_door: float = 0.0
    last: dict = field(default_factory=dict)

    # -- per-part checks ---------------------------------------------------------------------------------------
    def in_cup(self, pose: Pose | None) -> bool:
        if pose is None:
            return False
        g = self.g
        e = np.hypot(pose.p[0] - g.cup_xy[0], pose.p[1] - g.cup_xy[1])
        return bool(e < g.cup_r - 1 * MM and g.z0 < pose.p[2] < g.z0 + g.cup_h)

    def in_cradle(self, s: Snapshot, name: str) -> tuple[bool, bool]:
        """(lies in the cradle's place, + side to the + mark) for a cell, in the door's frame."""
        g = self.g
        door, c = s.pose["door"], s.pose[name]
        q = door.local(c.p[None])[0] - np.asarray(g.cradle_rel)
        inside = bool(np.hypot(q[0], q[1]) < SEAT_XY_TOL and abs(q[2]) < SEAT_Z_TOL + g.cap_h)
        up = float(np.dot(door.R[:, 2], c.R[:, 2]))
        return inside, up > np.cos(np.radians(TILT_TOL_DEG))

    def seated_up(self, s: Snapshot) -> bool:
        g = self.g
        inside, up = self.in_cradle(s, "new_cell")
        q = s.pose["door"].local(s.pose["new_cell"].p[None])[0] - np.asarray(g.cradle_rel)
        return inside and up and abs(q[2]) < SEAT_Z_TOL

    def near_cradle(self, s: Snapshot) -> bool:
        door, c = s.pose["door"], s.pose["new_cell"]
        q = door.local(c.p[None])[0] - np.asarray(self.g.cradle_rel)
        return bool(np.hypot(q[0], q[1]) < self.g.cell_r and abs(q[2]) < 3 * MM)

    def clear(self, s: Snapshot) -> bool:
        g = self.g
        pts = np.r_[s.pose["aid"].world(g.aid_pts), s.pose["door"].world(g.door_pts)]
        for b in s.boxes:
            if np.min(b.distance(pts)) < CLEAR:
                return False
        stick = s.pose["magnet"].world(g.stick_pts)
        d = np.min(np.linalg.norm(stick[:, None, :] - pts[None, :, :], axis=2))
        return bool(d >= CLEAR)

    # -- the trial -------------------------------------------------------------------------------------------------
    def update(self, s: Snapshot) -> None:
        g = self.g
        self.max_door = max(self.max_door, s.door_deg)
        if s.door_deg > DOOR_NEAR_DEG:
            self.opened = True
        for name in ("old_cell", "new_cell"):
            pose = s.pose[name]
            pts = cell_points(g, pose)
            for b in s.boxes:
                if b.finger and name not in self.touched and np.min(b.distance(pts)) < TOUCH:
                    self.touched.append(name)
            low = float(pts[:, 2].min())
            if name not in self.dropped and not self.in_cup(pose):
                if low < g.z0 + DROP_Z or pose.p[2] < g.top:
                    self.dropped.append(name)
        a = s.pose["aid"].p
        if abs(a[0] - g.mat_c[0]) > g.mat_size[0] / 2 or abs(a[1] - g.mat_c[1]) > g.mat_size[1] / 2 or a[2] < g.z0 - 2 * MM:
            self.aid_off = True
        tab = s.pose.get("tab")
        if tab is not None and not self.tab_off:
            want = s.pose["new_cell"].world(np.asarray(g.tab_rel)[None])[0]
            if np.linalg.norm(tab.p - want) > TAB_OFF:
                self.tab_off = True
        if tab is None:
            self.tab_off = True
        if self.clear(s):
            if self.clear_since is None:
                self.clear_since = s.t
        else:
            self.clear_since = None

    def tier(self, s: Snapshot) -> tuple[float, str]:
        if self.touched:
            return 0.0, f"a finger touched {self.touched}"
        if self.dropped:
            return 0.0, f"dropped {self.dropped}"
        if self.aid_off:
            return 0.0, "the aid is off the mat"
        inside, up = self.in_cradle(s, "new_cell")
        near = self.near_cradle(s)
        if (inside or near) and not up:
            return 0.0, "the new cell is upside down in the door"
        if (inside or near) and not self.tab_off:
            return 0.0, "the new cell is in the door with its tab on"
        if self.clear_since is None:
            return 0.0, "the aid is held or touched"
        old = self.in_cup(s.pose["old_cell"])
        tab_cup = self.in_cup(s.pose.get("tab"))
        seated = self.seated_up(s)
        if old and tab_cup and self.tab_off and seated:
            if s.door_deg < DOOR_SHUT_DEG and s.latched:
                if s.t - self.clear_since >= HOLD_S - 1e-6:
                    return 1.0, "done"
                return 0.5, "done, not yet held 1 s"
            if s.door_deg < DOOR_NEAR_DEG:
                return 0.5, "door shut to within 20 deg but not clicked"
            return 0.25, "door left open"
        if self.opened and old:
            return 0.1, "door opened, old cell in the cup"
        return 0.0, "nothing scored"
