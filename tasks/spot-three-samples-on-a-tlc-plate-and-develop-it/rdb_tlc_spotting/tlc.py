"""The stated rules. No liquid, silica layer or glass fracture is simulated: the spotters, the plate, the tank
and its lid are rigid bodies moved only by contact, and these rules, fed every 20 Hz control step from
the bodies' poses and contact forces, decide what the liquid and the layer would do. Pure Python.

- Filling: a spotter takes up a sample when its tip has been at least ``DIP_MIN`` under that sample's surface,
  inside the vial's bore, for ``FILL_S``. A spotter wetted in a second sample is mixed, and so is that vial.
- Spotting: a spot forms only where a filled spotter's tip presses the silica face with a force between
  ``TOUCH_MIN_N`` and ``TOUCH_MAX_N`` for at least ``DWELL_S``; its diameter then grows with the time in contact,
  sqrt(D0^2 + growth x seconds). A touch within ``MERGE`` of an earlier spot of the same sample grows that spot.
- Gouging: pressing harder than ``GOUGE_N`` for ``GOUGE_S`` scrapes 1.5 mm of layer off; sliding the tip more than
  ``SLIDE_MAX`` while pressing scrapes a track that long.
- Breaking: more than ``BREAK_N`` on a spotter from anything but the grippers' pads breaks it.
"""

import math

from . import layout as L


# -- poses --------------------------------------------------------------------------------------------------------
def quat_matrix(q):
    """Rows of the rotation matrix of a unit quaternion (x, y, z, w)."""
    x, y, z, w = (float(v) for v in q)
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def to_world(pose, point):
    position, q = pose
    m = quat_matrix(q)
    return tuple(float(position[i]) + sum(m[i][j] * point[j] for j in range(3)) for i in range(3))


def to_local(pose, point):
    position, q = pose
    m = quat_matrix(q)
    d = [float(point[i]) - float(position[i]) for i in range(3)]
    return tuple(sum(m[j][i] * d[j] for j in range(3)) for i in range(3))


def axis(pose, i):
    m = quat_matrix(pose[1])
    return (m[0][i], m[1][i], m[2][i])


def tilt_deg(pose, i=2):
    """Angle of the body's axis ``i`` from straight up."""
    return math.degrees(math.acos(max(-1., min(1., axis(pose, i)[2]))))


def spotter_tip(pose):
    """A spotter's lower end (its tip) on the table: its frame's -z end."""
    return to_world(pose, (0., 0., -L.CAP_LEN / 2))


def plate_corners(pose):
    return [to_world(pose, (sx * L.PLATE_LEN / 2, sy * L.PLATE_WIDE / 2, sz * L.PLATE_T / 2))
            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


# -- filling ------------------------------------------------------------------------------------------------------
class Loads:
    """What each spotter holds."""

    def __init__(self, hz=L.CONTROL_HZ):
        self.need = max(1, round(L.FILL_S * hz))
        self.under = [[0, 0, 0] for _ in range(3)]       # consecutive steps spotter k's tip is under sample j
        self.samples = [set() for _ in range(3)]         # samples spotter k has taken up
        self.wet = [set() for _ in range(3)]             # vials spotter k's tip has touched the liquid of
        self.mixed_vials = set()                         # vials a spotter already holding another sample wetted

    @staticmethod
    def in_bore(tip, j):
        vx, vy, _ = L.vial_point(j)
        return math.hypot(tip[0] - vx, tip[1] - vy) < L.VIAL_ID / 2 and tip[2] < L.VIAL_H

    def update(self, tips):
        for k, tip in enumerate(tips):
            for j in range(3):
                inside = self.in_bore(tip, j)
                if inside and tip[2] <= L.LIQUID_Z:
                    if self.samples[k] - {j}:
                        self.mixed_vials.add(j)
                    self.wet[k].add(j)
                under = inside and tip[2] <= L.LIQUID_Z - L.DIP_MIN
                self.under[k][j] = self.under[k][j] + 1 if under else 0
                if self.under[k][j] >= self.need:
                    self.samples[k].add(j)

    def sample(self, k):
        """Spotter k's sample (0, 1, 2), "mixed", or None while empty."""
        held = self.samples[k]
        if not held:
            return None
        if len(held) > 1 or len(self.wet[k]) > 1 or held & self.mixed_vials:
            return "mixed"
        return next(iter(held))


# -- spots and gouges ----------------------------------------------------------------------------------------------
class Spots:
    """Spots and gouges on the silica face, in the plate's frame."""

    def __init__(self, hz=L.CONTROL_HZ):
        self.dt = 1. / hz
        self.dwell = max(1, round(L.DWELL_S * hz))
        self.hard_steps = max(1, round(L.GOUGE_S * hz))
        self.spots = []                                  # {"x", "y", "sample", "seconds", "spotter"}
        self.gouges = []                                 # {"x", "y", "length"}
        self.touch = [None, None, None]
        self.touches = 0                                 # every touch (forming a spot or not)

    @staticmethod
    def on_face(tip):
        return abs(tip[0]) <= L.PLATE_LEN / 2 and abs(tip[1]) <= L.PLATE_WIDE / 2 and tip[2] > -.0005

    def update(self, k, force, tip, sample):
        """Spotter k pressing the silica face with ``force`` (N) this step, its tip at ``tip`` (plate frame)."""
        if force < L.TOUCH_MIN_N or not self.on_face(tip):
            self.touch[k] = None
            return
        touch = self.touch[k]
        if touch is None:
            touch = self.touch[k] = {"x": tip[0], "y": tip[1], "steps": 0, "spot": None, "pressed": False,
                                     "track": None, "hard": 0}
            self.touches += 1
        touch["steps"] += 1
        slide = math.hypot(tip[0] - touch["x"], tip[1] - touch["y"])
        touch["hard"] = touch["hard"] + 1 if force > L.GOUGE_N else 0
        if touch["hard"] >= self.hard_steps and not touch["pressed"]:
            touch["pressed"] = True
            self.gouges.append({"x": tip[0], "y": tip[1], "length": L.GOUGE_LEN, "why": "pressed"})
        if slide > L.SLIDE_MAX:
            if touch["track"] is None:
                touch["track"] = {"x": (tip[0] + touch["x"]) / 2, "y": (tip[1] + touch["y"]) / 2, "length": slide,
                                  "why": "slid"}
                self.gouges.append(touch["track"])
            else:
                touch["track"].update(length=max(touch["track"]["length"], slide))
        if force > L.TOUCH_MAX_N or sample is None or touch["steps"] < self.dwell:
            return
        if touch["spot"] is None:
            near = [s for s in self.spots if s["sample"] == sample
                    and math.hypot(s["x"] - touch["x"], s["y"] - touch["y"]) <= L.MERGE]
            if near:
                touch["spot"] = near[0]
                near[0]["seconds"] += touch["steps"] * self.dt
            else:
                touch["spot"] = {"x": touch["x"], "y": touch["y"], "sample": sample,
                                 "seconds": touch["steps"] * self.dt, "spotter": k}
                self.spots.append(touch["spot"])
        else:
            touch["spot"]["seconds"] += self.dt

    @staticmethod
    def diameter(spot):
        return L.spot_diameter(spot["seconds"])

    def lane(self, k):
        """Spot k's three checks: (a) exactly one spot within LANE_HALF of tick k, of sample k, covering the tick;
        (b) it fits the printed 3 mm circle; (c) no bare glass longer than 1 mm within LANE_HALF of the tick.
        (b) and (c) count only with (a)."""
        tx, ty, _ = L.tick_point(k)
        here = [s for s in self.spots if math.hypot(s["x"] - tx, s["y"] - ty) <= L.LANE_HALF]
        covered = (len(here) == 1 and here[0]["sample"] == k
                   and math.hypot(here[0]["x"] - tx, here[0]["y"] - ty) <= self.diameter(here[0]) / 2)
        small = covered and self.diameter(here[0]) <= L.SPOT_MAX_D
        bare = [g for g in self.gouges if g["length"] > L.BARE_MAX
                and math.hypot(g["x"] - tx, g["y"] - ty) <= L.LANE_HALF + g["length"] / 2]
        clean = covered and not bare
        return {"covered": bool(covered), "small": bool(small), "clean": bool(clean), "spots": len(here),
                "diameter_mm": round(1000 * self.diameter(here[0]), 2) if len(here) == 1 else None,
                "offset_mm": round(1000 * math.hypot(here[0]["x"] - tx, here[0]["y"] - ty), 2) if len(here) == 1 else None,
                "sample": here[0]["sample"] if len(here) == 1 else None, "gouged": bool(bare)}


# -- the plate in the tank, the lid ------------------------------------------------------------------------------
def yaw_deg(pose):
    """Turn of a body's x axis about the vertical."""
    a = axis(pose, 0)
    return math.degrees(math.atan2(a[1], a[0]))


def plate_in_tank(plate, tank):
    """The plate's state in the tank's frame (origin on the table under the tank's middle, x toward its back wall)."""
    ix, iy = L.TANK_IN

    def local(point):
        return to_local(tank, point)

    corners = [local(c) for c in plate_corners(plate)]
    inside = (max(abs(c[0]) for c in corners) <= ix / 2 + .002 and max(abs(c[1]) for c in corners) <= iy / 2 + .002
              and min(c[2] for c in corners) >= L.TANK_FLOOR - .002)
    bottom = [local(to_world(plate, (-L.PLATE_LEN / 2, sy * L.PLATE_WIDE / 2, 0.))) for sy in (-1, 1)]
    top = [local(to_world(plate, (L.PLATE_LEN / 2, sy * L.PLATE_WIDE / 2, sz * L.PLATE_T / 2)))
           for sy in (-1, 1) for sz in (-1, 1)]
    baseline = [local(to_world(plate, (-L.PLATE_LEN / 2 + L.BASELINE_UP, sy * L.PLATE_WIDE / 2, L.PLATE_T / 2)))
                for sy in (-1, 1)]
    m = quat_matrix(tank[1])
    up_w, n_w = axis(plate, 0), axis(plate, 2)
    up = tuple(sum(m[j][i] * up_w[j] for j in range(3)) for i in range(3))
    normal = tuple(sum(m[j][i] * n_w[j] for j in range(3)) for i in range(3))
    lean = math.degrees(math.acos(max(-1., min(1., up[2]))))
    return {"inside": bool(inside),
            "bottom_wet": bool(inside and max(p[2] for p in bottom) < L.SOLVENT_Z),
            "baseline_above_mm": round(1000 * (min(p[2] for p in baseline) - L.SOLVENT_Z), 2),
            "lean_deg": round(lean, 2),
            "face_out": bool(normal[0] < 0.),                       # spotted face away from the back wall
            "on_wall": bool(max(p[0] for p in top) >= ix / 2 - .0025),
            "right_way_up": bool(up[2] > 0. and min(p[2] for p in top) > max(p[2] for p in bottom))}


def lid_on(lid, tank):
    rel = to_local(tank, lid[0])
    off = math.hypot(rel[0], rel[1])
    yaw = (yaw_deg(lid) - yaw_deg(tank) + 90.) % 180. - 90.
    rest = L.TANK[2] + L.LID[2] / 2
    return {"off_centre_mm": round(1000 * off, 2), "tilt_deg": round(tilt_deg(lid), 2), "yaw_deg": round(yaw, 2),
            "height_error_mm": round(1000 * (rel[2] - rest), 2),
            "on": bool(off <= L.LID_CENTRE and tilt_deg(lid) <= L.LID_TILT_DEG and abs(yaw) <= L.LID_YAW_DEG
                       and abs(rel[2] - rest) <= .003)}


def in_rack(pose):
    """A spotter standing in any rack hole: its tip within 3 mm of the hole's axis and no more than 4 mm above its
    floor, the spotter within 12 degrees of upright. Returns the hole's index or None."""
    tip = spotter_tip(pose)
    for k in range(3):
        hx, hy, hz = L.hole_point(k)
        if math.hypot(tip[0] - hx, tip[1] - hy) <= L.HOLE / 2 + .0005 and tip[2] <= hz + .004 and tilt_deg(pose) <= 12.:
            return k
    return None
