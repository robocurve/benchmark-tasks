"""Pose-based scoring for a two-brick joining trial, independent of the simulator.

Dimensions are metres, quaternions use x/y/z/w, and rotations map local to world.
Roots are body centres. Studs protrude along each brick's local +Z axis. A seated
joint has the upper body's underside at the lower body's top surface. Seating
is reconstructed from measured poses. Retention is demonstrated by an actual
unsupported lift, not an attachment flag or simulator-created constraint.

Finger boxes are conservative collision envelopes, not TCP point estimates.
Missing envelopes fail closed for the unsupported-lift and final-release checks.
The runner must include both grippers and any other hand/palm part that could
support the lower brick. This scorer does not infer contact forces from video.
"""

from __future__ import annotations

from itertools import product
import math
from typing import Any

import numpy as np


TABLE = 0.7475
EPS = 1e-9
ROBOT_COLLISION_LINKS = frozenset(('base', 'gripper', 'tip_left', 'tip_right',
                                 'link1', 'link2', 'link3', 'link4', 'link5'))
DEFAULTS = {
    "pitch": 0.016,
    "body_height": 0.0192,
    "stud_height": 0.0048,
    "stud_radius": 0.0048,
    "body_clearance": 0.0002,
    "lateral_tolerance": 0.001,
    "seat_tolerance": 0.00075,
    "tilt_tolerance_deg": 3.0,
    "yaw_tolerance_deg": 3.0,
    "max_sample_gap": 0.075,
}


def _vec(value: Any, size: int) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f"expected {size} finite numbers")
    return array


def quaternion_matrix(quat_xyzw: Any) -> np.ndarray:
    q = _vec(quat_xyzw, 4)
    length = float(np.linalg.norm(q))
    if not math.isfinite(length) or length < 1e-12:
        raise ValueError("invalid quaternion")
    x, y, z, w = q / length
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def rotation_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Full orientation difference in radians (including yaw)."""
    return math.acos(float(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1)))


def _rotation(value: Any) -> np.ndarray:
    r = np.asarray(value, dtype=float)
    if r.shape != (3, 3) or not np.isfinite(r).all():
        raise ValueError("invalid box rotation")
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or abs(np.linalg.det(r)-1) > 1e-5:
        raise ValueError("box rotation must be orthonormal and right-handed")
    return r


def _config(config: dict) -> dict:
    cfg = {**DEFAULTS, **config}
    for key in ("lower_shape", "upper_shape"):
        shape = tuple(cfg[key])
        if len(shape) != 2 or any(isinstance(v, bool) or int(v) != v or v < 1 for v in shape):
            raise ValueError(f"invalid {key}")
        cfg[key] = tuple(int(v) for v in shape)
    required = cfg["required_studs"]
    if isinstance(required, bool) or int(required) != required or required < 1:
        raise ValueError("required_studs must be a positive integer")
    if required > min(math.prod(cfg["lower_shape"]), math.prod(cfg["upper_shape"])):
        raise ValueError("required_studs exceeds available studs/sockets")
    cfg["required_studs"] = int(required)
    for key, default in DEFAULTS.items():
        value = float(cfg[key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"invalid {key}")
        cfg[key] = value
    return cfg


def stud_centres(shape: tuple[int, int], pitch: float = 0.016) -> np.ndarray:
    return np.array([[(x-(shape[0]-1)/2)*pitch, (y-(shape[1]-1)/2)*pitch]
                     for x in range(shape[0]) for y in range(shape[1])], dtype=float)


def _pose(body: dict) -> tuple[np.ndarray, np.ndarray]:
    return _vec(body["pos"], 3), quaternion_matrix(body["quat_xyzw"])


def body_box(body: dict, shape: tuple[int, int], config: dict) -> dict:
    cfg = {**DEFAULTS, **config}
    p, r = _pose(body)
    half = np.array([(shape[0]*cfg["pitch"]-cfg["body_clearance"])/2,
                     (shape[1]*cfg["pitch"]-cfg["body_clearance"])/2,
                     cfg["body_height"]/2])
    return {"center": p, "rotation": r, "half_size": half}


def brick_envelope(body: dict, shape: tuple[int, int], config: dict) -> list[dict]:
    """Body OBB plus one conservative box around each cylindrical stud."""
    cfg = {**DEFAULTS, **config}
    box = body_box(body, shape, cfg)
    boxes = [box]
    for xy in stud_centres(shape, cfg["pitch"]):
        local = np.array([*xy, (cfg["body_height"]+cfg["stud_height"])/2])
        boxes.append({"center": box["center"]+box["rotation"] @ local,
                      "rotation": box["rotation"],
                      "half_size": np.array([cfg["stud_radius"], cfg["stud_radius"], cfg["stud_height"]/2])})
    return boxes


def _box(box: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    p = _vec(box["center"], 3)
    r = _rotation(box["rotation"])
    h = _vec(box["half_size"], 3)
    if np.any(h <= 0):
        raise ValueError("box half sizes must be positive")
    return p, r, h


_SIGNS = np.array(list(product((-1., 1.), repeat=3)))
_EDGES = [(i, j) for i in range(8) for j in range(i+1, 8)
          if np.count_nonzero(_SIGNS[i] != _SIGNS[j]) == 1]


def box_vertices(box: dict) -> np.ndarray:
    p, r, h = _box(box)
    return p + (_SIGNS*h) @ r.T


def _segment_distance(p0: np.ndarray, p1: np.ndarray, q0: np.ndarray, q1: np.ndarray) -> float:
    """Shortest distance between two finite line segments, including parallels."""
    u, v, w = p1-p0, q1-q0, p0-q0
    a, b, c, d, e = (float(u@u), float(u@v), float(v@v), float(u@w), float(v@w))
    denom = a*c-b*b
    s = float(np.clip((b*e-c*d)/denom, 0., 1.)) if denom > 1e-20 else 0.
    t = (b*s+e)/c
    if t < 0:
        t, s = 0., float(np.clip(-d/a, 0., 1.))
    elif t > 1:
        t, s = 1., float(np.clip((b-d)/a, 0., 1.))
    return float(np.linalg.norm(w+s*u-t*v))


def obb_clearance(first: dict, second: dict) -> float:
    """Exact distance between closed boxes; zero for touching or overlapping.

    Uses separating axes to detect overlap, then vertex/face and edge/edge
    candidates for the shortest Euclidean distance. This is conservative with
    respect to the cylindrical studs because their envelope is a larger box.
    """
    ap, ar, ah = _box(first)
    bp, br, bh = _box(second)
    axes = [*ar.T, *br.T, *(np.cross(a, b) for a in ar.T for b in br.T)]
    separated = False
    for axis in axes:
        norm = float(np.linalg.norm(axis))
        if norm < 1e-10:
            continue
        axis = axis/norm
        gap = abs(float((bp-ap) @ axis)) - float(ah @ np.abs(ar.T @ axis)) - float(bh @ np.abs(br.T @ axis))
        if gap > 1e-12:
            separated = True
            break
    if not separated:
        return 0.
    av = ap+(_SIGNS*ah)@ar.T
    bv = bp+(_SIGNS*bh)@br.T
    local_a = (av-bp)@br
    local_b = (bv-ap)@ar
    da = np.linalg.norm(local_a-np.clip(local_a, -bh, bh), axis=1)
    db = np.linalg.norm(local_b-np.clip(local_b, -ah, ah), axis=1)
    best = min(float(da.min()), float(db.min()))
    # Evaluate all 144 edge pairs in one array operation: this runs at every
    # recorded control step, so Python loops over pairs are unnecessarily slow.
    indices = np.asarray(_EDGES)
    p0, p1 = av[indices[:, 0]], av[indices[:, 1]]
    q0, q1 = bv[indices[:, 0]], bv[indices[:, 1]]
    u, v, w = (p1-p0)[:, None, :], (q1-q0)[None, :, :], p0[:, None, :]-q0[None, :, :]
    a, b, c = np.sum(u*u, axis=-1), np.sum(u*v, axis=-1), np.sum(v*v, axis=-1)
    d, e = np.sum(u*w, axis=-1), np.sum(v*w, axis=-1)
    denominator = a*c-b*b
    s = np.clip(np.divide(b*e-c*d, denominator, out=np.zeros_like(denominator), where=denominator > 1e-20), 0., 1.)
    t = (b*s+e)/c
    s = np.where(t < 0, np.clip(-d/a, 0., 1.), np.where(t > 1, np.clip((b-d)/a, 0., 1.), s))
    t = np.clip(t, 0., 1.)
    distances = np.linalg.norm(w+s[..., None]*u-t[..., None]*v, axis=-1)
    return min(best, float(distances.min()))


def minimum_clearance(boxes_a: list[dict], boxes_b: list[dict]) -> float:
    if not boxes_a or not boxes_b:
        return 0.  # Missing collision evidence must not manufacture clearance.
    # Axis-aligned bounding boxes contain each oriented box. Their separation
    # is therefore a lower bound on the exact distance. Examine closest bounds
    # first; omit a pair only when its lower bound cannot improve the best
    # exact distance already found. This preserves the exact result while
    # avoiding expensive edge comparisons for remote robot links.
    parsed_a, parsed_b = [_box(b) for b in boxes_a], [_box(b) for b in boxes_b]
    centres_a, centres_b = np.array([p for p, _, _ in parsed_a]), np.array([p for p, _, _ in parsed_b])
    extents_a = np.array([np.abs(r) @ h for _, r, h in parsed_a])
    extents_b = np.array([np.abs(r) @ h for _, r, h in parsed_b])
    gaps = np.maximum(np.abs(centres_a[:, None, :]-centres_b[None, :, :])
                      -extents_a[:, None, :]-extents_b[None, :, :], 0.)
    lower_bounds = np.linalg.norm(gaps, axis=-1)
    best = math.inf
    for flat in np.argsort(lower_bounds, axis=None):
        i, j = np.unravel_index(flat, lower_bounds.shape)
        if lower_bounds[i, j] >= best+1e-12:
            break
        best = min(best, obb_clearance(boxes_a[i], boxes_b[j]))
        if best == 0.:
            break
    return best


def _bounds(boxes: list[dict]) -> tuple[np.ndarray, np.ndarray]:
    vertices = np.concatenate([box_vertices(box) for box in boxes], axis=0)
    return vertices.min(axis=0), vertices.max(axis=0)


def geometric_connection(upper: dict, lower: dict, config: dict) -> dict:
    """Check geometry without trusting connector state or object AABB telemetry."""
    cfg = _config(config)
    up, ur = _pose(upper)
    lp, lr = _pose(lower)
    relative_r = lr.T @ ur
    relative_p = lr.T @ (up-lp)
    tilt_deg = math.degrees(math.acos(float(np.clip(relative_r[2, 2], -1, 1))))
    yaw = math.atan2(float(relative_r[1, 0]), float(relative_r[0, 0]))
    yaw_error_deg = abs(math.degrees((yaw+math.pi/4) % (math.pi/2)-math.pi/4))
    # A single circular socket has no privileged yaw. Multi-socket square
    # bricks admit all four 90-degree symmetries, not a chosen front face.
    single_socket = math.prod(cfg["upper_shape"]) == 1
    orientation_ok = tilt_deg <= cfg["tilt_tolerance_deg"]+EPS and (
        single_socket or yaw_error_deg <= cfg["yaw_tolerance_deg"]+EPS)
    lower_xy = stud_centres(cfg["lower_shape"], cfg["pitch"])
    upper_xy = stud_centres(cfg["upper_shape"], cfg["pitch"])
    holes = np.column_stack([upper_xy, np.full(len(upper_xy), -cfg["body_height"]/2)])
    mouths = relative_p + holes @ relative_r.T
    used = set()
    matches = []
    for socket, mouth in enumerate(mouths):
        distances = np.linalg.norm(lower_xy-mouth[:2], axis=1)
        stud = int(np.argmin(distances))
        lateral = float(distances[stud])
        seat_gap = float(mouth[2]-cfg["body_height"]/2)
        if lateral <= cfg["lateral_tolerance"]+EPS and abs(seat_gap) <= cfg["seat_tolerance"]+EPS and stud not in used:
            used.add(stud)
            matches.append({"socket": socket, "stud": stud, "lateral_error": lateral, "seat_gap": seat_gap})
    count = len(matches)
    return {"valid": bool(orientation_ok and count == cfg["required_studs"]),
            "engaged_studs": count, "matches": matches,
            "tilt_deg": tilt_deg, "yaw_error_deg": yaw_error_deg,
            "relative_position": relative_p.tolist()}


class BrickScorer:
    """Stateful scorer; report the final score, never the maximum historic score.

    0.25: upper brick held with its bottom >=20 mm above the table for 0.20 s.
    0.50: after pickup, the specified seated alignment persists for 0.20 s.
    1.00: after seating, lift the connected lower brick >=20 mm for 1 s while
          gripping only the upper brick, then leave the connected assembly on
          the table for 1 s, both hands >=5 mm clear and pose stable to 2 mm/5°.

    Losing the connection erases connection/full credit and the lift proof.
    Pickup credit survives ordinary failed attempts. Leaving the permitted
    table bounds or malformed/nonfinite evidence permanently invalidates all
    credit. Timers use measured timestamps and reset over missing intervals.
    """

    def __init__(self, config: dict):
        self.config = _config(config)
        self.score = 0.
        self.invalid_reason = None
        self.previous_time = None
        self.pickup_time = None
        self.seated_time = None
        self.proof_time = None
        self.rest_time = None
        self._pickup_start = None
        self._seat_start = None
        self._proof_start = None
        self._rest_start = None
        self._rest_anchor = None
        self.geometry = None
        self.lower_finger_clearance = None
        self.assembly_finger_clearance = None
        self._finger_evidence_complete = False

    def _invalidate(self, reason: str) -> float:
        self.invalid_reason = reason
        self.score = 0.
        return self.score

    def _reset_connection(self):
        self._seat_start = self.seated_time = None
        self._proof_start = self.proof_time = None
        self._rest_start = self.rest_time = self._rest_anchor = None

    def _held(self, body: dict, hands: list[dict]) -> bool:
        p, _ = _pose(body)
        envelope = body_box(body, self.config["upper_shape"], self.config)
        for hand in hands:
            if float(np.linalg.norm(_vec(hand["tcp"], 3)-p)) > .035+EPS:
                continue
            # Measured jaw geometry scales with the brick. A fixed normalized
            # opening cutoff could reject a sound grasp of the wider brick.
            tips = {b.get("link"): b for b in hand.get("finger_boxes", [])
                    if b.get("link") in ("tip_left", "tip_right")}
            if len(tips) == 2 and all(obb_clearance(tip, envelope) <= .001+EPS for tip in tips.values()):
                return True
        return False

    @staticmethod
    def _window(start: float | None, condition: bool, now: float, duration: float):
        if not condition:
            return None, False
        start = now if start is None else start
        return start, now-start >= duration-EPS

    def update(self, sample: dict) -> float:
        if self.invalid_reason:
            return 0.
        try:
            return self._update(sample)
        except (KeyError, TypeError, ValueError, OverflowError, FloatingPointError) as exc:
            return self._invalidate(f"invalid evidence: {exc}")

    def _update(self, sample: dict) -> float:
        now = float(sample["time"])
        if not math.isfinite(now) or now < 0:
            raise ValueError("timestamp must be finite and nonnegative")
        if self.previous_time is not None:
            dt = now-self.previous_time
            if dt <= EPS:
                raise ValueError("timestamps must strictly increase")
            if dt > self.config["max_sample_gap"]+EPS:
                self._pickup_start = None
                self._reset_connection()
        self.previous_time = now
        upper, lower = sample["upper"], sample["lower"]
        up, ur = _pose(upper)
        lp, lr = _pose(lower)
        upper_boxes = brick_envelope(upper, self.config["upper_shape"], self.config)
        lower_boxes = brick_envelope(lower, self.config["lower_shape"], self.config)
        upper_low, upper_high = _bounds(upper_boxes)
        lower_low, lower_high = _bounds(lower_boxes)
        for low, high in ((upper_low, upper_high), (lower_low, lower_high)):
            if low[0] < -.17-EPS or high[0] > .89+EPS or low[1] < -.63-EPS or high[1] > .63+EPS or low[2] < TABLE-.015-EPS:
                return self._invalidate("a brick left the permitted table bounds")
        hands = sample["hands"]
        if not isinstance(hands, list) or len(hands) != 2:
            raise ValueError("both hands must be recorded")
        for hand in hands:
            _vec(hand["tcp"], 3)
            opening = float(hand["opening"])
            if not math.isfinite(opening) or not 0 <= opening <= 1:
                raise ValueError("invalid normalized hand opening")
        # Per-hand envelopes demonstrate that neither gripper was omitted.
        finger_boxes = [box for hand in hands for box in hand.get("finger_boxes", [])]
        complete = all(ROBOT_COLLISION_LINKS.issubset({box.get("link") for box in hand.get("finger_boxes", [])})
                       for hand in hands)
        for box in finger_boxes:
            _box(box)
        self._finger_evidence_complete = complete
        self.lower_finger_clearance = minimum_clearance(finger_boxes, lower_boxes) if complete else None
        self.assembly_finger_clearance = minimum_clearance(finger_boxes, lower_boxes+upper_boxes) if complete else None
        held_upper = self._held(upper, hands)
        self._pickup_start, picked = self._window(self._pickup_start, held_upper and upper_low[2] >= TABLE+.020-EPS, now, .20)
        if picked and self.pickup_time is None:
            self.pickup_time = now
        self.geometry = geometric_connection(upper, lower, self.config)
        connected = self.geometry["valid"]
        if not connected or self.pickup_time is None:
            self._reset_connection()
            self.score = .25 if self.pickup_time is not None else 0.
            return self.score
        self._seat_start, seated = self._window(self._seat_start, True, now, .20)
        if seated and self.seated_time is None:
            self.seated_time = now
        if self.seated_time is None:
            self.score = .25
            return self.score
        proof_condition = (held_upper and lower_low[2] >= TABLE+.020-EPS and complete
                           and self.lower_finger_clearance >= .002-EPS)
        self._proof_start, proved = self._window(self._proof_start, proof_condition, now, 1.)
        if proved and self.proof_time is None:
            self.proof_time = now
        lower_level = float(lr[2, 2]) >= math.cos(math.radians(3.))-EPS
        rest_condition = (self.proof_time is not None and complete
                          and self.assembly_finger_clearance >= .005-EPS
                          and abs(float(lower_low[2])-TABLE) <= .003+EPS and lower_level)
        if rest_condition:
            if self._rest_anchor is None:
                self._rest_anchor = (up.copy(), ur.copy(), lp.copy(), lr.copy())
            au, aru, al, arl = self._rest_anchor
            stable = (float(np.linalg.norm(up-au)) <= .002+EPS
                      and float(np.linalg.norm(lp-al)) <= .002+EPS
                      and rotation_distance(ur, aru) <= math.radians(5.)+EPS
                      and rotation_distance(lr, arl) <= math.radians(5.)+EPS)
            if not stable:
                self._rest_anchor = (up.copy(), ur.copy(), lp.copy(), lr.copy())
                self._rest_start = None
        else:
            self._rest_anchor = None
        self._rest_start, rested = self._window(self._rest_start, rest_condition, now, 1.)
        self.rest_time = now if rested and self.rest_time is None else self.rest_time
        if not rest_condition or not rested:
            self.rest_time = None
        self.score = 1. if rested else .5
        return self.score

    def result(self) -> dict:
        return {"score": self.score, "invalid_reason": self.invalid_reason,
                "pickup_time": self.pickup_time, "seated_time": self.seated_time,
                "proof_time": self.proof_time, "rest_time": self.rest_time,
                "geometry": self.geometry,
                "lower_finger_clearance": self.lower_finger_clearance,
                "assembly_finger_clearance": self.assembly_finger_clearance,
                "finger_evidence_complete": self._finger_evidence_complete}


def score_samples(samples, config: dict) -> dict:
    scorer = BrickScorer(config)
    for sample in samples:
        scorer.update(sample)
    return scorer.result()
