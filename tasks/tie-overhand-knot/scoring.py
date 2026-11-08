"""Replayable, fail-closed numerical evidence for the overhand-knot task.

No pickup credit.  0.5 needs a released overhand candidate clear of the jig
at the end of the robot policy. This result is frozen before the
assessor acts. 1.0 additionally requires the knot to persist through
a two-second seating ramp and five seconds of opposing endpoint load. Defaults
use 0.981 N (a 100 g hanging mass) per end, not a claimed exact 1 N. Genuine
assessor-induced failure preserves the pre-test half credit; invalid geometry
or malformed evidence erases all credit. Assessor actions cannot create credit.

This scorer checks supplied telemetry for internal consistency; it cannot
authenticate a recording or prove that logged forces were actually applied.
The assay producer must read its applied external-force buffer and preserve
that evidence, rather than logging requests as measurements.  A source string
alone is not independent verification.  The load assay must also be physically
validated; endpoint force is not a direct measurement of internal rope tension.

Core span, crossing-envelope motion and inferred tail lengths are diagnostics
only. Tightening and changes of view can move that envelope; none is a scoring
condition or a ruler measurement of the physical knot.

Call ``summary(final=True)`` or ``finalize()`` at the end of every recording or
replay. Per-frame ``summary()`` is cheap; policy topology is refreshed every
0.25 s and at release transitions. Assessor topology, geometry, timestamp and
force audits run on every sample. Terminal validation is compulsory.

Phases run once in order ``policy``, ``seating``, ``proof``. Both assessor phases
use the existing ``proof`` force record, with ``elapsed_s`` reset on entering
each phase. A first elapsed value of zero or exactly the preceding capture
interval is accepted. The seating force must follow a monotone linear ramp;
the interval's start-to-end expected force envelope accommodates an applied
force read back after stepping. The proof begins exactly two seconds after
seating begins and needs five continuous seconds within the force tolerance.

``diagnostic_only=True`` in configuration or ANY sample permanently disables
task credit for that recording.  Preformed-knot material probes can expose
candidate diagnostics, but their task score remains zero even after a full
assay.  Later samples cannot remove the diagnostic-only designation.
"""

from __future__ import annotations

import math
import json
from pathlib import Path

import numpy as np

from topology import analyze_open_rope, _pair_distances


ROBOT_COLLISION_LINKS = frozenset(("base", "gripper", "tip_left", "tip_right",
                                 "link1", "link2", "link3", "link4", "link5"))
DEFAULTS = {
    "radius": .004, "segment": .01, "length": .64, "control_hz": 20.,
    "joint_gap_max_m": .002, "hand_clearance_m": .03,
    "proof_load_n": .981, "seating_seconds": 2., "proof_seconds": 5.,
    "proof_force_relative_tolerance": .05,
    "topology_interval_s": .25,
}
PROOF_PROTOCOL = "opposing_endpoint_load_v1"
PROOF_SOURCE = "simulator_applied_external_force"


def _array(value, shape, label):
    a = np.asarray(value, dtype=float)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError(f"{label}: expected finite shape {shape}")
    return a.copy()


def _number(value, label):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label}: non-finite number")
    return number


def _rotation(value):
    r = _array(value, (3, 3), "box rotation")
    if not np.allclose(r.T@r, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1., atol=1e-6):
        raise ValueError("box rotation is not a proper orthonormal matrix")
    return r


def _box_clearance(endpoints, radius, box):
    """Exact segment/AABB distance in box coordinates, then subtract radius.

    Squared distance is a convex piecewise quadratic along each segment.  Its
    breakpoints are the six box planes.  Check each interval's minimum, avoiding
    point-sampling gaps that could miss an intervening finger contact.
    """
    center = _array(box["center"], (3,), "box center")
    rotation = _rotation(box["rotation"])
    half = _array(box["half_size"], (3,), "box half_size")
    if np.any(half <= 0):
        raise ValueError("box half_size must be positive")
    a = (endpoints[:, 0]-center)@rotation
    delta = (endpoints[:, 1]-endpoints[:, 0])@rotation
    safe = np.where(np.abs(delta) > 1e-15, delta, 1.)
    times = np.concatenate([np.zeros((len(a), 1)), np.ones((len(a), 1)),
                            np.clip((-half-a)/safe, 0, 1), np.clip((half-a)/safe, 0, 1)], axis=1)
    times.sort(axis=1)
    mid = (times[:, :-1]+times[:, 1:])/2
    positions = a[:, None, :]+mid[:, :, None]*delta[:, None, :]
    active = np.abs(positions) > half
    offset = a[:, None, :]-np.sign(positions)*half
    aa = np.sum(active*delta[:, None, :]**2, axis=2)
    bb = np.sum(active*delta[:, None, :]*offset, axis=2)
    optimum = np.clip(-bb/np.where(aa > 1e-30, aa, 1.), times[:, :-1], times[:, 1:])
    positions = a[:, None, :]+optimum[:, :, None]*delta[:, None, :]
    outside = np.maximum(np.abs(positions)-half, 0.)
    distance = float(np.sqrt(np.min(np.sum(outside**2, axis=2))))
    return max(0., distance-radius)


def _read_cord(cord, config):
    n = int(round(config["length"]/config["segment"]))
    if n < 3:
        raise ValueError("configuration requires at least three rope links")
    points = _array(cord["points"], (n, 3), "cord points")
    quats = _array(cord["quats_xyzw"], (n, 4), "cord quaternions")
    norms = np.linalg.norm(quats, axis=1)
    if not np.allclose(norms, 1., atol=1e-5):
        raise ValueError("cord quaternion is not unit length")
    x, y, z, w = (quats/norms[:, None]).T
    axes = np.stack([1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)], axis=1)
    seg = _number(cord["seg_len"], "segment length")
    radius = _number(cord["radius"], "radius")
    if not np.isclose(seg, config["segment"], rtol=0, atol=1e-8) or not np.isclose(radius, config["radius"], rtol=0, atol=1e-8):
        raise ValueError("rope dimensions differ from fixed configuration")
    ends = np.stack([points-axes*seg/2, points+axes*seg/2], axis=1)
    recorded = _array(cord["endpoints"], (n, 2, 3), "cord endpoints")
    if not np.allclose(ends, recorded, rtol=0, atol=1e-6):
        raise ValueError("recorded endpoints contradict centres and orientations")
    gaps = np.linalg.norm(ends[:-1, 1]-ends[1:, 0], axis=1)
    reported_gaps = _array(cord["joint_gaps"], (n-1,), "joint_gaps")
    if not np.allclose(gaps, reported_gaps, rtol=0, atol=1e-6):
        raise ValueError("reported joint gaps contradict measured endpoints")
    maximum_gap = float(gaps.max())
    if not math.isclose(maximum_gap, _number(cord["max_joint_gap"], "max_joint_gap"), rel_tol=0, abs_tol=1e-6):
        raise ValueError("reported maximum gap contradicts joint gaps")
    if maximum_gap > config["joint_gap_max_m"]+1e-9:
        raise ValueError("rope joint gap exceeds configured limit")
    polyline = np.vstack([ends[0, 0], (ends[:-1, 1]+ends[1:, 0])/2, ends[-1, 1]])
    if not np.allclose(polyline, _array(cord["polyline"], (n+1, 3), "polyline"), rtol=0, atol=1e-6):
        raise ValueError("polyline contradicts individual link endpoints")
    velocities = _array(cord["linear_velocities"], (n, 3), "linear velocities")
    angular = _array(cord["angular_velocities"], (n, 3), "angular velocities")
    # Check actual capsule centreline segments, not averaged joint midpoints.
    i, j = np.triu_indices(n, k=2)
    nonlocal_pairs = (j-i-1)*seg >= 4*radius
    distance = _pair_distances(ends[i, 0], ends[i, 1], ends[j, 0], ends[j, 1])
    if np.any(distance <= max(1e-12, radius*2e-6)):
        raise ValueError("actual capsule centrelines intersect")
    minimum = float(distance[nonlocal_pairs].min()) if np.any(nonlocal_pairs) else None
    if minimum is not None and minimum < 2*radius*.95:
        raise ValueError("nonlocal capsule penetration exceeds 5% diameter tolerance")
    return {"points": points, "endpoints": ends, "polyline": polyline,
            "velocities": velocities, "angular_velocities": angular,
            "max_joint_gap_m": maximum_gap, "minimum_nonlocal_distance_m": minimum,
            "radius": radius, "segment": seg, "count": n}


def _hand_clearance(hands, cord):
    if not isinstance(hands, list) or len(hands) != 2:
        raise ValueError("exactly two measured hands are required")
    clearance = math.inf
    for hand in hands:
        _array(hand["tcp"], (3,), "hand TCP")
        opening = _number(hand["opening"], "hand opening")
        if not 0 <= opening <= 1:
            raise ValueError("hand opening outside [0,1]")
        boxes = hand["finger_boxes"]
        names = [box["link"] for box in boxes]
        if len(names) != len(set(names)) or not ROBOT_COLLISION_LINKS.issubset(names):
            raise ValueError("incomplete or duplicate robot collision envelopes")
        for box in boxes:
            clearance = min(clearance, _box_clearance(cord["endpoints"], cord["radius"], box))
    return clearance


def _jig_clearance(cord, config):
    """Surface clearance from the declared static jig, excluding its clamp."""
    if not config.get('development_jig'):
        return None
    if config['development_jig'] != 'three-peg':
        raise ValueError('Unknown jig geometry')
    from jig_layout import PEGS, PEG_Y_ENDS, PEG_RADIUS, SUPPORTS
    geometry = config.get('jig_geometry', dict(pegs=PEGS, peg_y_ends=PEG_Y_ENDS,
                                             peg_radius=PEG_RADIUS, supports=SUPPORTS))
    endpoints, radius = cord['endpoints'], cord['radius']
    a, b = endpoints[:, 0], endpoints[:, 1]
    clearance = math.inf
    for _, x, z in geometry['pegs']:
        c = np.tile([x, geometry['peg_y_ends'][0], z], (len(a), 1))
        d = np.tile([x, geometry['peg_y_ends'][1], z], (len(a), 1))
        clearance = min(clearance, float(_pair_distances(a, b, c, d).min())-radius-geometry['peg_radius'])
    for name, center, size in geometry['supports']:
        box = {'link': name, 'center': center, 'rotation': np.eye(3),
               'half_size': np.asarray(size)/2}
        clearance = min(clearance, _box_clearance(endpoints, radius, box))
    return clearance


def _core_metrics(topology, cord):
    poly = cord["polyline"]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    bounds = _array(topology["core_arclength_m"], (2,), "topology core bounds")
    if not 0 <= bounds[0] < bounds[1] <= arc[-1]:
        raise ValueError("invalid topology core interval")
    boundary_points = np.column_stack([np.interp(bounds, arc, poly[:, axis]) for axis in range(3)])
    selected = np.vstack([boundary_points[0], poly[(arc > bounds[0]) & (arc < bounds[1])], boundary_points[1]])
    diameter = float(np.sqrt(np.max(np.sum((selected[:, None]-selected[None, :])**2, axis=2))))+2*cord["radius"]
    material = np.arange(len(poly))*cord["segment"]
    material_bounds = np.interp(bounds, arc, material)
    return {"core_span_m": diameter, "material_core_bounds_m": material_bounds.tolist(),
            "tail_lengths_m": [float(material_bounds[0]), float(material[-1]-material_bounds[1])],
            "diagnostics_only": True,
            "method": "projection_dependent_crossing_envelope_not_physical_knot_boundary"}


class KnotScorer:
    def __init__(self, config=None):
        self.config = {**DEFAULTS, **(config or {})}
        for key in DEFAULTS:
            self.config[key] = _number(self.config[key], key)
            if self.config[key] <= 0:
                raise ValueError(f"{key} must be positive")
        self.value = self.candidate_value = 0.
        self._diagnostic_only = bool(self.config.get("diagnostic_only", False))
        self.samples = 0
        self.time = self.invalid = self.topology = self.topology_time = None
        self.metrics = self.clearance = self._last_cord = self._last_phase = None
        self.jig_clearance = None
        self.jig_clear = True
        self.released = False
        self.status = "diagnostic_only" if self._diagnostic_only else "no qualifying released knot"
        self.candidate_status = "no qualifying released knot"
        self._policy_result = None
        self._seating_start = self._proof_start = self._proof_baseline = None
        self._seating_elapsed = self._proof_elapsed = self._proof_max_slip = 0.
        self._seating_last_force = None
        self._seating_completed = self._proof_completed = False
        self._proof_failure = None
        self._proof_force_range = [None, None]
        self._seating_force_range = [None, None]
        self._proof_attempts = 0
        self._finalized = False

    def _invalidate(self, reason):
        self.invalid = str(reason)
        self.value = self.candidate_value = 0.
        self.candidate_status = "invalid evidence"
        self.status = "diagnostic_only" if self._diagnostic_only else "invalid evidence"
        self._proof_completed = False

    def _check_topology(self):
        self.topology = analyze_open_rope(self._last_cord["polyline"], 2*self._last_cord["radius"])
        self.topology_time = self.time
        self.metrics = None
        if self.topology["status"] == "invalid":
            self._invalidate("topology geometry: "+self.topology["reason"])
        elif self.topology["is_overhand"] is True:
            self.metrics = _core_metrics(self.topology, self._last_cord)

    def _qualifies(self):
        return bool(self.released and self.jig_clear and self.topology and
                    self.topology["is_overhand"] is True)

    def _fail_proof(self, reason):
        if self._proof_failure is None:
            self._proof_failure = reason
        self._proof_completed = False

    def _freeze_policy_result(self):
        self._check_topology()
        self._policy_result = {"time": self.time, "qualified": self._qualifies(),
                               "released": self.released,
                               "topology_status": self.topology["status"],
                               "core": self.metrics}
        self._proof_baseline = (np.asarray(self.metrics["material_core_bounds_m"])
                                if self.metrics else None)
        self._proof_attempts = 1
        if not self._policy_result["qualified"]:
            self._fail_proof("no qualifying released knot at the end of policy")

    def _phase_onset(self, elapsed, previous_time):
        if abs(elapsed) <= 1e-6:
            return self.time
        if previous_time is not None and math.isclose(
                elapsed, self.time-previous_time, rel_tol=0, abs_tol=1e-6):
            # Force-before-step / readback-after-step includes precisely this
            # preceding captured interval, never earlier unrecorded duration.
            return previous_time
        raise ValueError("assessor onset elapsed_s must be zero or exactly the preceding capture interval")

    def _retention_sample(self):
        if not self._qualifies():
            self._fail_proof("knot, robot release or jig clearance lost during assay")
        if self.metrics and self._proof_baseline is not None:
            slip = float(np.max(np.abs(np.asarray(self.metrics["material_core_bounds_m"])-self._proof_baseline)))
            self._proof_max_slip = max(self._proof_max_slip, slip)

    def _assessor_sample(self, proof, cord, phase, entering, previous_time):
        if not isinstance(proof, dict) or proof.get("protocol") != PROOF_PROTOCOL or proof.get("source") != PROOF_SOURCE:
            raise ValueError("missing applied-force assay provenance; requested force alone is insufficient")
        if proof.get("active") is not True:
            raise ValueError("assessor phase must record an active load")
        if proof.get("endpoint_indices") != [0, cord["count"]-1]:
            raise ValueError("assay loads are not assigned to the two endpoint links")
        forces = _array(proof["endpoint_forces_n"], (2, 3), "applied endpoint forces")
        magnitudes = np.linalg.norm(forces, axis=1)
        applied = _number(proof["applied_force_n"], "applied_force_n")
        if not math.isclose(applied, float(magnitudes.min()), rel_tol=0, abs_tol=1e-6):
            raise ValueError("reported scalar force contradicts applied endpoint vectors")
        elapsed = _number(proof["elapsed_s"], "assessor elapsed_s")
        if elapsed < -1e-6:
            raise ValueError("negative assessor elapsed_s")
        if entering:
            onset = self._phase_onset(elapsed, previous_time)
            if phase == "seating":
                self._seating_start = onset
            else:
                self._proof_start = onset
                duration = onset-self._seating_start
                duration_valid = math.isclose(duration, self.config["seating_seconds"], rel_tol=0, abs_tol=1e-6)
                if not duration_valid:
                    self._fail_proof("seating phase did not last the required duration")
                self._seating_completed = duration_valid and self._proof_failure is None
        start = self._seating_start if phase == "seating" else self._proof_start
        if start is None or abs(elapsed-(self.time-start)) > 1e-6:
            raise ValueError("assessor timer contradicts continuous sample timestamps")
        target, tolerance = self.config["proof_load_n"], self.config["proof_force_relative_tolerance"]
        low, high = float(magnitudes.min()), float(magnitudes.max())
        observed_range = self._seating_force_range if phase == "seating" else self._proof_force_range
        observed_range[0] = low if observed_range[0] is None else min(low, observed_range[0])
        observed_range[1] = high if observed_range[1] is None else max(high, observed_range[1])
        if phase == "seating":
            self._seating_elapsed = elapsed
            duration = self.config["seating_seconds"]
            interval = self.time-previous_time
            expected_low = target*np.clip((elapsed-interval)/duration, 0., 1.)
            expected_high = target*np.clip(elapsed/duration, 0., 1.)
            if elapsed > duration+1e-6:
                self._fail_proof("seating phase exceeded the required duration")
            if np.any(magnitudes < expected_low-target*tolerance-1e-9) or np.any(magnitudes > expected_high+target*tolerance+1e-9):
                self._fail_proof("seating force does not follow the prescribed linear ramp")
            if self._seating_last_force is not None and np.any(magnitudes < self._seating_last_force-1e-6):
                self._fail_proof("seating force decreased")
            self._seating_last_force = magnitudes
        else:
            self._proof_elapsed = elapsed
            if np.any(np.abs(magnitudes-target) > target*tolerance+1e-9):
                self._fail_proof("applied endpoint load outside tolerance")
        # A zero load at the beginning of seating has no direction. Every
        # nonzero pair must still point oppositely and outward from the ends.
        separation = cord["endpoints"][-1, 1]-cord["endpoints"][0, 0]
        norm = float(np.linalg.norm(separation))
        if norm < 1e-6 or (np.any(magnitudes < 1e-9) and high >= 1e-9):
            self._fail_proof("load direction cannot be established")
        elif low >= 1e-9:
            direction = separation/norm
            directions = forces/magnitudes[:, None]
            if float(directions[0]@directions[1]) > -math.cos(math.radians(5)) or float(directions[0]@(-direction)) < math.cos(math.radians(10)) or float(directions[1]@direction) < math.cos(math.radians(10)):
                self._fail_proof("endpoint loads are not opposed and outward")
        self._retention_sample()
        if phase == "proof" and elapsed >= self.config["proof_seconds"]-1e-9 and self._seating_completed and self._proof_failure is None:
            self._proof_completed = True

    def _set_score(self):
        if self.invalid:
            self.value, self.status = 0., "invalid evidence"
        elif self._policy_result is not None:
            if not self._policy_result["qualified"]:
                self.value, self.status = 0., "no qualifying knot before assessor actions"
            elif self._proof_completed:
                self.value, self.status = 1., "policy knot retained through seating and recorded load assay"
            else:
                self.value, self.status = .5, "policy-end knot; full retention assay not established"
        elif self._qualifies():
            self.value, self.status = .5, "released knot; assessor has not begun"
        else:
            self.value, self.status = 0., "no qualifying released knot"
        self.candidate_value, self.candidate_status = self.value, self.status
        if self._diagnostic_only:
            self.value, self.status = 0., "diagnostic_only"

    def update(self, sample):
        self.samples += 1
        self._finalized = False
        if isinstance(sample, dict) and sample.get("diagnostic_only"):
            self._diagnostic_only = True
            self.value, self.status = 0., "diagnostic_only"
        if self.invalid:
            return self.value
        try:
            time = _number(sample["time"], "sample time")
            period = 1/self.config["control_hz"]
            if self.time is None:
                if abs(time) > 1e-6:
                    raise ValueError("recording must include initial time zero")
            elif time <= self.time or time-self.time > 1.5*period+1e-9:
                raise ValueError("non-increasing timestamp or missing capture interval")
            phase = sample.get("phase", "policy")
            if phase not in ("policy", "seating", "proof"):
                raise ValueError("unknown capture phase")
            entering = phase != self._last_phase
            if entering and phase == "seating":
                if self._last_phase != "policy" or self._policy_result is not None:
                    raise ValueError("seating requires a recorded policy phase and may occur only once")
                self._freeze_policy_result()
                if self.invalid:
                    return self.value
            elif entering and phase == "proof" and self._last_phase != "seating":
                raise ValueError("proof requires the recorded seating phase")
            elif phase == "policy":
                if self._policy_result is not None:
                    raise ValueError("robot policy cannot resume after the assessor starts")
                if isinstance(sample.get("proof"), dict) and sample["proof"].get("active"):
                    raise ValueError("active assessor forces cannot be hidden in the policy phase")
            cord = _read_cord(sample["cord"], self.config)
            clearance = _hand_clearance(sample["hands"], cord)
            self.jig_clearance = _jig_clearance(cord, self.config)
            self.jig_clear = self.jig_clearance is None or self.jig_clearance >= .010-1e-9
            released = clearance >= self.config["hand_clearance_m"]-1e-9
            transition = entering or released != self.released
            previous_time = self.time
            self.time, self._last_cord = time, cord
            self.clearance, self.released = clearance, released
            if phase != "policy" or transition or self.topology_time is None or time-self.topology_time >= self.config["topology_interval_s"]-1e-9:
                self._check_topology()
            if self.invalid:
                return self.value
            if phase != "policy":
                self._assessor_sample(sample.get("proof"), cord, phase, entering, previous_time)
            self._last_phase = phase
            self._set_score()
        except (KeyError, ValueError, TypeError, ArithmeticError, IndexError) as exc:
            self._invalidate(exc)
        return self.value

    def finalize(self):
        """Refresh terminal evidence; assessor outcomes cannot rewrite policy credit."""
        if not self.invalid and self._last_cord is not None:
            try:
                self._check_topology()
                if self._policy_result is not None:
                    self._retention_sample()
                self._set_score()
            except (ValueError, TypeError, ArithmeticError, KeyError) as exc:
                self._invalidate(exc)
        self._finalized = True
        return self.value

    def summary(self, final=False):
        if final and not self._finalized:
            self.finalize()
        return {
            "score": self.value, "samples": self.samples, "time": self.time,
            "invalid": self.invalid, "status": self.status, "terminal_validated": self._finalized,
            "diagnostic_only": self._diagnostic_only,
            "candidate_score": self.candidate_value, "candidate_status": self.candidate_status,
            "released": self.released, "minimum_robot_clearance_m": self.clearance,
            "jig_clear": self.jig_clear, "minimum_jig_clearance_m": self.jig_clearance,
            "max_joint_gap_m": self._last_cord["max_joint_gap_m"] if self._last_cord else None,
            "minimum_nonlocal_distance_m": self._last_cord["minimum_nonlocal_distance_m"] if self._last_cord else None,
            "topology_status": self.topology["status"] if self.topology else None,
            "topology_reason": self.topology.get("reason") if self.topology else None,
            "topology_time": self.topology_time, "core": self.metrics,
            "policy_end": self._policy_result,
            "seating": {"elapsed_s": self._seating_elapsed, "required_seconds": self.config["seating_seconds"],
                        "completed": self._seating_completed, "observed_force_range_n": self._seating_force_range},
            "proof": {"protocol": PROOF_PROTOCOL, "attempts": self._proof_attempts,
                      "completed": self._proof_completed, "elapsed_s": self._proof_elapsed,
                      "target_force_n": self.config["proof_load_n"],
                      "observed_force_range_n": self._proof_force_range,
                      "maximum_boundary_slip_m": self._proof_max_slip,
                      "slip_method": "projected_crossing_envelope_in_material_coordinates",
                      "slip_and_core_span_are_diagnostics_only": True,
                      "failure": self._proof_failure},
        }


def replay_samples(samples, config=None):
    """Replay measured samples; existing ``metrics`` fields are never trusted."""
    scorer = KnotScorer(config)
    for sample in samples:
        scorer.update(sample)
    if not scorer.samples:
        scorer._invalidate("empty recording")
    return scorer.summary(final=True)


def replay_file(path, config=None):
    """Read JSONL (optionally gzip) and return a terminally validated score."""
    import gzip

    path = Path(path)
    opener = gzip.open if path.suffix == ".gz" else open
    scorer = KnotScorer(config)
    try:
        with opener(path, "rt", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, 1):
                try:
                    scorer.update(json.loads(line))
                except (json.JSONDecodeError, TypeError) as exc:
                    scorer._invalidate(f"malformed JSONL sample at line {line_number}: {exc}")
                    break
    except (OSError, EOFError) as exc:
        scorer._invalidate(f"recording could not be read: {exc}")
    if not scorer.samples and scorer.invalid is None:
        scorer._invalidate("empty recording")
    return scorer.summary(final=True)


if __name__ == "__main__":
    import argparse
    from config import CONFIG

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = replay_file(args.trajectory, CONFIG)
    encoded = json.dumps(result, indent=2, allow_nan=False)+"\n"
    if args.output:
        args.output.write_text(encoded)
    print(encoded, end="")
    raise SystemExit(1 if result["invalid"] else 0)
