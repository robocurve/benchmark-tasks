"""Conservative static topology diagnostics for an *open* polygonal rope.

Public API: ``analyze_open_rope(vertices, diameter, ...)`` returns JSON-safe
diagnostics.  It does not score the task and does not certify physics.  An open
arc has no invariant knot type: its ends can always be pulled back through it.
We therefore report an ``overhand_candidate`` only under an explicit closure
convention and agreement checks; uncertainty is not a negative knot finding.

Closure convention: extend each end away from the vertex centroid, with five
deterministic small changes to the exit directions.  Join the exits by a polygon
on a sphere of radius 12 times the rope's bounding radius.  The connecting
polygon lies outside the rope's bounding sphere.  Reject exits intersecting the
rope, and require at least three valid closures to agree.  This is a finite
closure sensitivity check, not the minimally-interfering-closure algorithm.

For each closure, inspect several generic projections.  At each crossing the
Fox/Wirtinger coloring row is 2*over - incoming_under - outgoing_under.  An exact
integer cofactor determinant is |Alexander(-1)|.  Determinant 3 ALONE does not
identify a trefoil.  A positive certificate additionally requires a diagram
reduced to exactly three alternating crossings by Reidemeister I/II removals,
all three crossings on the original rope.  Both handednesses are accepted.
Determinant 1 ALONE does not identify an unknot either: a zero-crossing reduction
is required for ``unknot``.  No RIII search is attempted, so some genuine knots
remain ambiguous.  Projection failures and missing certificates are explicit.

References for the invariant and open-chain ambiguity (not copied code):
  Kauffman, Remarks on Formal Knot Theory, arXiv:math/0605622, section on
  Fox coloring: https://arxiv.org/abs/math/0605622
  Tubiana, Orlandini & Micheletti, 2011, closure-scheme comparison:
  https://doi.org/10.1143/PTPS.191.192

Coordinates are metres.  The supplied vertices define straight centreline
segments.  Capsule clearance is a geometric check of that representation;
nearby portions along the rope are excluded to avoid adjacent-link overlap.
Actual link continuity, bending, contact calibration, between-frame crossings,
and the separate load/retention test must be checked elsewhere.
"""

from __future__ import annotations

from bisect import bisect_right
from collections import Counter
import math

import numpy as np


_VIEWS = np.asarray([
    [0.137, 0.271, 1.0], [0.619, -0.233, 1.0], [-0.481, 0.713, 1.0],
    [1.0, 0.357, 0.193], [0.283, 1.0, -0.429], [-0.317, 1.0, 0.811],
    [0.817, -0.611, 0.449], [0.913, 0.719, -0.541],
], dtype=float)


def _unit(v):
    norm = float(np.linalg.norm(v))
    if norm <= 1e-15:
        raise ValueError("zero direction")
    return np.asarray(v, dtype=float) / norm


def _basis(direction):
    n = _unit(direction)
    axis = np.eye(3)[int(np.argmin(np.abs(n)))]
    u = _unit(np.cross(n, axis))
    return u, np.cross(n, u), n


def _pair_distances(a, b, c, d):
    """Exact closest distances of paired line segments, vectorized in rows."""
    a, b, c, d = [np.asarray(x, dtype=float) for x in (a, b, c, d)]
    u, v, w = b - a, d - c, a - c
    aa = np.einsum("ij,ij->i", u, u)
    bb = np.einsum("ij,ij->i", u, v)
    cc = np.einsum("ij,ij->i", v, v)
    dd = np.einsum("ij,ij->i", u, w)
    ee = np.einsum("ij,ij->i", v, w)

    def point_segment(p, q, delta, length2):
        t = np.clip(np.einsum("ij,ij->i", p-q, delta) / length2, 0.0, 1.0)
        r = p - q - t[:, None] * delta
        return np.einsum("ij,ij->i", r, r)

    dist2 = np.minimum.reduce([
        point_segment(a, c, v, cc), point_segment(b, c, v, cc),
        point_segment(c, a, u, aa), point_segment(d, a, u, aa),
    ])
    denom = aa * cc - bb * bb
    regular = denom > 1e-12 * aa * cc
    safe = np.where(regular, denom, 1.0)
    s = (bb * ee - cc * dd) / safe
    t = (aa * ee - bb * dd) / safe
    interior = regular & (s >= 0) & (s <= 1) & (t >= 0) & (t <= 1)
    delta = w + s[:, None] * u - t[:, None] * v
    dist2 = np.where(interior, np.minimum(dist2, np.einsum("ij,ij->i", delta, delta)), dist2)
    return np.sqrt(np.maximum(0.0, dist2))


def _geometry(points, diameter, expected, stretch_tolerance):
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc = np.r_[0.0, np.cumsum(lengths)]
    result = {
        "valid": True, "reasons": [], "vertex_count": len(points),
        "length_m": float(arc[-1]), "diameter_m": float(diameter),
        "minimum_segment_length_m": float(lengths.min()),
        "maximum_segment_length_m": float(lengths.max()),
        "continuity_checked_against_rest_lengths": expected is not None,
        "nonlocal_exclusion_arclength_m": float(2.0 * diameter),
        "penetration_tolerance_fraction": 0.05,
    }
    if lengths.min() <= max(1e-12, diameter * 1e-8):
        result["valid"] = False
        result["reasons"].append("zero_or_near_zero_segment")
        return result, arc
    if expected is not None:
        rest = np.asarray(expected, dtype=float)
        if rest.ndim == 0:
            rest = np.full(len(lengths), float(rest))
        if rest.shape != lengths.shape or not np.all(np.isfinite(rest)) or np.any(rest <= 0):
            raise ValueError("expected_segment_lengths must be positive scalar or one value per segment")
        relative = np.abs(lengths / rest - 1.0)
        result["maximum_relative_length_error"] = float(relative.max())
        if np.any(relative > stretch_tolerance):
            result["valid"] = False
            result["reasons"].append("segment_length_outside_tolerance")

    i, j = np.triu_indices(len(lengths), k=2)
    # Never exempt an actual centreline crossing, even between locally nearby
    # segments.  Only the finite-radius capsule test needs a locality exclusion.
    all_distances = _pair_distances(points[i], points[i+1], points[j], points[j+1])
    result["minimum_nonadjacent_distance_m"] = float(all_distances.min()) if len(i) else None
    if len(i) and all_distances.min() <= max(1e-12, diameter*1e-6):
        result["valid"] = False
        result["reasons"].append("centreline_self_intersection")
    # Distance along the intervening centreline, not just index separation.
    keep = arc[j] - arc[i+1] >= 2.0 * diameter
    i, j = i[keep], j[keep]
    result["nonlocal_segment_pairs_checked"] = len(i)
    result["minimum_nonlocal_distance_m"] = None
    result["minimum_nonlocal_pair"] = None
    if len(i):
        distances = all_distances[keep]
        closest = int(np.argmin(distances))
        result["minimum_nonlocal_distance_m"] = float(distances[closest])
        result["minimum_nonlocal_pair"] = [int(i[closest]), int(j[closest])]
        if distances[closest] < diameter * 0.95:
            result["valid"] = False
            result["reasons"].append("nonlocal_capsule_penetration")
    return result, arc


def _closure(points, diameter, variant):
    centre = points.mean(axis=0)
    radius = float(np.linalg.norm(points-centre, axis=1).max())
    if radius <= diameter:
        return None, "rope_extent_too_small_for_closure"
    exits = []
    offsets = [(0.0, 0.0), (0.18, 0.13), (-0.18, -0.13), (0.13, -0.18), (-0.13, 0.18)]
    dx, dy = offsets[variant]
    for end_index in (len(points)-1, 0):
        end = points[end_index]
        try:
            u, v, radial = _basis(end-centre)
        except ValueError:
            return None, "endpoint_at_closure_centre"
        direction = _unit(radial + dx*u + dy*v)
        relative = end-centre
        distance = -float(relative @ direction) + math.sqrt(
            float(relative @ direction)**2 + (12.0*radius)**2 - float(relative @ relative)
        )
        exits.append(end+distance*direction)

    a, b = [_unit(e-centre) for e in exits]
    angle = math.acos(float(np.clip(a @ b, -1, 1)))
    if angle < 1e-6:
        return None, "coincident_closure_exits"
    if math.pi-angle < 1e-6:
        perpendicular = _basis(a)[0]
        arc_dirs = [math.cos(t)*a + math.sin(t)*perpendicular for t in np.linspace(0, math.pi, 17)]
    else:
        arc_dirs = [
            (math.sin((1-t)*angle)*a + math.sin(t*angle)*b)/math.sin(angle)
            for t in np.linspace(0, 1, 17)
        ]
    outer_arc = centre + 12.0*radius*np.asarray(arc_dirs)
    closed = np.vstack([points, outer_arc, points[0]])

    # Test both virtual exits against remote portions of the actual rope.
    rope_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    rope_arc = np.r_[0.0, np.cumsum(rope_lengths)]
    for end, far, at_start in ((points[-1], outer_arc[0], False), (points[0], outer_arc[-1], True)):
        remote = (rope_arc[:-1] >= 2*diameter) if at_start else (rope_arc[-1]-rope_arc[1:] >= 2*diameter)
        if np.any(remote):
            count = int(remote.sum())
            distances = _pair_distances(np.tile(end, (count, 1)), np.tile(far, (count, 1)), points[:-1][remote], points[1:][remote])
            if distances.min() <= diameter*0.55:
                return None, "virtual_exit_intersects_rope"
    return closed, None


def _cross2(a, b):
    return a[..., 0]*b[..., 1] - a[..., 1]*b[..., 0]


def _project_diagram(closed, direction, rope_segment_count):
    u, v, normal = _basis(direction)
    projected = closed @ np.stack([u, v], axis=1)
    depth = closed @ normal
    delta = np.diff(projected, axis=0)
    count = len(delta)
    scale = max(float(np.ptp(projected, axis=0).max()), 1e-12)
    eps = scale*1e-9
    plen = np.linalg.norm(delta, axis=1)
    base = {"valid": False, "crossings": [], "events": [], "direction": normal.tolist()}
    if np.any(plen <= eps):
        return {**base, "reason": "segment_projects_to_point"}
    ii, jj = np.triu_indices(count, k=2)
    keep = ~((ii == 0) & (jj == count-1))
    ii, jj = ii[keep], jj[keep]
    # Broad-phase boxes include tolerance so vertex hits cannot disappear.
    lo = np.minimum(projected[:-1], projected[1:])
    hi = np.maximum(projected[:-1], projected[1:])
    keep = np.all(hi[ii]+eps >= lo[jj], axis=1) & np.all(hi[jj]+eps >= lo[ii], axis=1)
    ii, jj = ii[keep], jj[keep]
    if not len(ii):
        return {**base, "valid": True, "reason": None}
    p, r, q, s = projected[ii], delta[ii], projected[jj], delta[jj]
    denom = _cross2(r, s)
    gap = q-p
    parallel = np.abs(denom) <= 1e-10*plen[ii]*plen[jj]
    if np.any(parallel):
        # Collinear overlapping non-adjacent segments are a singular view.
        collinear = np.abs(_cross2(gap, r)) <= eps*plen[ii]
        if np.any(parallel & collinear):
            return {**base, "reason": "projected_collinear_overlap"}
    safe = np.where(parallel, 1.0, denom)
    ts = _cross2(gap, s)/safe
    us = _cross2(gap, r)/safe
    inside = (~parallel) & (ts >= -1e-8) & (ts <= 1+1e-8) & (us >= -1e-8) & (us <= 1+1e-8)
    crossings = []
    events = []
    positions = []
    for k in np.flatnonzero(inside):
        i, j, t, w = int(ii[k]), int(jj[k]), float(ts[k]), float(us[k])
        if min(t, w, 1-t, 1-w) < 1e-7:
            return {**base, "reason": "crossing_at_vertex"}
        if abs(float(denom[k])) / (plen[i]*plen[j]) < 1e-5:
            return {**base, "reason": "near_tangent_crossing"}
        zi = depth[i]+t*(depth[i+1]-depth[i])
        zj = depth[j]+w*(depth[j+1]-depth[j])
        if abs(float(zi-zj)) <= eps:
            return {**base, "reason": "ambiguous_crossing_depth"}
        xy = projected[i]+t*delta[i]
        if positions and np.linalg.norm(np.asarray(positions)-xy, axis=1).min() <= 10*eps:
            return {**base, "reason": "coincident_projected_crossings"}
        crossing_id = len(crossings)
        i_over = bool(zi > zj)
        sign = (1 if denom[k] > 0 else -1)*(1 if i_over else -1)
        crossings.append({"id": crossing_id, "segments": [i, j], "fractions": [t, w], "sign": sign, "on_rope": j < rope_segment_count})
        events.extend([
            {"id": crossing_id, "over": i_over, "sign": sign, "position": i+t},
            {"id": crossing_id, "over": not i_over, "sign": sign, "position": j+w},
        ])
        positions.append(xy)
    events.sort(key=lambda e: e["position"])
    if len(events) > 1 and min(events[k+1]["position"]-events[k]["position"] for k in range(len(events)-1)) < 1e-8:
        return {**base, "reason": "crossing_order_ambiguous"}
    return {**base, "valid": True, "reason": None, "crossings": crossings, "events": events}


def _bareiss_determinant(matrix):
    """Integer-preserving elimination; no rounded floating determinant."""
    a = [list(map(int, row)) for row in matrix]
    n = len(a)
    if n == 0:
        return 1
    sign, previous = 1, 1
    for k in range(n-1):
        if not a[k][k]:
            pivot = next((i for i in range(k+1, n) if a[i][k]), None)
            if pivot is None:
                return 0
            a[k], a[pivot] = a[pivot], a[k]
            sign = -sign
        pivot = a[k][k]
        for i in range(k+1, n):
            for j in range(k+1, n):
                numerator = a[i][j]*pivot-a[i][k]*a[k][j]
                if numerator % previous:
                    raise ArithmeticError("non-exact Bareiss elimination")
                a[i][j] = numerator//previous
        for i in range(k+1, n):
            a[i][k] = 0
        previous = pivot
    return sign*a[-1][-1]


def _alexander_determinant(events):
    if not events:
        return 1
    under_positions = [i for i, e in enumerate(events) if not e["over"]]
    under_index = {events[p]["id"]: k for k, p in enumerate(under_positions)}
    n = len(under_positions)
    matrix = [[0]*n for _ in range(n)]
    for position, event in enumerate(events):
        if event["over"]:
            row = under_index[event["id"]]
            over_arc = (bisect_right(under_positions, position)-1) % n
            matrix[row][over_arc] += 2
            matrix[row][(row-1) % n] -= 1
            matrix[row][row] -= 1
    return abs(_bareiss_determinant([row[:-1] for row in matrix[:-1]]))


def _reduce_diagram(events):
    """Apply only Gauss-word RI/RII cancellations, retaining original IDs."""
    events = list(events)
    moves = {"reidemeister_I": 0, "reidemeister_II": 0}
    while events:
        n = len(events)
        remove = None
        for i in range(n):
            if events[i]["id"] == events[(i+1) % n]["id"]:
                remove = {events[i]["id"]}
                moves["reidemeister_I"] += 1
                break
        if remove is None:
            for i in range(n):
                a, b = events[i], events[(i+1) % n]
                if a["id"] == b["id"] or a["over"] != b["over"] or a["sign"] == b["sign"]:
                    continue
                for j in range(n):
                    c, d = events[j], events[(j+1) % n]
                    if {a["id"], b["id"]} == {c["id"], d["id"]} and c["over"] == d["over"] != a["over"]:
                        remove = {a["id"], b["id"]}
                        moves["reidemeister_II"] += 1
                        break
                if remove:
                    break
        if remove is None:
            break
        events = [e for e in events if e["id"] not in remove]
    return events, moves


def _classify_projection(diagram, rope_arc):
    result = {"valid": diagram["valid"], "reason": diagram["reason"], "direction": diagram["direction"]}
    if not diagram["valid"]:
        return result
    events = diagram["events"]
    if len(events) > 300:
        return {**result, "valid": False, "reason": "crossing_budget_exceeded", "crossing_count": len(events)//2}
    determinant = _alexander_determinant(events)
    reduced, moves = _reduce_diagram(events)
    reduced_determinant = _alexander_determinant(reduced)
    if determinant != reduced_determinant:
        return {**result, "valid": False, "reason": "reduction_changed_determinant"}
    result.update({
        "crossing_count": len(events)//2, "reduced_crossing_count": len(reduced)//2,
        "alexander_determinant": determinant, "reduction_moves": moves,
        "trefoil_certificate": False, "unknot_certificate": not reduced,
        "core_arclength_m": None,
        "reduced_gauss_code": [
            {"crossing": e["id"], "over": e["over"], "sign": e["sign"]}
            for e in reduced
        ],
    })
    alternating = bool(reduced) and all(reduced[k]["over"] != reduced[(k+1) % len(reduced)]["over"] for k in range(len(reduced)))
    if len(reduced) == 6 and determinant == 3 and alternating:
        retained = {e["id"] for e in reduced}
        result["closure_trefoil_certificate"] = True
        result["retained_crossings"] = [c for c in diagram["crossings"] if c["id"] in retained]
        physical = all(c["on_rope"] for c in diagram["crossings"] if c["id"] in retained)
        result["trefoil_certificate"] = physical
        if physical:
            locations = []
            for event in reduced:
                segment = int(event["position"])
                fraction = event["position"]-segment
                locations.append(float(rope_arc[segment]+fraction*(rope_arc[segment+1]-rope_arc[segment])))
            result["core_arclength_m"] = [min(locations), max(locations)]
        else:
            result["reason"] = "trefoil_crossings_include_virtual_closure"
    return result


def analyze_open_rope(vertices, diameter, *, expected_segment_lengths=None, stretch_tolerance=0.10):
    """Classify static centreline evidence, conservatively and without scoring.

    ``is_overhand`` is True only for an overhand candidate, False for certified
    unknotted closures / definitely incompatible determinants, and None when
    invalid or ambiguous.  A missing certificate is never treated as false.
    ``tail_lengths_m`` uses the union of crossing intervals from all retained
    three-crossing certificates.  It is a projection-dependent envelope, not
    the minimal physical knot core or a verification of accessible tail ends.

    If rest lengths are omitted, breaks/stretch have NOT been checked: an
    ordered point array cannot establish whether physical links are connected.
    """
    result = {
        "status": "invalid", "is_overhand": None, "reason": None,
        "geometry": None, "closures": [], "core_arclength_m": None,
        "core_vertex_indices": None, "tail_lengths_m": None,
        "confidence": "none", "closure_convention": "five_perturbed_radial_exits_outer_sphere_12r",
        "physical_knot_proven": False,
        "limitations": [
            "Open-chain classification depends on closure; five closures are a finite sensitivity check.",
            "Determinant 3 alone is insufficient; three-crossing diagram certificates are required.",
            "Static centreline data cannot prove load retention, absence of past tunneling, or material realism.",
            "Crossing-envelope tail estimates are projection-dependent and do not prove graspable tails.",
        ],
    }
    try:
        points = np.asarray(vertices, dtype=float)
        diameter = float(diameter)
        if points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 4:
            raise ValueError("vertices must be an N by 3 array with N >= 4")
        if len(points) > 2000:
            raise ValueError("vertex budget exceeded (2000)")
        if not np.all(np.isfinite(points)) or not math.isfinite(diameter) or diameter <= 0:
            raise ValueError("coordinates and positive diameter must be finite")
        if not math.isfinite(stretch_tolerance) or not 0 <= stretch_tolerance < 1:
            raise ValueError("stretch_tolerance must lie in [0,1)")
        geometry, rope_arc = _geometry(points, diameter, expected_segment_lengths, stretch_tolerance)
        result["geometry"] = geometry
        if not geometry["valid"]:
            result["reason"] = "; ".join(geometry["reasons"])
            return result
    except (ValueError, TypeError) as exc:
        result["reason"] = str(exc)
        return result

    core_ranges = []
    accepted = []
    for variant in range(5):
        closed, error = _closure(points, diameter, variant)
        item = {"variant": variant, "valid": False, "reason": error, "projections": []}
        if closed is None:
            result["closures"].append(item)
            continue
        views = [_classify_projection(_project_diagram(closed, view, len(points)-1), rope_arc) for view in _VIEWS]
        item["projections"] = views
        valid_views = [v for v in views if v["valid"]]
        determinants = {v["alexander_determinant"] for v in valid_views}
        item["valid_projection_count"] = len(valid_views)
        if len(valid_views) < 3:
            item["reason"] = "insufficient_generic_projections"
        elif len(determinants) != 1:
            item["reason"] = "projection_invariants_disagree"
        else:
            item["valid"] = True
            item["reason"] = None
            item["alexander_determinant"] = next(iter(determinants))
            item["trefoil_certificate_count"] = sum(v["trefoil_certificate"] for v in valid_views)
            item["unknot_certificate_count"] = sum(v["unknot_certificate"] for v in valid_views)
            accepted.append(item)
            core_ranges.extend(v["core_arclength_m"] for v in valid_views if v["core_arclength_m"] is not None)
        result["closures"].append(item)

    result["valid_closure_count"] = len(accepted)
    result["projection_failure_counts"] = dict(Counter(v["reason"] for c in result["closures"] for v in c["projections"] if not v["valid"]))
    if len(accepted) < 3:
        result.update(status="ambiguous", reason="fewer_than_three_valid_exterior_closures")
        return result
    determinants = {c["alexander_determinant"] for c in accepted}
    result["observed_determinants"] = sorted(determinants)
    if len(determinants) != 1:
        result.update(status="ambiguous", reason="closure_sensitive_topology", confidence="closure_disagreement")
    elif determinants == {3} and all(c["trefoil_certificate_count"] >= 2 for c in accepted):
        low = min(r[0] for r in core_ranges)
        high = max(r[1] for r in core_ranges)
        result.update(
            status="overhand_candidate", is_overhand=True,
            reason="agreeing_exterior_closures_with_three_crossing_certificates",
            confidence="finite_closure_and_projection_consensus",
            core_arclength_m=[low, high],
            core_vertex_indices=[int(np.searchsorted(rope_arc, low, side="right")-1), min(len(points)-1, int(np.searchsorted(rope_arc, high, side="left")))],
            tail_lengths_m=[low, float(rope_arc[-1]-high)],
        )
    elif determinants == {1} and all(c["unknot_certificate_count"] >= 2 for c in accepted):
        result.update(status="unknot", is_overhand=False, reason="agreeing_exterior_closures_reduce_to_zero_crossings", confidence="finite_closure_and_projection_consensus")
    elif determinants.isdisjoint({1, 3}):
        result.update(status="other", is_overhand=False, reason="closure_determinant_incompatible_with_trefoil", confidence="finite_closure_and_projection_consensus")
    else:
        result.update(status="ambiguous", reason="invariant_consistent_but_diagram_certificate_missing", confidence="invariant_only")
    return result
