"""Preformed geometry for PHYSICS DIAGNOSTICS ONLY, never task/demo resets.

``preformed`` (alias ``loose-overhand``) is deliberately already threaded. It tests whether the rope
model can settle, tighten and retain an existing knot. It cannot demonstrate
robotic knot tying, earn a task score, or replace the untied task starting layout.

The curve is an open T(2,3) torus-knot parameterization with two exterior tails,
the same mathematical construction independently exercised by test_topology.py.
Equal-chord resampling produces actual 10 mm straight links, not approximately
equal arclength bins. The last incomplete tail link is discarded. No simulation
state is read or written by this module. It depends only on NumPy until a caller
explicitly requests the local topology diagnostic in geometry_report().

An open rope has no closure-independent knot invariant. The reported candidate
certificate uses analyze_open_rope's documented exterior-closure convention;
neither that certificate nor clearance proves physical knot retention.
"""

from __future__ import annotations

import json
import math

import numpy as np


SEGMENT = 0.01
DIAMETER = 0.008
TABLE_TOP = 0.7475
CENTRE = np.array((0.28, 0.0, TABLE_TOP + 0.06))
DIAGNOSTIC_ONLY = True
LAYOUT_NAMES = ("straight", "preformed")


def _equal_chords(path: np.ndarray, step: float) -> np.ndarray:
    """Advance to each next sphere/path intersection, preserving link length."""
    path = np.asarray(path, dtype=np.float64)
    nodes = [path[0].copy()]
    cursor = path[0].copy()
    edge = 0
    while edge < len(path) - 1:
        delta = path[edge + 1] - cursor
        distance = float(np.linalg.norm(delta))
        if distance < 1e-12:
            edge += 1
            if edge < len(path) - 1:
                cursor = path[edge].copy()
            continue
        direction = delta / distance
        relative = cursor - nodes[-1]
        projection = float(relative @ direction)
        discriminant = projection**2 + step**2 - float(relative @ relative)
        travel = -projection + math.sqrt(max(0.0, discriminant)) if discriminant >= -1e-12 else math.inf
        if 1e-10 < travel <= distance + 1e-10:
            cursor = cursor + min(travel, distance) * direction
            nodes.append(cursor.copy())
        else:
            edge += 1
            if edge < len(path) - 1:
                cursor = path[edge].copy()
    return np.asarray(nodes)


def diagnostic_nodes(name: str) -> np.ndarray:
    """Return a NEW array of diagnostic nodes. Never call from task.initial_nodes.

    Both fixtures start elevated for a deliberate gravity-settling experiment.
    ``straight`` has 72 links, matching ``preformed`` for comparative probes.
    ``preformed`` contains a pre-existing knot candidate and is unsuitable
    as any tying demonstration's initial condition.
    """
    name = "preformed" if name == "loose-overhand" else name
    if name == "straight":
        return CENTRE + np.column_stack((np.zeros(73), np.linspace(-.36, .36, 73), np.zeros(73)))
    if name != "preformed":
        raise ValueError(f"Unknown diagnostic layout {name!r}; choose one of {LAYOUT_NAMES}")
    t = np.linspace(.12, 2 * math.pi - .12, 2501)
    curve = .018 * np.column_stack(((2 + np.cos(3*t)) * np.cos(2*t),
                                    (2 + np.cos(3*t)) * np.sin(2*t), np.sin(3*t)))
    # Tails leave the cut side and splay outward by .35 radians, giving separate
    # loading ends approximately 89 mm apart and at least 80 mm straight tails.
    tail = .095 * np.array((math.cos(.35), math.sin(.35), 0.0))
    path = np.vstack((curve[0] + tail, curve, curve[-1] + tail * (1., -1., 1.)))
    nodes = _equal_chords(path, SEGMENT) + CENTRE
    lengths = np.linalg.norm(np.diff(nodes, axis=0), axis=1)
    if len(nodes) != 73 or not np.allclose(lengths, SEGMENT, rtol=1e-8, atol=1e-11):
        raise AssertionError("Diagnostic geometry no longer has its validated 72 equal links")
    return nodes


def geometry_report(name: str) -> dict:
    """Compute geometric evidence. This report deliberately proves no physics."""
    from topology import analyze_open_rope, _pair_distances

    name = "preformed" if name == "loose-overhand" else name
    nodes = diagnostic_nodes(name)
    vectors = np.diff(nodes, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    directions = vectors / lengths[:, None]
    bends = np.arccos(np.clip(np.einsum("ij,ij->i", directions[:-1], directions[1:]), -1., 1.))
    i, j = np.triu_indices(len(vectors), k=2)
    distances = _pair_distances(nodes[i], nodes[i+1], nodes[j], nodes[j+1])
    closest = int(np.argmin(distances))
    first_turn = next((i for i, d in enumerate(directions) if d @ directions[0] < 1 - 1e-10), len(directions))
    last_turn = next((i for i, d in enumerate(directions[::-1]) if d @ directions[-1] < 1 - 1e-10), len(directions))
    result = analyze_open_rope(nodes, DIAMETER, expected_segment_lengths=SEGMENT, stretch_tolerance=1e-6)
    report = {
        "name": name, "diagnostic_only": True,
        "prohibited_use": "Task starting layout or robotic tying demonstration",
        "preformed_knot": name == "preformed", "physical_retention_proven": False,
        "link_count": len(vectors), "segment_length_m": SEGMENT, "diameter_m": DIAMETER,
        "length_m": float(lengths.sum()),
        "maximum_link_length_error_m": float(np.max(np.abs(lengths - SEGMENT))),
        "maximum_bend_radians": float(bends.max()),
        "minimum_nonadjacent_axis_distance_m": float(distances[closest]),
        "minimum_nonadjacent_pair": [int(i[closest]), int(j[closest])],
        "minimum_capsule_surface_clearance_m": float(distances[closest] - DIAMETER),
        "bounds_min_m": nodes.min(axis=0).tolist(), "bounds_max_m": nodes.max(axis=0).tolist(),
        "minimum_capsule_table_clearance_m": float(nodes[:, 2].min() - DIAMETER/2 - TABLE_TOP),
        "endpoint_separation_m": float(np.linalg.norm(nodes[-1] - nodes[0])),
        "topology_status": result["status"], "is_overhand": result["is_overhand"],
        "topology_reason": result["reason"], "valid_closures": result.get("valid_closure_count", 0),
        "observed_determinants": result.get("observed_determinants", []),
        "estimated_tail_lengths_m": result.get("tail_lengths_m"),
        "straight_end_sections_m": [float(lengths[:first_turn].sum()), float(lengths[len(lengths)-last_turn:].sum())],
        "nonlocal_geometry": result.get("geometry"),
        "closure_certificates": [{"variant": c["variant"], "valid": c["valid"],
                                   "trefoil_certificates": c.get("trefoil_certificate_count", 0),
                                   "unknot_certificates": c.get("unknot_certificate_count", 0)}
                                  for c in result.get("closures", [])],
    }
    if name == "preformed":
        if result["status"] != "overhand_candidate" or result["is_overhand"] is not True:
            raise AssertionError("Preformed diagnostic lost its exterior-closure candidate certificate")
        if min(result["tail_lengths_m"]) < .08:
            raise AssertionError("Diagnostic tails are shorter than 8 cm")
        if min(report["straight_end_sections_m"]) < .08 - 1e-10 or report["endpoint_separation_m"] < .08:
            raise AssertionError("Diagnostic straight tails or separated loading ends are shorter than 8 cm")
        if result["geometry"]["minimum_nonlocal_distance_m"] < .01:
            raise AssertionError("Nonlocal diagnostic axes are closer than 10 mm")
    elif result["status"] != "unknot":
        raise AssertionError("Straight diagnostic should certify as unknotted")
    if distances.min() < DIAMETER or bends.max() >= 1.5:
        raise AssertionError("Diagnostic violates capsule clearance or rope joint limits")
    return report


if __name__ == "__main__":
    print(json.dumps([geometry_report(name) for name in LAYOUT_NAMES], indent=2, allow_nan=False))
