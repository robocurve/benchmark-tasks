"""Scoring from the simulation state. NumPy only, so it can be tested offline (test_scoring.py).

Score (judged at the end of the trial; see DESIGN.md for the real-trial version of each check):

- 1.0  one tortilla is all the way out of the bag and lies flat on the plate (at least 90 % of it over the plate's
       well-and-rim circle, its footprint at least 85 % of its area, so not folded over, at rest), the other five are
       still in the bag (no more than a quarter of any of them past the zip), and both hands are open, holding
       nothing and at least 10 cm from it;
- 0.75 one tortilla is all the way out, the others are inside, but it is not lying flat on the plate (or a hand
       still holds something);
- 0.5  one tortilla is at least half out of the bag past the zip and the others are inside;
- 0.25 the zip is open along at least 150 mm of its 240 mm;
- 0    otherwise. Two or more tortillas at least half out at the end scores 0 whatever else happened.
"""

from __future__ import annotations

import numpy as np

import layout as L

ZIP_OPEN_FOR_CREDIT = 0.150  # m of zip open for 0.25
FLAT_AREA = 0.85
ON_PLATE = 0.90
REST_SPEED = 0.03  # m/s, the fastest vertex
HANDS_CLEAR = 0.10  # m from each fingertip to the tortilla on the plate, for full credit
OTHERS_OUT_MAX = 0.25  # the other tortillas count as still in the bag while no more than a quarter of each is past the zip


def zip_line(bottom_points: np.ndarray, zip_bottom_idx: np.ndarray):
    """The zip's current line as (y samples, x samples), sorted by y, from the bottom film's zip vertices."""
    z = bottom_points[zip_bottom_idx]
    o = np.argsort(z[:, 1])
    return z[o, 1], z[o, 0]


def fraction_out(points: np.ndarray, zy: np.ndarray, zx: np.ndarray, bottom_points: np.ndarray | None = None) -> float:
    """The share of a tortilla's vertices outside the bag: past the zip (toward the arms), or beside the bag, or
    behind its bottom seal, or more than 60 mm above the bag's bottom film."""
    x_zip = np.interp(points[:, 1], zy, zx)
    inside = points[:, 0] >= x_zip - 0.002
    inside &= (points[:, 1] >= zy.min() - 0.006) & (points[:, 1] <= zy.max() + 0.006)
    if bottom_points is not None:
        inside &= points[:, 0] <= bottom_points[:, 0].max() + 0.005
        inside &= points[:, 2] <= np.median(bottom_points[:, 2]) + 0.060
    return float(np.mean(~inside))


def projected_area(points: np.ndarray, faces: np.ndarray, cell: float = 0.002) -> float:
    """Area of the tortilla's footprint seen from above (the union of its triangles, on a 2 mm grid): a fold
    covers less area than the tortilla has."""
    p = points[:, :2]
    lo = p.min(axis=0)
    n = np.ceil((p.max(axis=0) - lo) / cell).astype(int) + 1
    grid = np.zeros(n, bool)
    tri = p[faces]
    for t in tri:
        a, b, c = t
        tlo = np.floor((t.min(axis=0) - lo) / cell).astype(int)
        thi = np.ceil((t.max(axis=0) - lo) / cell).astype(int)
        ii, jj = np.meshgrid(np.arange(tlo[0], thi[0] + 1), np.arange(tlo[1], thi[1] + 1), indexing="ij")
        q = lo + (np.stack([ii, jj], -1) + 0.5) * cell
        v0, v1, v2 = c - a, b - a, q - a
        d00, d01, d11 = v0 @ v0, v0 @ v1, v1 @ v1
        den = d00 * d11 - d01 * d01
        if abs(den) < 1e-18:
            continue
        d20, d21 = v2 @ v0, v2 @ v1
        u = (d11 * d20 - d01 * d21) / den
        v = (d00 * d21 - d01 * d20) / den
        m = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
        ii, jj = ii[m], jj[m]
        ok = (ii < n[0]) & (jj < n[1])
        grid[ii[ok], jj[ok]] = True
    return float(grid.sum() * cell * cell)


def true_area(points: np.ndarray, faces: np.ndarray) -> float:
    a = points[faces[:, 1]] - points[faces[:, 0]]
    b = points[faces[:, 2]] - points[faces[:, 0]]
    return float(0.5 * np.linalg.norm(np.cross(a, b), axis=1).sum())


def on_plate(points, plate_pos) -> float:
    """The share of the tortilla's vertices over the plate (inside its rim circle) and low on it (within 30 mm)."""
    d = np.hypot(points[:, 0] - plate_pos[0], points[:, 1] - plate_pos[1])
    low = points[:, 2] < plate_pos[2] + L.PLATE_H + 0.030
    return float(np.mean((d <= L.PLATE_R) & low))


def evaluate(tortillas: list[np.ndarray], tortilla_faces: list[np.ndarray], tortilla_speed: list[float],
             bottom_points: np.ndarray, zip_bottom_idx: np.ndarray, zip_open_u: list[float], plate_pos,
             hands_open: bool, hands_empty: bool, tips=()) -> dict:
    zy, zx = zip_line(bottom_points, zip_bottom_idx)
    out = [fraction_out(p, zy, zx, bottom_points) for p in tortillas]
    best = int(np.argmax(out))
    others = [o for i, o in enumerate(out) if i != best]
    others_in = max(others) <= OTHERS_OUT_MAX if others else True
    p = tortillas[best]
    flat = projected_area(p, tortilla_faces[best]) / max(1e-9, true_area(p, tortilla_faces[best]))
    plate = on_plate(p, plate_pos)
    resting = tortilla_speed[best] < REST_SPEED
    zip_open = _open_length(zip_open_u)
    half_out = sum(o >= 0.5 for o in out)
    full = out[best] >= 1.0
    lies = plate >= ON_PLATE and flat >= FLAT_AREA and resting
    clear = all(np.linalg.norm(p - np.asarray(t), axis=1).min() >= HANDS_CLEAR for t in tips)
    if half_out >= 2:
        score = 0.0
    elif full and others_in and lies and hands_open and hands_empty and clear:
        score = 1.0
    elif full and others_in:
        score = 0.75
    elif out[best] >= 0.5 and others_in:
        score = 0.5
    elif zip_open >= ZIP_OPEN_FOR_CREDIT:
        score = 0.25
    else:
        score = 0.0
    return {"score": score, "out": [round(o, 3) for o in out], "best": best, "flat": round(flat, 3),
            "on_plate": round(plate, 3), "resting": bool(resting), "zip_open_mm": round(zip_open * 1000, 1),
            "others_in": bool(others_in), "hands_open": bool(hands_open), "hands_empty": bool(hands_empty), "hands_clear": bool(clear)}


def _open_length(us: list[float]) -> float:
    """Length of zip open: each open cell counts for the zip length it covers (cells are one grid column wide)."""
    if not us:
        return 0.0
    pitch = (L.IN_W + 2 * L.SEAL_W) / (int(round((L.IN_W + 2 * L.SEAL_W) / L.EDGE)))
    return min(L.IN_W, len(us) * pitch)
