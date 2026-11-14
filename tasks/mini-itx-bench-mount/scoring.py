"""Geometric seating score for an open Mini-ITX test bench."""
import math
import numpy as np
import geometry as g


def _rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < .99:
        return None
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def measures(board):
    p = np.asarray(board.pos, dtype=float)
    r = _rotation(board.quat_xyzw)
    if p.shape != (3,) or not np.isfinite(p).all() or r is None:
        return None
    hole_error = []
    hole_axis_error = []
    for dx, dy in g.HOLES:
        world = p + r @ np.array((dx, dy, 0.0))
        target = np.array((g.BENCH_XY[0]+dx, g.BENCH_XY[1]+dy))
        offset = world[:2] - target
        hole_error.append(float(np.linalg.norm(offset)))
        hole_axis_error.append(float(np.max(np.abs(offset))))
    port = p + r @ np.array((.106, 0.0, .011))
    return {
        "center_error": float(np.linalg.norm(p[:2]-g.BOARD_GOAL[:2])),
        "max_hole_error": max(hole_error),
        "max_hole_axis_error": max(hole_axis_error),
        "board_bottom_error": abs(float(p[2]-g.BOARD_THICKNESS/2-(g.TOP+.036))),
        "tilt_deg": math.degrees(math.acos(float(np.clip(r[2, 2], -1, 1)))),
        "yaw_deg": math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0]))),
        "port_tip_x": float(port[0]),
        "port_center_y": float((p+r@np.array((.087,0.,.011)))[1]),
    }


def grade(objects, hands_clear):
    board = objects["motherboard"]
    m = measures(board)
    if m is None or m["tilt_deg"] > 8 or abs(m["yaw_deg"]) > 12:
        return 0.0
    if m["center_error"] > .020:
        return 0.0
    if m["max_hole_error"] > .004 or m["board_bottom_error"] > .006:
        return .25
    # A round pin fits a square aperture when its centre remains within
    # half the aperture width minus the pin radius on both axes.
    if m["max_hole_axis_error"] > g.HOLE_SIDE/2-g.PIN_RADIUS \
       or m["board_bottom_error"] > .002 \
       or m["tilt_deg"] > 3 or abs(m["yaw_deg"]) > 3:
        return .5
    # The rear I/O housing must enter the wall opening in the correct direction.
    if m["port_tip_x"] < g.BENCH_XY[0]+.103 or abs(m["port_center_y"]) > .030:
        return .5
    if not board.resting(.025, .5) or not hands_clear:
        return .75
    return 1.0
