"""Geometric seating score for the Mini-ITX assembly fixture."""
import math
import numpy as np
import geometry as g

# Every check below is one a person can make on the real fixture by eye or with a feeler gauge (review TK-WMXND9):
# a pin is engaged when its axis is inside its hole at both PCB faces, which a real trial reads as the pin's tip seen
# through the hole from above. The pins and PCB collide, so a board resting on all four standoffs has all four pins
# through its holes; the 0.2 mm on the 0.78 mm radial clearance covers the collision strips that approximate each
# round hole, and is not a measurement limit.
ENGAGED = (g.HOLE_RADIUS - g.PIN_RADIUS) + .0002
# Seated: a 1.0 mm feeler gauge does not enter between the board's underside and the top of any standoff.
SEATED = .001


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
    support_error = []
    for dx, dy in g.HOLES:
        world = p + r @ np.array((dx, dy, 0.0))
        target = np.array((g.BENCH_XY[0]+dx, g.BENCH_XY[1]+dy))
        # A tilted board can clear a pin at one PCB face but strike it at the
        # other. Both faces must fit inside the round aperture.
        for face in (-g.BOARD_THICKNESS/2, g.BOARD_THICKNESS/2):
            face_world = p + r @ np.array((dx, dy, face))
            hole_error.append(float(np.linalg.norm(face_world[:2] - target)))
        underside = p + r @ np.array((dx, dy, -g.BOARD_THICKNESS/2))
        support_error.append(abs(float(underside[2] - (g.TOP + .036))))
    port = p + r @ np.array((.106, 0.0, .011))
    return {
        "center_error": float(np.linalg.norm(p[:2]-g.BOARD_GOAL[:2])),
        "max_hole_error": max(hole_error),
        "max_support_height_error": max(support_error),
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
    if m["max_hole_error"] > .002 or m["max_support_height_error"] > .006:
        return .25
    # All four pins engaged and the board seated on all four standoffs.
    if m["max_hole_error"] > ENGAGED \
       or m["max_support_height_error"] > SEATED \
       or m["tilt_deg"] > 3 or abs(m["yaw_deg"]) > 3:
        return .5
    # The rear I/O housing must enter the wall opening in the correct direction.
    if m["port_tip_x"] < g.BENCH_XY[0]+.103 or abs(m["port_center_y"]) > .030:
        return .5
    if not board.resting(.025, .5) or not hands_clear:
        return .75
    return 1.0
