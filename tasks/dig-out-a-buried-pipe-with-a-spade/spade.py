"""Physical spade built from the dimensions and rest pose in Claude's digplan.

No external asset or simulator import is needed. The dished blade is a compound
of thin convex panels: taking one convex hull of the entire blade would fill its
concavity. The shaft and open D-handle belong to the same dynamic rigid body.
"""

import bodykit
import digplan as design
import numpy as np
from bodykit import Body, BoxShape, CylShape, MeshShape, segment
from yam_arena.ik import matrix_to_quat_xyzw

STEEL = (0.43, 0.47, 0.50)
ASH = (0.68, 0.48, 0.27)
GRIP = (0.20, 0.24, 0.21)
bodykit.FINISH[STEEL] = (0.32, 0.85)


def _panel(top):
    """Closed convex plate, with outward-facing triangles and no collider seams."""
    top = np.asarray(top, dtype=float)
    if np.cross(top[1] - top[0], top[2] - top[0])[2] < 0:
        top = top[::-1]
    normal = np.array([-np.sin(design.LIFT), 0.0, np.cos(design.LIFT)])
    points = np.vstack((top, top - design.BLADE_T * normal))
    n = len(top)
    faces = [(0, i, i + 1) for i in range(1, n - 1)]
    faces += [(n, n + i + 1, n + i) for i in range(1, n - 1)]
    for i in range(n):
        j = (i + 1) % n
        faces.extend(((i, n + i, n + j), (i, n + j, j)))
    return MeshShape(
        tuple(map(tuple, points)), tuple(faces), STEEL, friction=(0.35, 0.25)
    )


def blade_shapes():
    # Include the point where the tapered outline meets the flat centre, so
    # triangles on either side of each blade crease remain planar.
    crossing = 0.070 + (design.BLADE_LEN - 0.070) * (
        1 - (design.FLAT_HALF / (design.BLADE_W / 2)) ** 2
    ) ** (1 / 1.6)
    stations = sorted(
        [
            0.0,
            0.0125,
            0.025,
            0.035,
            0.070,
            0.090,
            crossing,
            0.115,
            0.122,
            design.BLADE_LEN,
        ]
    )
    normal = np.array([-np.sin(design.LIFT), 0.0, np.cos(design.LIFT)])

    def flat_point(s, w):
        return (
            design.blade_point(s, w)
            - design.HEEL_RISE * max(0.0, 1 - s / 0.025) * normal
        )

    # One collider per planar region prevents small grains catching on internal
    # edges between the triangles of the same flat surface.
    floor = [
        flat_point(s, -min(design.FLAT_HALF, design.half_width(s))) for s in stations
    ]
    floor += [
        flat_point(s, min(design.FLAT_HALF, design.half_width(s)))
        for s in stations[-2::-1]
    ]
    shapes = [_panel(floor)]
    wing_stations = [s for s in stations if s < crossing] + [crossing]
    for sign in (-1, 1):
        wing = [
            flat_point(0, sign * design.FLAT_HALF),
            flat_point(crossing, sign * design.FLAT_HALF),
        ]
        wing += [
            flat_point(s, sign * design.half_width(s)) for s in wing_stations[-2::-1]
        ]
        shapes.append(_panel(wing))
    shapes.append(
        _panel(
            [
                design.blade_point(s, w)
                for s, w in (
                    (0, -design.FLAT_HALF),
                    (0.025, -design.FLAT_HALF),
                    (0.025, design.FLAT_HALF),
                    (0, design.FLAT_HALF),
                )
            ]
        )
    )
    return shapes


def build_spade():
    shapes = blade_shapes()
    for (a, b, radius), color in ((design.SOCKET, STEEL), (design.SHAFT, ASH)):
        shapes.append(CylShape(((a + b) / 2, 0.0, 0.0), radius, b - a, color, axis="X"))
    for side in (-1, 1):
        shapes.append(
            segment(
                (design.D_JOIN, side * design.SHAFT[2], 0.0),
                (design.D_BAR, side * design.D_HALF, 0.0),
                2 * design.D_R,
                GRIP,
            )
        )
    shapes.append(
        CylShape((design.D_BAR, 0.0, 0.0), design.D_R, 2 * design.D_HALF, ASH, axis="Y")
    )
    origin, rotation = design.REST_POSE
    # Flat sleeves resist shaft roll with parallel fingers. They are ordinary
    # collision geometry; nothing attaches the tool to the robot.
    for arm, (point, grip) in design.grips_local().items():
        size = (0.032, 0.050, 0.032) if arm == "right" else (0.026, 0.065, 0.026)
        shapes.append(
            BoxShape(
                tuple(point),
                size,
                GRIP,
                quat_xyzw=tuple(float(v) for v in matrix_to_quat_xyzw(grip)),
            )
        )
    return Body.build(
        "spade",
        pos=origin,
        mass=design.SPADE_MASS,
        shapes=shapes,
        start_xyzw=tuple(float(v) for v in matrix_to_quat_xyzw(rotation)),
        com=design.SPADE_COM,
        friction=(1.2, 1.0),
        contact_offset=0.001,
        solver_iterations=32,
        solver_velocity_iterations=4,
    )


def build_rests():
    """Two fixed blocks support the loose tool above the table for grasping."""
    origin, rotation = design.REST_POSE
    shaft = origin + rotation @ np.array([-0.16, 0.0, 0.0])
    blade = origin + rotation @ design.blade_point(0.040, 0.0)
    shaft_top = shaft[2] - design.SHAFT[2] - 0.002
    blade_top = blade[2] - design.BLADE_T - 0.002
    shapes = []
    for centre, top in ((shaft, shaft_top), (blade, blade_top)):
        height = float(top - design.TOP)
        shapes.append(
            BoxShape(
                (float(centre[0]), float(centre[1]), height / 2),
                (0.025, 0.080, height),
                ASH,
                yaw=design.YAW,
            )
        )
    return Body.build(
        "spade_rests",
        pos=(0.0, 0.0, design.TOP),
        mass=1.0,
        kinematic=True,
        shapes=shapes,
    )
