"""Near-successes and unintended strategies, each with the score task.py must give it. Cases run in order against one
copy of the task module, so the whole-trial record (a roll that fell from a face onto the table) carries from case to case
until a scene reset puts both rolls back at their start."""

import math

import numpy as np

import parts
from task import (FACE_A, FACE_B, FACE_T, R, ROLL_A, ROLL_B, ROOF, SLOPE, TOP)

UP = {"face_a": np.array([1.0, 0.0, 0.0]), "face_b": np.array([0.0, 1.0, 0.0])}  # up the slope, link frame
ACROSS = {"face_a": np.array([0.0, 1.0, 0.0]), "face_b": np.array([-1.0, 0.0, 0.0])}
HINGE = {"face_a": FACE_A, "face_b": FACE_B}


def roof(a=SLOPE, b=SLOPE):
    return ROOF.state(joints={"face_a": a, "face_b": b})


def on_face(roll, face, u=-0.11, v=0.0, yaw_deg=0.0, lift=0.0, vel=None, pitch=SLOPE, stand=False):
    """The roll lying on the face, its centre u m along the slope from the high edge (negative is down), v m across,
    its axis turned yaw_deg in the face plane from the fall line, lifted lift m off the face; stand: on its end."""
    h = HINGE[face]
    rot = parts._rot(h.axis, pitch)
    U, V = UP[face], ACROSS[face]
    W = np.cross(U, V)
    y = math.radians(yaw_deg)
    axis_l = math.cos(y) * U + math.sin(y) * V
    if stand:
        axis_l, side = W, U
        centre_l = u * U + v * V + (FACE_T / 2 + 0.07 + lift) * W
    else:
        side = W
        centre_l = u * U + v * V + (FACE_T / 2 + R + lift) * W
    by = rot @ axis_l  # the roll's body y (its axis) in the world
    bz = rot @ side
    bx = np.cross(by, bz)
    m = np.column_stack([bx, by, bz])
    pos = np.asarray(h.pivot, float) + np.array([0.0, 0.0, TOP]) + rot @ centre_l
    spec = ROLL_A if roll == "roll_left" else ROLL_B
    return spec.state(pos=pos, quat_xyzw=parts._matrix_to_quat(m), lin_vel=vel)


def on_table(roll, x=0.25, y=0.0):
    spec = ROLL_A if roll == "roll_left" else ROLL_B
    return spec.state(pos=(x, y, TOP + R))


DONE = {"roof": roof(), "roll_left": on_face("roll_left", "face_a"), "roll_right": on_face("roll_right", "face_b")}

SINGLE = [
    ("both rolls along their faces' fall lines", DONE, 1.0, True),
    ("either roll on either face (swapped)", {"roof": roof(), "roll_left": on_face("roll_left", "face_b"),
                                             "roll_right": on_face("roll_right", "face_a")}, 1.0, True),
    ("only the left face holds a roll", {"roof": roof(), "roll_left": on_face("roll_left", "face_a")}, 0.0, False),
    ("only the right face holds a roll", {"roof": roof(), "roll_right": on_face("roll_right", "face_b")}, 0.0, False),
    ("both rolls side by side on the left face", {"roof": roof(), "roll_left": on_face("roll_left", "face_a", v=0.04),
                                                 "roll_right": on_face("roll_right", "face_a", v=-0.04)}, 0.0, False),
    ("left roll held across the slope (it would roll off)",
     {**DONE, "roll_left": on_face("roll_left", "face_a", yaw_deg=90.0)}, 0.0, False),
    ("left roll turned 19.9 degrees off the fall line", {**DONE, "roll_left": on_face("roll_left", "face_a",
                                                                                      yaw_deg=19.9)}, 1.0, True),
    ("left roll turned 20.1 degrees off the fall line", {**DONE, "roll_left": on_face("roll_left", "face_a",
                                                                                      yaw_deg=20.1)}, 0.0, False),
    ("left roll's low end exactly at the low edge", {**DONE, "roll_left": on_face("roll_left", "face_a",
                                                                                  u=-0.20 + 0.07)}, 1.0, True),
    ("left roll's low end 1 mm past the low edge", {**DONE, "roll_left": on_face("roll_left", "face_a",
                                                                                 u=-0.20 + 0.069)}, 0.0, False),
    ("right roll's side 1 mm past the face's side edge", {**DONE, "roll_right": on_face("roll_right", "face_b",
                                                                                        v=0.08 - R + 0.001)},
     0.0, False),
    ("right roll's side exactly at the face's side edge", {**DONE, "roll_right": on_face("roll_right", "face_b",
                                                                                         v=0.08 - R)}, 1.0, True),
    ("right roll held 4 mm above its face", {**DONE, "roll_right": on_face("roll_right", "face_b", lift=0.004)},
     0.0, False),
    ("right roll 3 mm above its face (within tolerance)", {**DONE, "roll_right": on_face("roll_right", "face_b",
                                                                                         lift=0.003)}, 1.0, True),
    ("right roll still moving", {**DONE, "roll_right": on_face("roll_right", "face_b", vel=(-0.05, 0.0, 0.0))},
     0.0, False),
    ("right roll stood on its end on its face", {**DONE, "roll_right": on_face("roll_right", "face_b", stand=True)},
     0.0, False),
    ("left face pushed up to 15 degrees", {**DONE, "roof": roof(a=math.radians(15.0))}, 0.0, False),
]

# A whole-trial chain: these build on each other, in order.
CHAIN = [
    ("both rolls on the table, away from the roof", {"roof": roof(), "roll_left": on_table("roll_left", 0.25, 0.0),
                                                     "roll_right": on_table("roll_right", 0.25, -0.30)}, 0.0, False),
    ("then left roll lifted onto its face: never fell off one, counts", DONE, 1.0, True),
    ("left roll rolled off its face onto the table", {**DONE, "roll_left": on_table("roll_left", 0.25, 0.15)},
     0.0, False),
    ("then put back on its face: the fall still counts", DONE, 0.0, False),
    ("then the other roll on the left face instead", {"roof": roof(), "roll_left": on_face("roll_left", "face_b"),
                                                      "roll_right": on_face("roll_right", "face_a")}, 0.0, False),
    ("scene reset: both rolls back at their start", {"roof": roof()}, 0.0, False),
    ("after the reset, done again", DONE, 1.0, True),
    ("scene reset again", {"roof": roof()}, 0.0, False),
    ("left roll laid across its face (it will roll)", {"roof": roof(), "roll_left": on_face("roll_left", "face_a",
                                                                                            yaw_deg=90.0)}, 0.0, False),
    ("it rolls off onto the table", {"roof": roof(), "roll_left": on_table("roll_left", 0.25, 0.15)}, 0.0, False),
    ("then both laid correctly: the fall still counts", DONE, 0.0, False),
]

RESET = ("(scene reset: both rolls back at their start)", {"roof": roof()}, 0.0, False)
# Each single case starts from a fresh scene reset, so no case inherits another's history.
CASES = [c for case in SINGLE for c in (RESET, case)] + [RESET] + CHAIN


def _run() -> int:
    """Score every case in order with task.py's own score() and success(); print and count mismatches."""
    import task

    start = {o.name: o.state() for o in task.OBJECTS if hasattr(o, "state")}
    from yam_arena.task_file import ObjectState

    for o in task.OBJECTS:
        if o.name not in start:
            p = np.asarray(o.pos, float)
            h = np.asarray(o.size, float) / 2
            start[o.name] = ObjectState(p, np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3), np.zeros(3), p - h, p + h)
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start) | changes
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
