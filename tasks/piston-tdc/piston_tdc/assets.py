"""USD bodies of the piston-stop engine trainer (original simplified geometry). Import after Isaac Sim starts.

Bodies and frames (all origins listed in design.start_poses):
    frame   kinematic: base plate, opaque housing (walls, lid with the stop slot), cylinder head wall, pointer
    rotor   crankshaft: hidden throw and pin (visual), degree wheel, handle arm (colliders above the lid)
    handle  the orange handle sleeve, spinning on its pin (own revolute joint)
    rod     connecting rod, origin at the big end (hidden, no colliders)
    piston  piston, origin at the wrist pin (hidden; collides with the stop)
    stop    loose T-shaped piston stop: shank in the slot, plate resting on the lid, grip block on top
"""

from __future__ import annotations

import math
from pathlib import Path

from . import design as d
from .usd_bodies import Body

assert tuple(d.U) == (0.0, -1.0), "the shapes below assume the cylinder points toward -y"

DARK, HOUSING, STEEL = (0.18, 0.19, 0.21), (0.16, 0.22, 0.34), (0.55, 0.57, 0.60)
YELLOW, BLACK, RED, BLUE = (0.98, 0.82, 0.10), (0.03, 0.03, 0.03), (0.85, 0.08, 0.08), (0.10, 0.30, 0.90)
ORANGE, GREEN, WHITE = (0.98, 0.45, 0.05), (0.05, 0.70, 0.20), (0.92, 0.92, 0.92)


def frame() -> Path:
    b = Body("frame", mass=3.0, friction=d.FRICTION["frame"], kinematic=True)
    hx, back, front = d.HOUSE_HALF_X, d.HOUSE_BACK, d.HOUSE_FRONT
    lid0, lid1 = d.LID_Z
    b.box((0.0, (back - front) / 2, d.PLATE_T / 2), (2 * hx + 0.018, back + front + 0.018, d.PLATE_T), DARK)
    wall_h = lid0 - d.PLATE_T
    for sx in (-1, 1):  # side walls
        b.box((sx * (hx - d.WALL / 2), (back - front) / 2, d.PLATE_T + wall_h / 2), (d.WALL, back + front, wall_h),
              HOUSING)
    b.box((0.0, back - d.WALL / 2, d.PLATE_T + wall_h / 2), (2 * hx, d.WALL, wall_h), HOUSING)  # crank end
    head_h = lid1 - d.PLATE_T  # cylinder head wall, up to the lid top: carries the stop's load
    b.box((0.0, -(d.HEAD_S + d.HEAD_T / 2), d.PLATE_T + head_h / 2), (2 * hx, d.HEAD_T, head_h), HOUSING)
    slot_near = d.STOP_FACE_S - d.SLOT_CLEAR  # slot edge on the piston side
    lid_len = back + slot_near
    b.box((0.0, (back - slot_near) / 2, (lid0 + lid1) / 2), (2 * hx, lid_len, lid1 - lid0), HOUSING)
    half_slot = d.STOP_W / 2 + d.SLOT_CLEAR
    side_w = hx - half_slot
    for sx in (-1, 1):  # lid beside the slot
        b.box((sx * (half_slot + side_w / 2), -(slot_near + d.HEAD_S) / 2, (lid0 + lid1) / 2),
              (side_w, d.HEAD_S - slot_near, lid1 - lid0), HOUSING)
    # pointer: a post outside the wheel rim and a green needle pointing at the rim
    a = math.radians(d.POINTER_WORLD_DEG)
    ca, sa = math.cos(a), math.sin(a)
    rp = d.WHEEL_R + 0.014
    b.box((rp * ca, rp * sa, d.WHEEL_Z[1] / 2), (0.010, 0.010, d.WHEEL_Z[1]), DARK, yaw=a)
    tip, base, hw = d.WHEEL_R + 0.002, d.WHEEL_R + 0.020, 0.006
    pts = [(tip * ca, tip * sa), (base * ca - hw * sa, base * sa + hw * ca), (base * ca + hw * sa, base * sa - hw * ca)]
    b.prism_z(pts, d.WHEEL_Z[1] - 0.001, d.WHEEL_Z[1] + 0.002, GREEN, collide=False)
    return b.save()


def rotor() -> Path:
    zc = d.Z_MECH
    b = Body("rotor", mass=d.MASS["rotor"], friction=d.FRICTION["rotor"], com=(0.0, 0.0, 0.03),
             inertia=d.INERTIA["rotor"])
    top = d.ARM_Z[1] - zc
    b.cylinder((0.0, 0.0, (top - 0.020) / 2), 0.008, top + 0.020, STEEL, collide=False)  # shaft
    b.box((d.R / 2, 0.0, 0.0), (d.R + 0.02, 0.016, 0.010), STEEL, collide=False)  # crank web (hidden)
    b.box((-0.015, 0.0, 0.0), (0.030, 0.050, 0.010), STEEL, collide=False)  # counterweight (hidden)
    b.cylinder((d.R, 0.0, 0.0), 0.006, 0.014, STEEL, collide=False)  # crank pin (hidden)
    wz0, wz1 = d.WHEEL_Z[0] - zc, d.WHEEL_Z[1] - zc
    b.cylinder((0.0, 0.0, (wz0 + wz1) / 2), d.WHEEL_R, wz1 - wz0, YELLOW)  # degree wheel
    for k in range(0, 360, d.TICK_EVERY_DEG):
        a = math.radians(d.WHEEL_ZERO_LOCAL_DEG + k)
        if k == 0:
            r0, w, color = d.WHEEL_R - 0.026, 0.004, RED
        elif k % 90 == 0:
            r0, w, color = d.WHEEL_R - 0.018, 0.003, BLUE
        else:
            r0, w, color = d.WHEEL_R - 0.012, 0.0015, BLACK
        r1 = d.WHEEL_R - 0.001
        rm = (r0 + r1) / 2
        b.box((rm * math.cos(a), rm * math.sin(a), wz1 + 0.0005), (r1 - r0, w, 0.001), color, yaw=a, collide=False)
    h = math.radians(d.HANDLE_OFFSET_DEG)
    az0, az1 = d.ARM_Z[0] - zc, d.ARM_Z[1] - zc
    reach = d.HANDLE_RADIUS + 0.010
    b.box((reach / 2 * math.cos(h), reach / 2 * math.sin(h), (az0 + az1) / 2), (reach + 0.016, 0.016, az1 - az0),
          DARK, yaw=h)  # handle arm
    b.cylinder((0.0, 0.0, (az0 + az1) / 2), 0.014, az1 - az0 + 0.002, DARK, collide=False)  # hub
    s = d.sleeve_local()
    pin_h = d.SLEEVE_Z0 - d.ARM_Z[1] + d.SLEEVE_H
    b.cylinder((s[0], s[1], az1 + pin_h / 2), 0.004, pin_h, STEEL, collide=False)  # handle pin
    return b.save()


def handle() -> Path:
    b = Body("handle", mass=d.MASS["handle"], friction=d.FRICTION["handle"], com=(0.0, 0.0, 0.0),
             inertia=d.INERTIA["handle"])
    b.cylinder((0.0, 0.0, 0.0), d.SLEEVE_R, d.SLEEVE_H, ORANGE)
    return b.save()


def rod() -> Path:
    b = Body("rod", mass=d.MASS["rod"], com=(d.L / 2, 0.0, 0.0), inertia=d.INERTIA["rod"])
    b.box((d.L / 2, 0.0, 0.0), (d.L, 0.012, 0.008), STEEL, collide=False)
    b.cylinder((0.0, 0.0, 0.0), 0.011, 0.012, STEEL, collide=False)
    b.cylinder((d.L, 0.0, 0.0), 0.007, 0.010, STEEL, collide=False)
    return b.save()


def piston() -> Path:
    b = Body("piston", mass=d.MASS["piston"], friction=d.FRICTION["piston"], com=(0.0, -0.005, 0.0),
             inertia=d.INERTIA["piston"])
    w, length, h = d.PISTON
    b.box((0.0, (d.SKIRT - d.CROWN) / 2, 0.0), (w, length, h), STEEL)
    return b.save()


def stop() -> Path:
    b = Body("stop", mass=d.STOP_MASS, friction=d.FRICTION["stop"])  # inertia from its colliders
    z0, z1 = d.STOP_SHANK_Z
    mid = (z0 + z1) / 2
    b.box((0.0, 0.0, 0.0), (d.STOP_W, d.STOP_T, z1 - z0), RED)
    pz = z1 + d.STOP_PLATE[2] / 2 - mid
    b.box((0.0, 0.0, pz), d.STOP_PLATE, RED)
    gz = z1 + d.STOP_PLATE[2] + d.STOP_GRIP[2] / 2 - mid
    b.box((0.0, 0.0, gz), d.STOP_GRIP, RED)
    return b.save()


BUILDERS = {"frame": frame, "rotor": rotor, "handle": handle, "rod": rod, "piston": piston, "stop": stop}
