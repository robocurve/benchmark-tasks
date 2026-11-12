"""Write the drill and battery-pack USD assets from geometry.py.

    python build_assets.py          # Isaac Sim's Python (pxr, PIL); no simulator needed

- assets/drill.usda: a rigid body: the drill-driver lying on its side, with the T-rail foot (flange, stem, block),
  the notch the pack's catch drops into, and the stop at the rail's back end.
- assets/pack_flat.usda, assets/pack_charged.usda: floating articulations, identical but for the fuel gauge (one red
  bar lit, or four green ones). Root ``pack``: the case, its two rail walls with inward lips; ``latch`` on prismatic
  ``catch`` along x: the tooth standing up out of the top face and the button ahead of the walls, one part, pushed
  out by a spring (task.py's drive).
"""

from __future__ import annotations

from pathlib import Path

import geometry as g
from usdkit import articulation_root, body, box, cylinder, joint, material, no_collide

OUT = Path(__file__).resolve().parent / "assets"


def build_drill() -> None:
    stage = _stage("drill")
    r = "/drill"
    grey = material(stage, f"{r}/Looks/grey", (0.16, 0.17, 0.18), roughness=0.6)
    accent = material(stage, f"{r}/Looks/accent", (0.95, 0.45, 0.08), roughness=0.5)
    steel = material(stage, f"{r}/Looks/steel", (0.55, 0.55, 0.57), roughness=0.3, metallic=0.9)
    body(stage, r, g.DRILL_MASS)
    H, L, W = g.H, g.L, g.W
    y_back = L / 2
    # the T-rail: flange in two pieces either side of the notch, stem, foot block, stop
    fx0, fx1 = g.FLANGE_X
    front = (g.FOOT_Y0, g.FOOT_Y0 + g.FRONT_PIECE)
    rear = (front[1] + g.NOTCH, y_back)
    for n, (y0, y1) in (("front", front), ("rear", rear)):
        _box(stage, f"{r}/flange_{n}", (fx0, fx1), (y0, y1), (-g.FLANGE_Z, g.FLANGE_Z), steel)
    _box(stage, f"{r}/stem", g.STEM_X, (g.FOOT_Y0, y_back), (-g.STEM_Z, g.STEM_Z), steel)
    _box(stage, f"{r}/foot", g.FOOT_BLOCK_X, (g.FOOT_Y0, y_back + g.STOP_T), (-W / 2, W / 2), grey)
    _box(stage, f"{r}/stop", (H / 2 + g.CLR, g.FOOT_BLOCK_X[0]), (y_back + g.CLR, y_back + g.STOP_T), (-W / 2, W / 2),
         grey)
    # handle, motor housing (lying on its side: its lower face on the table, like the foot's), chuck, trigger
    ht = g.HANDLE_T / 2
    _box(stage, f"{r}/handle", g.HANDLE_X, g.HANDLE_Y, (-ht, ht), grey)
    _box(stage, f"{r}/grip_pad", (g.HANDLE_X[0] + 0.01, g.HANDLE_X[1] - 0.01), (g.HANDLE_Y[1], g.HANDLE_Y[1] + 0.002),
         (-ht + 0.003, ht - 0.003), accent, collide=False)
    _box(stage, f"{r}/motor", g.MOTOR_X, g.MOTOR_Y, (-W / 2, -W / 2 + g.MOTOR_T), accent)
    cx = (g.MOTOR_X[0] + g.MOTOR_X[1]) / 2
    cz = -W / 2 + g.MOTOR_T / 2
    cylinder(stage, f"{r}/chuck", g.CHUCK_R, g.CHUCK_LEN, (cx, g.MOTOR_Y[0] - g.CHUCK_LEN / 2, cz), axis="Y",
             mat=grey)
    cylinder(stage, f"{r}/bit", 0.003, 0.05, (cx, g.MOTOR_Y[0] - g.CHUCK_LEN - 0.025, cz), axis="Y", mat=steel,
             collide=False)
    _box(stage, f"{r}/trigger", (g.HANDLE_X[1] - 0.035, g.HANDLE_X[1] - 0.012), (g.HANDLE_Y[0] - 0.012, g.HANDLE_Y[0]),
         (-0.008, 0.008), grey)
    stage.GetRootLayer().Save()


def build_pack(name: str, gauge: tuple) -> None:
    stage = _stage(name)
    r = f"/{name}"
    case = material(stage, f"{r}/Looks/case", (0.13, 0.13, 0.14), roughness=0.7)
    accent = material(stage, f"{r}/Looks/accent", (0.95, 0.45, 0.08), roughness=0.5)
    H, L, W = g.H, g.L, g.W
    p = f"{r}/pack"
    body(stage, p, g.PACK_MASS)
    articulation_root(stage, p, fixed=False)
    _box(stage, f"{p}/case", (-H / 2, H / 2), (-L / 2, L / 2), (-W / 2, W / 2), case)
    _box(stage, f"{p}/band", (-H / 2 + 0.008, -H / 2 + 0.016), (-L / 2 - 0.0005, L / 2 + 0.0005),
         (-W / 2 - 0.0005, W / 2 + 0.0005), accent, collide=False)
    for s in (-1, 1):
        zo = s * (W / 2 - g.WALL_T / 2)
        _box(stage, f"{p}/wall_{'u' if s > 0 else 'd'}", (H / 2, H / 2 + g.WALL_H), (g.WALL_Y0, L / 2),
             (zo - g.WALL_T / 2, zo + g.WALL_T / 2), case)
        zi = s * (W / 2 - g.WALL_T - g.LIP_IN / 2)
        _box(stage, f"{p}/lip_{'u' if s > 0 else 'd'}", (H / 2 + g.WALL_H - g.LIP_T, H / 2 + g.WALL_H),
             (g.WALL_Y0, L / 2), (zi - g.LIP_IN / 2, zi + g.LIP_IN / 2), case)
    # fuel gauge on the front face (the end with the button): four bars, lit as given
    for i, lit in enumerate(gauge):
        z = -0.018 + i * 0.012
        col = lit if lit else (0.08, 0.08, 0.08)
        box(stage, f"{p}/gauge_{i}", (0.004, 0.0012, 0.008), (H / 2 - 0.012, -L / 2 - 0.0006, z), color=col,
            collide=False)
    # the latch: tooth and button in one part; joint position 0 = up (locked), negative = pressed
    k = f"{r}/latch"
    body(stage, k, g.LATCH_MASS, (H / 2, 0, 0))
    tx, ty, tz = g.TOOTH
    box(stage, f"{k}/tooth", (tx, ty, tz), (tx / 2, g.TOOTH_Y, 0), color=(0.55, 0.55, 0.57))
    # its back face (+y, the end that meets the rail first) is a 45-degree ramp: the rail presses the catch down as
    # the pack slides on; its front face is square, so the catch holds against pulling off
    import math

    from usdkit import quat_axis_angle

    t = 0.006
    yb = g.TOOTH_Y + ty / 2                     # the ramp runs from the block's top back corner down to the top face
    mid = (tx / 2, yb + tx / 2)
    n = math.sqrt(0.5)
    box(stage, f"{k}/ramp", (math.hypot(tx, tx), t, tz), (mid[0] - n * t / 2, mid[1] - n * t / 2, 0),
        color=(0.55, 0.55, 0.57), rot_wxyz=quat_axis_angle((0, 0, 1), math.radians(135)))
    bx, by, bz = g.BUTTON
    box(stage, f"{k}/button", (bx + 0.003, by, bz), ((bx - 0.003) / 2, g.BUTTON_Y, g.BUTTON_Z), color=(0.95, 0.45, 0.08))
    box(stage, f"{k}/stem", (0.004, g.TOOTH_Y - g.BUTTON_Y, 0.006), (-0.004, (g.TOOTH_Y + g.BUTTON_Y) / 2, 0.0),
        collide=False, color=(0.3, 0.3, 0.3))
    joint(stage, f"{r}/joints/catch", "linear", p, k, (H / 2, 0, 0), axis="X", limits=(-g.TRAVEL, 0.0))
    no_collide(stage, k, p)
    stage.GetRootLayer().Save()


def _stage(name: str):
    from usdkit import stage_for

    return stage_for(OUT, name)


def _box(stage, path, xr, yr, zr, mat, collide=True):
    size = (xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0])
    pos = ((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, (zr[0] + zr[1]) / 2)
    return box(stage, path, size, pos, mat=mat, collide=collide)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    build_drill()
    build_pack("pack_flat", ((0.9, 0.1, 0.05), None, None, None))
    build_pack("pack_charged", ((0.1, 0.85, 0.2),) * 4)
    for f in sorted(OUT.rglob("*.*")):
        print(f.relative_to(OUT), f.stat().st_size, "bytes")
