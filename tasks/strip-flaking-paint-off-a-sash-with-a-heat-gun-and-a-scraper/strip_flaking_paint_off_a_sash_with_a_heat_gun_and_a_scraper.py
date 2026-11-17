"""On the station table, covered with a grey dust sheet, a wooden window sash lies flat, its glass pane in the middle.
The sash has been stripped back to bare wood except for three patches of old, flaking cream paint along its near
rail. To the left an orange heat gun lies on its wire stand, already running; to the right lies a paint scraper (a
wooden handle and a steel head with a carbide blade sloping forward to its edge) with its handle on a little wooden
rest.

Task: strip the three patches. Hold the heat gun aimed at a patch from a few centimetres away until the paint blisters,
and while it is still hot, push the scraper's blade along the rail through the patch, toward the heat, so the softened paint comes off;
then the next patch, and the next. Never point the heat gun at the glass: heated, it cracks. Then stand the heat gun
back on its stand and lay the scraper back with its handle on its rest, and take both arms back to where they started.
Old paint softens only while it is hot: it blisters after about two seconds of heat and hardens again about two
seconds after the heat leaves it, so the heat gun has to stay on the patch while the other hand scrapes it.

Run it:  inspect-robots run --embodiment yam_arena \\
           -E task=strip_flaking_paint_off_a_sash_with_a_heat_gun_and_a_scraper.py --instruction "..." --scorer task_score
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset,  # noqa: E402
                     on_step, plate, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SHEET = TOP + 0.002  # the dust sheet's top

# -- the sash, lying flat on two bearers ----------------------------------------------------------------------------
SX0, SX1, SY0, SY1 = 0.185, 0.465, -0.170, 0.170
RAIL = 0.045
BEARER_H, SASH_T = 0.0, 0.030  # (laid straight on the dust sheet)
SASH_TOP = SHEET + BEARER_H + SASH_T
GX0, GX1, GY0, GY1 = SX0 + RAIL, SX1 - RAIL, SY0 + RAIL, SY1 - RAIL  # the glass
GLASS_TOP = SASH_TOP - 0.010
NEAR_X = SX0 + RAIL / 2  # the near rail's middle line
# -- the three patches of old paint on the near rail ------------------------------------------------------------------
PATCH_Y = (0.075, 0.000, -0.075)
PATCH_L, PATCH_W = 0.060, 0.034  # along the rail, across it
N_PATCH = len(PATCH_Y)
BINS = 30
STRIPPED_FRAC = 0.8
HEAT_REACH = 0.100  # the gun heats what its nozzle points at within this distance
HEAT_MARGIN = 0.020  # the hot air spreads about this far round where the nozzle points
WARM, COOL, T_MAX, T_BLISTER, T_SET = 0.5, 0.5, 1.6, 1.0, 0.6  # per second; blistered from 1.0 until it falls to 0.6
GLASS_CRACK_S = 1.0
HIDE_X = 0.330  # under the glass, where unused marks wait
# -- the heat gun: its frame's origin on the housing's axis under the hand, +x toward the nozzle ----------------------
PSI = math.radians(40.0)  # it points this far round from -x toward -y (from the far left at the near rail)
AIM_PITCH = math.radians(19.0)  # nose down while heating
# commanded this much less nose-down at each patch (none needed once the grip cleared the sash)
PITCH_SAG = (0.0, 0.0, 0.0)
AIM_DZ = (0.0, 0.006, 0.012)  # the loaded arm sits this much lower at the middle and right patches (measured)
TIP_X = 0.110
GUN_REST = np.array([0.250, 0.250, SHEET + 0.093])  # its stand at the near left, clear of the work
GUN_MASS = 0.40
GUN_GRIP_OFF = np.array([0.0, 0.019, 0.0])  # the gun hangs this far along its y from the left TCP (measured)
GRIP_DZ = 0.010  # the TCP above the housing's axis (the fingers reach 3 cm down its sides)
# -- the scraper: its frame's origin at the middle of the blade's edge, +y from the handle to the blade -------------
SCRAPER_REST = np.array([0.300, -0.212, SHEET])  # on the sheet to the right of the sash
SCRAPER_GRIP = np.array([0.0, -0.090, 0.047])  # the hand on the handle 9 cm behind the blade
SCRAPER_MASS = 0.20
BLADE_W = 0.040

ORANGE, ORANGE_DARK, BLACK, GREY = (0.93, 0.42, 0.08), (0.70, 0.30, 0.05), (0.08, 0.08, 0.08), (0.45, 0.46, 0.48)
STEEL, STEEL_DARK, YELLOW = (0.70, 0.72, 0.75), (0.30, 0.31, 0.33), (0.96, 0.80, 0.12)
WOOD, WOOD_DARK, PAINT, PAINT_OLD = (0.80, 0.63, 0.43), (0.62, 0.46, 0.29), (0.93, 0.91, 0.82), (0.88, 0.83, 0.66)
CRACK, BUBBLE, GLASS, PUTTY = (0.55, 0.50, 0.40), (0.82, 0.74, 0.52), (0.66, 0.80, 0.83), (0.82, 0.76, 0.60)
SHEET_C, SHEET_FOLD, BEECH, RED = (0.55, 0.56, 0.55), (0.48, 0.49, 0.48), (0.78, 0.62, 0.42), (0.75, 0.12, 0.10)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _sheet() -> Body:
    s = [BoxShape((0.0, 0.0, 0.001), (0.400, 0.640, 0.002), SHEET_C)]
    for y in (-0.21, 0.04, 0.23):  # folds in the cloth
        s.append(BoxShape((0.0, y, 0.0021), (0.400, 0.006, 0.0003), SHEET_FOLD, collide=False))
    return Body.build("sheet", pos=(0.310, 0.0, TOP), mass=1.0, kinematic=True, shapes=s, color=SHEET_C,
                      friction=(0.8, 0.7))


def _sash() -> Body:
    """A wooden sash, stripped to bare wood, on two pine bearers; the glass sits in its rebate with putty round it."""
    cx, cy = (SX0 + SX1) / 2, (SY0 + SY1) / 2
    lx, ly = SX1 - SX0, SY1 - SY0
    z0 = BEARER_H
    s = []
    zc = z0 + SASH_T / 2
    for sx in (-1.0, 1.0):
        s.append(BoxShape((sx * (lx - RAIL) / 2, 0.0, zc), (RAIL, ly, SASH_T), WOOD))
        s.append(BoxShape((0.0, sx * (ly - RAIL) / 2, zc), (lx - 2 * RAIL, RAIL, SASH_T), WOOD))
        s.append(BoxShape((sx * (lx / 2 - RAIL - 0.004), 0.0, SASH_T + z0 - 0.0105), (0.008, ly - 2 * RAIL, 0.003),
                          PUTTY, collide=False))
        s.append(BoxShape((0.0, sx * (ly / 2 - RAIL - 0.004), SASH_T + z0 - 0.0105), (lx - 2 * RAIL, 0.008, 0.003),
                          PUTTY, collide=False))
    for k in range(4):  # grain on the near rail
        s.append(BoxShape((-(lx - RAIL) / 2 - 0.012 + 0.008 * k, 0.0, z0 + SASH_T + 0.0002), (0.0012, ly - 0.01, 0.0003),
                          WOOD_DARK, collide=False))
    s.append(BoxShape((0.0, 0.0, GLASS_TOP - SHEET - 0.0015), (lx - 2 * RAIL, ly - 2 * RAIL, 0.003), GLASS))
    s.append(BoxShape((0.02, 0.03, GLASS_TOP - SHEET + 0.0002), (0.08, 0.010, 0.0004), (0.80, 0.90, 0.92),
                      collide=False, yaw=0.6))  # a reflection
    return Body.build("sash", pos=(cx, cy, SHEET), mass=3.0, kinematic=True, shapes=s, color=WOOD,
                      friction=(0.35, 0.30))


def _patch_shapes(k: int, blistered: bool) -> list:
    rng = np.random.default_rng(11 + k)
    s = [BoxShape((0.0, 0.0, 0.0), (PATCH_W, PATCH_L, 0.0012), PAINT_OLD if blistered else PAINT, collide=False)]
    for _ in range(7):  # cracks through the old paint
        a = float(rng.uniform(0, math.pi))
        p = (float(rng.uniform(-0.012, 0.012)), float(rng.uniform(-0.024, 0.024)), 0.0007)
        s.append(BoxShape(p, (0.0008, float(rng.uniform(0.008, 0.020)), 0.0003), CRACK, collide=False, yaw=a))
    if blistered:
        for _ in range(12):
            p = (float(rng.uniform(-0.013, 0.013)), float(rng.uniform(-0.026, 0.026)), -0.001)
            s.append(SphereShape(p, float(rng.uniform(0.0025, 0.0045)), BUBBLE, collide=False))
    else:
        for _ in range(4):  # flakes lifting at the edges
            p = (float(rng.uniform(-0.012, 0.012)), float(rng.uniform(-0.025, 0.025)), 0.0012)
            s.append(BoxShape(p, (0.008, 0.006, 0.0008), (0.96, 0.95, 0.88), collide=False,
                              quat_xyzw=(0.10, 0.0, 0.0, 0.995)))
    return s


def _patches() -> list[Body]:
    out = []
    for k, y in enumerate(PATCH_Y):
        out.append(Body.build(f"paint{k}", pos=(NEAR_X, y, SASH_TOP + 0.0006), mass=0.001, kinematic=True,
                              shapes=_patch_shapes(k, False), color=PAINT))
        out.append(Body.build(f"blister{k}", pos=(HIDE_X, y, SHEET + 0.006), mass=0.001, kinematic=True,
                              shapes=_patch_shapes(k, True), color=PAINT_OLD))
        rng = np.random.default_rng(31 + k)
        flakes = []
        for _ in range(7):  # curled strips of paint on the dust sheet
            p = (float(rng.uniform(-0.012, 0.012)), float(rng.uniform(-0.028, 0.028)), 0.003)
            a = float(rng.uniform(0, math.pi))
            flakes.append(BoxShape(p, (0.010, 0.016, 0.0015), PAINT, collide=False,
                                   quat_xyzw=(0.0, math.sin(0.35), math.sin(a / 2) * 0.3, math.cos(0.35))))
        out.append(Body.build(f"flakes{k}", pos=(HIDE_X, y, SHEET + 0.006), mass=0.001, kinematic=True, shapes=flakes,
                              color=PAINT))
    return out


def _crack() -> Body:
    s = []
    for a in (0.2, 1.1, 1.9, 2.6, 3.5, 4.4, 5.3):
        d = np.array([math.cos(a), math.sin(a), 0.0])
        s.append(plate((0.0, 0.0, 0.0), 0.07 * d, 0.0012, 0.0004, (0.0, 0.0, 1.0), (0.95, 0.97, 0.97), collide=False))
    return Body.build("crack", pos=(HIDE_X, 0.0, SHEET + 0.006), mass=0.001, kinematic=True, shapes=s,
                      color=(0.95, 0.97, 0.97))


def _gun() -> Body:
    s = [BoxShape((-0.025, 0.0, 0.0), (0.130, 0.048, 0.056), ORANGE),
         BoxShape((-0.025, 0.0, 0.0282), (0.110, 0.030, 0.0006), ORANGE_DARK, collide=False),
         BoxShape((-0.092, 0.0, 0.0), (0.006, 0.046, 0.054), BLACK)]
    for z in (-0.016, -0.005, 0.006, 0.017):  # air intake slots in the back
        s.append(BoxShape((-0.0953, 0.0, z), (0.0006, 0.034, 0.004), GREY, collide=False))
    s.append(CylShape((0.048, 0.0, 0.0), 0.024, 0.016, BLACK, axis="X"))
    s.append(CylShape((0.083, 0.0, 0.0), 0.0155, 0.054, STEEL, axis="X"))
    s.append(CylShape((TIP_X - 0.002, 0.0, 0.0), 0.0125, 0.004, STEEL_DARK, collide=False, axis="X"))
    rake = math.radians(37.0)  # the pistol grip rakes back
    s.append(BoxShape((-0.0575, 0.0, -0.049), (0.032, 0.026, 0.058), BLACK,
                      quat_xyzw=(0.0, math.sin(rake / 2), 0.0, math.cos(rake / 2))))  # the handle
    s.append(BoxShape((-0.035, 0.0, -0.042), (0.004, 0.014, 0.014), YELLOW, collide=False,
                      quat_xyzw=(0.0, math.sin(rake / 2), 0.0, math.cos(rake / 2))))  # the switch
    s.append(CylShape((-0.079, 0.0, -0.077), 0.005, 0.006, GREY, collide=False))  # the cable
    return Body.build("gun", pos=tuple(GUN_REST.tolist()), mass=GUN_MASS, shapes=s, color=ORANGE, friction=(1.1, 1.0),
                      com=(0.0, 0.0, GRIP_DZ), start_xyzw=_yaw_quat(math.pi + PSI), damping=(0.0, 0.2))


def _stand() -> Body:
    """A bent-steel heat-gun stand: two cradles under the housing, each with sloping guides either side that centre the
    housing as it is set down; the grip hangs behind them."""
    yaw = math.pi + PSI
    c, s_ = math.cos(yaw), math.sin(yaw)

    def to_stand(xb, yb, z):  # a point (or direction) in the gun's frame, in the stand's
        return (xb * c - yb * s_, xb * s_ + yb * c, z)

    hb = GUN_REST[2] - 0.028 - SHEET - 0.001  # 1 mm under the housing: the gun settles onto it
    s = [BoxShape((0.0, 0.0, 0.003), (0.17, 0.08, 0.006), (0.20, 0.20, 0.21), yaw=yaw)]
    for d in (0.030, -0.010):
        s.append(BoxShape(to_stand(d, 0.0, hb / 2), (0.012, 0.050, hb), STEEL_DARK, yaw=yaw))
        for side in (-1.0, 1.0):
            s.append(plate(to_stand(d, side * 0.027, hb - 0.002), to_stand(d, side * 0.045, hb + 0.018), 0.012, 0.004,
                           to_stand(0.0, -side, 1.0), STEEL_DARK))
    return Body.build("stand", pos=(float(GUN_REST[0]), float(GUN_REST[1]), SHEET), mass=1.0, kinematic=True,
                      shapes=s, color=STEEL_DARK, friction=(0.6, 0.5))


def _scraper() -> Body:
    s = [plate((0.0, -0.001, 0.0015), (0.0, -0.022, 0.026), BLADE_W, 0.002, (0.0, 0.76, 0.65), STEEL),
         BoxShape((0.0, -0.0015, 0.0015), (BLADE_W, 0.004, 0.003), STEEL_DARK),  # the carbide edge
         BoxShape((0.0, -0.022, 0.030), (0.032, 0.014, 0.018), STEEL_DARK),
         CylShape((0.0, -0.022, 0.040), 0.004, 0.004, STEEL, collide=False, axis="Y"),
         BoxShape((0.0, -0.083, 0.032), (0.020, 0.134, 0.022), BEECH),
         CylShape((0.0, -0.152, 0.032), 0.0125, 0.008, RED, axis="Y")]
    for y in (-0.040, -0.120):
        s.append(BoxShape((0.0, y, 0.0432), (0.016, 0.003, 0.0006), WOOD_DARK, collide=False))
    return Body.build("scraper", pos=tuple(SCRAPER_REST.tolist()), mass=SCRAPER_MASS, shapes=s, color=BEECH,
                      friction=(1.1, 1.0), com=(0.0, -0.050, 0.030), slick=True)


def _scraper_rest() -> Body:
    h = 0.0195  # the handle's red end cap rests on it
    s = [BoxShape((0.0, 0.0, h / 2), (0.040, 0.030, h), BEECH),
         BoxShape((0.0, 0.0151, h / 2), (0.036, 0.0004, h - 0.004), WOOD_DARK, collide=False)]
    return Body.build("scraper_rest", pos=(float(SCRAPER_REST[0]), float(SCRAPER_REST[1]) - 0.150, SHEET), mass=1.0,
                      kinematic=True, shapes=s, color=BEECH, friction=(0.8, 0.7))


OBJECTS = [_sheet(), _sash(), _stand(), _scraper_rest(), _gun(), _scraper(), _crack()] + _patches()


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"T": [0.0] * N_PATCH, "blister": [False] * N_PATCH, "bins": [set() for _ in range(N_PATCH)],
            "stripped": [False] * N_PATCH, "glass_s": 0.0, "cracked": False, "edge_y": None, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("STRIP_DEBUG")), "steps": 0}
DT = 1.0 / 20.0


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _place(base, origin, name, pos) -> None:
    import torch

    asset = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                        device=asset.device)
    asset.write_root_pose_to_sim(pose)


def _in_patch(p, k, margin) -> bool:
    return abs(p[0] - NEAR_X) <= PATCH_W / 2 + margin and abs(p[1] - PATCH_Y[k]) <= PATCH_L / 2 + margin


def hot_spot(tip, d):
    """What the gun's nozzle points at within reach: ("patch", k) / ("glass", None) / ("wood", None) / None, and the
    point."""
    if d[2] >= -1e-6:
        return None, None
    t1 = (SASH_TOP - tip[2]) / d[2]
    if t1 < 0:
        return None, None
    p1 = tip + t1 * d
    over_glass = GX0 <= p1[0] <= GX1 and GY0 <= p1[1] <= GY1
    if not over_glass:
        if t1 > HEAT_REACH:
            return None, p1
        for k in range(N_PATCH):
            if _in_patch(p1, k, HEAT_MARGIN):
                return ("patch", k), p1
        return ("wood", None), p1
    t2 = (GLASS_TOP - tip[2]) / d[2]
    p2 = tip + t2 * d
    if t2 <= HEAT_REACH and GX0 <= p2[0] <= GX1 and GY0 <= p2[1] <= GY1:
        return ("glass", None), p2
    return None, p2


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pg, rg = _pose(base, origin, "gun")
    ps, rs = _pose(base, origin, "scraper")
    d = rg[:, 0]
    tip = pg + rg @ np.array([TIP_X, 0.0, 0.0])
    what, spot = hot_spot(tip, d)
    # heat: the patch the gun points at warms, the others cool; the glass cracks after a second of heat
    for k in range(N_PATCH):
        if _EP["stripped"][k]:
            continue
        warm = what is not None and what[0] == "patch" and what[1] == k
        _EP["T"][k] = min(T_MAX, max(0.0, _EP["T"][k] + (WARM if warm else -COOL) * DT))
        was = _EP["blister"][k]
        now = _EP["T"][k] >= T_BLISTER or (was and _EP["T"][k] > T_SET)
        if now != was:
            _EP["blister"][k] = now
            y = PATCH_Y[k]
            _place(base, origin, f"blister{k}" if now else f"paint{k}", (NEAR_X, y, SASH_TOP + 0.0006))
            _place(base, origin, f"paint{k}" if now else f"blister{k}", (HIDE_X, y, SHEET + 0.006))
    if what is not None and what[0] == "glass" and not _EP["cracked"]:
        _EP["glass_s"] += DT
        if _EP["glass_s"] >= GLASS_CRACK_S:
            _EP["cracked"] = True
            _place(base, origin, "crack", (spot[0], spot[1], GLASS_TOP + 0.0004))
    # scraping: the blade's edge drawn through a blistered patch, on the wood, takes the paint off where it passes
    edge = ps + rs @ np.array([0.0, 0.0, 0.0])
    across = rs[:, 0]
    level = abs(float(across[2])) <= math.sin(math.radians(15.0))
    on_wood = SASH_TOP - 0.004 <= edge[2] <= SASH_TOP + 0.007
    prev = _EP["edge_y"]
    _EP["edge_y"] = float(edge[1])
    for k in range(N_PATCH):
        if _EP["stripped"][k] or not _EP["blister"][k] or prev is None:
            continue
        if not (level and on_wood and abs(edge[0] - NEAR_X) <= 0.012):
            continue
        y0, y1 = sorted((prev, float(edge[1])))
        lo = PATCH_Y[k] - PATCH_L / 2
        for b in range(BINS):
            yb = lo + (b + 0.5) * PATCH_L / BINS
            if y0 - 0.002 <= yb <= y1 + 0.002:
                _EP["bins"][k].add(b)
        if len(_EP["bins"][k]) >= STRIPPED_FRAC * BINS:
            _EP["stripped"][k] = True
            _place(base, origin, f"paint{k}", (HIDE_X, PATCH_Y[k], SHEET + 0.006))
            _place(base, origin, f"blister{k}", (HIDE_X, PATCH_Y[k], SHEET + 0.006))
            _place(base, origin, f"flakes{k}", (SX0 - 0.030, PATCH_Y[k], SHEET + 0.0015))
    gun_home = (float(np.linalg.norm(pg - GUN_REST)) <= 0.040
                and math.degrees(math.acos(max(-1.0, min(1.0, float(rg[2, 2]))))) <= 10.0)
    scraper_home = (float(np.linalg.norm(ps - SCRAPER_REST)) <= 0.030
                    and math.degrees(math.acos(max(-1.0, min(1.0, float(rs[2, 2]))))) <= 8.0)
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"gun_home": gun_home, "scraper_home": scraper_home}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, trc = _EP["tcps"]
            sp = "-" if spot is None else np.round((spot - [0, 0, TOP]) * 1000).astype(int).tolist()
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, -float(d[2])))))
            gyaw = math.degrees(math.atan2(d[1], d[0]))
            off = np.round((rg.T @ (pg - tl)) * 1000).astype(int).tolist()
            print(f"[strip] t={_DEBUG['steps'] / 20:5.1f}s aim={what} spot={sp} pitch={pitch:5.1f} gyaw={gyaw:6.1f} gun-L={off}"
                  f" dist={0 if spot is None else float(np.linalg.norm(spot - tip)) * 1000:.0f}"
                  f" T={[round(v, 2) for v in _EP['T']]} bl={_EP['blister']} bins={[len(b) for b in _EP['bins']]}"
                  f" stripped={_EP['stripped']} glass={_EP['glass_s']:.2f}"
                  f" edge={np.round((edge - [0, 0, TOP]) * 1000).astype(int).tolist()} level={level}"
                  f" gun={np.round((pg - [0, 0, TOP]) * 1000).astype(int).tolist()} gh={gun_home} sh={scraper_home}"
                  f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
                  f" R={np.round((trc - [0, 0, TOP]) * 1000).astype(int).tolist()} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 once the glass has cracked. Otherwise, at the end: 1.0 with all three patches stripped, the heat gun back on
    its stand, the scraper back with its handle on its rest and both arms back where they started; 0.5 with all three
    stripped; 0.25 with at least one stripped; else 0.0. (A stripped patch stays stripped.)"""
    if _EP["cracked"]:
        return 0.0
    n = sum(_EP["stripped"])
    if n == N_PATCH:
        s = _EP["s"]
        if s.get("gun_home") and s.get("scraper_home") and _arms_home():
            return 1.0
        return 0.5
    return 0.25 if n >= 1 else 0.0


_HOME: dict = {}


def _arms_home() -> bool:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with the heat gun and the scraper still: the trial ends there."""
    return score(objects) >= 1.0 and objects["gun"].resting() and objects["scraper"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand takes the heat gun off its stand by its housing and aims it, nose down, at the first patch from the far
# left, its nozzle 8 cm from the paint. Meanwhile the right takes the scraper by its handle, 9 cm behind the blade. Once
# the paint has blistered (3 s), the right sets the blade down on the near side of the patch and pushes it through the
# patch toward the gun while the gun keeps heating; then the scraper moves on, and the gun is lifted, carried along and
# lowered onto the next patch. Finally the gun goes back on its stand, the scraper back on the sheet with its handle on
# its rest, and both arms go home.

LEAD_S, LEAD_STEPS = 3.0, 15
GUN_FINGER = -0.040  # (wide: close to its base the left arm comes down up to 1.5 cm off)
SCRAPER_FINGER = -0.015
HEAT_S = 3.0
PULL_S = 1.8
SPOT_X = NEAR_X - 0.003  # aimed at the rail's middle (the glass starts 2.5 cm beyond)
NOZZLE_GAP = 0.080  # (the nozzle's tip stays clear of the scraper's head)
BLADE_IN = -0.003  # the blade's edge aimed this far into the wood (the arm stops on it)


def gun_frame(pitch: float):
    """The gun's rotation (columns: its axes in the world) pointing round PSI and ``pitch`` nose down."""
    yaw = math.pi + PSI
    cz, sz, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return rz @ ry


def gun_grip(pitch: float):
    """The left's TCP rotation on the housing: approach down the gun's -z, fingers closing along its y."""
    r = gun_frame(pitch)
    xg, zg = r[:, 1], -r[:, 2]
    return np.column_stack([xg, np.cross(zg, xg), zg])


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.3, 1.9), (1.0, 1.7), (-1.3, -0.8), (-0.4, 0.0, 0.4), (-0.4, 0.4)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def demo():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    L, R = plan["left"], plan["right"]

    def clock(arm):
        return sum(w.duration for w in plan[arm])

    def wait_until(arm, t, label):
        last = plan[arm][-1]
        dt = t - clock(arm)
        if dt > 1e-6:
            plan[arm].append(_w(dt, last.pos, last.finger, label, last.yaw))

    # -- the left takes the heat gun off its stand -------------------------------------------------------------
    g_rest = GUN_REST + gun_frame(0.0) @ np.array([0.0, 0.0, GRIP_DZ])
    g_put = g_rest - gun_frame(0.0) @ GUN_GRIP_OFF  # setting it back: the gun sits off-centre in the hand
    r_rest = gun_grip(0.0)
    L += _joint_path("left", np.array(READY, float), _good_ik("left", g_rest + [0, 0, 0.025], r_rest),
                     "over the heat gun", GUN_FINGER)
    L.append(_w(0.8, g_rest + [0, 0, 0.012], GUN_FINGER, "down round the heat gun's housing", r_rest))
    L.append(_w(0.6, g_rest, GUN_FINGER, "down round the heat gun's housing", r_rest))
    L.append(_w(0.6, g_rest, FINGER_CLOSED, "take hold of the heat gun", r_rest))
    L.append(_w(0.8, g_rest + [0, 0, 0.070], FINGER_CLOSED, "lift it off its stand", r_rest))
    fa = gun_frame(AIM_PITCH)  # how the gun should point (and does, with the commands below)
    d = fa[:, 0]

    def aim_at(k):
        spot = np.array([SPOT_X, PATCH_Y[k], SASH_TOP])
        tip = spot - d * NOZZLE_GAP
        return (tip - fa @ np.array([TIP_X, 0.0, 0.0]) + fa @ np.array([0.0, 0.0, GRIP_DZ]) - fa @ GUN_GRIP_OFF
                + [0.0, 0.0, AIM_DZ[k]])

    def r_aim_at(k):
        return gun_grip(AIM_PITCH - PITCH_SAG[k])

    # -- the right takes the scraper ---------------------------------------------------------------------------
    r_s = grip_rotation(math.pi / 2)
    s_rest = SCRAPER_REST + SCRAPER_GRIP
    R += _joint_path("right", np.array(READY, float), _good_ik("right", s_rest + [0, 0, 0.050], r_s),
                     "over the scraper's handle", SCRAPER_FINGER)
    R.append(_w(1.0, s_rest + [0, 0, 0.010], SCRAPER_FINGER, "down round the handle, near the blade", r_s))
    R.append(_w(0.5, s_rest, SCRAPER_FINGER, "down round the handle, near the blade", r_s))
    R.append(_w(0.6, s_rest, FINGER_CLOSED, "take hold of the scraper", r_s))
    R.append(_w(0.8, s_rest + [0, 0, 0.060], FINGER_CLOSED, "lift it", r_s))

    def blade_at(y, dz):
        return np.array([NEAR_X, y, SASH_TOP + dz]) + SCRAPER_GRIP

    for k, y in enumerate(PATCH_Y):
        a = aim_at(k)
        if k:  # lift the gun clear before moving it along to the next patch
            L.append(_w(0.6, aim_at(k - 1) + [0, 0, 0.050], FINGER_CLOSED, "lift the heat gun", r_aim_at(k - 1)))
        L.append(_w(1.4 if k == 0 else 1.1, a + [0, 0, 0.050 if k else 0.020], FINGER_CLOSED,
                    "aim the heat gun at the next patch of paint", r_aim_at(k)))
        L.append(_w(0.7 if k else 0.5, a, FINGER_CLOSED, "aim the heat gun at the patch, 8 cm from it", r_aim_at(k)))
        t_hot = clock("left") + HEAT_S
        start, end = y - PATCH_L / 2 - 0.014, y + PATCH_L / 2 - 0.004  # toward the heat, stopping short of the nozzle
        R.append(_w(1.2 if k == 0 else 0.8, blade_at(start, 0.025), FINGER_CLOSED, "the blade over the patch's near end",
                    r_s))
        wait_until("right", t_hot - 0.5, "wait for the paint to blister")
        R.append(_w(0.5, blade_at(start, BLADE_IN), FINGER_CLOSED, "set the blade down on the rail", r_s))
        for j in range(1, 5):
            R.append(_w(PULL_S / 4, blade_at(start + (end - start) * j / 4, BLADE_IN), FINGER_CLOSED,
                        "push the blade through the hot paint, toward the heat", r_s))
        R.append(_w(0.4, blade_at(end, 0.025), FINGER_CLOSED, "lift the blade", r_s))
        # the gun moves on only once the scraper hand is on its way to the next patch (the wrists would meet)
        wait_until("left", clock("right") + 0.6, "keep the heat on the patch while it is scraped")
    # -- put both tools back -----------------------------------------------------------------------------------
    L.append(_w(0.8, aim_at(N_PATCH - 1) + [0, 0.060, 0.020], FINGER_CLOSED, "lift the heat gun away",
                r_aim_at(N_PATCH - 1)))
    lev = aim_at(N_PATCH - 1) + [0, 0.060, 0.040]
    L.append(_w(0.8, lev, FINGER_CLOSED, "level the heat gun", r_rest))
    L.append(_w(2.2, g_put + [0, 0, 0.070], FINGER_CLOSED, "carry it back over its stand", r_rest))
    L.append(_w(0.8, g_put + [0, 0, 0.070], FINGER_CLOSED, "let it steady over its stand", r_rest))
    L.append(_w(1.2, g_put - [0, 0, 0.003], FINGER_CLOSED, "set it down on its stand", r_rest))
    L.append(_w(0.5, g_put - [0, 0, 0.003], FINGER_CLOSED, "set it down on its stand", r_rest))
    L.append(_w(0.6, g_put - [0, 0, 0.003], GUN_FINGER, "let go of it", r_rest))
    L.append(_w(0.8, g_put + [0, 0, 0.070], GUN_FINGER, "rise clear", r_rest))
    R.append(_w(1.6, s_rest + [0, 0, 0.040], FINGER_CLOSED, "carry the scraper back", r_s))
    R.append(_w(1.0, s_rest - [0, 0, 0.002], FINGER_CLOSED, "lay it down, its handle on the rest", r_s))
    R.append(_w(0.6, s_rest - [0, 0, 0.002], SCRAPER_FINGER, "let go of it", r_s))
    R.append(_w(0.8, s_rest + [0, 0, 0.050], SCRAPER_FINGER, "rise clear", r_s))
    L += _joint_path("left", _good_ik("left", g_put + [0, 0, 0.070], r_rest), np.array(READY, float),
                     "back to where it started", GUN_FINGER)
    R += _joint_path("right", _good_ik("right", s_rest + [0, 0, 0.050], r_s), np.array(READY, float),
                     "back to where it started", SCRAPER_FINGER)
    t_end = max(clock("left"), clock("right"))
    for arm in ("left", "right"):
        wait_until(arm, t_end, "hold still")
        last = plan[arm][-1]
        plan[arm].append(_w(3.0, last.pos, last.finger, "hold still: the trial ends once the full score has held 1 s",
                            last.yaw))
    return plan
