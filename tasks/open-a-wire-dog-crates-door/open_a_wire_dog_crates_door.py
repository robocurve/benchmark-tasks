"""On the station table: an 18-inch folding wire dog crate (MidWest iCrate size, 45.7 x 30.5 x 35.6 cm, 18 x 12 x 14
in) standing on its plastic pan, its front door facing the robot, shut and held by its two slide-bolt latches, one near
the top of the door and one near the bottom, on the door's right-hand edge (the robot's right). The door is hinged on
its left-hand edge and swings out toward the robot.

Each latch is a 4.5 mm steel bolt that slides sideways in two guides on the door; its outer end sits in a wire keeper
on the crate's front frame, which holds the door shut. Its inner end is bent down into a wire-loop handle (two legs 17
mm apart, 3 cm long) that hangs straight down by its own weight. Next to the keeper the bolt carries a short locking
tip that, with the handle down, reaches back behind the front frame's vertical wire: the bolt cannot slide. Lifting
the handle 90 degrees (out, toward the robot) turns the bolt in its guides and swings the tip down clear of the wire;
only then can the bolt be slid back out of its keeper. Let go, the handle drops again; a bolt slid back stays back.

Task (instruction): open the crate's front door fully (90 degrees or more) and let go of it; it does not say how. The
procedure is the maker's (MidWest wire crate instructions): "Lift the handle 90 degrees so the locking tip will clear
the wire on the front panel, then slide the bolt to the left and swing the door open." Pulled without its latches
drawn, the door does not move: each bolt is caught in its keeper.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475; heights
below are above the table.
- crate: fixed; front frame in the plane x 0.31, back at x 0.767, sides at y -0.1525 and 0.1525, 35.6 cm tall, pan
  3 cm tall. Door opening between the frame's vertical wires (5 mm) at y -0.135 and 0.135.
- door: 0.40 kg, 25 x 29 cm wire panel (5 mm frame, 3 mm wires 2.5 cm apart), its bottom 4 cm and top 33 cm up, hinged
  about the vertical line x 0.31, y 0.128; opens 0 to 120 degrees, outward only.
- bolts: 0.025 kg each, 4.5 mm rods, axes 1 cm in front of the door at 8.5 cm (bottom) and 29.5 cm (top) up; shut, the
  outer end is at y -0.168, inside a keeper loop at y -0.165 to -0.150; each slides 0 to 32 mm toward +y (drag 120 N
  s/m) and turns 0 to 95 degrees; the handle's legs at y -0.0965 and -0.0795, 3 cm long; the locking tip at y -0.142,
  1.4 cm long, 2.5 mm short of the frame wire at y -0.135. A lift of about 67 degrees or more swings it clear.

Score (best stage reached; 0 for the rest of the run if the door swings out past 8 degrees while a bolt was still in
its keeper, slid back less than 18 mm, when the door was last shut: the door was forced): 0.25 one bolt slid back 20
mm or more (its end 2 mm clear of its keeper); 0.5 both; 0.75 the door swung open 45 degrees or more; full score
(success): the door open 90 degrees or more, both grippers' centres 4 cm or more from the door and its handles, the
door still (turning less than 0.5 degrees) for 1 s.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Joint, allow_matrix_grips, as_numpy, on_reset, on_step  # noqa: E402
from bodykit import capsule, rotation, segment  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
# The file keeps the best stage reached itself and a forced door latches 0, so the score is read at the end of the run.
SCORE_OVER = "end"

# -- layout (world frame, heights above the table) --------------------------------------------------------------------
PX = 0.31  # the crate's front frame (and the closed door) in the plane x = PX
CR_L, CR_HW, CR_H, PAN_H = 0.457, 0.1525, 0.356, 0.03  # crate length (x), half width (y), height, pan height
FRAME_Y = 0.135  # the front frame's vertical wires either side of the door opening (y = -FRAME_Y, +FRAME_Y)
FW_R = 0.0025  # frame wire (5 mm)
DOOR_Y, DOOR_Z = (-0.125, 0.125), (0.04, 0.33)  # the door's extent
DF, DW = 0.005, 0.003  # door frame bar and door mesh wire (square sections, m)
DOOR_KG = 0.40
HINGE = np.array([PX, 0.128])  # the hinge's vertical axis (x, y)
OPEN_MAX = 120.0
BOLT_R = 0.00225  # 4.5 mm bolt
BOLT_DX = 0.010  # the bolt's axis this far in front of the door's plane
BOLT_Z = {"bottom": 0.085, "top": 0.295}
BOLT_END = -0.168  # the bolt's outer end (y) when shut
HANDLE_Y = -0.088  # the handle's middle (y) when shut
LEG_DY, LEG_L = 0.0085, float(os.environ.get("CRATE_LEG_L", "0.030"))  # legs at HANDLE_Y +- LEG_DY, LEG_L long
BOLT_IN = HANDLE_Y + LEG_DY  # the bolt's inner end (y): its bend down into the handle's inner leg
TRAVEL = 0.032  # the bolt slides back (+y) at most this far
TURN_MAX = 95.0
TIP_Y, TIP_L, TIP_R = -0.142, 0.014, 0.002  # the locking tip at y TIP_Y, reaching TIP_L back (+x), handle down
KEEPER_Y = (-0.165, -0.150)  # the keeper loop on the front frame (y)
KW = 0.003  # keeper wire (square section)
KEEP_GAP = 0.002  # clearance round the bolt inside the keeper
BOLT_KG = 0.025
BOLT_DAMP = 120.0  # N s/m: the bolt's drag in its guides

BLACK, PAN, STEEL, ZINC = (0.08, 0.08, 0.09), (0.20, 0.20, 0.22), (0.72, 0.73, 0.75), (0.80, 0.78, 0.70)


def _v(p) -> tuple:
    return tuple(float(v) for v in p)


def _crate_shapes() -> list:
    """The crate in the frame of its front frame's bottom middle (x PX, y 0, table top): pan, front frame with the two
    keepers, sides, top and back (wire panels; the mesh is drawn but only the frame wires collide)."""
    out = []
    L, hw, H = CR_L, CR_HW, CR_H
    # pan
    out.append(BoxShape((L / 2, 0.0, PAN_H / 2), (L - 0.01, 2 * hw - 0.01, PAN_H), PAN))
    # front frame: outer verticals, the two verticals beside the door, top and bottom bars, a bar over the door
    for y in (-hw + FW_R, hw - FW_R, -FRAME_Y, FRAME_Y):
        out.append(CylShape((0.0, y, H / 2), FW_R, H, BLACK))
    for z in (DOOR_Z[0] - 0.008, DOOR_Z[1] + 0.008, H - FW_R):
        out.append(segment((0.0, -hw, z), (0.0, hw, z), 2 * FW_R, BLACK))
    for y0, y1 in ((-hw, -FRAME_Y), (FRAME_Y, hw)):  # short horizontals beside the door
        for z in np.linspace(0.08, 0.30, 5):
            out.append(segment((0.0, y0, float(z)), (0.0, y1, float(z)), 0.003, BLACK, collide=False))
    # keepers: a wire loop round each bolt's outer end, standing out of the front frame
    for z in BOLT_Z.values():
        xf = -BOLT_DX - BOLT_R - KEEP_GAP - KW / 2  # the keeper's front bar (in front of the bolt)
        ym, yl = (KEEPER_Y[0] + KEEPER_Y[1]) / 2, KEEPER_Y[1] - KEEPER_Y[0]
        out.append(BoxShape((xf, ym, z), (KW, yl, 2 * (BOLT_R + KEEP_GAP) + 2 * KW), ZINC))
        for s in (-1.0, 1.0):
            zz = z + s * (BOLT_R + KEEP_GAP + KW / 2)
            out.append(BoxShape(((xf - KW / 2) / 2, ym, zz), (-(xf - KW / 2), yl, KW), ZINC))
    # sides, top, back: frame wires (collide) and mesh (drawn only)
    for y in (-hw + FW_R, hw - FW_R):
        for z in (PAN_H + 0.005, H - FW_R):
            out.append(segment((0.0, y, z), (L, y, z), 2 * FW_R, BLACK))
        out.append(CylShape((L, y, H / 2), FW_R, H, BLACK))
        for x in np.arange(0.025, L, 0.025):
            out.append(segment((float(x), y, PAN_H), (float(x), y, H), 0.003, BLACK, collide=False))
        for z in (0.12, 0.24):
            out.append(segment((0.0, y, z), (L, y, z), 0.003, BLACK, collide=False))
    for x in np.arange(0.025, L, 0.025):  # top mesh
        out.append(segment((float(x), -hw, H - FW_R), (float(x), hw, H - FW_R), 0.003, BLACK, collide=False))
    for y in (-0.09, -0.03, 0.03, 0.09):
        out.append(segment((0.0, y, H - FW_R), (L, y, H - FW_R), 0.003, BLACK, collide=False))
    for y in np.arange(-hw + 0.025, hw, 0.025):  # back mesh
        out.append(segment((L, float(y), PAN_H), (L, float(y), H), 0.003, BLACK, collide=False))
    out.append(segment((L, -hw, H / 2), (L, hw, H / 2), 0.003, BLACK, collide=False))
    return out


DOOR_C = np.array([PX, (DOOR_Y[0] + DOOR_Y[1]) / 2, TOP + (DOOR_Z[0] + DOOR_Z[1]) / 2])


def _door_shapes() -> list:
    """The door in the frame of its middle: a 5 mm frame, 3 mm vertical wires 2.5 cm apart, two horizontal wires, the
    bolts' guides (drawn) and the hinge loops round the frame (drawn)."""
    cy, cz = DOOR_C[1], DOOR_C[2] - TOP
    y0, y1, z0, z1 = DOOR_Y[0] - cy, DOOR_Y[1] - cy, DOOR_Z[0] - cz, DOOR_Z[1] - cz
    out = []
    for y in (y0 + DF / 2, y1 - DF / 2):
        out.append(BoxShape((0.0, y, 0.0), (DF, DF, z1 - z0), BLACK))
    for z in (z0 + DF / 2, z1 - DF / 2):
        out.append(BoxShape((0.0, 0.0, z), (DF, y1 - y0, DF), BLACK))
    for y in np.arange(y0 + 0.025, y1 - 0.01, 0.025):
        out.append(BoxShape((0.0, float(y), 0.0), (DW, DW, z1 - z0 - 2 * DF), BLACK))
    for z in (z0 + 0.10, z0 + 0.19):
        out.append(BoxShape((-DW, 0.0, z), (DW, y1 - y0 - 2 * DF, DW), BLACK))
    for name, zb in BOLT_Z.items():  # the bolt's guides: a bar across the door and two loops (drawn)
        z = zb - cz
        for gy in (-0.117, -0.104):
            out.append(BoxShape((-BOLT_DX / 2, gy - cy, z), (BOLT_DX + 0.004, 0.003, 2 * BOLT_R + 0.004), ZINC,
                                collide=False))
    for z in (z0 + 0.03, z1 - 0.03):  # hinge loops (drawn)
        out.append(CylShape((0.0, HINGE[1] - cy, z), 0.005, 0.012, ZINC, collide=False))
    return out


def _bolt_shapes(zb: float) -> list:
    """A bolt in the frame of its axis at the bolt's middle (y), handle down: the rod, the locking tip reaching back
    (+x) at TIP_Y, and the wire-loop handle hanging down at its inner end."""
    cy = (BOLT_END + BOLT_IN) / 2
    out = [capsule((0.0, BOLT_END - cy + BOLT_R, 0.0), (0.0, BOLT_IN - cy, 0.0), BOLT_R, STEEL)]
    out.append(capsule((0.0, TIP_Y - cy, 0.0), (TIP_L, TIP_Y - cy, 0.0), TIP_R, STEEL))
    for s in (-1.0, 1.0):
        y = HANDLE_Y + s * LEG_DY - cy
        out.append(capsule((0.0, y, 0.0), (0.0, y, -LEG_L), BOLT_R, STEEL))
    out.append(capsule((0.0, HANDLE_Y - LEG_DY - cy, -LEG_L), (0.0, HANDLE_Y + LEG_DY - cy, -LEG_L), BOLT_R, STEEL))
    return out


def _bolt_c(name: str) -> np.ndarray:
    return np.array([PX - BOLT_DX, (BOLT_END + BOLT_IN) / 2, TOP + BOLT_Z[name]])


CRATE = Body.build("crate", pos=(PX, 0.0, TOP), mass=6.0, color=BLACK, fixed=True, friction=(0.5, 0.4),
                   shapes=_crate_shapes())
DOOR = Body.build("door", pos=_v(DOOR_C), mass=DOOR_KG, color=BLACK, friction=(0.6, 0.5), damping=(0.0, 0.3),
                  solver_iterations=32, shapes=_door_shapes(),
                  joint=Joint(anchor=(float(HINGE[0]), float(HINGE[1]), float(DOOR_C[2])), kind="revolute", axis="Z",
                              parent="crate", limits=(-OPEN_MAX, 0.0), damping=0.05))
BOLTS = {name: Body.build(f"bolt_{name}", pos=_v(_bolt_c(name)), mass=BOLT_KG, color=STEEL, friction=(0.6, 0.5),
                          damping=(0.5, 0.5), solver_iterations=32, shapes=_bolt_shapes(BOLT_Z[name]),
                          joint=Joint(anchor=_v(_bolt_c(name)), kind="cylinder", axis="Y", parent="door",
                                      limits=(-3.0, TURN_MAX), axial=(0.0, BOLT_DAMP, TRAVEL),
                                      slide_range=(0.0, TRAVEL)))
         for name in ("bottom", "top")}
# The door's latch-side frame bar beside each bolt, as its own body fixed to the door so that the bolt's locking tip
# collides with it (a joint keeps its two bodies from colliding, and the bolt is jointed to the door): with the handle
# down, the tip of a bolt slid back stands between this bar and the next door wire, so the bolt stays back.
CATCH_H = 0.04
CATCHES = {name: Body.build(f"catch_{name}", pos=(PX, DOOR_Y[0] + DF / 2, TOP + BOLT_Z[name]), mass=0.005, color=BLACK,
                            shapes=[BoxShape((0.0, 0.0, 0.0), (DF, DF, CATCH_H), BLACK)],
                            joint=Joint(anchor=(PX, DOOR_Y[0] + DF / 2, TOP + BOLT_Z[name]), kind="fixed",
                                        parent="door"))
           for name in ("bottom", "top")}
OBJECTS = [CRATE, DOOR, BOLTS["bottom"], BOLTS["top"], CATCHES["bottom"], CATCHES["top"]]

# -- scoring ----------------------------------------------------------------------------------------------------------
OUT_MM = 20.0  # slid back this far, a bolt's end is 2 mm clear of its keeper (keeper face at y -0.150, end at -0.168)
OPEN_PART, OPEN_FULL, FORCED_DEG = 45.0, 90.0, 8.0
SHUT_DEG = 2.0  # the door counts as shut up to this angle (a bolt in its keeper lets it open about 0.4 degrees)
IN_KEEPER_MM = 18.0  # slid back less than this, a bolt's end is still inside its keeper's span
CLEAR = 0.04
STILL_STEPS, STILL_DEG = 20, 0.5
_DEBUG = os.environ.get("CRATE_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("CRATE_DEBUG_EVERY", "10"))
_EP: dict = {}
_TCP: dict = {}
_ARM_IDS: dict = {}
_BOLT0 = {n: np.array([0.0, (BOLT_END + BOLT_IN) / 2 - DOOR_C[1], BOLT_Z[n] - (DOOR_C[2] - TOP)]) - [BOLT_DX, 0, 0]
          for n in BOLT_Z}  # each bolt's origin in the door's frame, shut


@on_reset
def _reset() -> None:
    _EP.update(step=0, tick=0, counted=-1, best=0.0, failed="", recent=[], angle=0.0, slide={n: 0.0 for n in BOLT_Z},
               turn={n: 0.0 for n in BOLT_Z}, near=1.0, shut_slide={n: 0.0 for n in BOLT_Z})
    _TCP.clear()


_reset()


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("CRATE_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _door_points(dp, dr) -> np.ndarray:
    """Points over the door's panel and both bolts' handles (world), for the grippers' clearance."""
    cy, cz = DOOR_C[1], DOOR_C[2] - TOP
    pts = [[0.0, y - cy, z - cz] for y in np.linspace(DOOR_Y[0], DOOR_Y[1], 11) for z in np.linspace(*DOOR_Z, 13)]
    return dp + np.asarray(pts) @ dr.T


def _track_inner(env) -> None:
    """Each gripper's centre (TCP) from its joints; the door's opening angle; each bolt's slide (back, +y along the
    door) and turn (handle lifted) relative to the door; the forced-door rule."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _ARM_IDS:
            _ARM_IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0]
        _TCP[side] = np.asarray(ik.tcp_pose(q[_ARM_IDS[side]], station.BASES[side])[0], float)
    origin = as_numpy(base.scene.env_origins[0])
    d = as_numpy(base.scene["door"].data.root_link_pose_w)[0]
    dp, dr = d[:3] - origin, rotation(d[3:7])
    angle = -math.degrees(math.atan2(dr[1, 0], dr[0, 0]))  # opening (outward) is a turn toward -z
    pts = [_door_points(dp, dr)]
    for n in BOLT_Z:
        b = as_numpy(base.scene[f"bolt_{n}"].data.root_link_pose_w)[0]
        bp, br = b[:3] - origin, rotation(b[3:7])
        rel = dr.T @ (bp - dp) - _BOLT0[n]
        rr = dr.T @ br
        _EP["slide"][n] = float(rel[1])
        _EP["turn"][n] = math.degrees(math.atan2(rr[0, 2], rr[0, 0]))
        cy = (BOLT_END + BOLT_IN) / 2
        pts.append(bp + np.array([[0.0, HANDLE_Y - cy, -LEG_L * k] for k in (0.0, 0.5, 1.0)]) @ br.T)
    pts = np.vstack(pts)
    _EP["near"] = float(min(np.linalg.norm(pts - t, axis=1).min() for t in _TCP.values())) if _TCP else 1.0
    out = [s * 1000 >= OUT_MM for s in _EP["slide"].values()]
    if angle <= SHUT_DEG:
        _EP["shut_slide"] = dict(_EP["slide"])
    caught = [n for n, v in _EP["shut_slide"].items() if v * 1000 < IN_KEEPER_MM]
    if _EP["angle"] <= FORCED_DEG < angle and caught and not _EP["failed"]:
        _EP["failed"] = f"the door was forced open past {FORCED_DEG:.0f} deg with the {caught[0]} bolt in its keeper"
    _EP["angle"] = angle
    stage = 0.75 if angle >= OPEN_PART else 0.25 * sum(out)
    if not _EP["failed"]:
        _EP["best"] = max(_EP["best"], stage)


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        recent.append(_EP["angle"])
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and max(recent) - min(recent) < STILL_DEG
    done = bool(_EP["angle"] >= OPEN_FULL and _EP["near"] >= CLEAR and still)
    return dict(still=still, done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    s, u = _EP["slide"], _EP["turn"]
    print(f"DBG step={_EP['step']} door={_EP['angle']:.1f}deg "
          f"slide b={s['bottom'] * 1000:.1f} t={s['top'] * 1000:.1f}mm "
          f"turn b={u['bottom']:.0f} t={u['top']:.0f}deg near={_EP['near']:.3f} still={t['still']} best={_EP['best']} "
          f"failed={_EP['failed'] or '-'}", flush=True)
    if len(_TCP) == 2:
        print("DBG   " + " | ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f})" for k, v in _TCP.items()),
              flush=True)


def score(objects) -> float:
    """The best stage reached (0 for the rest of the run once the door has been forced): 0.25 one bolt out of its
    keeper; 0.5 both; 0.75 the door open 45 degrees or more. Full score (1.0) is success()."""
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    return 0.0 if _EP["failed"] else _EP["best"]


def success(objects) -> bool:
    """The door open 90 degrees or more, both grippers 4 cm or more from the door and its bolts, the door still 1 s."""
    t = _tally(objects)
    return bool(t["done"] and not _EP["failed"])


# -- scripted demo ----------------------------------------------------------------------------------------------------
# As the maker's instructions say, latch by latch, with the right gripper: tipped forward, fingers closing across the
# handle's two legs, it pinches the hanging handle 2 cm below the bolt and swings it up through 90 degrees, tipping
# itself further down as it goes (the legs turn the rest of the way between its pads); it slides the bolt back 3.8 cm
# (it stops at 3.2) and lets go (the handle drops; the bolt stays back): first the bottom latch, then the top one. The
# left gripper then pinches the top handle (hanging again), pulls the door open by it to about 75 degrees, turning with
# the door, lets go and rises over the door's top, out of its way; the right gripper, fingers closed, comes into the
# open doorway and pushes the door's inside face on to about 105 degrees (it swings on to its stop), and moves away.
# CRATE_YANK=1 runs it the wrong way: nobody opens the latches; the left gripper pinches the top handle and pulls the
# door straight out by it.
# CRATE_NOLIFT=1 (a check of the lock): the right gripper pinches the bottom handle and slides it without lifting it.

WRONG = os.environ.get("CRATE_YANK") == "1"


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


FWD = _ry(-math.pi / 2) @ grip_rotation(0.0)  # level, pointing forward (+x), fingers closing along y
NOLIFT = os.environ.get("CRATE_NOLIFT") == "1"
# the right gripper's pitch (degrees down from level) on the hanging handle and on the lifted one
PITCH = {"bottom": tuple(float(v) for v in os.environ.get("CRATE_PB", "15,60").split(",")),
         "top": tuple(float(v) for v in os.environ.get("CRATE_PT", "0,60").split(","))}
DOWN = grip_rotation(0.0)
OPEN_C, CLOSED = -0.024, 0.0
PINCH_C = -(LEG_DY + BOLT_R - 0.003)  # 3 mm inside each leg's outer face
PINCH_R = float(os.environ.get("CRATE_PINCH_R", "0.020"))  # the pinch this far down the handle from the bolt's axis
TCP_DX = float(os.environ.get("CRATE_TCP_DX", "0.008"))  # the TCP this far short of the legs (along its pointing)
LIFT_DEG = float(os.environ.get("CRATE_LIFT", "100"))  # commanded lift (the bolt stops at 95)
SLIDE_TO = TRAVEL + float(os.environ.get("CRATE_SLIDE_EXTRA", "0.006"))
LIFT_STEP = float(os.environ.get("CRATE_LIFT_STEP", "0.6"))  # seconds per eighth of the lift
PULL_DEG = float(os.environ.get("CRATE_PULL", "75"))
PUSH_TO = float(os.environ.get("CRATE_PUSH_TO", "105"))
PULL_BIAS = float(os.environ.get("CRATE_PULL_BIAS", "0.025"))  # pulled this far toward the hinge: the bolt stays back
PULL_PITCH = float(os.environ.get("CRATE_PULL_PITCH", "15"))
LPULL = _ry(math.radians(PULL_PITCH)) @ FWD


def _tilt(deg: float) -> np.ndarray:
    return _ry(math.radians(deg)) @ FWD


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    for sd in itertools.product((-0.4, 0.0, 0.4), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (-1.2, 0.0, 1.2),
                                (-1.5, 0.0, 1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _axis(name: str) -> np.ndarray:
    return np.array([PX - BOLT_DX, HANDLE_Y, TOP + BOLT_Z[name]])


def _grip_pitch(name: str, deg: float) -> float:
    a0, a1 = PITCH[name]
    return a0 + (a1 - a0) * min(max(deg, 0.0), 90.0) / 90.0


def _pinch(name: str, deg: float, slide: float = 0.0, pitch: float | None = None) -> np.ndarray:
    """The TCP on the handle turned up ``deg`` (0: hanging), the bolt slid back ``slide``: TCP_DX short of the legs
    along the gripper's pointing direction (pitched ``pitch`` degrees down)."""
    a = math.radians(deg)
    leg = _axis(name) + np.array([-PINCH_R * math.sin(a), slide, -PINCH_R * math.cos(a)])
    g = math.radians(_grip_pitch(name, deg) if pitch is None else pitch)
    return leg - TCP_DX * np.array([math.cos(g), 0.0, -math.sin(g)])


def _latch(name: str) -> list:
    """Pinch the hanging handle, swing it up, slide the bolt back, let go and back off."""
    r0 = _tilt(_grip_pitch(name, 0.0))
    p0 = _pinch(name, 0.0)
    out = [_w(1.4, p0 + [-0.05, 0.0, 0.0], OPEN_C, f"{name} latch: in front of its handle", r0),
           _w(1.0, p0, OPEN_C, "fingers either side of the handle's legs", r0),
           _w(0.4, p0, OPEN_C, "settle", r0),
           _w(0.7, p0, PINCH_C, "pinch the handle", r0)]
    if NOLIFT:
        return out + [_w(1.5, p0 + [0.0, SLIDE_TO, 0.0], PINCH_C, "slide it without lifting the handle", r0),
                      _w(1.0, p0 + [0.0, SLIDE_TO, 0.0], PINCH_C, "keep pushing", r0),
                      _w(0.6, p0 + [0.0, SLIDE_TO, 0.0], OPEN_C, "let go", r0),
                      _w(0.8, p0 + [-0.06, SLIDE_TO, 0.0], OPEN_C, "back off", r0)]
    n = 8
    for k in range(1, n + 1):
        deg = LIFT_DEG * k / n
        out.append(_w(LIFT_STEP, _pinch(name, deg), PINCH_C, "swing the handle up 90 degrees",
                      _tilt(_grip_pitch(name, deg))))
    r1 = _tilt(_grip_pitch(name, LIFT_DEG))
    end = _pinch(name, LIFT_DEG, SLIDE_TO)
    out += [_w(0.4, _pinch(name, LIFT_DEG), PINCH_C, "settle", r1),
            _w(1.8, end, PINCH_C, "slide the bolt back out of its keeper", r1),
            _w(0.4, end, PINCH_C, "settle", r1),
            _w(0.6, end, OPEN_C, "let the handle go (it drops; the bolt stays back)", r1),
            _w(0.6, end + [-0.035, 0.0, -0.01], OPEN_C, "back off the handle", r1),
            _w(0.8, end + [-0.08, 0.0, -0.03], OPEN_C, "back off", r1)]
    return out


def _door_turn(p, deg: float) -> np.ndarray:
    """Point ``p`` (world, on the shut door) carried round the hinge by the door opened ``deg``."""
    r = _rz(-math.radians(deg))
    h = np.array([HINGE[0], HINGE[1], 0.0])
    return h + r @ (np.asarray(p, float) - h)


def _right() -> list:
    """Both latches, bottom then top (the wrong way: nothing)."""
    start = _pinch("bottom", 0.0) + [-0.05, 0.0, 0.03]
    r0 = _tilt(PITCH["bottom"][0])
    out = _lead_in("right", start, r0, OPEN_C, "in front of the door")
    if WRONG:
        return out + [_w(40.0, start, OPEN_C, "stay put", r0)]
    out += _latch("bottom")
    if NOLIFT or os.environ.get("CRATE_ONLY") == "bottom":
        return out + [_w(1.4, _away("right"), OPEN_C, "move away", _tilt(45.0)),
                      _w(10.0, _away("right"), OPEN_C, "stay", _tilt(45.0))]
    out += _latch("top")
    out += [_w(1.4, _away("right"), OPEN_C, "move away", _tilt(45.0))]
    t_left = _until(_left(), LEFT_CLEAR)
    t_here = float(sum(w.duration for w in out))
    p0 = _push_point(PUSH_FROM)
    out += [_w(max(t_left - t_here, 0.5), _away("right"), OPEN_C, "wait for the door to be pulled open", _tilt(45.0)),
            _w(1.4, p0 + [-0.04, -0.06, 0.0], CLOSED, "in front of the doorway, fingers closed", RPUSH),
            _w(1.0, p0, CLOSED, "inside the open door, by its inside face", RPUSH)]
    n = 9
    for k in range(1, n + 1):
        deg = PUSH_FROM + (PUSH_TO - PUSH_FROM) * k / n
        out.append(_w(0.5, _push_point(deg), CLOSED, "swing the door on, past 90 degrees", RPUSH))
    end = _push_point(PUSH_TO)
    away = end + _rz(-math.radians(PUSH_TO)) @ np.array([0.05, 0.0, 0.0])  # along the door's inside face normal
    out += [_w(0.4, end, CLOSED, "settle", RPUSH),
            _w(1.0, away, CLOSED, "back off the door", RPUSH),
            _w(1.4, _away("right"), OPEN_C, "move away", _tilt(45.0)),
            _w(6.0, _away("right"), OPEN_C, "stay clear", _tilt(45.0))]
    return out


def _until(ws: list, label: str) -> float:
    """The time at which the waypoint labelled ``label`` ends."""
    t = 0.0
    for w in ws:
        t += w.duration
        if w.label == label:
            return t
    raise ValueError(label)


def _away(arm: str) -> np.ndarray:
    return np.array([0.14, -0.26 if arm == "right" else 0.26, TOP + 0.25])


T_LATCH = 1.4 + 1.0 + 0.4 + 0.7 + 8 * LIFT_STEP + 0.4 + 1.8 + 0.4 + 0.6 + 0.6 + 0.8
T_RIGHT_DONE = 3.0 + 2 * T_LATCH + 0.8  # the right gripper is off the top handle and moving away
PUSH_FROM = float(os.environ.get("CRATE_PUSH_FROM", "60"))
PARK_L = np.array([0.25, 0.18, TOP + 0.41])  # the left gripper waits here, over the door's top
LEFT_CLEAR = "move away, clear of the door's swing"


def _left() -> list:
    """The door pulled open by the top handle (its bolt slid back); then over the door's top and a push on its
    inside face past 90 degrees."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    if NOLIFT:
        pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
        return [Waypoint(30.0, tuple(float(v) for v in pos), CLOSED, "stay put", rot)]
    rot = LPULL
    slide = 0.0 if WRONG else TRAVEL
    p0 = _pinch("top", 0.0, slide, PULL_PITCH)
    wait = p0 + [-0.08, 0.06, 0.05]
    out = _lead_in("left", wait, rot, OPEN_C, "in front of the door, to the left")
    out.append(_w((6.0 if WRONG else T_RIGHT_DONE + 0.6) - 3.0, wait, OPEN_C, "wait for the latches", rot))
    out += [_w(1.4, p0 + [-0.05, 0.0, 0.0], OPEN_C, "in front of the top handle", rot),
            _w(1.0, p0, OPEN_C, "fingers either side of the handle's legs", rot),
            _w(0.4, p0, OPEN_C, "settle", rot),
            _w(0.7, p0, PINCH_C, "pinch the handle", rot)]
    if WRONG:  # pull straight out by the hanging handle, nothing slid
        return out + [_w(1.5, p0 + [-0.04, 0.0, 0.0], PINCH_C, "pull the door by the handle, latches shut", rot),
                      _w(1.5, p0 + [-0.05, 0.0, 0.0], PINCH_C, "keep pulling", rot),
                      _w(0.6, p0 + [-0.05, 0.0, 0.0], OPEN_C, "let go", rot),
                      _w(0.8, p0 + [-0.10, 0.0, 0.0], OPEN_C, "back off", rot),
                      _w(1.4, _away("left"), OPEN_C, "move away", _tilt(45.0)),
                      _w(8.0, _away("left"), OPEN_C, "stay clear", _tilt(45.0))]
    n = 14
    for k in range(1, n + 1):
        deg = PULL_DEG * k / n
        out.append(_w(0.4, _door_turn(p0 + [0.0, PULL_BIAS, 0.0], deg), PINCH_C,
                      "pull the door open by the top handle", _rz(-math.radians(deg)) @ rot))
    last = _door_turn(p0 + [0.0, PULL_BIAS, 0.0], PULL_DEG)
    r1 = _rz(-math.radians(PULL_DEG)) @ rot
    a = math.radians(PULL_DEG)
    outward = np.array([-math.cos(a), math.sin(a), 0.0])  # the door's outside face normal, the door opened PULL_DEG
    back = last + 0.06 * outward
    out += [_w(0.4, last, PINCH_C, "settle", r1),
            _w(1.0, last, OPEN_C, "let go of the handle", r1),
            _w(0.4, last, OPEN_C, "settle", r1),
            _w(1.2, back, OPEN_C, "back off, away from the door's face", r1),
            _w(1.0, np.array([back[0], back[1], TOP + 0.36]), OPEN_C, "rise", r1),
            _w(1.2, PARK_L, OPEN_C, LEFT_CLEAR, r1),
            _w(30.0, PARK_L, OPEN_C, "stay clear", r1)]
    return out


PUSH_R = float(os.environ.get("CRATE_PUSH_R", "0.13"))  # the push this far out from the hinge
PUSH_Z = float(os.environ.get("CRATE_PUSH_Z", "0.16"))  # and this high
PUSH_IN = float(os.environ.get("CRATE_PUSH_IN", "0.022"))  # the TCP this far inside the door's plane
RPUSH = _ry(math.radians(float(os.environ.get("CRATE_RPITCH", "50")))) @ FWD  # tipped forward and down, fixed


def _push_point(deg: float) -> np.ndarray:
    """The right TCP on the door's inside face, PUSH_R out from the hinge, the door opened ``deg``."""
    p = np.array([PX + PUSH_IN, HINGE[1] - PUSH_R, TOP + PUSH_Z])
    return _door_turn(p, deg)


def demo():
    return {"left": _left(), "right": _right()}
