"""On the station table: a banker's desk lamp (brass base and stem, green glass shade, 38 cm tall, 1.6 kg) stands
switched off, loose on the table. Its bulb socket inside the shade is a pull-chain socket: a brass ball chain (2.4 mm
beads) hangs 128 mm from under the shade with a small brass bell pull on its end. Task (instruction): switch the lamp
on and leave it standing. The instruction does not say how.

How it is done: a pull-chain socket (replacement lamp sockets are sold "With a 6.3-inch chain on the side for easy
on/off performance", 1000Bulbs listing SOCK-801052) turns the lamp on or off once per full pull: the chain is pulled straight down until the switch clicks, then let
back. A pull that stops short does nothing; a second pull turns the lamp off again.

The hand skill: catch a small pull on a light chain that swings away when touched (the bell is 11 mm across and hangs
under the shade, 17 cm above the table, over the lamp's base), close on it, and pull it straight down 10 mm against
3-6 N until it clicks, exactly once, then let it back up and let go. The lamp is loose: a pull out toward the robot
drags or tips it; a pull more than 30 deg off the chain's hanging line binds the chain on the
bushing's rim and does not work the switch.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475. Heights
below are above the table top.
- lamp: one loose rigid body. Base: a brass slab 13 x 22 cm, 2.6 cm tall in two steps, middle at
  (0.31, 0.00); stem 18 mm across at x 0.36 up to 0.37, a yoke bar forward to the shade's top. Shade: green glass, half
  a cylinder of radius 65 mm (upper half), 25 cm long along y, its axis at x 0.29, z 0.30 (front lip at x 0.225, back
  lip at x 0.355, both 0.30 up), brass end caps. Socket: a brass shell 35 mm across, 57 mm long along y, axis at
  (0.285, z 0.325), y 0.022-0.078; an A19 bulb (60 mm) on it toward -y. 1.6 kg in all.
- switch: the chain enters the socket through a bushing under it at (0.285, 0.060, 0.3075). Inside, the chain's top end
  is a plunger on a slide (straight up and down): 0 to 13 mm of travel, a spring (300 N/m, 3.0 N preload: 3.0 N to
  start it, 6.0 N at 10 mm, 6.9 N at the stop). The click (a rule on the plunger's travel): reaching 10 mm toggles the
  lamp once; it acts again only after the plunger has come back above 3 mm. A pull whose top chain length is more than
  30 deg off the lamp's vertical when the plunger reaches 10 mm binds on the bushing: no click, and that pull is spent
  (the plunger must come back above 3 mm first).
- chain: five rigid lengths of 25.6 mm (8 beads 2.4 mm across at 3.2 mm pitch, 2 g each), each hung from the one above
  (the top one from the plunger) by a ball joint: free swinging, the top one stopped at 50 deg by the bushing's rim, the
  others at 80 deg. A brass bell pull, 11 mm across and 16 mm long, on the last length (8 g with it): its middle 0.1705
  up, its bottom 0.1625 up.

Score (latched hazard first, then the end state):
- 0 if the lamp ever tips over (base tilted more than 25 deg at any moment: latched).
- 1.0 (success, held 1 s) when the lamp is on after exactly one click, the chain is let go (every finger point 10 mm or
  more from the chain and bell, plunger back within 2 mm of its top), the lamp stands upright (under 5 deg) within
  2 cm of where it started and it is still (under 1 mm in 1 s).
- 0.5 when the lamp is on otherwise (three or more clicks, the chain still held, or the lamp pushed more than 2 cm).
- 0.3 when the lamp is off after an even number of clicks (2 or more: it was switched on and off again).
- 0 when it never clicked.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Joint, SphereShape, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, plate, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

# -- the layout ----------------------------------------------------------------------------------------------------
LAMP_X, LAMP_Y = 0.31, 0.0
ORIGIN = (LAMP_X, LAMP_Y, TOP + 0.001)  # the lamp's origin: the base's bottom middle (spawned 1 mm up)
BASE_X, BASE_Y, BASE_H1, BASE_H2 = 0.13, 0.22, 0.018, 0.008
STEM_X, STEM_R, STEM_TOP = 0.36, 0.009, 0.37
SHADE_X, SHADE_Z, SHADE_R, SHADE_L, SHADE_T = 0.29, 0.30, 0.065, 0.25, 0.003
SOCK_X, SOCK_Z, SOCK_R, SOCK_Y0, SOCK_Y1 = 0.285, 0.325, 0.0175, 0.022, 0.078
BULB_R = 0.030
CHAIN_X, CHAIN_Y, CHAIN_Z = 0.285, 0.060, 0.3075  # the bushing: where the chain leaves the socket
BEAD_R, PITCH, PER_SEG, N_SEG = 0.0012, 0.0032, 8, 5
SEG_L = PITCH * PER_SEG  # 25.6 mm
BELL_R, BELL_L = 0.0055, 0.016
CHAIN_END = CHAIN_Z - N_SEG * SEG_L  # the last bead's centre
BELL_MID = CHAIN_END - 0.001 - BELL_L / 2  # 0.1705
TRAVEL_MAX = 13.0 * MM
CLICK = 10.0 * MM
REARM = 3.0 * MM
K_SPRING = float(os.environ.get("PCHAIN_K", "300"))  # N/m
PRE = 10.0 * MM  # the spring's anchor this far above the top: 3.0 N preload
JAM_DEG = 30.0
RIM_DEG = 50.0
LAMP_MASS = 1.6
PLUNGER_MASS, SEG_MASS, BELL_SEG_MASS = 0.004, 0.002, 0.008
CHAIN_DAMP = (float(os.environ.get("PCHAIN_LDAMP", "0.1")), float(os.environ.get("PCHAIN_ADAMP", "0.3")))

BRASS, DARK_BRASS, GLASS, GLASS_IN = (0.78, 0.6, 0.25), (0.55, 0.42, 0.18), (0.04, 0.36, 0.16), (0.85, 0.85, 0.78)
BULB, FELT = (0.86, 0.86, 0.8), (0.12, 0.25, 0.14)
GLOW_GLASS = (0.15, 1.0, 0.35)  # emissive colours while the lamp is on
GLOW_BULB = (1.0, 0.9, 0.6)


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _r(x: float, y: float, z: float) -> tuple:
    """A world point (x, y, height above the table) relative to the lamp's origin."""
    return (x - ORIGIN[0], y - ORIGIN[1], TOP + z - ORIGIN[2])


def _base_shapes() -> list:
    sh = [BoxShape(_r(LAMP_X, LAMP_Y, BASE_H1 / 2), (BASE_X, BASE_Y, BASE_H1), BRASS),
          BoxShape(_r(LAMP_X + 0.01, LAMP_Y, BASE_H1 + BASE_H2 / 2), (BASE_X - 0.04, BASE_Y - 0.03, BASE_H2),
                   DARK_BRASS),
          BoxShape(_r(LAMP_X, LAMP_Y, 0.0006), (BASE_X - 0.006, BASE_Y - 0.006, 0.0012), FELT, collide=False),
          # stem and yoke
          CylShape(_r(STEM_X, LAMP_Y, (BASE_H1 + BASE_H2 + STEM_TOP) / 2), STEM_R, STEM_TOP - BASE_H1 - BASE_H2,
                   BRASS),
          SphereShape(_r(STEM_X, LAMP_Y, STEM_TOP), 0.012, BRASS),
          CylShape(_r((STEM_X + SHADE_X) / 2, LAMP_Y, STEM_TOP), 0.005, STEM_X - SHADE_X, BRASS, axis="X"),
          CylShape(_r(SHADE_X, LAMP_Y, (STEM_TOP + SHADE_Z + SHADE_R) / 2), 0.006, STEM_TOP - SHADE_Z - SHADE_R
                   + 0.004, BRASS),
          # the socket (brass shell) and its bushing, the bulb (on it, toward -y)
          CylShape(_r(SOCK_X, (SOCK_Y0 + SOCK_Y1) / 2, SOCK_Z), SOCK_R, SOCK_Y1 - SOCK_Y0, BRASS, axis="Y"),
          CylShape(_r(SOCK_X, SOCK_Y1 + 0.004, SOCK_Z), 0.008, 0.008, BRASS, axis="Y"),
          CylShape(_r(CHAIN_X, CHAIN_Y, CHAIN_Z + 0.0035), 0.0035, 0.004, DARK_BRASS, collide=False),
          CylShape(_r(SOCK_X, SOCK_Y0 - 0.010, SOCK_Z), 0.013, 0.020, BULB, axis="Y"),
          SphereShape(_r(SOCK_X, SOCK_Y0 - 0.020 - BULB_R + 0.006, SOCK_Z), BULB_R, BULB)]
    # the shade: green glass outside, white inside (drawn), half a cylinder (upper half) along y
    n = 12
    for k in range(n):
        a0, a1 = math.pi * k / n, math.pi * (k + 1) / n
        p0 = (SHADE_X + SHADE_R * math.cos(a0), SHADE_Z + SHADE_R * math.sin(a0))
        p1 = (SHADE_X + SHADE_R * math.cos(a1), SHADE_Z + SHADE_R * math.sin(a1))
        am = (a0 + a1) / 2
        nrm = (math.cos(am), 0.0, math.sin(am))
        w = 2 * SHADE_R * math.sin(math.pi / n / 2) + 0.0015
        c0, c1 = np.array(_r(p0[0], LAMP_Y, p0[1])), np.array(_r(p1[0], LAMP_Y, p1[1]))
        mid = (c0 + c1) / 2
        sh.append(plate(mid - [0, SHADE_L / 2, 0], mid + [0, SHADE_L / 2, 0], w, SHADE_T, nrm, GLASS))
        inner = mid - np.array(nrm) * (SHADE_T / 2 + 0.0004)
        sh.append(plate(inner - [0, SHADE_L / 2 - 0.002, 0], inner + [0, SHADE_L / 2 - 0.002, 0], w, 0.0004, nrm,
                        GLASS_IN, collide=False))
    # brass end caps (half discs, as steps) and the brass rim along both lips
    for side in (-1, 1):
        y = LAMP_Y + side * (SHADE_L / 2 + 0.001)
        for k in range(6):
            z0, z1 = SHADE_R * k / 6, SHADE_R * (k + 1) / 6
            half = SHADE_R * math.sqrt(max(0.0, 1 - ((z0 + z1) / 2 / SHADE_R) ** 2))
            sh.append(BoxShape(_r(SHADE_X, y, SHADE_Z + (z0 + z1) / 2), (2 * half, 0.002, z1 - z0), BRASS,
                               collide=False))
    for xs in (SHADE_X - SHADE_R, SHADE_X + SHADE_R):
        sh.append(CylShape(_r(xs, LAMP_Y, SHADE_Z), 0.0025, SHADE_L, BRASS, collide=False, axis="Y"))
    return sh


def _at(origin, x: float, y: float, z: float) -> tuple:
    """A world point (x, y, height above the table) relative to a body's origin (world)."""
    return (x - origin[0], y - origin[1], TOP + z - origin[2])


PLUNGER_O = _v((CHAIN_X, CHAIN_Y, TOP + CHAIN_Z + 0.008))  # inside the socket's shell, over the bushing
SEG_O = [_v((CHAIN_X, CHAIN_Y, TOP + CHAIN_Z - i * SEG_L)) for i in range(N_SEG)]  # each length's top joint


def _segment(i: int) -> Body:
    o = SEG_O[i]
    z_top = CHAIN_Z - i * SEG_L
    sh = [SphereShape(_at(o, CHAIN_X, CHAIN_Y, z_top - k * PITCH), BEAD_R, BRASS) for k in range(1, PER_SEG + 1)]
    if i == N_SEG - 1:  # the bell pull on the last bead: a small cap, the bell, its rim
        zc = CHAIN_END - 0.001
        sh += [CylShape(_at(o, CHAIN_X, CHAIN_Y, zc), 0.0018, 0.003, BRASS),
               CylShape(_at(o, CHAIN_X, CHAIN_Y, zc - 0.0045), 0.0035, 0.006, BRASS),
               CylShape(_at(o, CHAIN_X, CHAIN_Y, BELL_MID - 0.002), BELL_R, BELL_L - 0.006, BRASS),
               CylShape(_at(o, CHAIN_X, CHAIN_Y, zc - BELL_L + 0.0015), BELL_R + 0.0004, 0.003, DARK_BRASS)]
    parent = "pull_plunger" if i == 0 else f"chain_{i - 1}"
    lim = RIM_DEG if i == 0 else 80.0  # the top length stops on the bushing's rim
    return Body.build(f"chain_{i}", pos=o, mass=SEG_MASS if i < N_SEG - 1 else BELL_SEG_MASS, shapes=sh,
                      color=BRASS, friction=(0.5, 0.4), damping=CHAIN_DAMP, contact_offset=0.001,
                      solver_iterations=32, joint=Joint(anchor=o, kind="ball", parent=parent, limits=(lim,)))


LAMP = Body.build("bankers_lamp", pos=ORIGIN, mass=LAMP_MASS, color=BRASS, friction=(0.5, 0.4),
                  shapes=_base_shapes(), contact_offset=0.001, solver_iterations=32)
# The switch's plunger: a part inside the shell on a slide (straight up and down, 0-13 mm), its spring preloaded
# against the top stop (the spring's anchor 10 mm above it: 3.0 N).
PLUNGER = Body.build("pull_plunger", pos=PLUNGER_O, mass=PLUNGER_MASS, color=DARK_BRASS, damping=CHAIN_DAMP,
                     shapes=[SphereShape((0.0, 0.0, 0.0), 0.003, DARK_BRASS)], contact_offset=0.001,
                     solver_iterations=32,
                     joint=Joint(anchor=PLUNGER_O, kind="slide", axis="Z", parent="bankers_lamp",
                                 axial=(K_SPRING, 0.6, TRAVEL_MAX), slide_range=(-TRAVEL_MAX, 0.0), slide_target=PRE))
CHAIN = [_segment(i) for i in range(N_SEG)]
OBJECTS = [LAMP, PLUNGER, *CHAIN]

# -- tracking and scoring -------------------------------------------------------------------------------------------
TIP_DEG, UPRIGHT_DEG, MOVE_OK = 25.0, 5.0, 0.02
CLEAR = 0.010
BACK_UP = 2.0 * MM
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = os.environ.get("PCHAIN_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("PCHAIN_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.clear()
    _EP.update(tick=0, step=0, counted=-1, on=False, clicks=0, armed=True, tipped=False, jammed_at_click=0,
               recent=[], near=1.0, gaps={}, tcp={}, events=[], peak=0.0, max_tilt=0.0, max_angle=0.0, lit=None)


_reset()


def _finger_points() -> dict:
    import struct
    import xml.etree.ElementTree as ET

    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    root = pathlib.Path(YAM_URDF).parent
    out = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        name = link.get("name")
        if name not in ("tip_left", "tip_right"):
            continue
        pts = []
        for col in link.findall("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
            rpy = np.array([float(v) for v in (o.get("rpy") or "0 0 0").split()]) if o is not None else np.zeros(3)
            b = (root / col.find("geometry").find("mesh").get("filename")).read_bytes()
            n = struct.unpack("<I", b[80:84])[0]
            tri = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
            v = tri["v"].reshape(-1, 3).astype(float)
            pts.append(np.unique(np.round(v, 4), axis=0) @ rpy_matrix(rpy).T + xyz)
        out[name] = np.concatenate(pts)
    return out


def _shaders() -> dict:
    """The lamp's glass and bulb shaders (UsdPreviewSurface), found by their diffuse colour."""
    import omni.usd
    from pxr import UsdShade

    from yam_arena.station import linear_rgb

    stage = omni.usd.get_context().get_stage()
    want = {"glass": np.array(linear_rgb(GLASS)), "bulb": np.array(linear_rgb(BULB))}
    out: dict = {"glass": [], "bulb": []}
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if "/bankers_lamp/Looks/" not in p or not p.endswith("/Shader"):
            continue
        inp = UsdShade.Shader(prim).GetInput("diffuseColor")
        if not inp:
            continue
        c = np.array(inp.Get(), float)
        for k, w in want.items():
            if np.abs(c - w).max() < 1e-4:
                out[k].append(UsdShade.Shader(prim))
    return out


def _set_light(on: bool) -> None:
    from pxr import Gf, Sdf

    if _EP["lit"] == on:
        return
    if "shaders" not in _IDS:
        _IDS["shaders"] = _shaders()
    for key, glow in (("glass", GLOW_GLASS), ("bulb", GLOW_BULB)):
        for sh in _IDS["shaders"][key]:
            inp = sh.GetInput("emissiveColor") or sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f)
            inp.Set(Gf.Vec3f(*(glow if on else (0.0, 0.0, 0.0))))
    _EP["lit"] = on


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("PCHAIN_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    from scipy.spatial import cKDTree

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    if "fingers" not in _IDS:
        _IDS["fingers"] = _finger_points()
    fingers = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
            _IDS[side + "_tips"] = [robot.body_names.index(n) for n in ("tip_left", "tip_right")]
        q = as_numpy(robot.data.joint_pos)[0]
        _EP["tcp"][side] = np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float)
        _EP["gaps"][side] = float(-q[_IDS[side + "_f"]].sum())
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        fingers[side] = np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                        for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])])
    if "chain_pts" not in _IDS:
        pts = {}
        for i in range(N_SEG):
            zs = [CHAIN_Z - i * SEG_L - k * PITCH for k in range(0, PER_SEG + 1)]
            if i == N_SEG - 1:
                zs += list(np.linspace(CHAIN_END - 0.001, CHAIN_END - 0.001 - BELL_L, 8))
            pts[i] = np.array([_at(SEG_O[i], CHAIN_X, CHAIN_Y, z) for z in zs])
        _IDS["chain_pts"] = pts
        _IDS["plunger0"] = np.asarray(PLUNGER_O, float) - np.asarray(ORIGIN, float)  # in the lamp's frame
        _set_light(False)

    def pose(name):
        q = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        return q[:3] - origin, rotation(q[3:7])

    p, r = pose("bankers_lamp")
    pp, _ = pose("pull_plunger")
    trav = -float((r.T @ (pp - p))[2] - _IDS["plunger0"][2])  # down from the top stop
    _, r0 = pose("chain_0")
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(r0[:, 2] @ r[:, 2])))))  # top length vs the lamp
    _EP["peak"] = max(_EP["peak"], trav)
    _EP["max_angle"] = max(_EP["max_angle"], ang)
    if trav < REARM:
        _EP["armed"] = True
    if _EP["armed"] and trav >= CLICK:
        if ang > JAM_DEG:
            _EP["jammed_at_click"] += 1  # bound on the bushing: the switch does not turn, and this pull is spent
            _EP["armed"] = False
            _EP["events"].append((_EP["tick"], f"jammed at {ang:.0f} deg"))
        else:
            _EP["clicks"] += 1
            _EP["on"] = not _EP["on"]
            _EP["armed"] = False
            _EP["events"].append((_EP["tick"], f"click {_EP['clicks']} -> {'on' if _EP['on'] else 'off'} "
                                               f"({ang:.0f} deg)"))
    _set_light(_EP["on"])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))
    _EP["max_tilt"] = max(_EP["max_tilt"], tilt)
    if tilt > TIP_DEG and not _EP["tipped"]:
        _EP["tipped"] = True
        _EP["events"].append((_EP["tick"], f"tipped {tilt:.0f} deg"))
    o = np.asarray(ORIGIN, float)
    cpts = []
    for i in range(N_SEG):
        sp, sr = pose(f"chain_{i}")
        cpts.append(sp + _IDS["chain_pts"][i] @ sr.T)
    cpts = np.concatenate(cpts)
    tree = cKDTree(cpts)
    near = {s: float(tree.query(fp, k=1)[0].min()) for s, fp in fingers.items()}
    _EP.update(trav=trav, ang=ang, p=p, tilt=tilt, near=min(near.values()), near_s=near,
               moved=float(np.linalg.norm(p[:2] - o[:2])), bell=cpts[-1])


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"] and "p" in _EP:
        _EP["counted"] = _EP["tick"]
        recent.append(np.asarray(_EP["p"], float).copy())
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    released = _EP["near"] >= CLEAR and _EP.get("trav", 0.0) <= BACK_UP
    upright = _EP.get("tilt", 0.0) < UPRIGHT_DEG
    in_place = _EP.get("moved", 0.0) <= MOVE_OK
    n = _EP["clicks"]
    if _EP["tipped"]:
        s = 0.0
    elif _EP["on"] and n == 1 and released and upright and in_place and still:
        s = 1.0
    elif _EP["on"]:
        s = 0.5
    elif n >= 2:
        s = 0.3
    else:
        s = 0.0
    return dict(still=still, released=released, upright=upright, in_place=in_place, score=s)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    p = _EP.get("p", np.zeros(3))
    b = _EP.get("bell", np.zeros(3))
    print(f"DBG step={_EP['step']} trav={_EP.get('trav', 0) * 1000:.1f}mm peak={_EP['peak'] * 1000:.1f} "
          f"ang={_EP.get('ang', 0):.1f} on={int(_EP['on'])} clicks={_EP['clicks']} jam={_EP['jammed_at_click']} "
          f"lamp=({p[0]:.3f},{p[1]:.3f},{p[2] - TOP:.4f}) tilt={_EP.get('tilt', 0):.1f} max_tilt={_EP['max_tilt']:.1f} "
          f"moved={_EP.get('moved', 0) * 1000:.1f}mm bell=({b[0]:.3f},{b[1]:.3f},{b[2] - TOP:.4f}) "
          f"near={_EP['near'] * 1000:.0f}mm still={int(t['still'])} rel={int(t['released'])} score={t['score']:.2f} "
          f"ev={_EP['events'][-3:]} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    return float(t["score"])


def success(objects) -> bool:
    return bool(_tally(objects)["score"] >= 0.999)


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The left gripper, level and pointing forward (+x), fingers open across y, comes in
# under the shade's front lip at the bell's height, so the bell hangs between its open fingers, and closes on the bell
# (a 6 mm commanded gap on the 11 mm bell: the fingers stop on it at about 11 mm and squeeze). It pulls straight down,
# commanded 50 mm: the level arm, soft at its wrist, follows about 20 mm against the switch's 3-7 N, which takes the
# plunger to its 13 mm stop (the click at 10 mm). It holds 0.6 s, lets the chain back up by rising to where it caught
# it, opens and backs out.
# PCHAIN_WRONG=side: the same catch, then a yank 8 cm out toward the robot, then 3 cm down.
# PCHAIN_WRONG=double: the same straight pull, done twice.
OPEN_C = -0.047
WRONG = os.environ.get("PCHAIN_WRONG", "")
GRIP = float(os.environ.get("PCHAIN_GRIP", "-0.003"))  # finger value: a 6 mm commanded gap on the 11 mm bell
PULL = float(os.environ.get("PCHAIN_PULL", "0.050"))
TCP_DX = float(os.environ.get("PCHAIN_DX", "-0.005"))  # the TCP this far behind the chain's line (x)
TCP_DZ = float(os.environ.get("PCHAIN_DZ", "0.0"))  # and this far above the bell's middle
ROT_LEVEL = np.column_stack([(0.0, -1.0, 0.0), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)])  # left: level, toward +x
ROT_DOWN_L = grip_rotation(-math.pi / 2)
ROT_LEVEL_R = np.column_stack([(0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)])  # right: level, toward +x


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (0.0,), (-1.8, -0.9, 0.0, 0.9, 1.8)):
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


def _stay(arm: str, t: float = 120.0) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


CATCH = np.array([CHAIN_X + TCP_DX, CHAIN_Y, TOP + BELL_MID + TCP_DZ])
OUT = CATCH + np.array([-0.09, 0.0, 0.0])
SAFE = np.array([0.19, 0.17, TOP + 0.22])  # the lead-in's end: clear of the lamp (its shade ends at y 0.125)


def _pull(label: str) -> list:
    down = CATCH - np.array([0.0, 0.0, PULL])
    return [_w(2.5, down, GRIP, label, ROT_LEVEL),
            _w(0.6, down, GRIP, "hold it at the bottom a moment", ROT_LEVEL),
            _w(1.5, CATCH, GRIP, "let the chain back up", ROT_LEVEL),
            _w(0.5, CATCH, GRIP, "settle", ROT_LEVEL)]


def _left() -> list:
    out = _lead_in("left", SAFE, ROT_LEVEL, OPEN_C, "level, to the left of and in front of the lamp, open")
    out += [_w(2.0, OUT + np.array([0.0, 0.0, 0.03]), OPEN_C, "in front of the bell", ROT_LEVEL),
            _w(1.5, OUT, OPEN_C, "down to the bell's height", ROT_LEVEL),
            _w(2.5, CATCH, OPEN_C, "in under the shade, the bell between the open fingers", ROT_LEVEL),
            _w(0.5, CATCH, OPEN_C, "let it settle", ROT_LEVEL),
            _w(1.2, CATCH, GRIP, "pinch the bell", ROT_LEVEL),
            _w(0.5, CATCH, GRIP, "settle", ROT_LEVEL)]
    if WRONG == "side":
        out_ = CATCH + np.array([-0.08, 0.0, 0.0])
        yank = out_ + np.array([0.0, 0.0, -0.03])
        out += [_w(1.0, out_, GRIP, "yank the chain out toward the robot", ROT_LEVEL),
                _w(0.8, yank, GRIP, "and down", ROT_LEVEL),
                _w(0.8, yank, GRIP, "hold", ROT_LEVEL),
                _w(1.5, CATCH, GRIP, "back", ROT_LEVEL)]
    else:
        out += _pull("pull the chain straight down past the click")
        if WRONG == "double":
            out += _pull("pull it again")
    out += [_w(1.0, CATCH, OPEN_C, "let go", ROT_LEVEL),
            _w(2.0, OUT, OPEN_C, "back out from under the shade", ROT_LEVEL),
            _w(1.5, OUT + np.array([0.0, 0.0, 0.03]), OPEN_C, "rise clear", ROT_LEVEL),
            _w(10.0, OUT + np.array([0.0, 0.0, 0.03]), OPEN_C, "stay clear", ROT_LEVEL)]
    return out


def demo():
    return {"left": _left(), "right": _stay("right")}
