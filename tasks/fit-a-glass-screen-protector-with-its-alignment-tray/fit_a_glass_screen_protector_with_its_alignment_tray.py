"""On the station table: an iPhone 16 lies screen up on the desk, no case; beside it lies a tempered-glass screen
protector kit with an alignment tray (the Spigen Glas.tR EZ Fit kind): the glass is held in the tray by the tray's
sticker "2", and the tray lies skirt down on its back film "1", whose pull tab stands up at the tray's -y end.
Task (instruction): fit the screen protector to the phone with its tray and leave the empty tray off the phone.
The instruction does not say how.

The kit's steps (Spigen: "Simply place the tray over the phone, press the center and watch the glass adhere in
place"; a review of the kit: "put the installation tray on the screen after removing the backing tape on the screen
protector", "Wait 30 seconds then pull the tab off"): peel the back film "1" off by its tab, set the tray level round
the phone, press the middle of the glass, wait 30 s, peel the sticker "2" off by its tab, lift the tray off.

The hand skill: film and sticker are held by adhesive, so a tab pulled with one hand drags the 25 g tray (and, seated,
the phone) along with it: one hand holds the tray while the other peels each tab. The tray has to come down level and
square round the phone (0.4 mm to spare a side, a 3 mm lead-in at its skirt's bottom edge); the glass is pressed down
in the middle through the tray's window; after the wait the sticker is peeled off the held tray; then the tray is
lifted straight off, leaving the glass on the screen.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- phone: loose, Apple iPhone 16, 71.6 x 147.6 x 7.8 mm, 170 g, long side along y (its top end toward +y), screen up;
  centre at (0.255, -0.090). Its screen's top face is 7.8 mm above the desk.
- tray: loose, 25 g, clear plastic (drawn grey). Outside 78.4 x 154.4 mm; a skirt that fits round the phone with
  0.4 mm to spare a side, 4.5 mm deep below the seat, its bottom inside edge chamfered 3 mm; a seat frame (inside
  68.8 x 144.8 mm) that rests on the phone's 1.8 mm black border; a 2 mm deck above with a 54 x 120 mm press window;
  a flat handle 30 x 6 mm at each end. Lies skirt down on the film "1", centre at (0.250, 0.090), long side along y.
- glass: 68 x 144 x 0.5 mm, 7 g, held in the tray's pocket by the sticker "2": it hangs 0.4 mm above the phone's
  screen when the tray is seated and is pressed down onto it through the window.
- film "1": 0.5 mm sheet under the whole tray with a pull tab (20 x 22 mm, 1.5 mm thick) standing up 4 mm past the
  tray's -y end, 22 mm toward -x from its middle line. Its adhesive is a slide joint (the peel) along the tray with
  25 N s/m of drag that lets go only at the end of the peel (165 mm) and only to a pull of 2 N or more.
- sticker "2": 8 mm wide strip along the deck's +x side (31.6 mm from the middle line) with a pull tab standing up past
  the tray's -y end. It holds the glass (the glass hangs from it on a 200 N/m spring, its sideways hold lets go at
  1.5 N); its own adhesive is a slide joint along the tray, 25 N s/m, that lets go at the end of 50 mm of peel to a
  pull of 2 N or more.
- adhesion (a rule): glass that has been pressed onto the screen sticks where it lies once the sticker has let it go
  and it lies flat on the screen; from then on it moves with the phone.

Score (judged at the end; the events are tracked over the run):
- film off: the film's sheet no longer under the glass (its centre 110 mm or more from the glass's centre, or 10 mm
  or more below it).
- seated: the tray within 1.5 mm and 2 deg of centred on the phone, resting on it (within 1 mm).
- pressed: the glass pressed onto the screen with 5 N or more (PhysX contact force) by a finger whose centre is within
  20 mm of the glass's centre, with the tray seated and the film off.
- sticker off: its centre 100 mm or more from the tray's centre, or 15 mm or more above it.
- waited: the sticker came off 30 s or more after the first press, with the tray seated then.
- glass on: the glass lies on the screen, flat (every corner within 0.5 mm of the screen), every corner within 1.5 mm
  of its true place along each edge (equal 1.8 mm borders), and the sticker off.
- tray off: no part of the tray within 10 mm of the phone's outline below 20 mm above its screen; hands clear: both
  grippers' centres 30 mm or more from the tray, the glass and the phone; still: tray, glass and phone moved under
  1 mm in the last 1 s.
1.0: film off, glass on, pressed (before the sticker came off), waited, tray off, hands clear, still.
0.75: the same, but the tray still on the phone or the hands not clear or not still.
0.5: glass on and pressed, film off, but the sticker came off less than 30 s after the press (or before it).
0.25: film off only.
0: the film still on the tray.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Joint, allow_matrix_grips, as_numpy, on_reset, on_step, plate  # noqa: E402
from bodykit import _quat_from_matrix, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

# -- the layout ----------------------------------------------------------------------------------------------------
PHONE_XY = (0.255, -0.090)
PHONE_W, PHONE_L, PHONE_T = 0.0716, 0.1476, 0.0078
PHONE_R = 0.009  # corner radius (in plan)
TRAY_XY = (0.250, 0.090)
CLEAR_SIDE = 0.0004  # the skirt's play round the phone, a side
IN_W, IN_L = PHONE_W + 2 * CLEAR_SIDE, PHONE_L + 2 * CLEAR_SIDE  # 72.4 x 148.4 inside the skirt
WALL = 0.0015  # skirt wall above the chamfer
CHAMF = 0.003  # the chamfer at the skirt's bottom edge (45 deg): the lead-in
OUT_W, OUT_L = IN_W + 2 * (WALL + 0.0015), IN_L + 2 * (WALL + 0.0015)  # 78.4 x 154.4 outside
SKIRT_D = 0.0045  # skirt depth below the seat
SEAT_H = 0.0015  # the seat frame (rests on the phone's border), 0 .. 1.5 mm
RECESS_W, RECESS_L = 0.0688, 0.1448  # inside the seat frame: the glass's pocket
DECK_Z0, DECK_Z1 = SEAT_H, 0.0035  # the deck, above the seat frame
WIN_W, WIN_L = 0.054, 0.120  # the press window
GLASS_W, GLASS_L, GLASS_T = 0.068, 0.144, 0.0005
GLASS_GAP = 0.0004  # the glass hangs this far above the screen with the tray seated (on its spring)
GLASS_SPRING_REST = 0.0008  # unloaded, the spring holds it 0.8 mm up; its own weight sags it 0.34 mm
FILM_T = 0.0005
FILM_Z = -SKIRT_D - FILM_T / 2  # film sheet centre in the tray frame
TAB_H = 0.022  # the pull tabs stand this tall
TAB_W = 0.020
TAB_T = 0.0015  # the tabs are folded-over film with a stiff card label: 1.5 mm
FILM_TAB_X = -0.022  # the film's tab, across the tray (tray frame)
STRIP_X = 0.0316  # the sticker's line on the deck (tray frame)
STRIP_W = 0.008
TAB_Y = -(OUT_L / 2 + 0.004)  # both tabs stand just past the tray's -y end
FILM_RUN = 0.165  # the film lets go at the end of this much peel
STRIP_RUN = 0.050  # the sticker lets go at the end of this much peel
PEEL_C = float(os.environ.get("FITGLASS_PEEL_C", "25"))  # N s/m: the adhesive's drag along the peel
LET_GO_N = float(os.environ.get("FITGLASS_LETGO", "2.0"))  # N: the last of the adhesive lets go to this pull
GLASS_HOLD_N = 1.5  # N: the sticker's sideways hold on the glass
TRAY_Z0 = TOP + FILM_T + SKIRT_D  # the tray frame's origin (the seat plane) on the desk, on its film

PHONE_FRAME, SCREEN, BEZEL, TRAY_C, GLASS_C, FILM_C, STICKER_C, TAB1_C, TAB2_C = (
    (0.40, 0.58, 0.58), (0.04, 0.05, 0.07), (0.02, 0.02, 0.02), (0.42, 0.44, 0.48), (0.62, 0.82, 0.95),
    (0.92, 0.92, 0.88), (0.95, 0.80, 0.20), (0.90, 0.30, 0.25), (0.20, 0.45, 0.85))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _phone() -> Body:
    w, l, t, r = PHONE_W, PHONE_L, PHONE_T, PHONE_R
    sh = [BoxShape((0.0, 0.0, t / 2), (w - 2 * r, l, t), PHONE_FRAME),
          BoxShape((0.0, 0.0, t / 2), (w, l - 2 * r, t), PHONE_FRAME)]
    for sx, sy in itertools.product((-1, 1), (-1, 1)):
        sh.append(CylShape((sx * (w / 2 - r), sy * (l / 2 - r), t / 2), r, t, PHONE_FRAME))
    sh += [BoxShape((0.0, 0.0, t + 0.00005), (w - 0.0012, l - 0.0012, 0.0001), BEZEL, collide=False),
           BoxShape((0.0, 0.0, t + 0.0001), (w - 0.0050, l - 0.0090, 0.0001), SCREEN, collide=False),
           BoxShape((0.0, l / 2 - 0.007, t + 0.00015), (0.020, 0.006, 0.0001), BEZEL, collide=False)]  # the island
    return Body.build("phone", pos=(PHONE_XY[0], PHONE_XY[1], TOP + 0.0005), mass=0.170, color=PHONE_FRAME,
                      friction=(0.5, 0.45), shapes=sh, contact_offset=0.001, solver_iterations=32)


def _chamfer(side: str, sgn: float) -> BoxShape:
    """The 45-deg lead-in plate along one wall's bottom inside edge (tray frame)."""
    th = 0.0015
    if side == "x":  # a long wall at x = sgn * IN_W / 2
        a = np.array([sgn * IN_W / 2, 0.0, -SKIRT_D + CHAMF])
        b = np.array([sgn * (IN_W / 2 + CHAMF), 0.0, -SKIRT_D])
        n_in = np.array([sgn * 1.0, 0.0, 1.0]) / math.sqrt(2)  # into the wall's material
        c = (a + b) / 2 + n_in * th / 2
        return plate(c - (b - a) / 2, c + (b - a) / 2, IN_L + 2 * CHAMF, th, n_in, TRAY_C)
    a = np.array([0.0, sgn * IN_L / 2, -SKIRT_D + CHAMF])
    b = np.array([0.0, sgn * (IN_L / 2 + CHAMF), -SKIRT_D])
    n_in = np.array([0.0, sgn * 1.0, 1.0]) / math.sqrt(2)
    c = (a + b) / 2 + n_in * th / 2
    return plate(c - (b - a) / 2, c + (b - a) / 2, IN_W + 2 * CHAMF, th, n_in, TRAY_C)


def _tray() -> Body:
    sh = []
    zw0, zw1 = -SKIRT_D + CHAMF, DECK_Z1  # upper inner wall (above the chamfer)
    for s in (-1, 1):
        # long walls: inner part above the chamfer, outer part full height
        sh.append(BoxShape((s * (IN_W / 2 + WALL / 2), 0.0, (zw0 + zw1) / 2), (WALL, IN_L, zw1 - zw0), TRAY_C))
        sh.append(BoxShape((s * (OUT_W / 2 - 0.00075), 0.0, (-SKIRT_D + zw1) / 2), (0.0015, OUT_L, zw1 + SKIRT_D),
                           TRAY_C))
        sh.append(_chamfer("x", s))
        # end walls
        sh.append(BoxShape((0.0, s * (IN_L / 2 + WALL / 2), (zw0 + zw1) / 2), (IN_W, WALL, zw1 - zw0), TRAY_C))
        sh.append(BoxShape((0.0, s * (OUT_L / 2 - 0.00075), (-SKIRT_D + zw1) / 2), (OUT_W, 0.0015, zw1 + SKIRT_D),
                           TRAY_C))
        sh.append(_chamfer("y", s))
        # seat frame round the glass's pocket (rests on the phone's border)
        fw = (IN_W - RECESS_W) / 2
        fl = (IN_L - RECESS_L) / 2
        sh.append(BoxShape((s * (RECESS_W / 2 + fw / 2), 0.0, SEAT_H / 2), (fw, IN_L, SEAT_H), TRAY_C))
        sh.append(BoxShape((0.0, s * (RECESS_L / 2 + fl / 2), SEAT_H / 2), (RECESS_W, fl, SEAT_H), TRAY_C))
        # deck round the window
        dz, dh = (DECK_Z0 + DECK_Z1) / 2, DECK_Z1 - DECK_Z0
        sh.append(BoxShape((s * (WIN_W / 2 + (IN_W / 2 - WIN_W / 2) / 2), 0.0, dz), (IN_W / 2 - WIN_W / 2, IN_L, dh),
                           TRAY_C))
        sh.append(BoxShape((0.0, s * (WIN_L / 2 + (IN_L / 2 - WIN_L / 2) / 2), dz), (WIN_W, IN_L / 2 - WIN_L / 2, dh),
                           TRAY_C))
        # flat end handle
        sh.append(BoxShape((0.0, s * (OUT_L / 2 + 0.003), DECK_Z1 - 0.001), (0.030, 0.006, 0.002), TRAY_C))
    return Body.build("tray", pos=(TRAY_XY[0], TRAY_XY[1], TRAY_Z0), mass=0.025, color=TRAY_C, friction=(0.5, 0.45),
                      shapes=sh, contact_offset=0.001, solver_iterations=32,
                      joint=None)


TRAY_BODY = _tray()


def _film() -> Body:
    """The back film '1' under the tray, its tab standing up past the tray's -y end. Frame: the tray's."""
    fw, fl = OUT_W + 0.002, OUT_L + 0.002
    zt = FILM_T / 2
    sh = [BoxShape((0.0, 0.0, 0.0), (fw, fl, FILM_T), FILM_C),
          # the tail out to the tab, then the tab standing up
          BoxShape((FILM_TAB_X, (-fl / 2 + TAB_Y) / 2, 0.0), (TAB_W, abs(TAB_Y + fl / 2) + 0.0005, FILM_T), TAB1_C),
          BoxShape((FILM_TAB_X, TAB_Y, zt + TAB_H / 2), (TAB_W, TAB_T, TAB_H), TAB1_C),
          BoxShape((FILM_TAB_X, TAB_Y - TAB_T / 2 - 0.00005, zt + TAB_H * 0.6), (0.006, 0.0001, 0.008), BEZEL, collide=False)]
    pos = (TRAY_XY[0], TRAY_XY[1], TOP + FILM_T / 2 + 0.0002)
    return Body.build("film", pos=pos, mass=0.003, color=FILM_C, friction=(0.6, 0.5), shapes=sh, contact_offset=0.001,
                      joint=Joint(anchor=pos, kind="slide", axis="Y", parent="tray", axial=(0.0, PEEL_C, FILM_RUN),
                                  slide_range=(-FILM_RUN, 0.0), break_force=LET_GO_N))


def _strip() -> Body:
    """The sticker '2' along the deck's +x side, its tab standing up past the tray's -y end. Frame origin: the
    sticker's middle on the deck."""
    run = OUT_L  # over the deck and the end walls
    sh = [BoxShape((0.0, 0.0, 0.0), (STRIP_W, run, 0.0003), STICKER_C),
          BoxShape((0.0, (-run / 2 + TAB_Y) / 2, 0.0), (STRIP_W, abs(TAB_Y + run / 2) + 0.0005, 0.0003), STICKER_C),
          BoxShape((0.0, TAB_Y, TAB_H / 2), (TAB_W, TAB_T, TAB_H), TAB2_C),
          BoxShape((0.0, TAB_Y - TAB_T / 2 - 0.00005, TAB_H * 0.6), (0.006, 0.0001, 0.008), BEZEL, collide=False)]
    pos = (TRAY_XY[0] + STRIP_X, TRAY_XY[1], TRAY_Z0 + DECK_Z1 + 0.00015)
    return Body.build("sticker", pos=pos, mass=0.001, color=STICKER_C, friction=(0.6, 0.5), shapes=sh,
                      contact_offset=0.001,
                      joint=Joint(anchor=pos, kind="slide", axis="Y", parent="tray", axial=(0.0, PEEL_C, STRIP_RUN),
                                  slide_range=(-STRIP_RUN, 0.0), break_force=LET_GO_N))


def _glass() -> Body:
    z = TRAY_Z0 + GLASS_SPRING_REST + GLASS_T / 2
    pos = (TRAY_XY[0], TRAY_XY[1], z)
    sh = [BoxShape((0.0, 0.0, 0.0), (GLASS_W, GLASS_L, GLASS_T), GLASS_C)]
    # The sticker holds the glass up in the tray's pocket: a vertical spring (200 N/m) within 0 .. -1.2 mm, and a
    # sideways hold that lets go at 1.5 N.
    return Body.build("glass", pos=pos, mass=0.007, color=GLASS_C, friction=(0.6, 0.5), shapes=sh,
                      contact_offset=0.0005, solver_iterations=32, report_contacts=True,
                      joint=Joint(anchor=pos, kind="slide", axis="Z", parent="sticker", axial=(200.0, 0.5, 0.0012),
                                  slide_range=(-0.0012, 0.0), break_force=GLASS_HOLD_N))


PHONE = _phone()
TRAY = TRAY_BODY
FILM = _film()
STRIP = _strip()
GLASS = _glass()
OBJECTS = [PHONE, TRAY, FILM, STRIP, GLASS]

# -- tracking and scoring -------------------------------------------------------------------------------------------
PRESS_N = 5.0
PRESS_R = 0.020
WAIT_S = 30.0
SEAT_XY, SEAT_DEG, SEAT_Z = 1.5 * MM, 2.0, 1.0 * MM
GLASS_TOL = 1.5 * MM
FLAT_TOL = 0.5 * MM
CLEAR = 0.030
STILL_STEPS, STILL_MM = 20, 1.0
CONTROL_HZ = 20.0
_DEBUG = os.environ.get("FITGLASS_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("FITGLASS_DEBUG_EVERY", "10"))
_EP: dict = {}
_IDS: dict = {}
_CV: dict = {}

GLASS_CORNERS = np.array([[sx * GLASS_W / 2, sy * GLASS_L / 2, -GLASS_T / 2] for sx in (-1, 1) for sy in (-1, 1)])
TRAY_LO, TRAY_HI = TRAY.local_bounds()
TRAY_CORNERS = np.array([[x, y, z] for x in (TRAY_LO[0], TRAY_HI[0]) for y in (TRAY_LO[1], TRAY_HI[1])
                         for z in (TRAY_LO[2], TRAY_HI[2])])


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, counted=-1, film_off=False, pressed=False, t_press=None, released=False,
               t_release=None, released_seated=False, waited=False, force=0.0, peak_f=0.0, recent=[], tcp={}, gaps={},
               seated=False, info={}, lock=None)
    _CV.pop("view", None)


_reset()


def _glass_force(base) -> float:
    """The contact force (N) between the glass and the phone, from PhysX's pair-wise contact forces."""
    from isaaclab_physx.physics import PhysxManager

    sim = PhysxManager.get_physics_sim_view()
    view = _CV.get("view") if _CV.get("sim") is sim else None
    if view is None:
        env = base.scene.env_prim_paths[0]
        view = sim.create_rigid_contact_view([f"{env}/glass/body"], filter_patterns=[[f"{env}/phone/body"]],
                                             max_contact_data_count=64)
        _CV.update(sim=sim, view=view, dt=float(base.physics_dt))
    forces = view.get_contact_force_matrix(dt=_CV["dt"])
    a = forces.numpy() if hasattr(forces, "numpy") else as_numpy(forces)
    return float(np.linalg.norm(np.asarray(a, float).reshape(-1, 3), axis=-1).max())


def _pose(base, name, origin):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("FITGLASS_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _hold_glass(base, origin, pp, pr) -> None:
    import torch

    rel_p, rel_r = _EP["lock"]
    p = pp + pr @ rel_p + origin
    q = _quat_from_matrix(pr @ rel_r)
    obj = base.scene["glass"]
    pose = torch.tensor([[p[0], p[1], p[2], q[0], q[1], q[2], q[3]]], dtype=torch.float32, device=base.device)
    obj.write_root_link_pose_to_sim_index(root_pose=pose)
    obj.write_root_com_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32,
                                                                       device=base.device))


def _yaw(r) -> float:
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _track_inner(env) -> None:
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    t_now = _EP["tick"] / CONTROL_HZ
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0]
        _EP["tcp"][side] = np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float)
        _EP["gaps"][side] = float(-q[_IDS[side + "_f"]].sum())
    pp, pr = _pose(base, "phone", origin)
    tp, tr = _pose(base, "tray", origin)
    gp, gr = _pose(base, "glass", origin)
    fp, _ = _pose(base, "film", origin)
    sp, _ = _pose(base, "sticker", origin)
    f = _glass_force(base)
    top = pp + pr @ np.array([0.0, 0.0, PHONE_T])  # the screen's middle
    # tray in the phone's frame
    t_rel = pr.T @ (tp - top)
    t_yaw = (_yaw(pr.T @ tr) + 180.0) % 360.0 - 180.0
    t_tilt = math.degrees(math.acos(max(-1.0, min(1.0, float((pr.T @ tr)[2, 2])))))
    seated = (math.hypot(t_rel[0], t_rel[1]) <= SEAT_XY and abs(t_yaw) <= SEAT_DEG and abs(t_rel[2]) <= SEAT_Z
              and t_tilt <= SEAT_DEG)
    # film off: its sheet no longer under the glass
    d_fg = gp - fp
    film_off = math.hypot(d_fg[0], d_fg[1]) >= 0.110 or d_fg[2] >= 0.010
    if film_off:
        _EP["film_off"] = True
    # the sticker off the tray
    d_st = sp - tp
    rel_st = tr.T @ d_st
    released = math.hypot(d_st[0], d_st[1]) >= 0.100 or rel_st[2] >= DECK_Z1 + 0.015
    # a finger pressing at the glass's middle
    near_mid = min(math.hypot(*(v[:2] - gp[:2])) for v in _EP["tcp"].values())
    if (f >= PRESS_N and near_mid <= PRESS_R and seated and _EP["film_off"] and not _EP["pressed"]):
        _EP["pressed"], _EP["t_press"] = True, t_now
        _EP["pressed_before_release"] = not _EP["released"]
    if released and not _EP["released"]:
        _EP["released"], _EP["t_release"] = True, t_now
        _EP["released_seated"] = seated
        _EP["waited"] = bool(_EP["pressed"] and seated and t_now - _EP["t_press"] >= WAIT_S)
    # the glass on the screen, in the phone's frame
    corners = (gp + GLASS_CORNERS @ gr.T - top) @ pr
    true = GLASS_CORNERS[:, :2]
    off = float(np.max(np.abs(corners[:, :2] - true)))  # the worst border error at any corner, along either edge
    lift = float(np.max(np.abs(corners[:, 2])))
    glass_on = bool(off <= GLASS_TOL and lift <= FLAT_TOL and _EP["released"])
    # The adhesive: glass that was pressed onto the screen sticks where it lies once the sticker has let it go
    # (lying flat on the screen, within 5 mm of true): from then on it moves with the phone.
    if _EP["lock"] is None and _EP["pressed"] and _EP["released"] and lift <= FLAT_TOL and off <= 0.005:
        _EP["lock"] = (pr.T @ (gp - pp), pr.T @ gr)
        _EP["t_lock"] = t_now
    if _EP["lock"] is not None:
        _hold_glass(base, origin, pp, pr)
    # tray off the phone: no tray corner within 10 mm of the phone's outline below 20 mm above its screen
    tc = (tp + TRAY_CORNERS @ tr.T - top) @ pr
    lo_x, lo_y = PHONE_W / 2 + 0.010, PHONE_L / 2 + 0.010
    box_lo, box_hi = tc.min(0), tc.max(0)
    overlaps = (box_lo[0] < lo_x and box_hi[0] > -lo_x and box_lo[1] < lo_y and box_hi[1] > -lo_y
                and box_lo[2] < 0.020)
    tray_off = not overlaps
    # hands clear of tray, glass and phone (finger centre to each body's centre-region)
    clear = 1.0
    for v in _EP["tcp"].values():
        for c, r, half in ((tp, tr, np.array([OUT_W / 2, OUT_L / 2 + 0.006, 0.006])),
                           (gp, gr, np.array([GLASS_W / 2, GLASS_L / 2, 0.001])),
                           (top, pr, np.array([PHONE_W / 2, PHONE_L / 2, PHONE_T]))):
            loc = r.T @ (v - c)
            d = np.maximum(np.abs(loc) - half, 0.0)
            clear = min(clear, float(np.linalg.norm(d)))
    _EP.update(force=f, peak_f=max(_EP["peak_f"], f), seated=seated, t_now=t_now,
               info=dict(t_rel=t_rel, t_yaw=t_yaw, off=off, lift=lift, glass_on=glass_on, tray_off=tray_off,
                         clear=clear, near_mid=near_mid, film_off=film_off, released=released,
                         pts=np.concatenate([pp, tp, gp])))


def _tally(o) -> dict:
    recent = _EP["recent"]
    info = _EP["info"]
    if _EP["counted"] != _EP["tick"] and "pts" in info:
        _EP["counted"] = _EP["tick"]
        recent.append(np.asarray(info["pts"], float).copy())
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    glass_on = bool(info.get("glass_on", False))
    pressed_ok = bool(_EP["pressed"] and _EP.get("pressed_before_release", False))
    tray_away = bool(info.get("tray_off", False) and info.get("clear", 0.0) >= CLEAR and still)
    if not _EP["film_off"]:
        s = 0.0
    elif glass_on and _EP["pressed"] and pressed_ok and _EP["waited"]:
        s = 1.0 if tray_away else 0.75
    elif glass_on and _EP["pressed"]:
        s = 0.5
    else:
        s = 0.25
    return dict(still=still, score=s)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    i = _EP["info"]
    if not i:
        return
    tr = i["t_rel"] * 1000
    print(f"DBG step={_EP['step']} t={_EP.get('t_now', 0):.1f} f={_EP['force']:.2f}N peak={_EP['peak_f']:.2f} "
          f"tray_rel=({tr[0]:.2f},{tr[1]:.2f},{tr[2]:.2f})mm yaw={i['t_yaw']:.2f} seated={int(_EP['seated'])} "
          f"film_off={int(_EP['film_off'])} pressed={int(_EP['pressed'])}@{_EP['t_press']} "
          f"rel={int(_EP['released'])}@{_EP['t_release']} waited={int(_EP['waited'])} lock={int(_EP['lock'] is not None)} "
          f"glass_off={i['off'] * 1000:.2f}mm lift={i['lift'] * 1000:.2f}mm on={int(i['glass_on'])} "
          f"tray_off={int(i['tray_off'])} clear={i['clear'] * 1000:.0f}mm near_mid={i['near_mid'] * 1000:.0f}mm "
          f"still={int(t['still'])} score={t['score']} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    return float(t["score"])


def success(objects) -> bool:
    return _tally(objects)["score"] >= 1.0


# -- scripted demo --------------------------------------------------------------------------------------------------
OPEN_C = -0.047
CLOSED = 0.0
ONE = os.environ.get("FITGLASS_ONE") == "1"  # wrong way: the right hand alone, no hand holding the tray
R_L = grip_rotation(-math.pi / 2)  # left: fingers close along x (across the tray)
R_R = grip_rotation(0.0)  # right: fingers close along y (on the standing tabs)
R_RP = grip_rotation(math.pi / 2)  # right: fingers close along x (pressing)
TCP_ABOVE_SEAT = 0.0095  # the left TCP above the tray's seat plane while holding its walls
TRAY_PINCH = -(OUT_W / 2 - 0.005)  # 5 mm inside each long wall
GRIP_Y = float(os.environ.get("FITGLASS_GRIP_Y", "0.060"))  # the left hand holds the tray this far toward +y
TAB_TCP = 0.019  # the right TCP this far above the tray's seat plane on a tab (pads 6 mm and more above it)
TAB_OPEN = -0.015  # fingers 30 mm apart round a tab
PULL_OVER = float(os.environ.get("FITGLASS_PULL_OVER", "0.025"))  # pull this far past the let-go point
PRESS_DZ = float(os.environ.get("FITGLASS_PRESS_DZ", "0.012"))  # the pressing TCP this far below first touch
SEAT_DZ = float(os.environ.get("FITGLASS_SEAT_DZ", "0.003"))  # the tray pushed this far below seated
WAIT_DEMO = float(os.environ.get("FITGLASS_WAIT", "31.0"))


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


def _park(arm: str, t: float) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "wait at the ready pose", rot)]


def _hold(p, finger, label, rot, t) -> Waypoint:
    return _w(t, p, finger, label, rot)


# where things are, from the known layout
T0 = np.array([TRAY_XY[0], TRAY_XY[1], TRAY_Z0])
SEATED0 = np.array([PHONE_XY[0], PHONE_XY[1], TOP + 0.0005 + PHONE_T])  # the tray's seat plane on the phone (approx.)
LIFT_H = 0.060  # the left hand holds the tray this far up while the film is peeled


class _Arm:
    """One arm's waypoints and its clock."""

    def __init__(self, name):
        self.name, self.wps, self.t = name, [], 0.0

    def add(self, *wps):
        for w in wps:
            self.wps.append(w)
            self.t += w.duration

    def wait_until(self, t, label):
        if t > self.t + 1e-6:
            last = self.wps[-1]
            self.add(_w(t - self.t, last.pos, last.finger, label, last.yaw))


def _plan() -> dict:
    """Both arms' waypoints on one timeline (each list is padded with waits so the two stay in step)."""
    L, R = _Arm("left"), _Arm("right")
    up = np.array([0.0, 0.0, 0.05])
    z = lambda h: np.array([0.0, 0.0, h])  # noqa: E731
    # 1. left: take the tray across its width by its long walls and hold it up, level
    grip_t = T0 + np.array([0.0, GRIP_Y, TCP_ABOVE_SEAT])
    if ONE:
        L.add(*_park("left", 1.0))
    else:
        L.add(*_lead_in("left", grip_t + up, R_L, OPEN_C, "left over the tray, open"))
        L.add(_w(1.0, grip_t + up, OPEN_C, "left: settle over the tray", R_L),
              _w(1.0, grip_t + z(0.015), OPEN_C, "left: down to just above the walls", R_L),
              _w(0.5, grip_t + z(0.015), OPEN_C, "left: settle", R_L),
              _w(1.0, grip_t, OPEN_C, "left: pads down beside the long walls", R_L),
              _w(1.0, grip_t, TRAY_PINCH, "left: hold the tray across its width", R_L),
              _w(2.0, grip_t + z(LIFT_H), TRAY_PINCH, "left: lift the tray, level", R_L),
              _w(1.0, grip_t + z(LIFT_H), TRAY_PINCH, "left: hold it up", R_L))
    # 2. right: the film's tab, standing up past the tray's -y end (under the raised tray, or on the desk)
    lift = 0.0 if ONE else LIFT_H
    tab_t = np.array([TRAY_XY[0] + FILM_TAB_X, TRAY_XY[1] + TAB_Y, TRAY_Z0 + TAB_TCP + lift])
    R.add(*_lead_in("right", tab_t + z(0.04), R_R, TAB_OPEN, "right over the film's tab, open"))
    R.wait_until(L.t - 2.5, "right: wait for the left hand")
    R.add(_w(1.0, tab_t + z(0.04), TAB_OPEN, "right: settle over the tab", R_R),
          _w(1.0, tab_t, TAB_OPEN, "right: fingers down either side of the tab", R_R),
          _w(0.5, tab_t, TAB_OPEN, "right: settle", R_R))
    L.wait_until(R.t, "left: hold the tray up")
    R.add(_w(1.0, tab_t, CLOSED, "right: pinch the tab", R_R))
    # 3. peel the film: pull the tab along under the tray and past its end; the left holds the tray up
    pull_end = tab_t + np.array([0.0, -FILM_RUN - PULL_OVER, 0.0])
    n = 10
    for k in range(1, n + 1):
        R.add(_w(8.0 / n, tab_t + (pull_end - tab_t) * (k / n), CLOSED, "right: peel the film off by its tab", R_R))
    drop_film = np.array([0.13, -0.22, TOP + 0.08])
    R.add(_w(1.0, pull_end, CLOSED, "right: hold the pull", R_R),
          _w(2.0, drop_film, CLOSED, "right: carry the film aside", R_R),
          _w(0.8, drop_film, OPEN_C, "right: drop the film", R_R),
          _w(1.5, np.array([0.13, -0.24, TOP + 0.12]), OPEN_C, "right: out of the way", R_R))
    L.wait_until(R.t - 4.3, "left: hold the tray up while the film is peeled")
    # 4. left: carry the tray over the phone, lower it round the phone, let go
    seat_t = SEATED0 + np.array([0.0, GRIP_Y, TCP_ABOVE_SEAT])
    carry = [_w(3.0, seat_t + up, TRAY_PINCH, "left: carry it over the phone, level", R_L),
             _w(1.0, seat_t + up, TRAY_PINCH, "left: hold over the phone", R_L),
             _w(2.0, seat_t + z(0.006), TRAY_PINCH, "left: lower to just above the phone", R_L),
             _w(1.0, seat_t + z(0.006), TRAY_PINCH, "left: hold just above", R_L),
             _w(2.0, seat_t + z(-SEAT_DZ), TRAY_PINCH, "left: lower it round the phone", R_L),
             _w(0.8, seat_t + z(-SEAT_DZ), TRAY_PINCH, "left: hold it seated", R_L),
             _w(0.8, seat_t + z(-SEAT_DZ), OPEN_C, "left: let go of the walls", R_L),
             _w(1.2, seat_t + up, OPEN_C, "left: rise off the tray", R_L),
             _w(1.5, np.array([0.24, 0.17, TOP + 0.10]), OPEN_C, "left: out of the way", R_L)]
    if ONE:
        carry = [_w(sum(w.duration for w in carry), L.wps[-1].pos, OPEN_C, "left: stays", L.wps[-1].yaw)]
    L.add(*carry)
    R.wait_until(L.t, "right: wait")
    # 5. right: press the middle of the glass through the window
    g_top = SEATED0[2] + GLASS_GAP + GLASS_T
    touch = np.array([PHONE_XY[0], PHONE_XY[1], g_top + 0.013])
    R.add(_w(2.0, touch + z(0.04), CLOSED, "right: over the middle of the glass, closed", R_RP),
          _w(1.5, touch + z(0.004), CLOSED, "right: fingertips onto the glass", R_RP),
          _w(1.0, touch + z(-PRESS_DZ), CLOSED, "right: press the middle", R_RP),
          _w(2.0, touch + z(-PRESS_DZ), CLOSED, "right: hold the press", R_RP))
    t_press = R.t - 3.0
    R.add(_w(1.0, touch + z(0.04), CLOSED, "right: lift the fingertips", R_RP))
    # 6. the maker's 30 s wait; meanwhile the left comes back and holds the tray across its width
    hold_t = seat_t + z(-0.001)
    L.wait_until(R.t, "left: wait")
    back = [_w(2.0, hold_t + up, OPEN_C, "left: back over the tray", R_L),
            _w(1.0, hold_t + z(0.015), OPEN_C, "left: down to just above the walls", R_L),
            _w(0.5, hold_t + z(0.015), OPEN_C, "left: settle", R_L),
            _w(1.0, hold_t, OPEN_C, "left: pads down beside the long walls", R_L),
            _w(1.0, hold_t, TRAY_PINCH, "left: hold the tray on the phone", R_L)]
    if ONE:
        back = [_w(sum(w.duration for w in back), L.wps[-1].pos, OPEN_C, "left: stays", L.wps[-1].yaw)]
    L.add(*back)
    # 7. right: the sticker's tab, pulled after the wait
    s_tab = np.array([PHONE_XY[0] + STRIP_X, PHONE_XY[1] + TAB_Y, SEATED0[2] + TAB_TCP])
    R.add(_w(2.0, s_tab + z(0.04), TAB_OPEN, "right: over the sticker's tab", R_R))
    R.wait_until(max(t_press + WAIT_DEMO - 2.5, L.t), "right: wait 30 s from the press")
    R.add(_w(1.0, s_tab + z(0.04), TAB_OPEN, "right: settle over the sticker's tab", R_R),
          _w(1.0, s_tab, TAB_OPEN, "right: fingers down either side of the sticker's tab", R_R),
          _w(0.5, s_tab, TAB_OPEN, "right: settle", R_R),
          _w(1.0, s_tab, CLOSED, "right: pinch the tab", R_R))
    s_end = s_tab + np.array([0.0, -STRIP_RUN - PULL_OVER, 0.0])
    for k in range(1, n + 1):
        R.add(_w(4.0 / n, s_tab + (s_end - s_tab) * (k / n), CLOSED, "right: peel the sticker off by its tab", R_R))
    drop_st = np.array([0.13, -0.22, TOP + 0.08])
    R.add(_w(1.0, s_end, CLOSED, "right: hold the pull", R_R),
          _w(2.0, s_end + z(0.06), CLOSED, "right: lift the sticker away", R_R),
          _w(2.0, drop_st, CLOSED, "right: carry it aside", R_R),
          _w(0.8, drop_st, OPEN_C, "right: drop it", R_R),
          _w(2.0, np.array([0.15, -0.25, TOP + 0.15]), OPEN_C, "right: clear", R_R))
    L.wait_until(R.t - 4.8, "left: hold the tray while the sticker is peeled")
    # 8. left: lift the tray straight off and put it down on the desk where it was
    put = np.array([TRAY_XY[0], TRAY_XY[1] + GRIP_Y, TRAY_Z0 - FILM_T + TCP_ABOVE_SEAT])
    off = [_w(2.5, hold_t + z(0.03), TRAY_PINCH, "left: lift the tray straight off", R_L),
           _w(1.0, hold_t + z(0.06), TRAY_PINCH, "left: clear of the phone", R_L),
           _w(3.0, put + up, TRAY_PINCH, "left: carry it aside", R_L),
           _w(1.5, put + z(0.002), TRAY_PINCH, "left: set it down", R_L),
           _w(0.8, put + z(0.002), OPEN_C, "left: let go", R_L),
           _w(1.5, put + up + z(0.05), OPEN_C, "left: rise clear", R_L),
           _w(4.0, put + up + z(0.05), OPEN_C, "left: stay clear", R_L)]
    if ONE:
        off = [_w(sum(w.duration for w in off), L.wps[-1].pos, OPEN_C, "left: stays", L.wps[-1].yaw)]
    L.add(*off)
    R.wait_until(L.t, "right: stay clear")
    return {"left": L.wps, "right": R.wps}


def demo():
    return _plan()
