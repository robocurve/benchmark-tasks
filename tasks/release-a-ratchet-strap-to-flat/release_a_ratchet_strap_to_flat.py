"""Release a tight ratchet tie-down: hold its release and fold the handle over flat, on ourkit.

On the station table: a section of pickup-bed floor (plywood with a rubber bed mat) carrying two lengths of L-track.
A Coleman 62 qt wheeled cooler stands on the mat, strapped down across its lid by a 1 in ratchet tie-down hooked to a
stud ring on each track. The ratchet sits on the strap's right leg, its spool axis pointing at the robot, handle
closed down the leg.

- The handle turns about the spool axis from closed (0 deg) to flat (180 deg, its hard stop). In the sim the hinge
  runs to negative angles (opening is -q); ``opened()`` gives the opening angle.
- The release (lock release slide) moves 8 mm along the handle toward the grip on a 10-14 N spring. In the sim it is a
  hinge on the handle with a 200 mm virtual pivot, straight to 0.2 mm over its travel; ``pulled()`` gives its mm.
- ``Ratchet`` (a rule on the real motion): with the release pulled 6 mm or more the handle is free; let go, the handle
  is held in its closed notch at 0 deg, stopped at the crank stop at 115 deg, free beyond it, and locked flat once it
  reaches 178 deg. A cam (the base pawl being lifted) pushes back toward closed with 0.3 N m over 160-178 deg. Folding
  the handle back with the release let go winds the spool one tooth per 33 deg (11 teeth). The strap holds 200 N
  (declared) until the handle locks flat, then reads 0 and its leg is drawn slack.

Scored at the end: locked flat (and so slack), the grippers clear and the handle still; milestones for getting past
the crank stop and reaching flat; latched zeros for shoving the cooler or winding the strap two teeth tighter.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import ourkit as k  # noqa: E402
from ourkit import demo as D  # noqa: E402
from ourkit import scene as S  # noqa: E402
from ourkit.rules import Rule  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the full-score rung already needs 1 s still and clear
TOP = TABLE_TOP[SCENE]
SPAWN_GAP = 0.0005
INSTRUCTION = "Release the strap so it goes slack, and let go."
R = math.radians

# -- the scene (x forward from the arm bases' midpoint, y left, z up from the table top; m) --------------------------
BOARD_X0, BOARD_X1, BOARD_Y0, BOARD_Y1 = 0.10, 0.90, -0.34, 0.40
PLY_T, MAT_T = 0.019, 0.006
FLOOR = PLY_T + MAT_T  # the mat's top, 25 mm
BED_KG = 12.0  # the floor section with its mat and track, its real mass (screwed down on the real bench)
TRACK_L, TRACK_W, TRACK_H = 0.610, 0.032, 0.010
TRACK_X0 = 0.15
RIGHT_TRACK_Y, LEFT_TRACK_Y = -0.285, 0.26
RING_D = 0.050
COOLER_X0, COOLER_D, COOLER_W, COOLER_H = 0.265, 0.508, 0.343, 0.419  # front face, depth (x), length (y), height
EDGE_Y = -0.135  # the cooler's right face (the strap leaves the lid here)
COOLER_KG = 9.6  # 3.6 kg empty plus six 1 L water bottles
LID_T = 0.040
LID_Z = FLOOR + COOLER_H  # 0.444
STRAP_W, WEB_T = 0.025, 0.0015
TENSION = 200.0  # N, declared

# -- the ratchet ------------------------------------------------------------------------------------------------------
H_X, H_Y, H_Z = 0.300, -0.210, 0.265  # the spool (mandrel) axis
LEAN = math.atan2(EDGE_Y - H_Y, LID_Z - H_Z)  # the strap leg's lean out from vertical (22 deg)
SL, CL = math.sin(LEAN), math.cos(LEAN)
U = np.array([0.0, SL, CL])  # up the leg, toward the lid edge
NV = np.array([0.0, CL, -SL])  # across the leg, in the swing plane
D0 = -U  # the closed handle's direction (down the leg)
HINGE = np.array([H_X, H_Y, H_Z])
RATCHET_W, RATCHET_H, RATCHET_L, RATCHET_KG = 0.059, 0.041, 0.121, 0.34
BASE_W = 0.033  # base side plates, outside to outside
HANDLE_R = 0.100  # spool axis to the grip's end
PLATE_W = 0.036  # handle side plates, outside to outside
PLATE_T = 0.0025
GRIP_T = 0.018  # the grip bar, along the handle
PINCH = 0.034  # the release tab's inner face to the grip's end
TAB_T = 0.003
TAB_R = HANDLE_R - PINCH  # 66 mm: the tab's inner face at rest
HANDLE_N = 0.025  # the handle's depth across the leg
HANDLE_KG = 0.12
HANDLE_FRICTION = 0.10  # N m, static = dynamic
TRAVEL = 0.008
F_REST, RATE = 10.0, 500.0  # N at rest (preload), N/m: 14 N fully pulled
PULL_FREE = 0.006  # pulled this far, the closed notch and the crank stop are passed
LEVER_L = 0.200  # the release's virtual pivot, 200 mm across the handle (its 8 mm stays straight to 0.2 mm)
CRANK_DEG, PAST_DEG = 115.0, 125.0
CAM0_DEG, CATCH_DEG, FLAT_DEG = 160.0, 178.0, 180.0
CAM_NM = 0.3  # the base pawl's cam, pushing the handle back toward closed over 160-178 deg
CAM_K = 0.2  # N m/rad: the handle's spring, aimed CAM_NM / CAM_K ahead while the cam acts, else at the handle
CLOSED_DEG = 1.0  # let go within this of closed, the release drops into the closed notch
LOCK_BAND, REACH_BAND = 3.0, 5.0
TEETH = 11
TOOTH_DEG = 360.0 / TEETH
TEETH_MAX = 2
SLACK_TURN = -0.10  # rad: the drawn webbing leg swings out this far once slack

# -- scoring ----------------------------------------------------------------------------------------------------------
CLEAR_MM = 30.0
STILL_DEG, STILL_S = 1.0, 1.0
COOLER_MOVE, COOLER_TILT = 0.010, 5.0

STEEL, ZINC, BLACK, RED = (0.55, 0.57, 0.60), (0.72, 0.74, 0.76), (0.08, 0.08, 0.09), (0.75, 0.12, 0.10)
PLY, MAT, ALU, WEB = (0.78, 0.68, 0.50), (0.12, 0.12, 0.12), (0.70, 0.72, 0.74), (0.85, 0.55, 0.10)
SHELL, LIDC = (0.15, 0.35, 0.60), (0.94, 0.94, 0.92)


def lbox(origin, s0, s1, n0, n1, x0, x1, color, ghost=False, label=None):
    """A box laid along the strap leg: ``s`` up the leg, ``n`` across it, ``x`` along the spool axis, from ``origin``."""
    o = np.asarray(origin, dtype=float)
    c = o + (s0 + s1) / 2 * U + (n0 + n1) / 2 * NV + np.array([(x0 + x1) / 2, 0.0, 0.0])
    kw = {"label": label} if label else {}
    return k.box(tuple(c), (x1 - x0, n1 - n0, s1 - s0), color, turn=(-LEAN, 0.0, 0.0), ghost=ghost, **kw)


def _floor() -> list:
    s = k.span
    out = [s((BOARD_X0, BOARD_Y0, 0.0), (BOARD_X1, BOARD_Y1, PLY_T), PLY),
           s((BOARD_X0, BOARD_Y0, PLY_T), (BOARD_X1, BOARD_Y1, FLOOR), MAT)]
    for x in np.arange(BOARD_X0 + 0.04, BOARD_X1, 0.08):  # the mat's ribs, drawn
        out.append(s((x - 0.006, BOARD_Y0, FLOOR), (x + 0.006, BOARD_Y1, FLOOR + 0.0008), (0.2, 0.2, 0.2), ghost=True))
    for y in (RIGHT_TRACK_Y, LEFT_TRACK_Y):
        out.append(s((TRACK_X0, y - TRACK_W / 2, FLOOR), (TRACK_X0 + TRACK_L, y + TRACK_W / 2, FLOOR + TRACK_H), ALU))
        # single-stud fitting with its O-ring standing up, at x 30 cm
        out.append(s((H_X - 0.012, y - 0.010, FLOOR + TRACK_H), (H_X + 0.012, y + 0.010, FLOOR + TRACK_H + 0.012), ALU))
        ring_z = FLOOR + TRACK_H + 0.012
        out.append(s((H_X - RING_D / 2, y - 0.003, ring_z), (H_X + RING_D / 2, y + 0.003, ring_z + 0.005), ZINC))
        out.append(s((H_X - RING_D / 2, y - 0.003, ring_z + RING_D - 0.005), (H_X + RING_D / 2, y + 0.003, ring_z + RING_D),
                     ZINC))
        for x in (H_X - RING_D / 2, H_X + RING_D / 2 - 0.005):
            out.append(s((x, y - 0.003, ring_z), (x + 0.005, y + 0.003, ring_z + RING_D), ZINC))
    return out


RING_TOP_Z = FLOOR + TRACK_H + 0.012 + RING_D


def seg(p0, p1) -> tuple:
    """A drawn length of webbing (no collision) at x 30 cm from (y, z) ``p0`` to ``p1``."""
    dy, dz = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dy, dz)
    c = (H_X, (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
    return k.box(c, (STRAP_W, WEB_T, length), WEB, turn=(math.atan2(-dy, dz), 0.0, 0.0), ghost=True)


def _webbing() -> list:
    """The drawn strap (no collision): over the lid, down the left leg to its ring, and the ratchet's short fixed end
    from the base down to the right ring."""
    g = dict(ghost=True)
    out = [k.span((H_X - STRAP_W / 2, EDGE_Y, LID_Z), (H_X + STRAP_W / 2, EDGE_Y + COOLER_W, LID_Z + WEB_T),
                  WEB, **g)]
    left_face = EDGE_Y + COOLER_W
    out.append(seg((left_face, LID_Z), (LEFT_TRACK_Y, RING_TOP_Z - 0.010)))
    base_end = RATCHET_L - 0.020  # the base runs 101 mm down the leg from the spool
    leg_to_ring = (H_Z - (RING_TOP_Z - 0.010)) / CL
    out.append(lbox(HINGE, -leg_to_ring, -base_end, -WEB_T / 2, WEB_T / 2, -STRAP_W / 2, STRAP_W / 2, WEB, ghost=True))
    return out


def _ratchet_base() -> list:
    """The ratchet's base frame on the strap (part of the bed body: held fixed on its strap, declared)."""
    out = []
    x_in, x_out = BASE_W / 2 - 0.002, BASE_W / 2
    for sx in (-1, 1):
        x0, x1 = sorted((sx * x_in, sx * x_out))
        out.append(lbox(HINGE, -(RATCHET_L - 0.020), 0.020, -0.015, 0.015, x0, x1, ZINC))
        # the gear wheel on the spool, inside each base plate
        out.append(lbox(HINGE, -0.017, 0.017, -0.017, 0.017, sx * x_in - (0.003 if sx > 0 else 0), sx * x_in +
                        (0.003 if sx < 0 else 0), STEEL))
    out.append(lbox(HINGE, -0.009, 0.009, -0.009, 0.009, -x_in, x_in, STEEL))  # the slotted spool
    out.append(lbox(HINGE, -(RATCHET_L - 0.020), -0.060, -0.015, -0.012, -x_in, x_in, ZINC))  # base floor plate
    out.append(lbox(HINGE, -0.075, -0.062, 0.006, 0.014, -x_in, x_in, STEEL))  # the base pawl
    return out


def _cooler() -> list:
    x0, x1 = COOLER_X0, COOLER_X0 + COOLER_D
    y0, y1 = EDGE_Y, EDGE_Y + COOLER_W
    out = [k.span((x0, y0, FLOOR), (x1, y1, LID_Z - LID_T), SHELL), k.span((x0, y0, LID_Z - LID_T), (x1, y1, LID_Z), LIDC)]
    for xa, xb in ((x0 + 0.010, x0 + 0.040), (x1 - 0.040, x1 - 0.010)):  # two wheels at the left end, drawn
        out.append(k.span((xa, y1 - 0.060, FLOOR + 0.002), (xb, y1 + 0.004, FLOOR + 0.082), BLACK, ghost=True))
    return out


def _handle_boxes() -> list:
    """The handle in its frame (origin on the spool axis), closed: radius r along D0 is s = -r up the leg."""
    o = np.zeros(3)
    out = []
    for sx in (-1, 1):
        x0, x1 = sorted((sx * (PLATE_W / 2 - PLATE_T), sx * PLATE_W / 2))
        out.append(lbox(o, -(HANDLE_R - GRIP_T + 0.002), 0.012, -HANDLE_N / 2, HANDLE_N / 2, x0, x1, ZINC))
    out.append(lbox(o, -HANDLE_R, -(HANDLE_R - GRIP_T), -HANDLE_N / 2, HANDLE_N / 2, -RATCHET_W / 2, RATCHET_W / 2, BLACK,
                    label="grip"))
    return out


HANDLE_BOXES = _handle_boxes()
TAB_C = HINGE + (TAB_R + TAB_T / 2) * D0  # the tab's centre at rest, in the bed frame
PIVOT = TAB_C + LEVER_L * NV  # the release's virtual pivot (positive turn pulls the tab toward the grip)


def _release_boxes() -> list:
    o = HINGE - PIVOT  # the spool axis in the release's frame
    tab = lbox(o, -(TAB_R + TAB_T), -TAB_R, -HANDLE_N / 2, HANDLE_N / 2, -RATCHET_W / 2, RATCHET_W / 2, RED,
               label="release")
    body = lbox(o, -TAB_R, -0.022, -0.006, 0.006, -(PLATE_W / 2 - PLATE_T - 0.001), PLATE_W / 2 - PLATE_T - 0.001, RED)
    return [tab, body]


EDGE = np.array([H_X, EDGE_Y, LID_Z])
LEG = float(np.linalg.norm(EDGE - HINGE))  # lid edge to the spool, 234 mm
HANDLE = k.hinge("handle", tuple(HINGE), "X", -math.pi, 0.0, HANDLE_BOXES, rate=CAM_K, target=0.0,
                 friction=HANDLE_FRICTION, damping=0.02, mass=HANDLE_KG, armature=0.002)
RELEASE = k.hinge("release", tuple(PIVOT), "X", 0.0, TRAVEL / LEVER_L, _release_boxes(), rate=RATE * LEVER_L ** 2,
                  target=-F_REST / RATE / LEVER_L, damping=0.05, mass=0.12, parent="handle", weightless=True,
                  armature=0.01)  # the slide's felt inertia on its 200 mm lever: keeps its stops stiff
WEBBING = k.hinge("webbing", tuple(EDGE), "X", -0.15, 0.0,
                  [lbox(np.zeros(3), -LEG, 0.0, -WEB_T / 2, WEB_T / 2, -STRAP_W / 2, STRAP_W / 2, WEB, ghost=True)],
                  rate=6.0, target=0.0, damping=1.0, mass=0.12, weightless=True)  # link masses within 100:1 of the bed
BED = k.mech("bed", (0.0, 0.0, TOP + SPAWN_GAP), _floor() + _webbing() + _ratchet_base(), BED_KG,
             joints=[HANDLE, RELEASE, WEBBING], color=PLY, surface=0.6)
COOLER = k.mech("cooler", (0.0, 0.0, TOP + SPAWN_GAP), _cooler(), COOLER_KG, color=SHELL, surface=0.6)
OBJECTS = [BED, COOLER]


def opened(st) -> float:
    """The handle's opening angle, deg (0 closed, 180 flat)."""
    return -math.degrees(float(st["bed"].q["handle"]))


def pulled(st) -> float:
    """How far the release is pulled toward the grip, m."""
    return float(st["bed"].q["release"]) * LEVER_L


# -- the ratchet's rules ----------------------------------------------------------------------------------------------


class Ratchet(Rule):
    """Closed notch, crank stop, flat lock (joint limits on the handle), the base pawl's cam (the handle's spring
    aimed ahead of it), the spool's teeth wound by folding back unreleased, and the strap's declared tension."""

    def __init__(self):
        super().__init__("handle", "ratchet")
        self.reset_state()

    def reset_state(self) -> None:
        self.locked, self.teeth, self.peak, self.limits, self.cam = False, 0, 0.0, None, False
        self.closed, self.past = True, False  # in the closed notch; riding the side plate past the crank stop

    def reset(self, ctx) -> None:
        super().reset(ctx)
        self.reset_state()
        self._set_limits(ctx, (-R(CLOSED_DEG), 0.0))

    @property
    def tension(self) -> float:
        return 0.0 if self.locked else TENSION

    def _set_limits(self, ctx, lim) -> None:
        if lim != self.limits:
            self.limits = lim
            S.set_limits(self.mech, "handle", lim[0], lim[1])

    def step(self, ctx) -> None:
        q = ctx.q("handle")
        a = -math.degrees(q)
        free = ctx.q("release") * LEVER_L >= PULL_FREE - 1e-9
        # the spool: folding back with the release let go winds a tooth per 33 deg, inside the crank range
        if free or self.past:
            self.peak = min(a, CRANK_DEG)
        else:
            self.peak = max(self.peak, a)
            if self.peak - a >= TOOTH_DEG:
                self.teeth += 1
                self.peak = a
                self.log(ctx, f"tooth {self.teeth}")
        # the flat lock
        if self.locked and free:
            self.locked = False
            self.log(ctx, "flat lock freed")
            S.set_targets(self.mech, {"webbing": 0.0})
        elif not self.locked and not free and a >= CATCH_DEG:
            self.locked = True
            self.log(ctx, "locked flat")
            S.set_targets(self.mech, {"webbing": SLACK_TURN})
        # where the release's nose is: in the closed notch, in the crank range, or past the crank stop on the plate.
        # Only the release pulled 6 mm takes it out of the notch or past the stop; pushing through a limit never does.
        if free:
            self.closed, self.past = False, a > CRANK_DEG
        elif self.past and a < CRANK_DEG - 0.5:
            self.past = False  # folded back into the crank range
        elif not self.past and a < CLOSED_DEG:
            self.closed = True
        if self.locked:
            lim = (-math.pi, -R(CATCH_DEG))
        elif free or self.past:
            lim = (-math.pi, 0.0)
        elif self.closed:
            lim = (-R(CLOSED_DEG), 0.0)
        else:
            lim = (-R(CRANK_DEG), 0.0)
        self._set_limits(ctx, lim)
        # the cam: a steady push back toward closed over 160-178 deg
        cam = (not self.locked) and CAM0_DEG <= a < CATCH_DEG
        if cam:
            S.set_targets(self.mech, {"handle": q + CAM_NM / CAM_K})
        elif self.cam or abs(S.target_of(self.mech, "handle") - q) > 0.01:
            S.set_targets(self.mech, {"handle": q})
        self.cam = cam


RATCHET = Ratchet()
RULES = k.Rules(BED, [RATCHET])

# -- the scorer -------------------------------------------------------------------------------------------------------
SHOVED, WOUND, PAST, REACH = k.Sticky(), k.Sticky(), k.Ever(), k.Ever()
END = k.Settle(STILL_S, R(STILL_DEG))
RUN = {"cooler0": None}
REC = k.run_record.RunRecord(__file__, "bed", "cooler")
CLEAR_BASE = _ratchet_base()


def locked_flat(st) -> bool:
    return RATCHET.locked and opened(st) >= FLAT_DEG - LOCK_BAND - 1e-9


def grade(states) -> float:
    if SHOVED or WOUND:
        return 0.0
    flat = locked_flat(states)
    if RATCHET.teeth >= 1 and not flat:  # tighter at the end than at the start
        return 0.0
    return k.ladder((0.25, bool(PAST)), (0.5, bool(REACH)), (0.75, flat), (1.0, flat and END.ok))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    return max(0.0, FLAT_DEG - opened(states))


@k.on_reset
def _reset(states) -> None:
    for t in (SHOVED, WOUND, PAST, REACH, END):
        t.reset()
    RUN["cooler0"] = np.asarray(states["cooler"].pos, dtype=float)
    REC.reset()


@k.on_step
def _watch(states) -> None:
    b, c = states["bed"], states["cooler"]
    a = opened(states)
    if np.linalg.norm(np.asarray(c.pos) - RUN["cooler0"]) > COOLER_MOVE or c.tilt_deg() > COOLER_TILT:
        SHOVED.trip("cooler moved")
    if RATCHET.teeth >= TEETH_MAX:
        WOUND.trip("strap wound tighter")
    PAST.update(a >= PAST_DEG)
    REACH.update(a >= FLAT_DEG - REACH_BAND)
    pts = np.concatenate([k.score.surface_points(b, "handle", HANDLE_BOXES, 0.02),
                          k.score.surface_points(b, "", CLEAR_BASE, 0.02)])
    clear = k.hands_clear(pts, CLEAR_MM)
    END.push([float(b.q["handle"])], clear)
    REC.tick(states, lambda: f"handle {a:.2f} deg, release {pulled(states) * 1000:.2f} mm, "
                             f"{'locked flat' if RATCHET.locked else 'unlocked'}, tension {RATCHET.tension:g} N, "
                             f"teeth {RATCHET.teeth}, clear {clear}, score {grade(states):g}")


# -- the demo ---------------------------------------------------------------------------------------------------------
AX = np.array([1.0, 0.0, 0.0])
OPEN_GAP = 0.074  # 20 mm clear of the grip and the tab (the arm sags on the way in)
TOUCH_GAP = 0.032  # on the 34 mm pinch, 1 mm into each face
PULL_GAP = 0.018  # the tab pulled 8 mm (a 26 mm pinch), 4 mm into each face (the pads give)
BAR_GAP = 0.014  # on the 18 mm grip bar alone
BAR_OPEN = 0.036  # round the grip bar, the inner pad between it and the tab
LET_GO_GAP = 0.036  # 1 mm off the 34 mm pinch each side: let go without swinging the outer finger at the cooler
R_PINCH0 = TAB_R + PINCH / 2  # 83 mm: the pinch's middle at rest
R_PINCH1 = TAB_R + TRAVEL + (PINCH - TRAVEL) / 2  # 87 mm: with the tab pulled
R_BAR = HANDLE_R - GRIP_T / 2  # 91 mm
TIP_X = H_X - PLATE_W / 2 - 0.003  # the pad tips 3 mm short of the handle's side plate
TCP_X = TIP_X - D.P.PAD_TIP
APPROACH = 0.060
STEP_DEG = 15.0
LEAD_DEG = 4.0  # past 150 deg the command leads the handle by up to this (joint 6 stays 4 deg off its limit)


def hdir(theta_deg: float) -> np.ndarray:
    return S.axis_rotation(-AX, R(theta_deg)) @ D0


def grip_at(theta_deg: float) -> D.Grip:
    d = hdir(theta_deg)
    return D.Grip.from_matrix(np.column_stack([d, np.cross(AX, d), AX]))


def tcp(theta_deg: float, r: float, back: float = 0.0) -> tuple:
    p = HINGE + r * hdir(theta_deg)
    return (TCP_X - back, float(p[1]), TOP + SPAWN_GAP + float(p[2]))


def lead(theta: float) -> float:
    return LEAD_DEG * min(1.0, max(0.0, (theta - 150.0) / 30.0))


def swing(p: D.Plan, t0: float, t1: float, r: float, seconds_per_step: float = 1.0) -> float:
    """Turn the hand with the handle from ``t0`` to ``t1`` deg in 15 deg waypoints; returns the last commanded deg."""
    n = max(1, int(math.ceil(abs(t1 - t0) / STEP_DEG)))
    cmd = t0
    for i in range(1, n + 1):
        th = t0 + (t1 - t0) * i / n
        cmd = th + (lead(th) if t1 > t0 else 0.0)
        p.move(seconds_per_step, tcp(cmd, r), grip=grip_at(cmd), label=f"swing the handle to {th:.0f} deg")
    return cmd


def pinch_release(p: D.Plan) -> D.Plan:
    """End-on onto the closed handle, pads on the grip's end and the tab, then pull the tab 8 mm."""
    p.move(2.5, tcp(0.0, R_PINCH0, APPROACH), finger=D.gap_finger(OPEN_GAP), grip=grip_at(0.0),
           label="in line with the handle's end")
    p.hold(0.5, "settle in line")
    p.move(2.0, tcp(0.0, R_PINCH0), label="slide the pads over the grip and the release")
    p.fingers(TOUCH_GAP, 0.8, "close on the grip and the release tab")
    p.move(1.0, tcp(0.0, R_PINCH1), finger=D.gap_finger(PULL_GAP), label="pull the release 8 mm toward the grip")
    return p.hold(0.4, "keep the release pulled")


def leave(p: D.Plan, theta: float, r: float) -> D.Plan:
    """Take the lead out (the hand back on the handle's own angle), let go, slide off end-on, open, go home."""
    p.move(0.6, tcp(theta, r), grip=grip_at(theta), label="ease off the stop")
    p.fingers(LET_GO_GAP, 0.8, "let go of the handle")
    p.move(1.5, tcp(theta, r - 0.003, APPROACH), label="slide off the handle's end")
    p.fingers(OPEN_GAP, 0.6, "open clear of the ratchet")  # wide only once off it: the open outer finger nears the cooler
    return p.home()


def right_hand(how: str) -> D.Plan:
    p = pinch_release(D.Plan("right"))
    if how in ("crank_stop", "pump"):
        swing(p, 0.0, 20.0, R_PINCH1)
        p.fingers(OPEN_GAP, 0.8, "let go of the release")
        p.move(1.2, tcp(20.0, R_PINCH1, APPROACH), label="slide off the handle's end")
        p.move(1.0, tcp(20.0, R_BAR, APPROACH), finger=D.gap_finger(BAR_OPEN), label="in line with the grip bar")
        p.move(1.5, tcp(20.0, R_BAR), label="slide the pads over the grip bar alone")
        p.fingers(BAR_GAP, 0.8, "hold the grip bar alone")
        swing(p, 20.0, 130.0, R_BAR, 0.8)
        p.hold(1.0, "push on at the stop")
        if how == "pump":
            swing(p, 130.0, 5.0, R_BAR, 0.8)
            return leave(p, 5.0, R_BAR)
        return leave(p, CRANK_DEG, R_BAR)
    stop = {"early_letgo": 140.0, "short_of_flat": 170.0}.get(how, FLAT_DEG)
    cmd = swing(p, 0.0, stop, R_PINCH1)
    p.hold(1.0, "push the handle into its stop")
    if how in ("early_letgo", "short_of_flat"):
        return leave(p, stop, R_PINCH1)
    if how == "flat_then_back":
        swing(p, cmd, 168.0, R_PINCH1)
        return leave(p, 168.0, R_PINCH1)
    p.fingers(TOUCH_GAP + 0.002, 0.8, "let the release spring back into the flat notch")
    p.hold(0.6, "the handle locks flat")
    if how == "held":
        return p.hold(12.0, "keep holding the handle")
    if how == "lock_then_close":
        p.fingers(PULL_GAP, 0.8, "pull the release again")
        swing(p, cmd, 140.0, R_PINCH1)
        return leave(p, 140.0, R_PINCH1)
    return leave(p, FLAT_DEG, R_PINCH1)


SHOVE = D.Grip.deg(yaw=0, pitch=90)  # pointing +x, horizontal
SHOVE_Y = 0.15  # on the cooler's front face


def shove() -> D.Plan:
    p = D.Plan("left")
    z = TOP + SPAWN_GAP + 0.25
    p.move(2.5, (COOLER_X0 - 0.08, SHOVE_Y, z), finger=D.gap_finger(0.010), grip=SHOVE, label="in front of the cooler")
    p.move(2.0, (COOLER_X0 + 0.03 - D.P.PAD_TIP, SHOVE_Y, z), label="push the cooler 30 mm")
    p.move(1.5, (COOLER_X0 - 0.08, SHOVE_Y, z), label="back off")
    return p.home()


def demo():
    def two(how):
        return D.together(D.idle_plan("left", 1.0), right_hand(how))

    modes = {m: (lambda m=m: two(m)) for m in ("crank_stop", "pump", "early_letgo", "short_of_flat", "flat_then_back",
                                                "held", "lock_then_close")}
    return D.choose(demo=lambda: two("demo"), shove=lambda: D.together(shove(), D.idle_plan("right", 1.0)), **modes)


# -- what the template tests read ---------------------------------------------------------------------------------------
TEXT_NUMBERS = [("100 mm from the axis", HANDLE_R), ("36 mm apart", PLATE_W), ("18 mm thick", GRIP_T),
                ("59 mm wide", RATCHET_W), ("41 mm high", RATCHET_H), ("121 mm long", RATCHET_L),
                ("34 mm from the grip's end", PINCH), ("8 mm toward the grip", TRAVEL), ("10 N at rest", F_REST),
                ("14 N fully pulled", F_REST + RATE * TRAVEL), ("6 mm or more", PULL_FREE),
                ("115 deg", CRANK_DEG), ("125 deg or more", PAST_DEG), ("178 deg or more", CATCH_DEG),
                ("0.3 N m", CAM_NM), ("180 deg", FLAT_DEG), ("within 3 deg of flat", LOCK_BAND),
                ("within 5 deg of flat", REACH_BAND), ("0.10 N m", HANDLE_FRICTION), ("0.12 kg", HANDLE_KG),
                ("0.34 kg", RATCHET_KG), ("200 N", TENSION), ("leans 23 deg", math.degrees(LEAN), 0.02),
                ("33 deg of handle", TOOTH_DEG, 0.02), ("508 mm long", COOLER_D), ("343 mm deep", COOLER_W),
                ("419 mm tall", COOLER_H), ("9.6 kg", COOLER_KG), ("610 mm", TRACK_L), ("real 12 kg", BED_KG),
                ("more than 10 mm", COOLER_MOVE), ("more than 5 deg", COOLER_TILT),
                ("30 mm or more", CLEAR_MM / 1000), ("less than 1 deg", STILL_DEG)]
CHEATS = {"none": 0.0, "crank_stop": 0.0, "pump": 0.0, "early_letgo": 0.25, "short_of_flat": 0.25,
          "flat_then_back": 0.5, "held": 0.75, "lock_then_close": 0.5, "shove": 0.0}
HOLD = {"bed.handle": 0.5, "bed.release": 0.2, "bed.webbing": 0.5}
