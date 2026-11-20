"""Salt half the rim of a rocks glass, on its outer edge only, then stand it upright on the mat's fill spot.

Wet and salted are rules on the rim's pose (see the README); nothing is moved or latched by code. The YAM can't point a
grip upward, so the demo hands the glass to the left hand to stand it up.
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
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the full score already needs 3 s still
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Salt half the rim of the upside-down rocks glass for a margarita: wet the outside of its rim on the "
               "lime, roll the outer edge through the salt, keep salt out of the glass, hold it still over the plate, "
               "then stand it upright on the taped circle on the mat.")

# -- scoring ----------------------------------------------------------------------------------------------------------
N_SECT = 36  # 10-degree sectors of the rim
WET_PRESS = 0.0007  # m the lime's face must be pressed down (1.0 N) ...
WET_TOUCH = 0.0010  # ... with a sector's outer edge within 1 mm of the face ...
FACE_R = 0.022  # ... and within 22 mm of the face's centre
SALT_MIN = 0.0005  # m: a wet sector's outer edge this far under the salt surface takes salt
INSIDE_MAX = 0.0010  # m: the inner edge this far under the surface puts salt inside (latched zero)
TOUCH_GAP = 0.0010  # m: a rim point this close to the mat, the plate's floor or its raised edge touches it
LIFT_MM = 0.030  # m: lifted clear of the mat
HOLD_ABOVE = 0.030  # m: the rim's lowest point above the plate's raised edge in the hold over the plate
STILL_S, STILL_TOL = 3.0, 0.002  # 3 s: the glass moves less than 2 mm (the hold over the plate, and the end)
RUNG_025, RUNG_05, HALF, MOST = 5, 9, 18, 24  # salted sectors: 0.25, 0.5, half the rim, at most two thirds
UPRIGHT_DEG, REST_GAP = 10.0, 0.002  # the end: axis within 10 degrees of vertical, base within 2 mm of the mat
MARK_XY, MARK_TOL, MARK_R = (0.27, -0.04), 0.030, 0.035  # the fill spot: centre, base-centre tolerance, tape radius
CLEAR_MM = 30.0  # both grippers at least 30 mm from the glass over the last 3 s
DROP_STEP, DROP_NEAR = 0.015, 0.010  # dropped: centre of mass falls 15 mm in one step, no gripper within 10 mm

# -- the props, at the real items' sizes ------------------------------------------------------------------------------
GAP = 0.0005  # spawn gap above what each body stands on
GLASS_C, MAT_C, PLATE_C, LIME_C = (0.80, 0.88, 0.90), (0.08, 0.08, 0.08), (0.95, 0.95, 0.96), (0.40, 0.66, 0.20)
SALT_C, FLESH_C, CUP_C, MARK_C = (0.97, 0.97, 0.97), (0.78, 0.86, 0.45), (0.92, 0.92, 0.88), (0.85, 0.85, 0.80)

# Tablecraft 1218BK bar mat: 305 x 457 x 9.4 mm; the bar body's frame is on the table top under the mat's centre
MAT_XY, MAT_SIZE = (0.30, -0.14), (0.305, 0.457, 0.0094)
MAT_T = MAT_SIZE[2]

# Libbey 15454 Winchester 8 oz rocks glass (frame: the rim's centre, +z up the axis toward the base)
GLASS_XY = (0.24, -0.12)
H, R_RIM, R_BASE, WALL, BASE_T = 0.086, 0.0395, 0.03175, 0.003, 0.017
R_RIM_IN = R_RIM - WALL  # 36.5 mm
GLASS_MASS, GLASS_MU = 0.260, 0.4
ALPHA = math.atan((R_RIM - R_BASE) / H)  # the wall's 5.15-degree taper
TILT_MIN_DEG = 19.5  # = asin((2.0 - 1.0) / 3.0): least tilt keeping the inner edge within 1 mm under 2 mm of salt


def r_out(z: float) -> float:
    """The glass's outside radius at ``z`` m from the rim."""
    return R_RIM - (R_RIM - R_BASE) * z / H


def _spokes(r, z0, z1, n, color, centre=(0.0, 0.0), ghost=False):
    """A disc of radius ``r`` from ``z0`` to ``z1`` as ``n`` boxes crossing at its centre (a 2n-sided prism)."""
    w = 2 * r * math.sin(math.pi / (2 * n))
    return [k.box((centre[0], centre[1], (z0 + z1) / 2), (2 * r, w, z1 - z0), color, turn=(0.0, 0.0, math.pi * i / n),
                  ghost=ghost) for i in range(n)]


def _glass_boxes() -> list:
    out = []
    z_wall = H - BASE_T
    w_wall = 2 * (R_RIM - WALL / 2) * math.tan(math.pi / N_SECT)
    for i in range(N_SECT):  # the wall: one slab per sector, leaning in with the taper
        phi = 2 * math.pi * (i + 0.5) / N_SECT
        rc = r_out(z_wall / 2) - WALL / 2
        out.append(k.box((rc * math.cos(phi), rc * math.sin(phi), z_wall / 2), (WALL, w_wall, z_wall / math.cos(ALPHA)),
                         GLASS_C, turn=(0.0, -ALPHA, phi)))
    zb, ro = z_wall + BASE_T / 2, r_out(z_wall + BASE_T / 2)
    for i in range(N_SECT):  # the solid base: an outer ring from 22 mm out to the taper ...
        phi = 2 * math.pi * (i + 0.5) / N_SECT
        rc = (0.022 + ro) / 2
        out.append(k.box((rc * math.cos(phi), rc * math.sin(phi), zb), (ro - 0.022, 2 * ro * math.tan(math.pi / N_SECT),
                                                                         BASE_T), GLASS_C, turn=(0.0, -ALPHA, phi)))
    for i in range(18):  # ... an inner ring from 10 to 22 mm ...
        phi = 2 * math.pi * (i + 0.5) / 18
        out.append(k.box((0.016 * math.cos(phi), 0.016 * math.sin(phi), zb),
                         (0.012, 2 * 0.022 * math.tan(math.pi / 18), BASE_T), GLASS_C, turn=(0.0, 0.0, phi)))
    out.append(k.box((0.0, 0.0, zb), (0.017, 0.017, BASE_T), GLASS_C))  # ... and the middle
    return out


GLASS_BOXES = _glass_boxes()


def _com_from_rim() -> float:
    v = np.array([np.prod(b[1]) for b in GLASS_BOXES])
    return float(np.sum(v * np.array([b[0][2] for b in GLASS_BOXES])) / v.sum())


COM_Z = _com_from_rim()  # about 59.6 mm from the rim (the boxes' volume centre: one density throughout)
GLASS = k.mech("glass", (*GLASS_XY, TOP + MAT_T + GAP), GLASS_BOXES, GLASS_MASS, color=GLASS_C, surface=GLASS_MU)

# Corelle 6003887 bread plate (171 mm, 22 mm tall; floor 5.6 mm up, flat to 60 mm, raised edge out to 85.5 mm)
PLATE_XY, PLATE_R, PLATE_H, FLOOR_UP, FLAT_R = (0.34, -0.23), 0.0855, 0.022, 0.0056, 0.060
SALT_R, SALT_T = 0.060, 0.0020  # Morton coarse kosher salt: a 120 mm disc levelled to 2 mm
FLOOR_Z = MAT_T + FLOOR_UP  # in the bar's frame
SALT_Z = FLOOR_Z + SALT_T
EDGE_Z = MAT_T + PLATE_H

# Dart 200PC 2 oz portion cup (60 mm top, 30 mm tall) with a Persian lime half, cut face up, its face 40 mm up
LIME_XY, LIME_FACE_UP, LIME_R, CUP_R, CUP_H = (0.36, -0.06), 0.040, 0.0285, 0.0265, 0.030
LIME_RATE, LIME_PRE, LIME_DAMP, LIME_MASS = 1000.0, 0.3, 2.0, 0.033  # N/m, N, N s/m, kg: the cut face's give
LIME_TRAVEL = 0.006
BAR_SIM_KG, BAR_MU = 30.0, 0.5  # the bar body stands in for the tape; wet lime and rubber, never grippier than real


def _rel(xy):
    return (xy[0] - MAT_XY[0], xy[1] - MAT_XY[1])


def _bar_boxes() -> list:
    out = [k.box((0.0, 0.0, MAT_T / 2), MAT_SIZE, MAT_C)]
    pc = _rel(PLATE_XY)
    out += _spokes(FLAT_R, MAT_T, FLOOR_Z, 6, PLATE_C, pc)
    beta = math.atan((EDGE_Z - FLOOR_Z) / (PLATE_R - FLAT_R))
    run = (PLATE_R - FLAT_R) / math.cos(beta)
    for i in range(24):  # the sloped raised edge, 3.5 mm thick
        phi = 2 * math.pi * i / 24
        rc, zc = (FLAT_R + PLATE_R) / 2, (FLOOR_Z + EDGE_Z) / 2 - 0.0015
        out.append(k.box((pc[0] + rc * math.cos(phi), pc[1] + rc * math.sin(phi), zc),
                         (run, 2 * rc * math.tan(math.pi / 24) + 0.003, 0.0035), PLATE_C, turn=(0.0, -beta, phi)))
    out += _spokes(SALT_R, FLOOR_Z, SALT_Z, 6, SALT_C, pc, ghost=True)  # the salt: drawn, no collider
    out += _spokes(CUP_R, MAT_T, MAT_T + CUP_H - 0.0006, 6, CUP_C, _rel(LIME_XY))
    out += _spokes(MARK_R, MAT_T, MAT_T + 0.0003, 6, MARK_C, _rel(MARK_XY), ghost=True)  # the fill spot: tape, drawn
    return out


LIME_AT = (*_rel(LIME_XY), MAT_T + LIME_FACE_UP)  # the lime's face centre in the bar frame
LIME = k.slider("lime", LIME_AT, "Z", -LIME_TRAVEL, 0.0,
                [*_spokes(LIME_R, -0.022, -0.001, 6, LIME_C), *_spokes(0.025, -0.001, 0.0, 6, FLESH_C)],
                rate=LIME_RATE, target=LIME_PRE / LIME_RATE, damping=LIME_DAMP, mass=LIME_MASS, weightless=True)
BAR = k.mech("bar", (*MAT_XY, TOP + GAP), _bar_boxes(), BAR_SIM_KG, joints=[LIME], color=MAT_C, surface=BAR_MU)
OBJECTS = [BAR, GLASS]

# -- rim geometry -----------------------------------------------------------------------------------------------------
_MID = np.radians((np.arange(N_SECT) + 0.5) * 360.0 / N_SECT)
RING_OUT = np.stack([R_RIM * np.cos(_MID), R_RIM * np.sin(_MID), 0 * _MID], axis=1)  # each sector's outer-edge point
_FINE = np.radians(np.arange(0.0, 360.0, 2.0))
RING_OUT_FINE = np.stack([R_RIM * np.cos(_FINE), R_RIM * np.sin(_FINE), 0 * _FINE], axis=1)
RING_IN = np.stack([R_RIM_IN * np.cos(_FINE), R_RIM_IN * np.sin(_FINE), 0 * _FINE], axis=1)


def rim(st, ring) -> np.ndarray:  # also any other points in the glass's frame
    return st["glass"].pos + np.asarray(ring, float) @ st["glass"].rotation().T


def places(st) -> dict:
    """World heights and centres read from the bar body."""
    b = st["bar"]
    pc = b.to_world((*_rel(PLATE_XY), 0.0))
    face = b.to_world((0.0, 0.0, 0.0), "lime")
    rest = b.to_world(LIME_AT)
    z0 = float(b.pos[2])
    return {"plate": pc[:2], "salt": z0 + SALT_Z, "floor": z0 + FLOOR_Z, "edge": z0 + EDGE_Z, "mat": z0 + MAT_T,
            "face": face, "face_rest": float(rest[2]), "press": float(-b.q["lime"]),
            "mark": b.to_world((*_rel(MARK_XY), MAT_T))[:2]}


_RING24 = np.radians(np.arange(0.0, 360.0, 15.0))
BASE_RING = np.stack([R_BASE * np.cos(_RING24), R_BASE * np.sin(_RING24), H + 0 * _RING24], axis=1)
GLASS_PTS = np.concatenate([np.stack([r_out(z) * np.cos(_RING24), r_out(z) * np.sin(_RING24), z + 0 * _RING24], axis=1)
                            for z in (0.0, H / 3, 2 * H / 3, H)] + [np.array([[0.0, 0.0, H]])])  # its outside


world = rim


def stood_on_mark(st) -> bool:
    """Upright (axis within UPRIGHT_DEG of vertical, rim up), base resting on the mat, base centre on the fill spot."""
    p = places(st)
    up = -float(st["glass"].rotation()[2, 2])  # the axis from the base to the rim, against world up
    base = world(st, BASE_RING)
    centre = world(st, [0.0, 0.0, H])
    return bool(up >= math.cos(math.radians(UPRIGHT_DEG)) - 1e-9
                and float(base[:, 2].min()) - p["mat"] <= REST_GAP + 1e-9
                and float(np.linalg.norm(centre[:2] - p["mark"])) <= MARK_TOL + 1e-9)


# -- the scorer -------------------------------------------------------------------------------------------------------
INSIDE, TOUCHED, LIFTED = k.Sticky(), k.Sticky(), k.Ever()
DROPPED = k.Sticky()  # a cap at 0.75, not a zero
OVER, END = k.Settle(STILL_S, STILL_TOL), k.Settle(STILL_S, STILL_TOL)  # held over the plate; stood on the spot
RUN = {"wet": np.zeros(N_SECT, bool), "salted": np.zeros(N_SECT, bool), "t": 0.0, "inner_mm": -99.0,
       "low": None, "over_n": -1, "com_z": None}
REC = k.run_record.RunRecord(__file__, "glass", "bar")
TRACE: dict = {}


def salted() -> int:
    return int(RUN["salted"].sum())


def high_and_rim_down(st) -> bool:
    p = places(st)
    low = float(rim(st, RING_OUT_FINE)[:, 2].min())
    return bool(st["glass"].rotation()[2, 2] > 0.5 and low >= p["edge"] + HOLD_ABOVE - 1e-9)


def grade(states) -> float:
    """1.0: 18-24 salted, held over the plate after the last (``over_n``), then stood on the spot, let go, still 3 s,
    never dropped; still held, tipped, off the spot or dropped stays at 0.75."""
    if INSIDE or TOUCHED:
        return 0.0
    n = salted()
    done = n <= MOST and RUN["over_n"] == n and END.ok and not DROPPED
    return k.ladder((0.1, bool(LIFTED)), (0.25, n >= RUNG_025), (0.5, n >= RUNG_05), (0.75, n >= HALF),
                    (1.0, done))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


@k.on_reset
def _reset(states) -> None:
    INSIDE.reset(), TOUCHED.reset(), LIFTED.reset(), DROPPED.reset(), OVER.reset(), END.reset()
    RUN.update(wet=np.zeros(N_SECT, bool), salted=np.zeros(N_SECT, bool), t=0.0, inner_mm=-99.0, low=None, over_n=-1,
               com_z=None)
    HAND.update(cmd=None)
    TRACE.clear()
    REC.reset()


@k.on_step
def _watch(states) -> None:
    RUN["t"] += 1.0 / D.HZ
    t, p = RUN["t"], places(states)
    out, fine, inn = rim(states, RING_OUT), rim(states, RING_OUT_FINE), rim(states, RING_IN)
    RUN["low"] = float(fine[:, 2].min())
    LIFTED.update(RUN["low"] >= p["mat"] + LIFT_MM - 1e-9, t)
    # wet: the outer edge on the pressed lime face
    near = (np.linalg.norm(out[:, :2] - p["face"][:2], axis=1) <= FACE_R) & (out[:, 2] - p["face"][2] <= WET_TOUCH)
    if p["press"] >= WET_PRESS - 1e-9:
        RUN["wet"] |= near
    # salted: a wet sector's outer edge under the salt surface, inside the salt disc
    in_o = np.linalg.norm(out[:, :2] - p["plate"], axis=1) <= SALT_R
    RUN["salted"] |= RUN["wet"] & in_o & (p["salt"] - out[:, 2] >= SALT_MIN - 1e-9)
    # salt inside: the inner edge deeper than 1 mm under the surface, inside the disc (any time)
    in_i = np.linalg.norm(inn[:, :2] - p["plate"], axis=1) <= SALT_R
    if in_i.any():
        worst = float((p["salt"] - inn[in_i, 2]).max())
        RUN["inner_mm"] = max(RUN["inner_mm"], worst * 1000)
        if worst > INSIDE_MAX:
            INSIDE.trip(f"inner edge {worst * 1000:.2f} mm under the salt", t)
    # touched: after the first wet sector, the rim on the mat, the plate's floor or its raised edge
    if RUN["wet"].any():
        r = np.linalg.norm(fine[:, :2] - p["plate"], axis=1)
        slope = p["floor"] + (np.clip(r, FLAT_R, PLATE_R) - FLAT_R) / (PLATE_R - FLAT_R) * (p["edge"] - p["floor"])
        on_plate = (r > FLAT_R) & (r <= PLATE_R) & (fine[:, 2] < slope + TOUCH_GAP)  # the raised edge; the flat floor under the salt is where the rim rolls
        on_mat = (r > PLATE_R) & (fine[:, 2] < p["mat"] + TOUCH_GAP)
        if on_plate.any() or on_mat.any():
            TOUCHED.trip("the rim touched the " + ("plate" if on_plate.any() else "mat"), t)
    # held still rim down over the plate (3 s), after the last sector took salt
    OVER.push(states["glass"].pos, high_and_rim_down(states))
    if OVER.ok:
        RUN["over_n"] = salted()
    # dropped: once wet, the centre of mass falls 15 mm in one step with no gripper within 10 mm (a cap, not a zero)
    com_z = float(world(states, [0.0, 0.0, COM_Z])[2])
    if (RUN["wet"].any() and RUN["com_z"] is not None and RUN["com_z"] - com_z >= DROP_STEP - 1e-9
            and k.hands_clear(world(states, GLASS_PTS), DROP_NEAR * 1000)):
        DROPPED.trip(f"fell {(RUN['com_z'] - com_z) * 1000:.0f} mm in one step", t)
    RUN["com_z"] = com_z
    # the end: stood upright on the fill spot, still, both grippers 30 mm clear, for 3 s
    stood = stood_on_mark(states)
    END.push(states["glass"].pos, stood and k.hands_clear(world(states, GLASS_PTS), CLEAR_MM))
    msg = (f"tilt {states['glass'].tilt_deg():.1f} low {(RUN['low'] - p['salt']) * 1000:+.1f} press "
           f"{p['press'] * 1000:.2f} wet {int(RUN['wet'].sum())} salt {salted()} in {RUN['inner_mm']:.1f} over "
           f"{RUN['over_n']} stood {int(stood)} dropped {int(bool(DROPPED))} score {grade(states):g}")
    if int(round(t * D.HZ)) % 10 == 0:  # every 0.5 s, 8 s to a key, for the offline check's report
        TRACE[f"t{int(t // 8) * 8:02d}"] = TRACE.get(f"t{int(t // 8) * 8:02d}", "") + f" | {t:.1f} {msg}"
    REC.tick(states, lambda: msg)


# -- the demo ---------------------------------------------------------------------------------------------------------
GUIDE_TILT = 45.0  # the bar guides' tilt
PITCH = 50.0  # the glass tilted 50 degrees, rim down (the arm's compliance lets it fall to about 37 at some wrist angles)
SPIN = (-110.0, 70.0)  # deg turned about the glass's axis: on the lime one way, in the salt back (180 degrees)
SPIN_STEP, SPIN_RATE = 10.0, 30.0  # one move per 10 degrees, 30 deg/s
DAB_STEP, DAB_LIFT, DAB_PRESS_S = 20.0, 0.0045, 0.9  # on the lime: turned 20 degrees at a time, lifted 4.5 mm between
TIPS_OVER_COM = 0.0  # m: the pad tips at the centre of mass (3 mm above dropped a 25% heavier glass offline)
GRIP_Z = COM_Z + TIPS_OVER_COM  # where the pads bear, from the rim
L_TCP_RIM = GRIP_Z + D.P.PAD_TIP  # the TCP 72.6 mm from the rim along the axis
GRIP_W = 2 * r_out(GRIP_Z)  # 68 mm across the body there
SQUEEZE = 0.007  # m into each side (about 35 N a finger: the pinch must resist the rim's push about the pads' axis)
LIME_PUSH = 0.0012  # m the face is pressed down (1.5 N)
LIME_LEAD = 0.010  # m: the servo commands about this far past the pressed face to push it through the arm's ~0.14 N/mm
DEPTH = 0.0015  # m (the slam cheat only) the rim dunked this far under the salt
PRESS = 0.001  # m the command goes past the rim's rest on the plate floor: a light press (the floor sets the depth)
SALT_LIFT = 0.005  # m: the rim lifted this far between dabs in the salt (outer edge 3 mm over the surface)
PRESS_V = 0.00025  # m per step (5 mm/s) the command may move as a dab comes down: a soft landing on the floor
INNER_KEEP = 0.0005  # m: lift if the inner edge goes more than 0.5 mm under (the tilt sags at some wrist angles)
SALT_AT = (0.33, -0.22)  # where the rim's low point dips (14 mm from the disc's centre)
ABOVE = 0.025  # m over the lime / salt before the servo brings the rim down
DIP_FROM = 0.006  # m over the dip's target, then slowly in
GAIN, FIX_MAX = 0.25, 0.045  # servo: share of the error per step, the most it moves the command in all (m)
DIP_V = 0.00005  # m per step (1 mm/s) the command may move while the rim dips into the salt
SALT_GAIN, LOWER_V, AHEAD = 0.08, 0.00025, 0.5  # lowering to the salt: servo share, 5 mm/s, 0.5 s lead for arm lag
Z = np.array([0.0, 0.0, 1.0])
NOM = {"mat": TOP + MAT_T, "salt": TOP + SALT_Z, "edge": TOP + EDGE_Z, "face": TOP + MAT_T + LIME_FACE_UP}


def G(spin: float, pitch: float = PITCH) -> D.Grip:
    """The right hand's grip turned ``spin`` degrees about its approach axis (the glass's axis) from Grip(0, pitch)."""
    return D.Grip.from_matrix(D.Grip.deg(yaw=0, pitch=pitch).rotation @ axis_angle_matrix(Z, math.radians(spin)))


def _down(a) -> np.ndarray:
    u = -Z - float(-Z @ a) * a
    return u / np.linalg.norm(u)


def tcp_for_low(target, g) -> np.ndarray:
    """The TCP that puts the rim's lowest outer point at ``target`` with the glass where the grasp put it."""
    a = g.approach
    return np.asarray(target, float) - L_TCP_RIM * a - R_RIM * _down(a)


def low_point(st) -> np.ndarray:
    fine = rim(st, RING_OUT_FINE)
    return fine[int(np.argmin(fine[:, 2]))]


class Servo:
    """Keeps the rim's real lowest point on a target: each step adds a clipped share of the error to the TCP command
    (the arm's sag taken out as the wrist turns). ``sag``, read on the first step after a seed, leads the next carry.
    ``ahead``: seconds of lead on the low point's measured speed (the arm lags)."""

    def __init__(self):
        self.cmd, self.nominal, self.low0, self.sag, self.k, self.z = None, None, None, None, -1, None

    def seed(self, over, low0) -> np.ndarray:
        """Hover at ``over`` (planned to put the low point at ``low0``) led by the last sag measured."""
        lead = self.lead()
        self.cmd = self.nominal = np.asarray(over, float) + lead
        self.low0, self.sag, self.k, self.z = np.asarray(low0, float) + lead, None, S.LATEST["steps"], None
        return self.cmd

    def lead(self) -> np.ndarray:
        return np.zeros(3) if self.sag is None else np.clip(self.sag, -FIX_MAX, FIX_MAX)

    def step(self, st, want, vmax: float = 0.0008, gain: float = GAIN, ahead: float = 0.0, press: float = 0.0) -> np.ndarray:
        if S.LATEST["steps"] != self.k:
            self.k = S.LATEST["steps"]
            if self.sag is None:  # the rim hangs this far below where the hover put it, sag included
                self.sag = self.low0 - low_point(st)
            err = np.asarray(want(st), float)
            z = float(low_point(st)[2])
            if ahead and self.z is not None:
                err[2] -= ahead * (z - self.z) * D.HZ
            self.z = z if ahead else None
            self.cmd = self.cmd + np.clip(gain * err, -vmax, vmax)
            self.cmd = self.nominal + np.clip(self.cmd - self.nominal, -FIX_MAX, FIX_MAX)
        return self.cmd - np.array([0.0, 0.0, press])


SERVO = Servo()


def on_lime(st) -> np.ndarray:
    """The low point onto the lime's centre, LIME_PUSH below the face's rest height."""
    p = places(st)
    return np.array([*(p["face"][:2]), p["face_rest"] - LIME_PUSH]) - low_point(st)


def in_salt(st) -> np.ndarray:
    """The low point onto the plate floor at SALT_AT, raised if the inner edge would go INNER_KEEP under."""
    p = places(st)
    low = low_point(st)
    dz = p["floor"] - low[2]
    guard = (p["salt"] - INNER_KEEP) - float(rim(st, RING_IN)[:, 2].min())
    if guard > dz:  # the glass has turned in the hand, so its inner edge is lower: lift faster
        dz = 2.0 * guard
    pc = np.asarray(SALT_AT) - np.asarray(PLATE_XY) + p["plate"]
    return np.array([pc[0] - low[0], pc[1] - low[1], dz])


def press_ok(st) -> float:
    """1 while the inner edge is within INNER_KEEP under the salt (the press may go on), else 0 (the guard lifts)."""
    return float((places(st)["salt"] - float(rim(st, RING_IN)[:, 2].min())) <= INNER_KEEP)


def spin_moves(p: D.Plan, a: float, b: float, want, label: str, gain: float = GAIN, press: float = 0.0) -> None:
    n = int(round(abs(b - a) / SPIN_STEP))
    for i in range(1, n + 1):
        s = a + (b - a) * i / n
        p.move(abs(b - a) / n / SPIN_RATE, lambda st: SERVO.step(st, want, gain=gain, press=press), grip=G(s), track=True,
               fallback=p.last.pos, label=f"{label} {i}/{n}")


def take_glass(p: D.Plan, s0: float) -> None:
    """Straight down round the body, pad tips at the centre of mass, grip, lift 60 mm."""
    tcp_z = NOM["mat"] + L_TCP_RIM
    where = (lambda st: (*st["glass"].pos[:2], st["glass"].pos[2] + L_TCP_RIM - D.P.GRIP_ABOVE_CENTRE))
    D.grasp(p, where, GRIP_W, height=2 * (H - L_TCP_RIM + D.P.GRIP_ABOVE_CENTRE), grip=G(s0, 0.0),
            fallback=(*GLASS_XY, tcp_z - D.P.GRIP_ABOVE_CENTRE), lift=0.060, squeeze=SQUEEZE,
            support_z=NOM["mat"], label="the glass's body", unproven=True)


def lime_pass(p: D.Plan, a: float, b: float) -> None:
    low0 = np.array([*LIME_XY, NOM["face"] + ABOVE])
    over = tcp_for_low(low0, G(a))
    p.move(3.0, lambda st: tuple(over + SERVO.lead() + [0, 0, 0.015]), fallback=tuple(over + [0, 0, 0.015]), grip=G(a),
           label="tilt the glass and carry it over the lime")
    p.move(0.8, lambda st: SERVO.seed(over, low0), fallback=tuple(over), label="settle over the lime")
    pressed = tuple(over - [0, 0, ABOVE + LIME_PUSH + LIME_LEAD])  # the servo's command at the press (the real one is live)
    p.move(2.0, lambda st: SERVO.step(st, on_lime), track=True, fallback=pressed,
           label="press the rim's outer edge on the lime")
    n = int(round(abs(b - a) / DAB_STEP))  # then dab round: lift a little, turn, press again (no sliding on the lime)
    for i in range(1, n + 1):
        s = a + (b - a) * i / n
        p.move(DAB_STEP / SPIN_RATE, lambda st: SERVO.step(st, lambda q: on_lime(q) + [0, 0, DAB_LIFT]), grip=G(s),
               track=True, fallback=p.last.pos, label=f"lift and turn on the lime {i}/{n}")
        p.move(DAB_PRESS_S, lambda st: SERVO.step(st, on_lime), track=True, fallback=pressed,
               label=f"press the rim's outer edge on the lime {i}/{n}")
    p.move(0.8, lambda st: SERVO.cmd + [0, 0, 0.020], fallback=tuple(over), label="up off the lime")


def salt_pass(p: D.Plan, a: float, b: float) -> None:
    low0 = np.array([*SALT_AT, NOM["salt"] + ABOVE])
    over = tcp_for_low(low0, G(a))
    p.move(2.5, lambda st: tuple(over + SERVO.lead() + [0, 0, 0.010]), fallback=tuple(over + [0, 0, 0.010]),
           grip=G(a), label="carry it over the salt, led by the sag")
    p.move(0.8, lambda st: SERVO.seed(over, low0), fallback=tuple(over),
           label="settle over the salt")
    p.move(4.0, lambda st: SERVO.step(st, lambda s: in_salt(s) + [0, 0, DIP_FROM], LOWER_V, SALT_GAIN, AHEAD),
           track=True, fallback=tuple(over - [0, 0, ABOVE - DIP_FROM]), label="lower the rim to just over the salt")
    p.move(7.5, lambda st: SERVO.step(st, in_salt, DIP_V, SALT_GAIN, AHEAD, press=press_ok(st) * PRESS), track=True,
           fallback=tuple(over - [0, 0, ABOVE]), label="dip the outer edge into the salt")
    n = int(round(abs(b - a) / DAB_STEP))  # dab round: lift clear of the salt, turn, press onto the floor again (the
    for i in range(1, n + 1):              # rim never slides on the floor, so its friction can't twist the glass)
        s = a + (b - a) * i / n
        p.move(DAB_STEP / SPIN_RATE, lambda st: SERVO.step(st, lambda q: in_salt(q) + [0, 0, SALT_LIFT]),
               grip=G(s), track=True, fallback=p.last.pos,
               label=f"lift and turn over the salt {i}/{n}")
        p.move(1.4, lambda st: SERVO.step(st, in_salt, PRESS_V, press=press_ok(st) * PRESS), track=True,
               fallback=p.last.pos,
               label=f"press the outer edge through the salt {i}/{n}")


def hold_up(p: D.Plan) -> None:
    def up(st):
        lift = (places(st)["edge"] + HOLD_ABOVE + 0.012) - RUN["low"] if RUN["low"] is not None else 0.045
        return SERVO.cmd + [0.0, 0.0, max(lift, 0.020)]

    p.move(1.5, up, fallback=tuple(np.asarray(p.last.pos) + [0, 0, 0.045]), label="lift the glass over the plate")
    p.hold(4.5, "hold the glass still, rim down")


# The set-down: the right hand brings the glass level, rim forward; the left grips it from above near the rim,
# pitches its grip forward 90 degrees about it (the rim comes up) and stands it on the spot, led by the base's pose.
HAND_TCP = np.array([0.2184, 0.0, TOP + 0.130])  # the right TCP at the hand-over
L_GRIP, L_EDGE, L_SQUEEZE = 0.036, 0.013, 0.004  # m from the rim: left grip point; its pads bear 13 mm nearer; squeeze
L_W = 2 * r_out(L_GRIP - L_EDGE)  # 75 mm across where the pads bear
TURN = (30.0, 60.0, 90.0)  # deg: the left grip's pitch, from straight down to level forward
HOVER, SET_PRESS = 0.015, 0.003  # m: the base over the mat before going down; the command past touching
HAND: dict = {"cmd": None}  # the left hand's last live command


def GL(beta: float) -> D.Grip:
    return D.Grip.deg(yaw=0.0, pitch=beta)


def _turned(tcp0, beta: float) -> np.ndarray:
    """The left TCP at pitch ``beta`` with the grip point where the grasp (``tcp0``) put it, 15 mm up."""
    return np.asarray(tcp0, float) + [0.0, 0.0, 0.015] + D.P.GRIP_ABOVE_CENTRE * (GL(0.0).approach - GL(beta).approach)


def _base_error(st, z_over: float) -> np.ndarray:
    """(fill spot - base centre) in x, y; (mat + ``z_over`` - base's lowest point) in z."""
    p, base, centre = places(st), world(st, BASE_RING), world(st, [0.0, 0.0, H])
    return np.array([p["mark"][0] - centre[0], p["mark"][1] - centre[1], p["mat"] + z_over - float(base[:, 2].min())])


def _keep(fn):
    def f(st):
        HAND["cmd"] = np.asarray(fn(st), float)
        return HAND["cmd"]
    return f


def hand_over(p: D.Plan, release: bool = True) -> D.Plan:
    """Right hand ``p`` after the hold over the plate; returns the left hand's plan (``release=False``: "held")."""
    w0 = p.last.pos
    up = lambda *_: np.asarray(w0, float) + [0.0, 0.0, 0.030]  # noqa: E731
    p.move(1.0, up, fallback=up, label="up from the plate")
    p.move(2.0, tuple(HAND_TCP + [0.03, -0.08, 0.02]), grip=G(-100.0, 67.5), label="turn the glass toward level")
    p.move(2.0, tuple(HAND_TCP), grip=G(-90.0, 90.0), label="the glass level, rim forward, to the hand-over")
    c0 = HAND_TCP + [L_TCP_RIM - L_GRIP, 0.0, 0.0]  # the grip point with no sag
    q = D.Plan("left").hold(p.duration - 3.0, "stay home")
    q.move(3.0, tuple(c0 + [0.0, 0.0, 0.05]), grip=GL(0.0), label="wait over the hand-over").hold(0.6)
    D.grasp(q, lambda st: world(st, [0.0, 0.0, L_GRIP]), L_W, height=2 * r_out(L_GRIP), grip=GL(0.0),
            fallback=tuple(c0), lift=None, squeeze=L_SQUEEZE, support_z=c0[2] - 0.05, approach=0.030,
            label="the glass's body near the rim", unproven=True)
    g, g0 = q.last.pos, c0 + [0, 0, 0.004]  # the grasp TCP (live) and as planned
    p.hold(q.duration + 0.5 - p.duration, "hold the glass for the left hand")
    p.fingers(GRIP_W + 0.016, 1.5, "let go of the glass").back(0.030, 1.5, "back off the glass's base")
    p.move(2.0, tuple(HAND_TCP + [-0.03, -0.12, 0.01]), grip=G(-60.0, 70.0), label="out to the right, low").home()
    q.hold(1.5, "keep it while the right hand opens")
    q.move(2.0, lambda st: _turned(g, 0.0), fallback=tuple(_turned(g0, 0.0)), label="lift it off the right hand")
    q.hold(p.duration - q.duration, "keep it still until the right hand is home")
    for b in TURN:
        q.move(1.2, lambda st, b=b: _turned(g, b), fallback=tuple(_turned(g0, b)), grip=GL(b), label=f"rim up {b:.0f}")
    q.hold(0.8, "let the glass settle upright")
    fb = np.asarray(q.last.pos, float) + [MARK_XY[0] - c0[0], MARK_XY[1] - c0[1], -0.07]
    q.move(2.5, _keep(lambda st, w=q.last.pos: np.asarray(w, float) + _base_error(st, HOVER)), fallback=tuple(fb),
           label="carry it over the fill spot").hold(0.8)
    q.move(1.0, _keep(lambda st: HAND["cmd"] + np.clip(_base_error(st, HOVER), -0.02, 0.02)), fallback=tuple(fb),
           label="take out the sag over the spot")
    q.move(3.0, lambda st: HAND["cmd"] + _base_error(st, -SET_PRESS) * [0, 0, 1.0],
           fallback=tuple(fb - [0, 0, HOVER + SET_PRESS]), label="lower it until the base meets the mat").hold(0.5)
    if not release:
        return q.hold(6.0, "keep holding it")
    q.release().back(0.060, 1.5, "back away from the glass")
    w1 = q.last.pos
    up1 = lambda *_: np.asarray(w1, float) + [0.0, 0.0, 0.08]  # noqa: E731
    return q.move(1.5, up1, fallback=up1, label="up, clear of the glass").home()


def salt_rim(how: str) -> dict:
    """``how``: "demo"; "slam" (no lime, dunked straight down); "wrong_order" (salt, then lime); "held" (never let go)."""
    s0, s1 = SPIN
    p = D.Plan("right")
    take_glass(p, s0)
    if how == "slam":
        dunk = np.array([*SALT_AT, NOM["salt"] - DEPTH + L_TCP_RIM])
        p.move(3.0, tuple(dunk + [0, 0, 0.030]), grip=G(s0, 0.0), label="over the salt, glass upright")
        p.move(2.0, tuple(dunk), label="dunk the rim straight down").hold(1.0)
        p.move(1.5, tuple(dunk + [0, 0, 0.060]), label="lift").hold(4.0)
    elif how == "wrong_order":
        salt_pass(p, s0, s1)
        p.move(0.8, lambda st: SERVO.cmd + [0, 0, 0.020], fallback=tuple(np.asarray(p.last.pos) + [0, 0, 0.02]),
               label="up out of the salt")
        lime_pass(p, s1, s0)
        hold_up(p)
    else:
        lime_pass(p, s0, s1)
        salt_pass(p, s1, s0)
        hold_up(p)
        return D.together(hand_over(p, release=how != "held"), p)
    return D.together(D.idle_plan("left", p.duration), p)


def demo():
    return D.choose(demo=lambda: salt_rim("demo"), slam=lambda: salt_rim("slam"),
                    wrong_order=lambda: salt_rim("wrong_order"), held=lambda: salt_rim("held"))


# -- what the template tests read -------------------------------------------------------------------------------------
TEXT_NUMBERS = [("86 mm", H), ("79 mm", 2 * R_RIM), ("63.5 mm", 2 * R_BASE), ("3 mm wall", WALL), ("260 g", GLASS_MASS),
                ("305 x 457 x 9.4 mm", MAT_T), ("171 mm", 2 * PLATE_R), ("22 mm", PLATE_H),
                ("5.6 mm", FLOOR_UP), ("120 mm", 2 * SALT_R), ("levelled to 2 mm", SALT_T),
                ("40 mm above the table", LIME_FACE_UP), ("1 mm per newton", 1.0 / LIME_RATE),
                ("0.3 N", LIME_PRE), ("0.7 mm", WET_PRESS), ("1.0 N", LIME_PRE + LIME_RATE * WET_PRESS),
                ("within 1 mm of the face", WET_TOUCH), ("0.5 mm or more under", SALT_MIN),
                ("more than 1 mm under", INSIDE_MAX), ("30 mm clear", LIFT_MM), ("30 mm above", HOLD_ABOVE),
                ("moves less than 2 mm", STILL_TOL), ("3 s", STILL_S),
                ("2.1 mm", WALL * math.sin(math.radians(GUIDE_TILT)), 0.02), ("within 2 mm of the mat", REST_GAP),
                ("within 30 mm of the circle", MARK_TOL), ("70 mm circle", 2 * MARK_R),
                ("at least 30 mm from it", CLEAR_MM / 1000), ("falls 15 mm", DROP_STEP),
                ("no gripper within 10 mm", DROP_NEAR)]
CHEATS = {"none": 0.0, "slam": 0.0, "wrong_order": 0.1, "held": 0.75}  # mode: the most it may score
HOLD = {"bar.lime": 0.5}  # joint: the most it may move with no command (mm)
REQUIRED_FORCES = {"press the rim's outer edge on the lime": 1.0}  # the wet rule's 0.7 mm press (1.0 N)
