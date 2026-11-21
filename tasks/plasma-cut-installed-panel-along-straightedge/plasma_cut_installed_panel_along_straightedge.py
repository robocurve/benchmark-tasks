"""Plasma-cut the overhanging edge strip off an installed equipment skid's deck plate, in place, along a hand-held
straightedge, until the strip drops.

The work is cut where it stands, as equipment is in decommissioning and scrap work: the skid stays bolted down with its
control cabinet beyond its far end, a junction box is still mounted on its deck, and nothing is moved to a cutting
table. Scene (station frame: x forward from the arm bases, y left, z up from the table top):

- skid: two 2 x 2 x 1/4 in steel angles along x carrying a 3 mm mild-steel deck plate, 140 mm long; the deck overhangs
  the right-hand angle by 55 mm plus the kerf. A white line marks the cut along x at y = LINE_Y, from the deck's far
  edge to its near edge. The skid's control cabinet stands 100 mm beyond the deck's far end.
- a steel junction box mounted on the deck's left part, a dross pan on the table under the overhang;
- a hand plasma torch (grip block on top of the handle, head at its front pointing down, copper drag shield) on its
  rest at the right; an aluminium straightedge with a handle block, laid loose on the deck along the line at setup.

Cut model (the arc, kerf and heat are not simulated; cutting is bookkeeping from the torch's pose, kit.on_step):
- arc on = the torch in a hand (a TCP within GRIP_R of the grip block), within ARC_TILT of square, the shield rim from
  1 mm into the deck's top to 2 mm above it, over the deck between its ends and within 10 mm of the line (the real
  trial's supervisor closes a hold-to-run switch on the same criterion; the robot presses no trigger);
- the line is N_SEG segments of SEG; each control step with the arc on adds DT of arc time to the segment under the
  nozzle. A segment is cut through from T_CUT (travel at V_MAX or slower) and overheated beyond T_HOT (slower than
  V_SLOW). The strip hangs on one weld (the uncut bridges) that is released once every segment is cut through, so it
  drops only after the whole line is cut; the arms cannot tear a 3 mm steel bridge off.
- a segment is SOUND when cut through, not overheated, and while the arc was over it: the nozzle never more than
  ON_LINE from the line, the torch never more than SQUARE_TILT from square, and for GUIDED_SHARE or more of its arc
  time the shield's side within GUIDE_GAP of the straightedge's working edge with the straightedge flat on the deck.

Score (``score()``): 0 if a tool falls off the table, or if the hand that is not holding the torch comes within
HAND_ARC of the nozzle while the arc is on (both latched for the trial). Otherwise 0.8 x the share of sound segments;
0.9 with all sound and the strip lying in the dross pan; 1.0 with, as well, the torch back on its rest, the
straightedge still where it was laid, both TCPs CLEAR from the torch, the straightedge and the strip, and those three still.
Holding still scores 0.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import mechkit as mk  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

STEEL, MILL, DARK, BLACK = (0.60, 0.61, 0.63), (0.42, 0.43, 0.45), (0.30, 0.31, 0.33), (0.08, 0.08, 0.09)
COPPER, WHITE, YELLOW, RED = (0.72, 0.45, 0.20), (0.95, 0.95, 0.95), (0.95, 0.80, 0.10), (0.80, 0.12, 0.10)
GREY_BOX, WALL_C, RUBBER, BRIGHT = (0.55, 0.58, 0.55), (0.80, 0.78, 0.72), (0.12, 0.12, 0.12), (0.78, 0.79, 0.80)

# -- skid and deck ----------------------------------------------------------------------------------------------------
X0, X1 = 0.215, 0.355  # deck ends along x: the 140 mm line
LINE_Y = -0.100  # the cut line
KERF = 0.0015
T = 0.003  # 3 mm deck plate
L_LEG, L_T = 0.0508, 0.00635  # 2 x 2 x 1/4 in equal-leg steel angle
PZ = TOP + L_LEG + T  # deck top
DECK_Y1 = 0.185  # deck's left edge
RAIL_R = LINE_Y + 0.015  # right angle's outer face, 15 mm back from the line (the jet clears it)
RAIL_L = DECK_Y1  # left angle's outer face
OFF_W = 0.055  # strip width
CAB_X = X1 + 0.100  # face of the control cabinet standing beyond the skid's far end


def _angle(y_out, inward):
    """A 2 x 2 x 1/4 in angle along x: vertical leg's outer face at y_out, foot leg on the table toward ``inward``."""
    L = X1 - X0
    return [mk.box((L, L_T, L_LEG), (0.0, y_out + inward * L_T / 2, L_LEG / 2)),
            mk.box((L, L_LEG, L_T), (0.0, y_out + inward * L_LEG / 2, L_T / 2))]


XM = (X0 + X1) / 2
kit = mk.Kit()
kit.part("skid", (XM, 0.0, TOP), color=DARK, boxes=_angle(RAIL_R, +1) + _angle(RAIL_L, -1))
KEEP_Y0 = LINE_Y + KERF / 2
kit.part("deck", (XM, (KEEP_Y0 + DECK_Y1) / 2, PZ - T / 2), color=MILL,
         boxes=[mk.box((X1 - X0, DECK_Y1 - KEEP_Y0, T)),
                mk.box((X1 - X0, 0.001, 0.0003), (0.0, -(DECK_Y1 - KEEP_Y0) / 2 + 0.0005, T / 2 + 0.00015), WHITE)])
OFF_Y = LINE_Y - KERF / 2 - OFF_W / 2
OFF_M = (X1 - X0) * OFF_W * T * 7850  # 0.18 kg
kit.part("strip", (XM, OFF_Y, PZ - T / 2), mass=OFF_M, color=MILL, boxes=[mk.box((X1 - X0, OFF_W, T))])
SEAM = kit.bond("seam", "deck", "strip", anchor=(XM, LINE_Y, PZ - T / 2))  # the uncut bridges

# junction box still mounted on the deck: 4-11/16 in square steel box, 2-1/8 in deep (119 x 119 x 54 mm)
BOX_X, BOX_Y, BOX_W, BOX_D, BOX_H = 0.285, 0.1195, 0.119, 0.119, 0.054
kit.part("junction_box", (BOX_X, BOX_Y, PZ), color=GREY_BOX,
         boxes=[mk.box((BOX_W, BOX_D, BOX_H), (0.0, 0.0, BOX_H / 2)),
                mk.box((0.022, 0.030, 0.022), (BOX_W / 2 - 0.025, BOX_D / 2 + 0.003, 0.027), DARK)])  # conduit connector
# the skid's control cabinet, bolted down beyond the deck's far end (steel enclosure, 250 mm tall at this scale)
CAB_D, CAB_W, CAB_H = 0.120, 0.420, 0.250
kit.part("cabinet", (CAB_X + CAB_D / 2, -0.050, TOP), color=WALL_C,
         boxes=[mk.box((CAB_D, CAB_W, CAB_H), (0.0, 0.0, CAB_H / 2)),
                mk.box((0.002, 0.300, 0.180), (-CAB_D / 2 - 0.001, 0.0, CAB_H / 2), GREY_BOX)])  # door

# dross pan on the table under the overhang
PAN_X0, PAN_X1, PAN_Y0, PAN_Y1, PAN_RIM = X0 - 0.020, X1 + 0.030, -0.190, RAIL_R - 0.002, 0.012
_pw = 0.0015
kit.part("dross_pan", ((PAN_X0 + PAN_X1) / 2, (PAN_Y0 + PAN_Y1) / 2, TOP), color=STEEL,
         boxes=[mk.box((PAN_X1 - PAN_X0, PAN_Y1 - PAN_Y0, _pw), (0.0, 0.0, _pw / 2)),
                mk.box((_pw, PAN_Y1 - PAN_Y0, PAN_RIM), (-(PAN_X1 - PAN_X0) / 2, 0.0, PAN_RIM / 2)),
                mk.box((_pw, PAN_Y1 - PAN_Y0, PAN_RIM), ((PAN_X1 - PAN_X0) / 2, 0.0, PAN_RIM / 2)),
                mk.box((PAN_X1 - PAN_X0, _pw, PAN_RIM), (0.0, -(PAN_Y1 - PAN_Y0) / 2, PAN_RIM / 2)),
                mk.box((PAN_X1 - PAN_X0, _pw, PAN_RIM), (0.0, (PAN_Y1 - PAN_Y0) / 2, PAN_RIM / 2))])

# -- torch: handle along y, head at its +y end pointing down; frame on the handle axis at the volume centroid --------
_TB = [((0.024, 0.190, 0.024), (0.0, -0.082, 0.0), BLACK),  # handle (the lead hangs from a balancer)
       ((0.026, 0.026, 0.060), (0.0, 0.0, -0.030), BLACK),  # head, axis at x = 0
       ((0.020, 0.020, 0.008), (0.0, 0.0, -0.064), COPPER)]  # drag shield, rim at z = -0.068
_V = np.array([np.prod(sz) for sz, _, _ in _TB])
CY_T = float(np.dot(_V, [at[1] for _, at, _ in _TB]) / _V.sum())
HEAD_DY, RIM_DZ, SHIELD_R = -CY_T, 0.068, 0.010  # head axis left (+y) of the grip block; rim below the handle axis
GRIP_DZ = 0.024  # grip block centre above the handle axis
TORCH_M = 0.10
REST = (0.250, -0.250)
REST_Z = TOP + 0.080  # handle axis on the rest posts
POST_H = REST_Z - 0.012 - TOP
kit.part("torch", (REST[0], REST[1], REST_Z), mass=TORCH_M, color=BLACK,
         boxes=[mk.box(sz, (at[0], at[1] - CY_T, at[2]), c) for sz, at, c in _TB]
         + [mk.box((0.030, 0.030, 0.024), (0.0, 0.0, GRIP_DZ), YELLOW)])  # grip block clamped on the handle
kit.part("torch_rest", (REST[0], REST[1], TOP), color=DARK,
         boxes=[mk.box((0.050, 0.100, 0.004), (0.0, -0.026, 0.002))]
         + [mk.box((0.040, 0.012, POST_H), (0.0, dy, POST_H / 2)) for dy in (-0.060, 0.008)])

# -- straightedge: aluminium flat bar along x, working edge toward -y, tab and handle block on its back edge ---------------
SE_L, SE_W, SE_T = 0.180, 0.0254, 0.00635  # 1 x 1/4 in 6061 aluminium flat bar, cut to 180 mm
HANDLE = 0.030  # handle block cube
SE_XC = (X0 - 0.010 + X1 + 0.030) / 2  # set on the deck from 10 mm short of its near end to 30 mm past its far end
HANDLE_OFF = 0.115  # handle centre from the working edge (in plan): the two hands stay ~95 mm apart
SE_M = 0.14  # kg: bar 96 g (2.70 g/cm3) + aluminium tab 18 g + handle block ~25 g
SE_START = (SE_XC, LINE_Y + 0.010 + 0.0)  # laid along the line at setup: working edge one shield radius off it
SE_Z0 = PZ + SE_T / 2
kit.part("straightedge", (SE_START[0], SE_START[1], SE_Z0), mass=SE_M, color=BRIGHT,
         boxes=[mk.box((SE_L, SE_W, SE_T), (0.0, SE_W / 2, 0.0)),  # frame on the working edge's midpoint
                mk.box((0.030, HANDLE_OFF + 0.010 - SE_W, SE_T), (0.0, SE_W + (HANDLE_OFF + 0.010 - SE_W) / 2, 0.0)),
                mk.box((HANDLE, HANDLE, HANDLE), (0.0, HANDLE_OFF, SE_T / 2 + HANDLE / 2), RED)])
OBJECTS = kit.objects()

# -- cut bookkeeping --------------------------------------------------------------------------------------------------
N_SEG = 14
SEG = (X1 - X0) / N_SEG  # 10 mm
DT = 0.05  # s per control step (20 Hz)
V_MAX = 1.245 / 60  # m/s: Powermax30 AIR, 120 V / 20 A, 3 mm mild steel, maximum cut speed (manual 808840 p.36)
V_RATED = 0.762 / 60  # m/s: its rated 3 mm cut (762 mm/min)
V_SLOW = V_RATED / 2  # modelling choice: half the rated speed = overheated (the manual gives no number)
T_CUT = SEG / V_MAX  # 0.482 s
T_HOT = SEG / V_SLOW  # 1.575 s
GRIP_R = 0.045
ARC_TILT = 30.0  # deg
ON_LINE = 0.002
SQUARE_TILT = 10.0
GUIDE_GAP = 0.001
GUIDED_SHARE = 0.8
SE_FLAT_DZ, SE_FLAT_DEG = 0.001, 3.0
HAND_ARC = 0.045
CLEAR = 0.060
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _state():
    m = mk.MEMORY.get("cut")
    if m is None:
        m = {"t": np.zeros(N_SEG), "guided": np.zeros(N_SEG), "off_line": np.zeros(N_SEG, bool),
             "tilted": np.zeros(N_SEG, bool), "hand_near": False, "dropped_tool": False, "min_hand": float("inf")}
        mk.MEMORY["cut"] = m
    return m


def _tilt_deg(objects, part) -> float:
    _, q = kit.pose(objects, part)
    return float(np.degrees(np.arccos(np.clip(mk.quat_to_mat(q)[2, 2], -1.0, 1.0))))


def nozzle(objects) -> np.ndarray:
    return kit.point(objects, "torch", (0.0, HEAD_DY, -RIM_DZ))


def guide_gap(objects) -> float:
    """Gap between the shield's side and the straightedge's working edge, and whether the bar lies flat on the deck
    with the shield beside it (inf if not)."""
    rim = nozzle(objects)
    pos, q = kit.pose(objects, "straightedge")
    r = mk.quat_to_mat(q)
    rel = r.T @ (rim - pos)  # rim in the bar's frame: x along the bar, y away from the working edge
    flat = abs(pos[2] - (PZ + SE_T / 2)) <= SE_FLAT_DZ and _tilt_deg(objects, "straightedge") <= SE_FLAT_DEG
    if not flat or abs(rel[0]) > SE_L / 2 or rel[2] > SE_T / 2:
        return float("inf")
    return float(max(0.0, -rel[1] - SHIELD_R))


def arc_on(objects, hands=None) -> bool:
    pos, _ = kit.pose(objects, "torch")
    hands = mk.tcps() if hands is None else hands
    held = any(np.linalg.norm(np.asarray(h) - pos) < GRIP_R for h in hands)
    rim = nozzle(objects)
    return bool(held and _tilt_deg(objects, "torch") <= ARC_TILT and PZ - 0.001 <= rim[2] <= PZ + 0.002
                and X0 <= rim[0] <= X1 and abs(rim[1] - LINE_Y) <= 0.010)


def cut_step(objects, hands=None) -> None:
    """One control step of the cut (sim: every step via kit.on_step; tests call it with fake states and hands)."""
    m = _state()
    hands = mk.tcps() if hands is None else hands
    for name in ("torch", "straightedge"):
        if kit.pose(objects, name)[0][2] < TOP - 0.010:
            m["dropped_tool"] = True
    if not arc_on(objects, hands):
        return
    pos, _ = kit.pose(objects, "torch")
    rim = nozzle(objects)
    holder = int(np.argmin([np.linalg.norm(np.asarray(h) - pos) for h in hands]))
    for i, h in enumerate(hands):
        if i != holder:
            d = float(np.linalg.norm(np.asarray(h) - rim))
            m["min_hand"] = min(m["min_hand"], d)
            if d < HAND_ARC:
                m["hand_near"] = True
    k = min(N_SEG - 1, int((rim[0] - X0) / SEG))
    m["t"][k] += DT
    if abs(rim[1] - LINE_Y) > ON_LINE:
        m["off_line"][k] = True
    if _tilt_deg(objects, "torch") > SQUARE_TILT:
        m["tilted"][k] = True
    if guide_gap(objects) <= GUIDE_GAP:
        m["guided"][k] += DT
    if np.all(m["t"] >= T_CUT):
        kit.release(SEAM)  # the last bridge is cut: the strip is free


kit.on_step(cut_step)


def sound(m=None) -> np.ndarray:
    m = _state() if m is None else m
    t = m["t"]
    return ((t >= T_CUT) & (t <= T_HOT) & ~m["off_line"] & ~m["tilted"]
            & (m["guided"] >= GUIDED_SHARE * t - 1e-9))


def strip_in_pan(objects) -> bool:
    p = kit.pose(objects, "strip")[0]
    return bool(p[2] <= PZ - T / 2 - 0.025 and PAN_X0 <= p[0] <= PAN_X1 and PAN_Y0 <= p[1] <= PAN_Y1)


def _yaw_off_deg(objects, part, yaw0=0.0) -> float:
    _, q = kit.pose(objects, part)
    r = mk.quat_to_mat(q)
    yaw = math.atan2(r[1, 0], r[0, 0])
    return abs(math.degrees((yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi))


def tools_back(objects) -> bool:
    tp = kit.pose(objects, "torch")[0]
    sp = kit.point(objects, "straightedge", (0.0, HANDLE_OFF, SE_T / 2 + HANDLE / 2))
    s0 = np.array([SE_START[0], SE_START[1] + HANDLE_OFF, SE_Z0 + SE_T / 2 + HANDLE / 2])
    return bool(np.hypot(*(tp[:2] - REST)) <= 0.015 and abs(tp[2] - REST_Z) <= 0.006
                and _tilt_deg(objects, "torch") <= 10.0 and _yaw_off_deg(objects, "torch") <= 20.0
                and np.hypot(*(sp[:2] - s0[:2])) <= 0.020 and abs(sp[2] - s0[2]) <= 0.006
                and _tilt_deg(objects, "straightedge") <= 3.0 and _yaw_off_deg(objects, "straightedge") <= 20.0)


def hands_clear(objects, hands=None) -> bool:
    hands = mk.tcps() if hands is None else hands
    for name in ("torch", "straightedge", "strip"):
        lo, hi = np.asarray(objects[name].low, float), np.asarray(objects[name].high, float)
        for h in hands:
            if np.linalg.norm(np.asarray(h) - np.clip(h, lo, hi)) < CLEAR:
                return False
    return True


def still(objects) -> bool:
    return all(objects[n].resting() for n in ("torch", "straightedge", "strip"))


def score(objects) -> float:
    m = _state()
    if m["hand_near"] or m["dropped_tool"]:
        return 0.0
    ok = sound(m)
    if not ok.all():
        return round(0.8 * float(ok.mean()), 3)
    if not strip_in_pan(objects):
        return 0.8
    if tools_back(objects) and hands_clear(objects) and still(objects):
        return 1.0
    return 0.9


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    m = _state()
    return float(np.sum(np.clip(T_CUT - m["t"], 0.0, None)) * V_MAX)


# -- scripted demo (fixed layout constants only; open loop) -----------------------------------------------------------
TIP = PAD_TIP_PAST_TCP - 0.002
OPEN = -0.030
YT = math.pi / 2  # torch hand: fingers close along x on the grip block, so the drag along x cannot pitch the torch in the hand
YS = 0.0  # straightedge hand: fingers close along y on the handle block (the bar pitches about x)


def torch_tcp(x, y, rim_z):
    """TCP over the grip block for a torch whose shield rim centre is at (x, y, rim_z)."""
    return (x, y - HEAD_DY, rim_z + RIM_DZ + GRIP_DZ + TIP)


def se_tcp(edge_y, z_bar_mid):
    """TCP on the straightedge's handle for a bar whose working edge is at y = edge_y (bar centred on SE_XC)."""
    return (SE_XC, edge_y + HANDLE_OFF, z_bar_mid + SE_T / 2 + HANDLE / 2 + TIP)


T_SET = 11.0  # s: straightedge pressed on the deck
EDGE_Y = LINE_Y + SHIELD_R  # straightedge's working edge on the deck
PRESS = 0.004  # m: handle target below contact (the hand presses the bar down)
RIM_Z = PZ + 0.0015  # shield rim target on the deck (the hand's give under the torch rides it 1 mm up)
BEAR = 0.004  # m: shield aimed this far into the straightedge's edge, so it bears on it
KNOT = 0.005  # m: drag waypoint spacing
START_LIFT = 0.006  # m: past the deck's far end the unsupported hand sags; start this much higher
FAR_PACE = 1.3  # the first centimetre onto the deck is drawn this much faster: the arm lags at full reach


def _timed(keys):
    out, t_prev = [], 0.0
    for t, pos, finger, label, yaw in keys:
        out.append(Waypoint(max(0.05, t - t_prev), pos, finger, label, yaw=yaw))
        t_prev = t
    return out


def demo(speed=V_RATED):
    o, c = OPEN, FINGER_CLOSED
    # left: fingertips down on the straightedge's handle, pressing it to the deck through the cut, then lifted clear
    top = (SE_START[0], SE_START[1] + HANDLE_OFF, SE_Z0 + SE_T / 2 + HANDLE)  # handle block's top centre
    press = (top[0], top[1], top[2] + TIP + 0.002 - PRESS)
    up = lambda p, dz: (p[0], p[1], p[2] + dz)  # noqa: E731
    x_far, x_near = X1 + 0.014, X0 - 0.060
    run = (x_far - x_near) / speed
    t_cut0 = T_SET + 3.5
    t_cut1 = t_cut0 + run + 3.0
    t_back = t_cut1 + 6.0
    left = _timed([
        (2.0, up(press, 0.08), c, "fingers closed, over the straightedge's handle", YS),
        (4.0, up(press, 0.02), c, "down to the handle", YS),
        (T_SET, press, c, "press the straightedge to the deck", YS),
        (t_cut1 + 0.5, press, c, "hold it down through the cut", YS),
        (t_cut1 + 2.0, up(press, 0.06), c, "lift the hand clear", YS),
    ])
    # right: torch off its rest, edge start against the straightedge at the far end, drag to the near edge, back
    rest_tcp = (REST[0], REST[1], REST_Z + GRIP_DZ + TIP)
    y0 = LINE_Y + BEAR
    # Edge start: the shield is set down past the deck's far end 10 mm clear of the straightedge (it cannot land on
    # the bar), slightly high (off the deck the unsupported hand sags), drawn sideways against the straightedge's
    # edge, then dragged onto the deck while the lift fades out over the first 15 mm.
    far_clear = torch_tcp(x_far, y0 - 0.010, RIM_Z + START_LIFT)
    far = torch_tcp(x_far, y0, RIM_Z + START_LIFT)
    near = torch_tcp(x_near, y0, RIM_Z)
    # the drag: a knot every KNOT along the line, each reached on the clock at the travel speed (every waypoint eases
    # in and out, so knots this close keep the pace even; one long waypoint would crawl at both ends). At full reach
    # the arm lags, so the first centimetre onto the deck is drawn FAR_PACE faster; past the near edge, where the
    # arm catches up its lag, the run-off is drawn at 60 % of the pace.
    n_knots = int(round((x_far - x_near) / KNOT))
    drag, t = [], t_cut0
    for k in range(n_knots):
        x = x_far - (k + 1) * (x_far - x_near) / n_knots
        pace = FAR_PACE if x >= X1 - SEG else 0.6 if X0 - 0.045 < x < X0 - 0.015 else 1.0
        t += KNOT / (speed * pace)
        z = RIM_Z + START_LIFT * max(0.0, 1.0 - (x_far - x) / 0.015)
        drag.append((t, torch_tcp(x, y0, z), c, "drag the shield along the straightedge toward the near edge", YT))
    right = _timed([
        (1.5, up(rest_tcp, 0.08), o, "above the torch's grip block", YT),
        (3.0, rest_tcp, o, "down to the grip block", YT),
        (3.8, rest_tcp, c, "grip the torch", YT),
        (5.0, up(rest_tcp, 0.08), c, "lift the torch off its rest", YT),
        (T_SET - 2.0, (far[0] - 0.08, far[1] - 0.03, far[2] + 0.05), c, "wait beside the deck", YT),
        (T_SET + 0.5, (far[0] - 0.08, far_clear[1], far_clear[2] + 0.02), c, "over the strip, short of the far end", YT),
        (T_SET + 2.5, (far_clear[0], far_clear[1], far_clear[2] + 0.009), c, "past the deck's far end, low (full reach)", YT),
        (t_cut0 - 1.0, far_clear, c, "shield down past the far end, clear of the straightedge", YT),
        (t_cut0, far, c, "shield against the straightedge's edge", YT),
        *drag,
        (t_cut1 - 0.5, near, c, "shield off the near edge", YT),
        (t_cut1, (near[0], near[1], near[2] + 0.04), c, "lift off", YT),
        (t_back - 1.5, up(rest_tcp, 0.06), c, "carry the torch over its rest", YT),
        (t_back, (rest_tcp[0], rest_tcp[1], rest_tcp[2] + 0.002), c, "lay it on its rest", YT),
        (t_back + 0.8, (rest_tcp[0], rest_tcp[1], rest_tcp[2] + 0.002), o, "let go", YT),
        (t_back + 2.0, up(rest_tcp, 0.10), o, "rise clear", YT),
    ])
    end = max(sum(w.duration for w in left), sum(w.duration for w in right)) + SUCCESS_HOLD_S + 2.0
    for plan in (left, right):
        plan.append(Waypoint(max(0.2, end - sum(w.duration for w in plan)), plan[-1].pos, OPEN, "hold clear",
                             yaw=plan[-1].yaw))
    return {"left": left, "right": right}
