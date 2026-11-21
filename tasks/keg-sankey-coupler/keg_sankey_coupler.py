"""On the bar's back counter stands a full sixtel beer keg (5.16 US gal, about 58 lb = 26.3 kg full; only its top is
modelled: the 230 mm stainless top with its chime ring and the Sankey D valve in the middle). It is not held down:
it stands loose on the wet counter on its bottom chime, under its own weight (friction 0.1), and it slides, spins
or tips if it is pushed hard enough. A Sankey D coupler (about 0.9 kg) stands on a drip tray beside it, handle up,
its beer outlet on top and its gas inlet on the side; a beer line runs from the outlet to a tap tower and a gas
line from the inlet to a CO2 regulator, both connected throughout.

Tapping the keg: lower the coupler over the valve neck with its bayonet ears lined up with the valve's lugs, turn
it a quarter turn clockwise (seen from above) to its stop, then pull the handle down past its over-centre point,
which drives the probe 11.5 mm down into the valve and holds it there. Turning the coupler under the lugs takes
0.92 N*m, and the same torque acts on the keg through the bayonet. The full keg's weight on its chime resists
about 2.7 N*m of turning (0.1 x 26.3 kg x 9.81 m/s2 x 0.105 m) and about 26 N of sideways push, so it takes the
turn without being held; nothing else keeps it in place. The turn is measured against the keg, so a keg that goes
round with the coupler earns no turn. Pressing the handle before the coupler is turned home drives the probe into
a coupler that is not locked to the keg (on a real keg it blows the coupler off or sprays beer), so that latches
the score to 0.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/keg-sankey-coupler/keg_sankey_coupler.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1400 \\
           --instruction "Tap the keg with the coupler." --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
STEEL = (0.76, 0.77, 0.79)
CHROME = (0.84, 0.85, 0.87)
DARK = (0.12, 0.12, 0.13)
ORANGE = (0.95, 0.45, 0.08)
BLACK = (0.08, 0.08, 0.09)

# -- keg top (sixtel: 9.25 in = 235 mm across, 5.16 US gal; about 15 lb empty, 58 lb = 26.3 kg full) -----------------
# A free body: nothing holds it but its weight and its friction on the counter (stainless on a wet stainless top).
KEG_XY = (0.280, 0.000)
KEG_D = 0.230
KEG_KG = 26.3  # a full sixtel
KEG_MU = 0.10  # keg on the counter, static and sliding: the slippery end for wet steel on steel
FOOT_R, FOOT_H = 0.105, 0.002  # the bottom chime ring it stands on
DOME = TOP + 0.012  # the top plate inside the chime
CHIME_TOP = TOP + 0.040
VALVE_H = 0.016  # valve neck above the top plate
VALVE_W = 0.034  # across flats (octagon)
VTOP = DOME + VALVE_H
VALVE_MU = 0.1  # the coupler's steel base on the wet steel neck
LUG_YAW = math.radians(45.0)  # the lugs' line, turned this far counter-clockwise (seen from above) from left-right

_keg_boxes = [mk.box((KEG_D, KEG_D * math.tan(math.pi / 12), DOME - TOP - FOOT_H),
                     at=(0, 0, FOOT_H + (DOME - TOP - FOOT_H) / 2), yaw=k * math.pi / 6)
              for k in range(6)]  # six strips make the 12-sided plate
# what the keg stands on: its bottom chime, a ring (twelve flats) of radius FOOT_R, the only part on the counter
_keg_boxes += [mk.box((0.008, 2 * FOOT_R * math.tan(math.pi / 12), FOOT_H), yaw=k * math.pi / 6, friction=KEG_MU,
                      at=(FOOT_R * math.cos(k * math.pi / 6), FOOT_R * math.sin(k * math.pi / 6), FOOT_H / 2))
               for k in range(12)]
_side = KEG_D * math.tan(math.pi / 12) + 0.002
for _k in range(12):
    _a = _k * math.pi / 6
    _r = KEG_D / 2 - 0.002
    _keg_boxes.append(mk.box((0.004, _side, CHIME_TOP - DOME),
                             at=(_r * math.cos(_a), _r * math.sin(_a), DOME - TOP + (CHIME_TOP - DOME) / 2), yaw=_a))
_keg_boxes += [mk.box((VALVE_W, VALVE_W, VALVE_H), at=(0, 0, DOME - TOP + VALVE_H / 2), yaw=y, color=CHROME,
                      friction=VALVE_MU) for y in (0.0, math.pi / 4)]
# the valve's two bayonet lugs, on opposite faces of the neck (the keg stands turned LUG_YAW)
_keg_boxes += [mk.box((0.010, 0.003, 0.006), color=DARK, yaw=LUG_YAW,
                      at=(-s * 0.0185 * math.sin(LUG_YAW), s * 0.0185 * math.cos(LUG_YAW), VTOP - TOP - 0.006))
               for s in (1, -1)]

kit = mk.Kit()
kit.part("keg", (KEG_XY[0], KEG_XY[1], TOP), _keg_boxes, mass=KEG_KG, color=STEEL)


# -- Sankey D coupler ----------------------------------------------------------------------------------------------
def _square_ring(outer, inner, h, z0, color=None):
    w = (outer - inner) / 2
    c = (outer + inner) / 4
    return [
        mk.box((outer, w, h), at=(0, c, z0 + h / 2), color=color),
        mk.box((outer, w, h), at=(0, -c, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(c, 0, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(-c, 0, z0 + h / 2), color=color),
    ]


TRAY_XY = (0.280, -0.200)
TRAY = (0.100, 0.100, 0.008)
TRAY_TOP = TOP + TRAY[2]
SKIRT = 0.006  # the coupler's skirt hangs this far below its seat face, around the valve neck
C0 = (TRAY_XY[0], TRAY_XY[1], TRAY_TOP + SKIRT)  # coupler frame (seat face centre) standing on the drip tray
SEAT = (KEG_XY[0], KEG_XY[1], VTOP + 0.0005)  # coupler frame seated on the valve
BODY = 0.046
PIVOT = (-0.027, 0.0, 0.044)  # handle pivot in the coupler frame
HANDLE_L = 0.080
HANDLE_UP = 35.0  # deg above horizontal at handle q = 0
HANDLE_MAX = 60.0  # stop: 25 deg below horizontal
OVER_CENTRE = 35.0  # handle q where the toggle goes over centre (bar horizontal)
LOCKED = 52.0
PROBE_TRAVEL = 0.0115
PROBE_FULL_AT = 54.0  # handle q giving full probe travel (cam)
COUPLER_KG, HANDLE_KG, PROBE_KG = 0.75, 0.10, 0.05

kit.part("coupler", C0, [
    *_square_ring(0.064, 0.056, SKIRT, -SKIRT),  # skirt round the valve neck
    *_square_ring(0.064, 0.016, 0.004, 0.0),  # seat face (the probe passes through its middle)
    mk.box((BODY, BODY, 0.046), at=(0, 0, 0.027)),  # body
    mk.box((0.016, 0.016, 0.025), at=(0, 0, 0.0625), color=BLACK),  # beer outlet
    mk.box((0.020, 0.012, 0.010), at=(0.017, 0, 0.046), color=BLACK),  # its elbow, turned down the far face
    mk.box((0.016, 0.012, 0.012), at=(0.031, 0, 0.030), color=DARK),  # gas inlet with its check valve
    mk.box((0.010, 0.020, 0.010), at=(PIVOT[0] + 0.001, 0, PIVOT[2])),  # handle pivot boss
    *[mk.box((0.010, 0.004, 0.008), at=(0, s * 0.034, -0.002), color=ORANGE) for s in (1, -1)],  # bayonet ears
], mass=COUPLER_KG, color=CHROME)

_P0 = np.array(C0) + np.array(PIVOT)
kit.part("handle", tuple(_P0),
         [mk.box((HANDLE_L - 0.012, 0.014, 0.008), at=(-(HANDLE_L - 0.012) / 2, 0, 0)),
          mk.box((0.020, 0.020, 0.012), at=(-HANDLE_L + 0.010, 0, 0), color=BLACK)],
         mass=HANDLE_KG, color=CHROME, quat=mk.mat_to_quat(mk.axis_angle((0, 1, 0), math.radians(HANDLE_UP))))
kit.hinge("handle", "coupler", "handle", anchor=tuple(_P0), axis=(0, -1, 0), lower_deg=0.0, upper_deg=HANDLE_MAX,
          stiffness=0.5, rest_deg=-10.0, damping=0.01)

kit.part("probe", (C0[0], C0[1], C0[2] + 0.012), [mk.box((0.014, 0.014, 0.030), at=(0, 0, 0.015))], mass=PROBE_KG,
         color=DARK)
kit.slider("probe", "coupler", "probe", axis=(0, 0, -1), lower=0.0, upper=PROBE_TRAVEL, stiffness=400.0,
           rest=-0.003, damping=2.0)

# -- beer and gas lines ----------------------------------------------------------------------------------------
# Each line is a chain of rigid 30 mm segments on spring hinges (bending sideways, up/down and twisting), hung
# from its port on the coupler and welded at its far end to a fixed fitting: the beer line from the top outlet to
# the shank of a tap tower, the gas line from the side inlet to a regulator with its shutoff. Both lie in a slack
# arc away from the keg; their weight and stiffness pull on the coupler wherever it is carried, seated or turned.
SEG = 0.030
N_SEG = 12  # 0.36 m of line each
LINE_RISE = 0.0  # deg: climb toward the fitting (0: the lines run level, low along the counter)
LINE_KG = 0.020  # per segment
LINE_K, LINE_C = 0.01, 0.002  # N*m/rad, N*m*s/rad at each bend or twist (about 11 N/m at the segment's length)
LINE_PULL_OFF = 60.0  # N: a line's end fitting lets go above this
BEER_PORT = (0.027, 0.0, 0.046)  # coupler frame: barb of the beer outlet's elbow, past the body's far (+x) face
GAS_PORT = (0.039, 0.0, 0.030)  # coupler frame: the gas inlet's barb
BEER_LINE, GAS_LINE = (0.86, 0.80, 0.55), (0.80, 0.12, 0.10)  # clear vinyl (beer-tinted), red gas hose
LINES = {"beer": (BEER_PORT, 0.010, -60.0, 18.0, BEER_LINE), "gas": (GAS_PORT, 0.012, -50.0, 15.5, GAS_LINE)}  # start heading, turn per segment (deg)
LINE_PARTS = {}
LINE_ENDS = {}
for _line, (_port, _d, _h0, _bend, _col) in LINES.items():
    _p = np.array(C0) + np.array(_port)
    _prev, _h = "coupler", _h0
    LINE_PARTS[_line] = []
    for _i in range(N_SEG):
        _name = f"{_line}_line_{_i}"
        _dz = 0.0
        _a = math.radians(_h)
        _pt = math.radians(LINE_RISE if 4 <= _i < N_SEG - 1 else 0.0)  # climbs to its fitting, which is raised
        _rot = mk.axis_angle((0, 0, 1), _a) @ mk.axis_angle((0, 1, 0), -_pt)
        kit.part(_name, (_p[0], _p[1], _p[2] + _dz), [mk.box((SEG, _d, _d), at=(SEG / 2, 0, 0))], mass=LINE_KG,
                 color=_col, quat=mk.mat_to_quat(_rot))
        # bends sideways, bends up/down and twists in turn: a hose has all three, and with both ends held a chain
        # without the twist would lock the coupler's orientation to the fitting's
        _axis = [(0, 0, 1), tuple(_rot[:, 1]), tuple(_rot[:, 0])][_i % 3]
        kit.hinge(f"{_line}_bend_{_i}", _prev, _name, anchor=(_p[0], _p[1], _p[2] + _dz), axis=_axis,
                  lower_deg=-100, upper_deg=100, stiffness=LINE_K, damping=LINE_C)
        LINE_PARTS[_line].append(_name)
        _p = _p + SEG * _rot[:, 0]
        _prev = _name
        if _i < N_SEG - 1:
            _h += _bend
    kit.tie(f"{_line}_end", _prev, local_xyz=(SEG, 0, 0), break_force=LINE_PULL_OFF)
    LINE_ENDS[_line] = (np.array([_p[0], _p[1], _p[2] + _dz]), math.radians(_h))

# The far ends: a tap tower (its shank takes the beer line) and a CO2 regulator with its shutoff (gas line), both
# fixed to the counter just beyond where each line ends.
_bp, _bh = LINE_ENDS["beer"]
_gp, _gh = LINE_ENDS["gas"]
TOWER_XY = tuple(float(v) for v in _bp[:2] + 0.026 * np.array([math.cos(_bh), math.sin(_bh)]))
REG_XY = tuple(float(v) for v in _gp[:2] + 0.024 * np.array([math.cos(_gh), math.sin(_gh)]))
TOWER_H = 0.280
REG_Z = float(_gp[2])
_anchors = [
    Box("tap_tower", pos=(TOWER_XY[0], TOWER_XY[1], TOP + TOWER_H / 2), size=(0.040, 0.040, TOWER_H),
        color=CHROME, yaw=_bh),
    Box("regulator_post", pos=(REG_XY[0], REG_XY[1], (TOP + REG_Z - 0.030) / 2), size=(0.020, 0.020,
        REG_Z - 0.030 - TOP), color=DARK, yaw=_gh),
    Box("regulator", pos=(REG_XY[0], REG_XY[1], REG_Z), size=(0.036, 0.040, 0.060), color=(0.15, 0.30, 0.70),
        yaw=_gh),
    Box("gas_shutoff", pos=(REG_XY[0], REG_XY[1], REG_Z + 0.036), size=(0.010, 0.030, 0.012),
        color=(0.95, 0.75, 0.1), yaw=_gh),
]

# The bayonet: seated over the neck with its ears on the lugs, the coupler turns clockwise (seen from above) a
# quarter turn to its stop under the lugs. A revolute catch between the coupler and the keg, armed once the coupler
# sits on the valve, so whatever turns the coupler turns the keg the other way through it.
# The lugs' ramps and the seal: turning takes BAYONET_TORQUE (Coulomb friction: a spring whose rest point trails
# the coupler by BAYONET_STICK once it has slipped that far, so the coupler stays where it is left), and over the
# last 15 deg the ramps draw the coupler home to the stop with BAYONET_PULL.
BAYONET_TORQUE, BAYONET_STICK, BAYONET_PULL, BAYONET_DRAW = 0.92, 10.0, 0.4, 75.0  # N*m, deg, N*m, deg
BAYONET_K = BAYONET_TORQUE / math.radians(BAYONET_STICK)  # N*m/rad
_KEG_REF = kit.parts["keg"].ref.copy()
_SEAT_T = mk._T(mk.axis_angle((0, 0, 1), LUG_YAW), SEAT)  # ears on the lugs: the handle points near-right
kit.hinge_catch("bayonet", "coupler", _SEAT_T, anchor=(KEG_XY[0], KEG_XY[1], VTOP), axis=(0, 0, -1),
                lower_deg=-2.0, upper_deg=90.0, parent="keg", parent_T=_KEG_REF, damping=0.1, stiffness=BAYONET_K)

FIXTURES = [Box("drip_tray", pos=(TRAY_XY[0], TRAY_XY[1], TOP + TRAY[2] / 2), size=TRAY, color=BLACK), *_anchors]
OBJECTS = [*kit.objects(), *FIXTURES]

DT = 0.05  # one control step at 20 Hz
ENGAGE_XY, ENGAGE_Z, ENGAGE_TILT, ENGAGE_YAW = 0.011, 0.003, 5.0, 10.0  # XY: the skirt's play round the neck
TURNED, FORCED_TURN, FORCED_HANDLE = 85.0, 80.0, 30.0
FORCED_S = 0.25  # the handle that far down for this long: pushed, not a flick as the coupler drops onto the neck
KEG_TILT, KEG_DROP, DROP_TILT = 20.0, 0.020, 30.0
STILL_S = 1.0


def _tilt_T(t) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(t[2, 2])))))


def seat_offset(objects):
    """The coupler's pose relative to its seated pose on the keg's valve (4x4)."""
    keg = kit._pose_T(objects, "keg")
    cpl = kit._pose_T(objects, "coupler")
    return mk._inv(mk._inv(_KEG_REF) @ _SEAT_T) @ mk._inv(keg) @ cpl


def turn_deg(objects) -> float:
    """How far the coupler has turned clockwise (seen from above) on the valve from the lug-entry position."""
    d = seat_offset(objects)
    return -math.degrees(math.atan2(float(d[1, 0]), float(d[0, 0])))


def handle_deg(objects) -> float:
    return kit.q_deg(objects, "handle")


def probe_m(objects) -> float:
    return kit.q(objects, "probe")


def keg_turn_deg(objects) -> float:
    """How far the keg itself has turned on the counter, clockwise seen from above (not scored: reported)."""
    t = kit._pose_T(objects, "keg")
    return -math.degrees(math.atan2(float(t[1, 0]), float(t[0, 0])))


def keg_slide_m(objects) -> float:
    """How far the keg's centre is from where it was set down (not scored: reported)."""
    t = kit._pose_T(objects, "keg")
    return float(np.linalg.norm(t[:2, 3] - np.array(KEG_XY)))


def keg_tipped(objects) -> bool:
    """Tipped over or off the counter. Sliding or spinning on the counter is not a failure."""
    t = kit._pose_T(objects, "keg")
    return bool(_tilt_T(t) > KEG_TILT or float(t[2, 3]) < TOP - KEG_DROP)


def coupler_dropped(objects) -> bool:
    """Knocked over, or down on the bare counter (not on the tray, not on the keg)."""
    t = kit._pose_T(objects, "coupler")
    return bool(_tilt_T(t) > DROP_TILT or float(t[2, 3]) < TOP + 0.010)


def seated(objects) -> bool:
    d = seat_offset(objects)
    yaw = -math.degrees(math.atan2(float(d[1, 0]), float(d[0, 0])))
    return bool(math.hypot(float(d[0, 3]), float(d[1, 3])) <= ENGAGE_XY and -0.002 <= float(d[2, 3]) <= ENGAGE_Z
                and _tilt_T(d) <= ENGAGE_TILT and abs(yaw) <= ENGAGE_YAW)


def _released(st, margin=0.020) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in mk.tcps())


def let_go_and_still(objects) -> bool:
    return bool(all(_released(objects[p]) and objects[p].resting() for p in ("coupler", "handle"))
                and objects["keg"].resting())


def _rules(objects):
    m = mk.MEMORY
    if not m.get("engaged") and seated(objects):
        kit.engage("bayonet")
        m["engaged"] = True
    if m.get("engaged"):
        tn = turn_deg(objects)
        if tn >= BAYONET_DRAW:  # on the ramps: pulled home (and held against the stop) with BAYONET_PULL
            m["lug_ref"] = tn
            kit.drive_catch("bayonet", tn + BAYONET_STICK * BAYONET_PULL / BAYONET_TORQUE)
        else:  # lug and seal friction: the rest point trails the coupler by at most BAYONET_STICK
            m["lug_ref"] = min(max(m.get("lug_ref", 0.0), tn - BAYONET_STICK), tn + BAYONET_STICK)
            kit.drive_catch("bayonet", m["lug_ref"])
    h = handle_deg(objects)
    kit.drive("handle", HANDLE_MAX + 15.0 if h > OVER_CENTRE else -10.0)  # the toggle's spring, over centre or not
    frac = min(1.0, max(0.0, h / PROBE_FULL_AT))
    kit.drive("probe", -0.003 + frac * (PROBE_TRAVEL + 0.005))  # the handle's cam drives the probe
    m["forced_steps"] = m.get("forced_steps", 0) + 1 if h > FORCED_HANDLE and turn_deg(objects) < FORCED_TURN else 0
    m["forced_longest"] = max(m.get("forced_longest", 0), m["forced_steps"])
    if m["forced_steps"] * DT >= FORCED_S - 1e-9:
        m["forced"] = True
    if keg_tipped(objects):
        m["keg_tipped"] = True
    if coupler_dropped(objects):
        m["dropped"] = True
    if any(kit.tie_off(objects, f"{ln}_end") for ln in LINES):
        m["line_off"] = True
    m["still_steps"] = m.get("still_steps", 0) + 1 if let_go_and_still(objects) else 0
    m["n"] = m.get("n", 0) + 1
    # the most the keg has slid and turned on the counter so far (not scored)
    m["keg_slide"] = max(m.get("keg_slide", 0.0), keg_slide_m(objects))
    m["keg_turn"] = max(m.get("keg_turn", 0.0), abs(keg_turn_deg(objects)))
    m["handle_peak"] = max(m.get("handle_peak", 0.0), h)  # since the last debug line
    if os.environ.get("KEG_DEBUG") and m["n"] % 20 == 0:
        d = seat_offset(objects)
        print(f"[keg] t={m['n'] * DT:.2f}s caught={bool(m.get('engaged'))} off={np.round(d[:3, 3] * 1000, 1)} "
              f"tilt={_tilt_T(d):.1f} turn={turn_deg(objects):.1f} handle={h:.1f} (peak {m['handle_peak']:.1f}) "
              f"probe={probe_m(objects) * 1000:.1f} "
              f"keg: slid {keg_slide_m(objects) * 1000:.1f} mm (most {m['keg_slide'] * 1000:.1f}), turned "
              f"{keg_turn_deg(objects):.1f} deg (most {m['keg_turn']:.1f}), tilt "
              f"{_tilt_T(kit._pose_T(objects, 'keg')):.1f} "
              f"flags={[k for k in ('forced', 'keg_tipped', 'dropped', 'line_off') if m.get(k)]} "
              f"handle early for {m['forced_longest'] * DT:.2f} s at most, still={m['still_steps']} "
              f"score={score(objects)}", flush=True)
        m["handle_peak"] = 0.0


kit.on_step(_rules)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def failed(objects) -> bool:
    m = mk.MEMORY
    return bool(m.get("forced") or m.get("keg_tipped") or m.get("dropped") or m.get("line_off") or keg_tipped(objects)
                or coupler_dropped(objects))


def score(objects) -> float:
    """0 (latched) if the handle was past 30 deg for 0.25 s while the coupler was turned less than 80 deg on the
    keg, if the keg was tipped over (more than 20 deg) or went off the counter, if the coupler was knocked over or
    put down on the bare counter, or if the beer or gas line was pulled off its fitting; also 0 unless the coupler
    caught on the valve. The keg sliding or spinning on the counter costs nothing by itself, but the turn is the
    coupler's angle on the keg's valve, so a keg that spins with the coupler earns no turn. Otherwise the sum of: caught
    0.1; turned 85 deg or more on the keg 0.25; handle locked over centre (52 deg or more) 0.35; probe 90% or more
    of its travel into the valve 0.2; turned home, the coupler and handle let go, and they and the keg still for
    1 s 0.1."""
    if failed(objects) or not mk.MEMORY.get("engaged"):
        return 0.0
    turned = turn_deg(objects) >= TURNED
    s = 0.1 + (0.25 if turned else 0.0)
    s += 0.35 if handle_deg(objects) >= LOCKED else 0.0
    s += 0.2 if probe_m(objects) >= 0.9 * PROBE_TRAVEL else 0.0
    s += 0.1 if turned and mk.MEMORY.get("still_steps", 0) * DT >= STILL_S else 0.0
    return round(s, 6)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    d = seat_offset(objects)
    return float(np.linalg.norm(d[:3, 3]) + math.radians(max(0.0, 90.0 - turn_deg(objects))) * 0.03
                 + math.radians(max(0.0, HANDLE_MAX - handle_deg(objects))) * 0.03)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Right arm: grips the coupler's body, carries it over the chime turned so its ears are over the lugs, lowers it
# onto the valve neck with a light push and works it round in a small spiral until it drops on and catches; then,
# still holding the body, twists it a quarter turn clockwise (a couple about the valve's axis, so little shoves
# the keg sideways), lets go and clears. Nothing touches the keg itself: its weight holds it against the turn.
# The left arm then rides the handle's top face down past over centre with closed fingers.

TIP = 0.013
TRAVEL = TOP + 0.140
GRIP = 0.040  # TCP above the coupler's seat face when holding it (pad tips 27 mm up the body)
PARK_R = (0.200, -0.220, TRAVEL)
PARK_L = (0.200, 0.220, TRAVEL)
PIVOT_Z = SEAT[2] + PIVOT[2]
BODY_OPEN = FINGER_OPEN * 0.75  # fingers 70 mm apart, round the 46 mm body
AIM = (-0.016, 0.036)  # the loaded arm sags toward its base: aim this far off the valve
SEAT_PRESS = 0.004  # the grip is commanded this far below the seated height: a light push down
WIGGLE_R, WIGGLE_TURNS = 0.022, 3  # the spiral that works the coupler onto the neck
TWIST = math.radians(100.0)  # commanded turn of the grip: the wrist trails it under the lug friction
TWIST_S = 8.0


def _place():
    """Carry the coupler to the valve turned LUG_YAW, lower it pressing lightly and spiral until it catches."""
    cx, cy, cz = C0
    ax, ay = KEG_XY[0] + AIM[0], KEG_XY[1] + AIM[1]
    o, c, yaw = BODY_OPEN, FINGER_CLOSED, LUG_YAW
    carry = VTOP + 0.060 + GRIP  # clear of the chime with the lines hanging off it
    down = SEAT[2] + GRIP - SEAT_PRESS
    plan = [
        Waypoint(2.0, (cx, cy, TRAVEL), o, "over the coupler", 0.0),
        Waypoint(1.5, (cx, cy, cz + GRIP), o, "down round its body", 0.0),
        Waypoint(0.5, (cx, cy, cz + GRIP), o, "settle", 0.0),
        Waypoint(1.0, (cx, cy, cz + GRIP), c, "grip it", 0.0),
        Waypoint(1.5, (cx, cy, carry), c, "lift it", 0.0),
        Waypoint(3.0, (ax, ay, carry), c, "over the valve, ears over the lugs", yaw),
        Waypoint(1.0, (ax, ay, carry), c, "settle", yaw),
        Waypoint(2.0, (ax, ay, down), c, "down onto the neck", yaw),
    ]
    n = 8 * WIGGLE_TURNS
    for k in range(1, n + 1):
        a, r = 2 * math.pi * k / 8, WIGGLE_R * k / n
        plan.append(Waypoint(0.25, (ax + r * math.cos(a), ay + r * math.sin(a), down), c,
                             "work it round until it drops on and catches", yaw))
    plan.append(Waypoint(0.6, (ax, ay, down), c, "back over the valve", yaw))
    return plan


def _twist():
    """Still holding the seated coupler's body: turn the grip a quarter turn clockwise."""
    at = (KEG_XY[0] + AIM[0], KEG_XY[1] + AIM[1], SEAT[2] + GRIP - SEAT_PRESS)
    return [
        Waypoint(TWIST_S, at, FINGER_CLOSED, "turn the coupler a quarter turn clockwise", LUG_YAW - TWIST),
        Waypoint(1.2, at, FINGER_CLOSED, "against the stop", LUG_YAW - TWIST),
    ]


def _let_go(yaw):
    at = (KEG_XY[0] + AIM[0], KEG_XY[1] + AIM[1], SEAT[2] + GRIP - SEAT_PRESS)
    return [
        Waypoint(0.8, at, BODY_OPEN, "let go", yaw),
        Waypoint(1.0, (at[0], at[1], TRAVEL), BODY_OPEN, "up off it", yaw),
        Waypoint(1.5, PARK_R, BODY_OPEN, "clear", 0.0),
    ]


def _press(toward_deg, park, lp=0.050):
    """Pad tips on the handle's top face, lp from the pivot, riding it down through over centre to the stop.
    toward_deg: where the handle points, round the keg from +x toward +y."""
    kx, ky = KEG_XY
    r0 = -PIVOT[0]
    ca, sa = math.cos(math.radians(toward_deg)), math.sin(math.radians(toward_deg))
    yaw = math.radians((toward_deg - 90.0 + 90.0) % 180.0 - 90.0)  # fingers closed along the handle

    def at(theta):  # theta: the bar's angle above horizontal
        t = math.radians(theta)
        rad = r0 + lp * math.cos(t) - 0.004 * math.sin(t)
        z = PIVOT_Z + lp * math.sin(t) + 0.004 * math.cos(t) + TIP
        return (kx + rad * ca, ky + rad * sa, z)

    plan = [Waypoint(2.0, (*at(HANDLE_UP)[:2], TRAVEL), FINGER_CLOSED, "over the handle", yaw),
            Waypoint(1.5, tuple(np.add(at(HANDLE_UP), (0, 0, 0.004))), FINGER_CLOSED, "onto it", yaw)]
    plan += [Waypoint(0.8, at(th), FINGER_CLOSED, f"press the handle to {th} deg", yaw)
             for th in range(int(HANDLE_UP) - 10, -31, -10)]
    plan.append(Waypoint(1.0, at(-30), FINGER_CLOSED, "hold it at the stop", yaw))
    return plan + [Waypoint(1.0, tuple(np.add(at(-30), (0, 0, 0.040))), FINGER_CLOSED, "up off it", yaw),
                   Waypoint(1.5, park, FINGER_CLOSED, "clear", 0.0)]


def _dur(plan):
    return sum(w.duration for w in plan)


VARIANTS = ("full", "no_turn", "no_handle", "idle")


def demo():
    """KEG_DEMO picks a wrong path instead of the full demo. no_turn: the right arm seats the coupler, lets go and
    presses the handle without turning it. no_handle: seated and turned home, but the handle is never pressed.
    idle: neither arm touches anything."""
    variant = os.environ.get("KEG_DEMO", "full")
    if variant not in VARIANTS:
        raise ValueError(f"KEG_DEMO must be one of {VARIANTS}, got {variant!r}")
    if variant == "idle":
        return {"left": [Waypoint(3.0, PARK_L, FINGER_CLOSED, "wait", 0.0)],
                "right": [Waypoint(3.0, PARK_R, BODY_OPEN, "wait", 0.0)]}
    right = _place()
    if variant == "no_turn":
        right += _let_go(LUG_YAW)[:2] + _press(180.0 + math.degrees(LUG_YAW), PARK_R)
        return {"left": [Waypoint(_dur(right), PARK_L, FINGER_CLOSED, "wait", 0.0)], "right": right}
    right += _twist() + _let_go(LUG_YAW - TWIST)
    wait = [Waypoint(_dur(right) - 1.5, PARK_L, FINGER_CLOSED, "wait until the right hand is up off the coupler", 0.0)]
    if variant == "no_handle":
        return {"left": wait, "right": right}
    return {"left": wait + _press(90.0 + math.degrees(LUG_YAW), PARK_L), "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    seat_T = _SEAT_T

    def keg_at(dx=0.0, dy=0.0, spin=0.0, tilt=0.0):
        """The keg slid (dx, dy), spun ``spin`` deg clockwise seen from above, tipped ``tilt`` deg."""
        rot = mk.axis_angle((0, 0, -1), math.radians(spin)) @ mk.axis_angle((1, 0, 0), math.radians(tilt))
        return mk._T(rot, (KEG_XY[0] + dx, KEG_XY[1] + dy, TOP))

    def pose(t):
        return (t[:3, 3], mk.mat_to_quat(t[:3, :3]))

    def st(turn=90.0, handle=0.0, probe=0.0, keg=None, dz=0.0, dx=0.0):
        """The coupler turned ``turn`` deg on the valve of a keg posed ``keg`` (4x4; None: where it spawns)."""
        keg = _KEG_REF if keg is None else keg
        on_valve = mk._T(mk.axis_angle((0, 0, 1), LUG_YAW - math.radians(turn)), (SEAT[0] + dx, SEAT[1], SEAT[2] + dz))
        moved = {"keg": pose(keg), "coupler": pose(keg @ mk._inv(_KEG_REF) @ on_valve)}
        return kit.fake_states(q={"handle": handle, "probe": probe}, deg=True, moved=moved)

    full = dict(turn=90.0, handle=60.0, probe=PROBE_TRAVEL)
    E = {"engaged": True}
    S = {"engaged": True, "still_steps": 25}
    tipped = (np.array(C0), mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(80))))
    cases = {  # name: (state, MEMORY, score)
        "start": (kit.fake_states(), {}, 0.0),
        "on the valve, not caught": (st(turn=0.0), {}, 0.0),
        "caught, not turned": (st(turn=0.0), E, 0.1),
        "turned 90": (st(), E, 0.35),
        "turned 90, let go and still": (st(), S, 0.45),
        "turned 70 only": (st(turn=70.0), S, 0.1),
        "turned, handle half down": (st(handle=30.0, probe=0.006), S, 0.45),
        "turned, locked, probe home, still": (st(**full), S, 1.0),
        "turned, locked, probe home, not yet still": (st(**full), E, 0.9),
        "locked but probe short": (st(turn=90, handle=60, probe=0.008), S, 0.8),
        "a line pulled off its fitting": (st(**full), {**S, "line_off": True}, 0.0),
        "handle forced before the turn": (st(**full), {**S, "forced": True}, 0.0),
        # the keg is loose: sliding and spinning cost nothing, and the turn is measured on the keg
        "keg slid 30 mm and spun 25 deg, coupler home on it": (st(**full, keg=keg_at(0.030, -0.010, 25.0)), S, 1.0),
        "keg spun 90 deg with the coupler (no turn on the keg)": (st(turn=0.0, keg=keg_at(spin=90.0)), S, 0.1),
        "keg spun 60 deg, coupler 90 deg in the room (30 on the keg)": (st(turn=30.0, keg=keg_at(spin=60.0)), S,
                                                                        0.1),
        "keg tipped over": (st(**full, keg=keg_at(tilt=25.0)), S, 0.0),
        "keg tipped earlier, stood back up": (st(**full), {**S, "keg_tipped": True}, 0.0),
        "keg tilted 10 deg": (st(**full, keg=keg_at(tilt=10.0)), S, 1.0),
        "coupler dropped earlier": (st(**full), {**S, "dropped": True}, 0.0),
        "coupler knocked over": (kit.fake_states(moved={"coupler": tipped}), E, 0.0),
    }
    try:
        for name, (s_, mem, want) in cases.items():
            mk.new_episode()
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        mk.new_episode()
        assert abs(turn_deg(st(turn=37.0)) - 37.0) < 1e-6 and seated(st(turn=8.0)) and not seated(st(turn=12.0))
        assert abs(turn_deg(st(turn=37.0, keg=keg_at(0.02, 0.01, 50.0))) - 37.0) < 1e-6
        assert abs(keg_turn_deg(st(turn=37.0, keg=keg_at(spin=50.0))) - 50.0) < 1e-6
        assert seated(st(turn=3.0, keg=keg_at(0.02, 0.0, 40.0))) and seated(st(turn=0.0, dx=0.008))
        assert not seated(st(turn=0.0, dx=0.014)) and not seated(st(turn=0.0, dz=0.005))  # beside / on top of the neck
        assert seat_T is _SEAT_T
    finally:
        mk.new_episode()
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for variant in VARIANTS:
        os.environ["KEG_DEMO"] = variant
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
            print(f"{variant} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
    os.environ.pop("KEG_DEMO", None)
