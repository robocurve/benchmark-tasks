"""A gravity-feed deli slicer on the bench, after service. Its 254 mm (10 in) knife is a fixed disc standing upright
across the back of the machine; in front of it a product carriage (the tray the meat rides on, 1.6 kg) sits on a
rail running left-right, parked mid-stroke in front of the knife. The gauge plate, which sets the slice thickness,
is open at 12 mm, so a strip of the knife's edge is bare where the carriage passes. The gauge knob (left, on its
pedestal) closes the plate: 7 deg of knob per mm, 12 detents of 2 mm (14 deg each). The carriage is locked to its rail by a threaded
tilt-release knob on the right (three turns out frees it), and it lifts off only at the right-hand end of its
stroke, and only with the gauge at zero: the interlock bar under the carriage arm catches it otherwise, as on real
slicers. A drain tray stands on the bench to the right.

The job, as in the NSF / OSHA slicer cleaning procedure: close the gauge to zero so the plate covers the edge, back
the release knob out, run the carriage to the end of its stroke, lift it off and set it down on the drain tray.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/slicer-gauge-zero-then-remove-carriage/slicer_gauge_zero_then_remove_carriage.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1600 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
ALU = (0.78, 0.79, 0.80)  # anodised aluminium (slicer body, carriage)
KNIFE = (0.86, 0.87, 0.90)
DARK = (0.16, 0.16, 0.17)
WHITE = (0.93, 0.93, 0.91)
KNOB = (0.12, 0.12, 0.13)
RED = (0.80, 0.14, 0.12)
TRAY_C = (0.55, 0.57, 0.60)
MARK = (0.15, 0.35, 0.75)

# -- knife and gauge plate -------------------------------------------------------------------------------------
BLADE_X = 0.360  # the knife's front face (plane y-z)
BLADE_C = (BLADE_X, 0.080, TOP + 0.150)  # hub
BLADE_R = 0.127  # 254 mm knife
GAUGE_DEG_PER_MM = 7.0  # 0-24 mm over 168 deg of knob
DETENT_MM = 2.0  # 12 detents
G0_MM = 12.0  # start: plate open 12 mm


def _disc(r, thick, at, color=None, axis="x", n=3):
    """A disc as n squares turned about its axis (a 4n-gon)."""
    side = 2 * r / math.sqrt(2) * 1.08
    ax = (1, 0, 0) if axis == "x" else (0, 0, 1)
    size = (thick, side, side) if axis == "x" else (side, side, thick)
    return [mk.box(size, at=at, color=color, quat=mk.mat_to_quat(mk.axis_angle(ax, math.pi / 2 * i / n)))
            for i in range(n)]


kit = mk.Kit()
# The knife: a fixed part (the gauge plate is jointed to it, so the two never collide).
kit.part("knife", (0, 0, 0), [*_disc(BLADE_R, 0.002, (BLADE_C[0] + 0.001, BLADE_C[1], BLADE_C[2]), KNIFE),
                              *_disc(0.040, 0.012, (BLADE_C[0] + 0.007, BLADE_C[1], BLADE_C[2]), DARK)],
         mass=None, color=KNIFE)
# Gauge plate: covers the front-lower arc of the knife where the carriage passes. At 0 its face stands 4 mm proud of
# the knife's face (edge covered); opening it slides it back behind the knife face by the slice thickness.
PLATE = (0.006, 0.120, 0.090)
PLATE_AT = (BLADE_X - 0.004 - PLATE[0] / 2, -0.005, TOP + 0.030 + PLATE[2] / 2)
kit.part("gauge_plate", (0, 0, 0), [mk.box(PLATE, at=PLATE_AT, color=ALU),
                                    mk.box((0.010, 0.004, PLATE[2]), at=(PLATE_AT[0] - 0.002, PLATE_AT[1] + PLATE[1] / 2,
                                                                         PLATE_AT[2]), color=DARK)],
         mass=0.350, color=ALU)
kit.slider("gauge_travel", "knife", "gauge_plate", axis=(1, 0, 0), lower=0.0, upper=0.024, start=G0_MM / 1000,
           stiffness=4000.0, rest=G0_MM / 1000, damping=40.0)

# Knobs: 50 mm, 22 mm tall, fluted (a 12-sided prism here), turned by pinching the rim.
KNOB_H = 0.022
KNOB_D = 0.050


def _knob_boxes(color=KNOB):
    return [*_disc(KNOB_D / 2 / 1.08, KNOB_H, (0, 0, KNOB_H / 2), color, axis="z", n=3),
            mk.box((0.012, 0.004, 0.001), at=(0.010, 0, KNOB_H + 0.0005), color=RED)]  # pointer


GAUGE_PED = (0.260, 0.240, 0.070, 0.045)  # x, y, side, height
GK = (GAUGE_PED[0], GAUGE_PED[1], TOP + GAUGE_PED[3] + 0.0005)
kit.part("gauge_knob", GK, _knob_boxes(), mass=0.080, color=KNOB)
kit.hinge("gauge_turn", None, "gauge_knob", anchor=GK, axis=(0, 0, 1), lower_deg=-2.0, upper_deg=170.0,
          start_deg=G0_MM * GAUGE_DEG_PER_MM, stiffness=0.15, rest_deg=G0_MM * GAUGE_DEG_PER_MM, damping=0.004)

RELEASE_PED = (0.335, -0.170, 0.040, 0.030)
RK = (RELEASE_PED[0], RELEASE_PED[1], TOP + RELEASE_PED[3] + 0.0005)
kit.part("release_knob", RK, _knob_boxes(), mass=0.060, color=KNOB)
# threaded: no stops in the model (the knob is captive), viscous thread drag only
kit.hinge("release_turn", None, "release_knob", anchor=RK, axis=(0, 0, 1), lower_deg=-math.inf, upper_deg=math.inf,
          damping=0.010)
RELEASE_TURNS_DEG = 1080.0
RELEASE_OUT_DEG = 1060.0  # backed out (the end detent clicks)

# -- carriage ---------------------------------------------------------------------------------------------------
CX, CY0 = 0.250, 0.030  # deck centre at the start (mid-stroke, in front of the knife)
RAIL_H = 0.021
DECK_Z = TOP + RAIL_H + 0.001  # deck underside, 1 mm over the rail
DECK = (0.140, 0.180, 0.008)
FENCE = (0.012, 0.180, 0.050)  # end weight / product pusher on the operator side: the COM sits off centre
FIN = (0.040, 0.006, 0.030)  # lifting handles at both ends
FIN_X = -0.020
FIN_Y = DECK[1] / 2 + FIN[1] / 2
_CARRIAGE = [mk.box(DECK, at=(0, 0, DECK[2] / 2)),
             mk.box(FENCE, at=(-DECK[0] / 2 + FENCE[0] / 2, 0, DECK[2] + FENCE[2] / 2)),
             mk.box(FIN, at=(FIN_X, FIN_Y, DECK[2] + FIN[2] / 2), color=DARK),
             mk.box(FIN, at=(FIN_X, -FIN_Y, DECK[2] + FIN[2] / 2), color=DARK)]
CARRIAGE_KG = 1.6
kit.part("carriage", (CX, CY0, DECK_Z), _CARRIAGE, mass=CARRIAGE_KG, color=ALU)
RAIL_LO, RAIL_HI = -0.090, 0.060
LIFT_AT = RAIL_LO + 0.005  # the lift-off slot at the right-hand (operator) end
kit.slider("rail", None, "carriage", axis=(0, 1, 0), lower=RAIL_LO, upper=RAIL_HI, damping=8.0)
LIFT_SEAT = np.array([CX, CY0 + RAIL_LO, DECK_Z])

# -- drain tray -------------------------------------------------------------------------------------------------
TRAY_XY = (0.225, -0.215)
TRAY = (0.170, 0.220, 0.006)
TRAY_TOP = TOP + TRAY[2]

FIXTURES = [
    # body: motor housing behind the knife, knife pillar, base
    Box("body", pos=(0.405, 0.090, TOP + 0.060), size=(0.030, 0.450, 0.120), color=ALU),
    Box("pillar", pos=(0.385, BLADE_C[1], TOP + 0.130), size=(0.020, 0.100, 0.150), color=ALU),
    Box("gauge_pedestal", pos=(GAUGE_PED[0], GAUGE_PED[1], TOP + GAUGE_PED[3] / 2),
        size=(GAUGE_PED[2], GAUGE_PED[2], GAUGE_PED[3]), color=ALU),
    Box("release_pedestal", pos=(RELEASE_PED[0], RELEASE_PED[1], TOP + RELEASE_PED[3] / 2),
        size=(RELEASE_PED[2], RELEASE_PED[2], RELEASE_PED[3]), color=ALU),
    Box("rail", pos=(CX, 0.040, TOP + RAIL_H / 2), size=(0.030, 0.270, RAIL_H), color=DARK),
    Box("gauge_scale", pos=(GAUGE_PED[0] - GAUGE_PED[2] / 2 - 0.0005, GAUGE_PED[1], TOP + GAUGE_PED[3] - 0.010),
        size=(0.001, 0.040, 0.012), color=WHITE),
    Box("drain_tray", pos=(TRAY_XY[0], TRAY_XY[1], TOP + TRAY[2] / 2), size=TRAY, color=TRAY_C),
    Box("tray_mark", pos=(TRAY_XY[0] - TRAY[0] / 2 + 0.006, TRAY_XY[1], TRAY_TOP + 0.0005), size=(0.006, 0.150, 0.001),
        color=MARK),
]
OBJECTS = [*kit.objects(), *FIXTURES]

# Blade zone: a box 45 mm deep in front of the whole knife disc (x 31.5-37, y -4.7 to 20.7, 2.3-27.7 cm up).
# The plate's window (y -6.5 to 5.5, 3-12 cm up) is covered only while the gauge reads 1 mm or less; the rest of the
# disc stays exposed with the gauge closed, so an arm there fails at any time.
EDGE_LO = np.array([BLADE_X - 0.045, BLADE_C[1] - BLADE_R, BLADE_C[2] - BLADE_R])
EDGE_HI = np.array([BLADE_X + 0.010, BLADE_C[1] + BLADE_R, BLADE_C[2] + BLADE_R])
COVER_LO = np.array([PLATE_AT[1] - PLATE[1] / 2, PLATE_AT[2] - PLATE[2] / 2])  # y, z of the plate
COVER_HI = np.array([PLATE_AT[1] + PLATE[1] / 2, PLATE_AT[2] + PLATE[2] / 2])
OPEN_MM = 1.0  # gauge above this: the plate window is bare too


def _unwrap(key, raw_deg, start_deg):
    prev = mk.MEMORY.get(key, start_deg)
    d = (raw_deg - prev + 180.0) % 360.0 - 180.0
    mk.MEMORY[key] = prev + d
    return prev + d


def gauge_mm() -> float:
    return mk.MEMORY.get("gauge_deg", G0_MM * GAUGE_DEG_PER_MM) / GAUGE_DEG_PER_MM


def release_deg() -> float:
    return mk.MEMORY.get("release_deg", 0.0)


_TEST_POINTS = None


def robot_points() -> list:
    """Every link origin of both arms plus both TCPs (world); [] offline unless a test sets them."""
    env = mk._EP["env"]
    if env is None:
        return list(_TEST_POINTS or [])
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        raw = base.scene[key].data.body_pos_w
        pos = (getattr(raw, "torch", raw)[0] - origin).detach().cpu().numpy().astype(float)
        out.extend(list(pos))
    return out + list(mk.tcps())


# -- arm collision surfaces -----------------------------------------------------------------------------------------
# Every collision piece of the YAM URDF (the convex STLs PhysX collides with), reduced to the 8 corners of its box in
# its link's frame. At every step the corners are posed with the link and the piece's world box is tested against
# the blade zone: a box that encloses the piece, so no finger pad or link surface can enter the zone unseen.


def _rpy(rpy):
    r_, p_, y_ = rpy
    cx, sx, cy, sy, cz, sz = math.cos(r_), math.sin(r_), math.cos(p_), math.sin(p_), math.cos(y_), math.sin(y_)
    return (np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]))


def _stl_points(path):
    raw = Path(path).read_bytes()
    if raw[:5] == b"solid" and b"facet" in raw[:400]:
        return np.array([[float(v) for v in ln.split()[1:4]] for ln in raw.decode().splitlines()
                         if ln.strip().startswith("vertex")])
    n = int.from_bytes(raw[80:84], "little")
    tri = np.frombuffer(raw[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return tri["v"].reshape(-1, 3).astype(float)


def _link_corners():
    """{link: (k*8, 3) corner points in the link frame}, from the URDF's collision elements."""
    import xml.etree.ElementTree as ET
    from yam_arena import YAM_URDF
    out = {}
    root = ET.parse(YAM_URDF).getroot()
    for link in root.iter("link"):
        pts = []
        for col in link.iter("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            R = _rpy([float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()])
            mesh = col.find("geometry/mesh")
            if mesh is None:
                continue
            sc = np.array([float(v) for v in mesh.get("scale", "1 1 1").split()])
            v = _stl_points(Path(YAM_URDF).parent / mesh.get("filename")) * sc
            lo, hi = v.min(0), v.max(0)
            c = np.array([[a, b, d] for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for d in (lo[2], hi[2])])
            pts.append(c @ R.T + xyz)
        if pts:
            out[link.get("name")] = np.concatenate(pts)
    return out


LINK_CORNERS = _link_corners()


def _quat_wxyz_mat(q):
    w, x, y, z = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


_TEST_BOXES = None


def robot_boxes() -> list:
    """World boxes (lo, hi) enclosing every collision piece of both arms, finger pads included."""
    env = mk._EP["env"]
    if env is None:
        return list(_TEST_BOXES or [])
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        rp, rq = art.data.body_pos_w, art.data.body_quat_w
        pos = (getattr(rp, "torch", rp)[0] - origin).detach().cpu().numpy().astype(float)
        quat = getattr(rq, "torch", rq)[0].detach().cpu().numpy().astype(float)
        for i, name in enumerate(art.body_names):
            c = LINK_CORNERS.get(name)
            if c is None:
                continue
            w = c.reshape(-1, 8, 3) @ _quat_wxyz_mat(quat[i]).T + pos[i]
            out.extend(zip(w.min(1), w.max(1)))
            if os.environ.get("SLICER_DEBUG"):
                for lo, hi in zip(w.min(1), w.max(1)):
                    if _box_in_edge(lo, hi) and not mk.MEMORY.get("cut"):
                        print(f"[slicer] step {mk._EP['step']} {key} {name} box {np.round(lo, 3)} {np.round(hi, 3)}"
                              f" pos {np.round(pos[i], 3)}", flush=True)
    return out


def _box_in_edge(lo, hi, gauge=None) -> bool:
    """A world box overlaps bare blade: the blade zone, less the plate window while the gauge is closed."""
    lo, hi = np.maximum(np.asarray(lo, float), EDGE_LO), np.minimum(np.asarray(hi, float), EDGE_HI)
    if np.any(lo > hi):
        return False
    g = gauge_mm() if gauge is None else gauge
    covered = g <= OPEN_MM and bool(np.all(lo[1:] >= COVER_LO) and np.all(hi[1:] <= COVER_HI))
    return not covered


def _in_edge(p, gauge=None) -> bool:
    """In front of bare blade: the blade zone, minus the plate's window while the gauge is closed."""
    p = np.asarray(p, float)
    if not (np.all(p >= EDGE_LO) and np.all(p <= EDGE_HI)):
        return False
    g = gauge_mm() if gauge is None else gauge
    covered = g <= OPEN_MM and bool(np.all(p[1:] >= COVER_LO) and np.all(p[1:] <= COVER_HI))
    return not covered


def _tilt(q) -> float:
    z = mk.quat_to_mat(q)[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def _mechanism_rule(objects):
    """Gauge knob detents and the plate coupled to it; release-knob turns; the carriage interlock; latches."""
    M = mk.MEMORY
    if not M.get("init"):  # a new episode: the rail holds the carriage again
        M["init"] = True
        mk._set_joint_enabled(kit, "rail", True)
        M["rail_free"] = False
    g = _unwrap("gauge_deg", kit.q_deg(objects, "gauge_turn"), G0_MM * GAUGE_DEG_PER_MM)
    det = min(168.0, max(0.0, round(g / (DETENT_MM * GAUGE_DEG_PER_MM)) * DETENT_MM * GAUGE_DEG_PER_MM))
    if M.get("detent") != det:
        M["detent"] = det
        kit.drive("gauge_turn", det)
    kit.drive("gauge_travel", max(0.0, min(24.0, g / GAUGE_DEG_PER_MM)) / 1000.0)
    _unwrap("release_deg", kit.q_deg(objects, "release_turn"), 0.0)

    travel = kit.q(objects, "rail")
    if "order_ok" not in M and abs(travel) > 0.005:  # the carriage first moved: was the gauge closed by then?
        M["order_ok"] = gauge_mm() <= 0.5
    p, q = kit.pose(objects, "carriage")
    if not M.get("rail_free"):
        if gauge_mm() <= 0.5 and release_deg() >= RELEASE_OUT_DEG and travel <= LIFT_AT:
            M["rail_free"] = True
            mk._set_joint_enabled(kit, "rail", False)
        elif abs(float(p[0]) - CX) > 0.010 or abs(float(p[2]) - DECK_Z) > 0.010:
            M["forced"] = True  # off its rail against the interlock
    boxes = robot_boxes()
    if any(_in_edge(pt) for pt in robot_points()) or any(_box_in_edge(lo, hi) for lo, hi in boxes):
        M["cut"] = True
    if os.environ.get("SLICER_DEBUG") and boxes:  # closest approach of any collision box to the blade zone
        gap = min(float(np.max(np.maximum(EDGE_LO - hi, lo - EDGE_HI))) for lo, hi in boxes)
        M["min_gap"] = min(M.get("min_gap", 1.0), gap)
    if _tilt(q) > 45.0:
        M["tipped"] = True
    if float(p[2]) < TOP - 0.050:
        M["dropped"] = True
    if os.environ.get("SLICER_DEBUG") and mk._EP["step"] % 20 == 0:
        print(f"[slicer] step {mk._EP['step']} gauge {gauge_mm():.2f} mm release {release_deg():.0f} deg "
              f"rail {travel * 1000:.1f} mm carriage {np.round(p, 4)} tilt {_tilt(q):.1f} "
              f"flags { {k: v for k, v in M.items() if k in ('order_ok', 'rail_free', 'cut', 'forced', 'tipped', 'dropped', 'min_gap')} }"
              f" score {score(objects)}", flush=True)


kit.on_step(_mechanism_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def failed(objects) -> bool:
    """Latched: a link in the bare-edge zone with the gauge open, the carriage forced off its rail, tipped over 45
    deg or dropped off the bench."""
    M = mk.MEMORY
    return bool(M.get("cut") or M.get("forced") or M.get("tipped") or M.get("dropped"))


def off_rail(objects) -> bool:
    """Lifted off (the interlock let it go) and 30 mm or more from its lift-off seat."""
    p, _ = kit.pose(objects, "carriage")
    return bool(mk.MEMORY.get("rail_free")) and float(np.linalg.norm(p - LIFT_SEAT)) >= 0.030


def over_tray(objects) -> bool:
    p, _ = kit.pose(objects, "carriage")
    c = kit.point(objects, "carriage", (0, 0, 0))
    return abs(float(c[0]) - TRAY_XY[0]) <= TRAY[0] / 2 and abs(float(c[1]) - TRAY_XY[1]) <= TRAY[1] / 2


def on_tray(objects) -> bool:
    """Flat (within 5 deg) on the tray: underside within 4 mm of the tray top, deck centre within 20 mm in x and
    25 mm in y of the tray's centre."""
    p, q = kit.pose(objects, "carriage")
    return (abs(float(p[0]) - TRAY_XY[0]) <= 0.020 and abs(float(p[1]) - TRAY_XY[1]) <= 0.025
            and abs(float(p[2]) - TRAY_TOP) <= 0.004 and _tilt(q) <= 5.0)


def gauge_closed(objects) -> bool:
    return gauge_mm() <= 0.5 and objects["gauge_knob"].resting(max_speed=0.01, max_spin=0.2)


def score(objects) -> float:
    """0 if failed() or the carriage moved 5 mm before the gauge was at 0. Otherwise staged, each stage needing the
    ones before: gauge at 0 (0.5 mm or less) and still 0.2; carriage freed (interlock released) 0.4; carriage
    lifted off, 30 mm or more from its seat 0.6; flat on the drain tray, let go (no gripper within 2 cm) and
    still 1.0."""
    if failed(objects) or mk.MEMORY.get("order_ok") is False:
        return 0.0
    if not gauge_closed(objects):
        return 0.0
    if not mk.MEMORY.get("rail_free"):
        return 0.2
    if not off_rail(objects):
        return 0.4
    st = objects["carriage"]
    if over_tray(objects) and on_tray(objects) and _released(st) and st.resting(max_speed=0.01, max_spin=0.2):
        return 1.0
    return 0.6


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    p, _ = kit.pose(objects, "carriage")
    return float(gauge_mm() / 1000.0 + np.linalg.norm(p - np.array([TRAY_XY[0], TRAY_XY[1], TRAY_TOP])))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Left arm: pinches the gauge knob and turns it down to 0 in one wrist turn. Right arm (at the same
# time, well clear of the knife): backs the release knob out three turns, regrasping every 120 deg. Then each arm pinches one
# end handle of the carriage; together they run it to the right-hand end of its stroke, lift it 25 mm off the rail,
# carry it right over the drain tray, set it down and let go.

TIP = 0.013
TRAVEL = TOP + 0.090
PARK_L = (0.18, 0.24, TRAVEL)
PARK_R = (0.18, -0.26, TRAVEL)
GRASP_Z = 0.004 + TIP  # pad tips 4 mm above the knob's foot
KNOB_OPEN = -0.032
OPEN = -0.012  # pads' inner faces ~11 mm off the TCP: round a 6 mm handle
FIN_TOP = DECK_Z + DECK[2] + FIN[2]
GRIP_Z = FIN_TOP - 0.010 + TIP
LIFT = 0.025
CARRY_DY = TRAY_XY[1] - (CY0 + RAIL_LO)
CARRY_DX = TRAY_XY[0] - CX
SET_DZ = TRAY_TOP - DECK_Z - 0.002


def _twist(c, start_yaw, stop_yaw, sense, label, per_grasp=120.0, turn_s=1.6):
    """Turn a knob by pinching its rim and turning the wrist, regrasping every `per_grasp` deg. The knob turns
    from `start_yaw` to `stop_yaw` (deg, unwrapped, in the sense `sense`); the wrist works within +-60 deg."""
    over, at = (c[0], c[1], c[2] + GRASP_Z + 0.030), (c[0], c[1], c[2] + GRASP_Z)
    total = abs(stop_yaw - start_yaw)
    lo = start_yaw if sense > 0 else start_yaw  # first grasp at the knob's start angle (within +-60 here)
    y0 = math.radians(lo)
    plan = [Waypoint(2.0, over, KNOB_OPEN, f"over the {label}", y0),
            Waypoint(1.5, at, KNOB_OPEN, f"down round the {label}", y0)]
    done = 0.0
    while done < total - 1e-6:
        step = min(per_grasp, total - done)
        if done > 0:  # let go, turn the wrist back, take a new grip
            plan += [Waypoint(0.6, at, KNOB_OPEN, "let go", plan[-1].yaw),
                     Waypoint(0.8, at, KNOB_OPEN, "turn the wrist back", math.radians(-sense * per_grasp / 2))]
        yb = plan[-1].yaw
        plan += [Waypoint(0.6, at, FINGER_CLOSED, f"grip the {label}", yb),
                 Waypoint(turn_s * step / per_grasp, at, FINGER_CLOSED, f"turn the {label}", yb + math.radians(sense * step))]
        done += step
    return plan + [Waypoint(0.6, at, KNOB_OPEN, "let go", plan[-1].yaw),
                   Waypoint(1.0, over, KNOB_OPEN, "up off the knob", 0.0)]


def _gauge_down():
    return _twist(GK, 40.0, -60.0, -1, "gauge knob", per_grasp=100.0, turn_s=3.0)


def _release_out():
    s = _twist(RK, -60.0, RELEASE_TURNS_DEG + 80.0 - 60.0, +1, "release knob")
    # draw back toward the robot before crossing to the carriage, so the forearm stays clear of the knife
    return s + [Waypoint(1.0, (0.230, RK[1], RK[2] + GRASP_Z + 0.030), KNOB_OPEN, "back off toward you", 0.0)]


def _dur(plan):
    return sum(w.duration for w in plan)


def _carriage(side, lift=True, variant=""):
    """One arm's part in moving the carriage: pinch its handle, run it to the lift-off end, lift, carry, set down."""
    s = 1 if side == "left" else -1
    x, y0 = CX + FIN_X, CY0 + s * FIN_Y
    y1 = y0 + RAIL_LO - 0.003  # to the stop
    plan = [Waypoint(2.0, (x, y0, GRIP_Z + 0.030), OPEN, "over the carriage handle", 0.0),
            Waypoint(1.5, (x, y0, GRIP_Z), OPEN, "down round the handle", 0.0),
            Waypoint(1.0, (x, y0, GRIP_Z), FINGER_CLOSED, "pinch the handle", 0.0),
            Waypoint(3.0, (x, y1, GRIP_Z), FINGER_CLOSED, "run the carriage to the lift-off end", 0.0),
            Waypoint(0.5, (x, y1, GRIP_Z), FINGER_CLOSED, "settle", 0.0)]
    if not lift:
        return plan
    up = GRIP_Z + LIFT
    plan += [Waypoint(1.5, (x, y1, up), FINGER_CLOSED, "lift it off the rail", 0.0)]
    if variant == "sideways":
        return plan[:-1] + [Waypoint(2.0, (x - 0.150, y1, GRIP_Z + 0.004), FINGER_CLOSED, "drag it off the rail forward", 0.0),
                            Waypoint(1.0, (x - 0.150, y1, GRIP_Z + 0.004), OPEN, "let go", 0.0),
                            Waypoint(1.0, (x - 0.150, y1, TRAVEL + 0.02), OPEN, "up", 0.0)]
    if variant == "on_body":  # set it back down on the slicer instead of the tray
        yd, zd = y1, GRIP_Z - 0.0005
    else:
        yd, zd = y1 + CARRY_DY, GRIP_Z + SET_DZ
        x += CARRY_DX
        plan += [Waypoint(1.0, (x, y1, up), FINGER_CLOSED, "draw it toward you, clear of the release knob", 0.0),
                 Waypoint(3.5, (x, yd, up), FINGER_CLOSED, "carry it over the drain tray", 0.0)]
    return plan + [Waypoint(1.5, (x, yd, zd), FINGER_CLOSED, "set it down", 0.0),
                   Waypoint(0.5, (x, yd, zd), FINGER_CLOSED, "settle", 0.0),
                   Waypoint(1.0, (x, yd, zd), OPEN, "let go", 0.0),
                   Waypoint(1.0, (x, yd + s * 0.010, zd + 0.045), OPEN, "up off the handle", 0.0),
                   Waypoint(1.5, (x - 0.04, yd + s * 0.030, zd + 0.050), OPEN, "clear", 0.0)]


def _wait(plan, until, label="wait"):
    t = _dur(plan)
    if until > t:
        last = plan[-1] if plan else Waypoint(0, PARK_L, FINGER_CLOSED)
        plan.append(Waypoint(until - t, last.pos, last.finger, label, last.yaw))
    return plan


def _plans(gauge=True, release=True, carriage=True, variant=""):
    left = _gauge_down() if gauge else [Waypoint(2.0, PARK_L, FINGER_CLOSED, "wait", 0.0)]
    right = _release_out() if release else [Waypoint(2.0, PARK_R, FINGER_CLOSED, "wait", 0.0)]
    if not carriage:
        return {"left": left + [Waypoint(2.0, PARK_L, FINGER_CLOSED, "clear", 0.0)],
                "right": right + [Waypoint(2.0, PARK_R, FINGER_CLOSED, "clear", 0.0)]}
    t = max(_dur(left), _dur(right)) + 0.5
    lift = variant != "lift_first"
    left = _wait(left, t) + _carriage("left", lift, variant)
    right = _wait(right, t) + _carriage("right", lift, variant)
    if not lift:  # try to lift it anyway, then give up
        for plan, s in ((left, 1), (right, -1)):
            x, y, z = plan[-1].pos
            plan += [Waypoint(1.5, (x, y, z + LIFT), FINGER_CLOSED, "try to lift it off", 0.0),
                     Waypoint(1.5, (x, y, z + LIFT), FINGER_CLOSED, "hold", 0.0),
                     Waypoint(1.0, (x, y, z), FINGER_CLOSED, "down", 0.0),
                     Waypoint(1.0, (x, y, z), OPEN, "let go", 0.0),
                     Waypoint(1.0, (x, y + s * 0.01, z + 0.045), OPEN, "up", 0.0)]
    return {"left": left, "right": right}


def demo():
    # DEMO_VARIANT selects a degenerate check (see README); unset runs the full-score demo
    v = os.environ.get("DEMO_VARIANT", "")
    if v == "lift_first":  # release knob out, carriage run to the end and lifted without closing the gauge
        return _plans(gauge=False, variant="lift_first")
    if v == "zero_only":  # close the gauge and stop
        return _plans(release=False, carriage=False)
    if v == "sideways":  # everything in order, then drag the carriage off the rail forward instead of lifting
        return _plans(variant="sideways")
    if v == "on_body":  # everything in order, but set the carriage back down on the slicer, not the tray
        return _plans(variant="on_body")
    return _plans()


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS, _TEST_POINTS, _TEST_BOXES
    up = mk.mat_to_quat(np.eye(3))
    seat = (LIFT_SEAT.copy(), up)
    lifted = (LIFT_SEAT + np.array([0, 0, LIFT]), up)
    on_tray_p = (np.array([TRAY_XY[0], TRAY_XY[1], TRAY_TOP]), up)
    over_p = (np.array([TRAY_XY[0], TRAY_XY[1], TRAY_TOP + 0.030]), up)
    on_body_p = (LIFT_SEAT + np.array([0, 0.0, 0.0]), up)
    tilt = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(8)))
    tip = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(60)))
    floor = (np.array([0.10, -0.06, TOP + 0.004]), tip)

    def st(c):
        return kit.fake_states(moved={"carriage": c})

    start = kit.fake_states()
    ok = {"order_ok": True, "rail_free": True, "gauge_deg": 0.0, "release_deg": 1090.0}
    grip = [kit.point(st(on_tray_p), "carriage", (FIN_X, FIN_Y, DECK[2] + FIN[2] - 0.010 + TIP))]
    cases = {  # name: (state, tcps, MEMORY, score)
        "start": (start, [], {}, 0.0),
        "gauge zeroed only": (start, [], {"gauge_deg": 0.0}, 0.2),
        "gauge zeroed, release out, carriage at the end": (st(seat), [], {**ok, "rail_free": False}, 0.2),
        "freed at the end, not lifted": (st(seat), [], ok, 0.4),
        "lifted 25 mm over the rail": (st(lifted), [], ok, 0.4),
        "held over the tray": (st(over_p), grip, ok, 0.6),
        "on the tray, still held": (st(on_tray_p), grip, ok, 0.6),
        "on the tray, let go": (st(on_tray_p), [], ok, 1.0),
        "on the tray, tilted 8 deg": (st((on_tray_p[0] + np.array([0, 0, 0.003]), tilt)), [], ok, 0.6),
        "on the tray, gauge reopened": (st(on_tray_p), [], {**ok, "gauge_deg": 50.0}, 0.0),
        "on the tray, carriage moved before zeroing": (st(on_tray_p), [], {**ok, "order_ok": False}, 0.0),
        "set back down on the slicer": (st(on_body_p), [], ok, 0.4),
        "dragged off forward onto the bench": (st(floor), [], ok, 0.6),
        "edge touched with the gauge open": (st(on_tray_p), [], {**ok, "cut": True}, 0.0),
        "forced off the rail": (st(on_tray_p), [], {**ok, "forced": True}, 0.0),
        "tipped": (st(on_tray_p), [], {**ok, "tipped": True}, 0.0),
    }
    try:
        # a finger pad reaches into the bare upper arc while the TCP and every link origin stay outside: fails, latched
        mk.new_episode()
        _TEST_TCPS, _TEST_POINTS = [], [np.array([BLADE_X - 0.060, 0.10, TOP + 0.22])]
        pad = (np.array([BLADE_X - 0.052, 0.095, TOP + 0.215]), np.array([BLADE_X - 0.040, 0.105, TOP + 0.235]))
        _TEST_BOXES = [pad]
        assert not any(_in_edge(p) for p in _TEST_POINTS) and _box_in_edge(*pad, gauge=0.0)
        mk.MEMORY.update({**ok, "gauge_deg": 0.0})
        _mechanism_rule(st(on_tray_p))
        _TEST_BOXES = None
        _mechanism_rule(st(on_tray_p))
        assert mk.MEMORY.get("cut") and score(st(on_tray_p)) == 0.0, "finger surface in the blade zone not caught"
        # the same pad inside the plate window with the gauge closed is covered; with the gauge open it fails
        win = (np.array([BLADE_X - 0.044, -0.02, TOP + 0.06]), np.array([BLADE_X - 0.040, 0.00, TOP + 0.08]))
        assert not _box_in_edge(*win, gauge=0.0) and _box_in_edge(*win, gauge=12.0)
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        # geometry: the start carriage is clear of every fixture and of the knife; the edge zone check
        c = kit.fake_states()["carriage"]
        assert c.high[0] < BLADE_X - 0.02 and c.low[2] > TOP + RAIL_H, (c.low, c.high)
        assert _in_edge((BLADE_X - 0.01, 0.0, TOP + 0.06), gauge=12.0)
        assert not _in_edge((BLADE_X - 0.01, 0.0, TOP + 0.06), gauge=0.0)  # behind the closed plate
        assert _in_edge((BLADE_X - 0.01, 0.10, TOP + 0.22), gauge=0.0)  # upper arc: bare with the gauge closed
        assert not _in_edge((CX + FIN_X, CY0 + FIN_Y, GRIP_Z), gauge=12.0)
        assert not _in_edge(RK) and not _in_edge(GK)
        assert abs(kit.q(start, "rail")) < 1e-9 and abs(kit.q(st(seat), "rail") - RAIL_LO) < 1e-9
    finally:
        mk.new_episode()
        _TEST_TCPS = _TEST_POINTS = _TEST_BOXES = None
    return len(cases) + 2


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
        # no link of the demo enters the bare-edge zone while the gauge is open: checked in sim (cut latch)
