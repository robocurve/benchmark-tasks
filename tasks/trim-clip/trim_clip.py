"""On the station table, 30 cm out: a section of a car's body, a grey steel flange held up on four posts, with a
square hole in it on the robot's left. A black plastic splash-shield panel lies on it, loose, with its own hole a
little out of line with the body's; the panel has a raised lip along its right-hand edge. To the left, nearer the
arms, a black push-in trim clip stands head up in a small tray. Fasten the panel to the body with the clip.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism  # noqa: E402
from parts import Part, Piece  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, DARK_STEEL, BLACK, PANEL_BLACK, TRAY_GREY = (0.55, 0.57, 0.60), (0.30, 0.31, 0.33), (0.07, 0.07, 0.08), \
    (0.12, 0.12, 0.13), (0.45, 0.45, 0.47)
MATERIALS = {"default": (1.2, 1.0), "metal": (0.3, 0.2)}


def _m(v):
    return tuple(x / 1000 for x in v)


# -- the body: a steel flange on posts, with a hole and the clip's catch under it (fixed) -----------------------------
# mm, in the body's frame: origin on the table under the flange's centre.
BODY_XY = (0.30, 0.02)
SHEET = (160.0, 220.0, 2.0)  # x, y, thickness
SHEET_TOP = 40.0  # above the table
# WHY 11.4 mm: the clip's 9.6 mm collar must pass both holes, so the panel has to be within 1.8 mm of lined up;
# with 10.8 mm holes the panel 2.2 mm out stopped the collar on the edges (official sim, 2026-11-14).
HOLE = 11.4  # the square hole through the flange (and the panel), for an 8 mm push-in clip
HOLE_XY = (-20.0, 70.0)  # in the body's frame: world (0.28, 0.09)
# The catch: two sprung jaws under the hole, closing to 8.4 mm round the clip's stem. The clip's 9.6 mm collar pushes
# them apart going in, and they close over it once it is through: then the clip holds. On a real clip the barbs are on
# the clip and flex; here they are in the hole and slide.
JAW_R, JAW_LEN, JAW_GAP = 2.0, 12.0, 8.4
JAW_Z = SHEET_TOP - SHEET[2] - JAW_R - 0.3
JAW_K, JAW_PRELOAD, JAW_TRAVEL = 300.0, 0.0005, 0.003  # N/m, m, m [tuned in sim]


def _box(c, s, color, material="metal"):
    return Shape("box", _m(c), _m(s), color, material=material, contact_offset=0.001)


def _cyl(c, r, length, axis, color, material="metal"):
    return Shape("cylinder", _m(c), (r / 1000, length / 1000), color, axis=axis, material=material,
                 contact_offset=0.001)


def _plate_with_hole(cx, cy, size_x, size_y, z, t, hx, hy, hole, color, maker):
    """A plate centred at (cx, cy) with a square hole at (hx, hy), as four pieces round the hole."""
    x0, x1, y0, y1 = cx - size_x / 2, cx + size_x / 2, cy - size_y / 2, cy + size_y / 2
    h = hole / 2
    pieces = [
        ((x0 + hx - h) / 2, cy, hx - h - x0, size_y),  # before the hole in x
        ((hx + h + x1) / 2, cy, x1 - hx - h, size_y),  # after it
        (hx, (y0 + hy - h) / 2, hole, hy - h - y0),  # beside it, -y
        (hx, (hy + h + y1) / 2, hole, y1 - hy - h),  # beside it, +y
    ]
    return [maker((px, py, z), (sx, sy, t), color) for px, py, sx, sy in pieces if sx > 0 and sy > 0]


def _body_base():
    sx, sy, t = SHEET
    shapes = _plate_with_hole(0.0, 0.0, sx, sy, SHEET_TOP - t / 2, t, *HOLE_XY, HOLE, STEEL, _box)
    for px in (-sx / 2 + 8, sx / 2 - 8):
        for py in (-sy / 2 + 8, sy / 2 - 8):
            shapes.append(_cyl((px, py, (SHEET_TOP - t) / 2), 6.0, SHEET_TOP - t, "Z", DARK_STEEL))
    return tuple(shapes)


def _jaw(side):
    hx, hy = HOLE_XY
    return (_cyl((hx, hy + side * (JAW_GAP / 2 + JAW_R), JAW_Z), JAW_R, JAW_LEN, "X", DARK_STEEL),)


BODY = mechanism(
    "body", (*BODY_XY, TOP), base=Link("flange", 1.0, _body_base()),
    links=(Link("jaw_left", 0.003, _jaw(+1)), Link("jaw_right", 0.003, _jaw(-1))),
    joints=(
        # each jaw slides outward (away from the hole) against its spring, preloaded against its stop at 0
        Joint("jaw_left_slide", "prismatic", "jaw_left", _m((HOLE_XY[0], HOLE_XY[1] + JAW_GAP / 2 + JAW_R, JAW_Z)),
              "Y", 0.0, JAW_TRAVEL, stiffness=JAW_K, damping=0.5, target=-JAW_PRELOAD),
        Joint("jaw_right_slide", "prismatic", "jaw_right", _m((HOLE_XY[0], HOLE_XY[1] - JAW_GAP / 2 - JAW_R, JAW_Z)),
              "Y", -JAW_TRAVEL, 0.0, stiffness=JAW_K, damping=0.5, target=JAW_PRELOAD),
    ),
    materials=MATERIALS)
HOLE_W = (BODY_XY[0] + HOLE_XY[0] / 1000, BODY_XY[1] + HOLE_XY[1] / 1000)

# -- the splash-shield panel: loose, polypropylene, with a lip along its right-hand edge -------------------------
# WHY 20 cm long, the hole 7 cm from its middle: with 11.5 cm between the hole and the lip the two arms met as the
# left came over the hole, and the panel turned 3 degrees (official sim, 2026-11-14).
PANEL = (80.0, 200.0, 2.5)  # x, y, thickness
PANEL_HOLE = (0.0, 70.0)  # in the panel's frame, centred on the plate
LIP_H, LIP_T = 15.0, 2.5  # the lip stands up along the panel's -y edge
PANEL_KG = 0.050
PANEL_TOP = SHEET_TOP + PANEL[2]
PANEL_LINED = (HOLE_W[0] - PANEL_HOLE[0] / 1000, HOLE_W[1] - PANEL_HOLE[1] / 1000)  # its centre with the holes in line
PANEL_OFFSET = (0.010, -0.008)  # where it starts: 10 mm away from the arms and 8 mm right of lined up
PANEL_START = (PANEL_LINED[0] + PANEL_OFFSET[0], PANEL_LINED[1] + PANEL_OFFSET[1])


def _piece(c, s, color=None):
    return Piece(_m(c), _m(s), color)


def _panel_pieces():
    sx, sy, t = PANEL
    pieces = _plate_with_hole(0.0, 0.0, sx, sy, 0.0, t, *PANEL_HOLE, HOLE, None, _piece)
    pieces.append(_piece((0.0, -sy / 2 + LIP_T / 2, t / 2 + LIP_H / 2), (sx, LIP_T, LIP_H)))
    return tuple(pieces)


PANEL_PART = Part("panel", pos=(*PANEL_START, TOP + (SHEET_TOP + PANEL[2] / 2 + 0.2) / 1000),
                  size=_m((PANEL[0], PANEL[1], PANEL[2] + LIP_H)), mass=PANEL_KG, color=PANEL_BLACK,
                  pieces=_panel_pieces())

# -- the push-in clip: loose nylon, an octagonal head, stem, collar and tip ------------------------------------------
HEAD_W, HEAD_T = 20.0, 3.0
STEM_W, STEM_L = 7.6, 11.0  # below the head
COLLAR_W, COLLAR_L = 9.6, 3.0  # below the stem: the part the jaws close over
TIP_W, TIP_L = 6.0, 2.0
CLIP_LEN = STEM_L + COLLAR_L + TIP_L  # below the head's underside
CLIP_KG = 0.005  # a real one is about 2 g


def _octagon(cz, across, height):
    """An octagonal prism ``across`` its flats: four bars, each as long as the flats and as wide as one side, turned
    by 45 degree steps; their ends make the eight faces.
    WHY not a square and a turned square: their union is an eight-point star (points 1.41 times out), on whose points
    the fingers slid and which wedged into the tray (official sim, 2026-11-14)."""
    side = across * math.tan(math.pi / 8)
    return [Piece(_m((0, 0, cz)), _m((across, side, height)), None, yaw=k * math.pi / 4) for k in range(4)]


def _clip_pieces():
    # frame origin at the underside of the head, on the clip's axis
    return tuple([
        *_octagon(HEAD_T / 2, HEAD_W, HEAD_T),
        *_octagon(-STEM_L / 2, STEM_W, STEM_L),
        *_octagon(-STEM_L - COLLAR_L / 2, COLLAR_W, COLLAR_L),
        *_octagon(-STEM_L - COLLAR_L - TIP_L / 2, TIP_W, TIP_L),
    ])


# The clip waits head up in a tray hole, its head on the tray's top.
TRAY_XY, TRAY_H, TRAY_W = (0.22, 0.20), 20.0, 40.0
CLIP_START = (*TRAY_XY, TOP + (TRAY_H + 0.2) / 1000)
CLIP = Part("clip", pos=CLIP_START, size=_m((HEAD_W, HEAD_W, HEAD_T + CLIP_LEN)), mass=CLIP_KG, color=BLACK,
            pieces=_clip_pieces())


def _tray():
    tx, ty = TRAY_XY
    out = []
    for i, (px, py, sx, sy) in enumerate([
        (-(TRAY_W + HOLE) / 4, 0.0, (TRAY_W - HOLE) / 2, TRAY_W),
        ((TRAY_W + HOLE) / 4, 0.0, (TRAY_W - HOLE) / 2, TRAY_W),
        (0.0, -(TRAY_W + HOLE) / 4, HOLE, (TRAY_W - HOLE) / 2),
        (0.0, (TRAY_W + HOLE) / 4, HOLE, (TRAY_W - HOLE) / 2),
    ]):
        out.append(Box(f"tray_{i}", pos=(tx + px / 1000, ty + py / 1000, TOP + TRAY_H / 2000), size=_m((sx, sy, TRAY_H)),
                       color=TRAY_GREY))
    return out


OBJECTS = [BODY, PANEL_PART, CLIP, *_tray()]
NAMES = ("panel", "clip")

# -- scoring ----------------------------------------------------------------------------------------------------------
LINED_UP = 0.0015  # the panel's hole within 1.5 mm of the body's (the 2.8 mm of play round the stem)
FLAT_DEG = 3.0
ON_TOL = 0.001  # resting on: within 1 mm
CLIP_UPRIGHT_DEG = 5.0
CLIP_IN_HOLE = 0.0015  # the clip's axis within 1.5 mm of the hole's
SEATED = 0.001  # the head's underside within 1 mm of the panel
COMPLETE_SCORE = 0.9


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _tilt_deg(s) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(s.quat_xyzw)[2, 2])))))


# The simulator reports a loose part's position as the centre of its world bounding box. The panel's plate is
# symmetric about its own centre in x and y (the lip lies inside its edge), and an upright clip's box is centred on
# its axis, so x and y come from that centre; heights come from the box's bottom and top.


def panel_hole(panel) -> np.ndarray:
    """Where the panel's hole is (world x, y)."""
    r = _rot(panel.quat_xyzw)
    return np.asarray(panel.pos, float)[:2] + (r @ np.array([*_m(PANEL_HOLE), 0.0]))[:2]


def panel_misalign(panel) -> float:
    return float(np.linalg.norm(panel_hole(panel) - np.asarray(HOLE_W)))


def _panel_on(panel) -> bool:
    return _tilt_deg(panel) <= FLAT_DEG and abs(float(panel.low[2]) - (TOP + SHEET_TOP / 1000)) <= ON_TOL


def head_underside(clip) -> float:
    return float(clip.high[2]) - HEAD_T / 1000


def clip_state(clip):
    """(how far the clip's axis is from the hole's, m; how far its head's underside is above the panel, m)."""
    return (float(np.linalg.norm(np.asarray(clip.pos, float)[:2] - np.asarray(HOLE_W))),
            head_underside(clip) - (TOP + PANEL_TOP / 1000))


def _clip_through(clip) -> bool:
    """The clip in the hole, upright, its collar below the jaws (so they hold it)."""
    off, _ = clip_state(clip)
    collar_top = head_underside(clip) - STEM_L / 1000
    return (off <= CLIP_IN_HOLE and _tilt_deg(clip) <= CLIP_UPRIGHT_DEG
            and collar_top < TOP + (JAW_Z - JAW_R) / 1000)


def _grade(objects) -> float:
    panel, clip = objects["panel"], objects["clip"]
    if not _panel_on(panel):
        return 0.0
    lined = panel_misalign(panel) <= LINED_UP
    if lined and _clip_through(clip):
        _, above = clip_state(clip)
        return 1.0 if above <= SEATED else 0.5
    return 0.25 if lined else 0.0


def score(objects):
    """0.9 for the panel lined up and the clip pushed home through both holes, held by the catch (1.0 once success()
    has held 1 s); 0.5 if the clip is through and held but its head stands more than 1 mm proud; 0.25 for the panel
    lined up with no clip through it; 0 otherwise (the panel off the flange or tilted, or out of line)."""
    return min(_grade(objects), COMPLETE_SCORE)


def success(objects):
    """The panel flat on the flange with its hole within 1.5 mm of the body's, and the clip through both, upright,
    held by the catch, its head within 1 mm of the panel; both still. Must hold for 1 s."""
    return _grade(objects) == 1.0 and all(objects[n].resting() for n in NAMES)


# -- the scripted demo ----------------------------------------------------------------------------------------------
# The right arm pinches the panel's lip, slides the panel into line and holds it. The left arm takes the clip from its
# tray, puts it into the holes until its collar sits on the catch, lets go, closes its fingers and pushes the head home.
DEMO_AIM_SHIFT_MM = 0.0  # both arms aim this far off (+x and +y), for margin runs
LIP_GRIP_ABOVE = 0.004  # the right pads' tips this far above the panel's top
HEAD_GRIP_ABOVE = 0.0005  # the left pads' tips this far above the head's underside
PINCH_LIP = FINGER_OPEN * 0.0005 / 0.094
GRIP_HEAD = FINGER_OPEN * 0.016 / 0.094
OPEN_LIP = FINGER_OPEN * 0.030 / 0.094
OPEN_HEAD = FINGER_OPEN * 0.040 / 0.094
CLOSED = 0.0
ENGAGE = 0.008  # the head's underside this far above the panel when the collar meets the catch
PRESS = 0.0015  # the closed fingertips pushed this far below the seated head's top
SLIDE_S, HOLD_PAD_S = 2.0, 0.5
# Where the panel came to rest, slid to lined up (mm, x and y; official sim, 2026-11-14): the slide aims that much
# the other way.
PANEL_LANDING = (2.1, -0.9)
# WHY carried 4 cm higher and lowered straight down, only once the panel is in line: carried low across, the left
# arm caught the panel's far end and turned it 4 degrees out of line (official sim, 2026-11-14).
CARRY_HIGH = 0.04
# WHY the left gripper turned a quarter round: its wrist camera stands 8 cm off one side; turned the other way it reached
# towards the right arm holding the lip, and the panel turned 3 degrees as the left arm came over (official sim,
# 2026-11-14). The clip's head is the same both ways round.
LEFT_YAW = -math.pi / 2  # (a half turn is out of reach; a quarter turn this way puts the camera towards the bases)


def _shift():
    s = DEMO_AIM_SHIFT_MM / 1000
    return np.array([s, s, 0.0])


def lip_tcp(panel_xy):
    x, y = panel_xy
    lip_y = y - PANEL[1] / 2000 + LIP_T / 2000
    return tuple(np.array([x, lip_y, TOP + PANEL_TOP / 1000 + LIP_GRIP_ABOVE + PAD_TIP_PAST_TCP]) + _shift())


def head_tcp(xy, head_bottom_z):
    return tuple(np.array([xy[0], xy[1], head_bottom_z + HEAD_GRIP_ABOVE + PAD_TIP_PAST_TCP]) + _shift())


def _above(p, dz):
    return (p[0], p[1], p[2] + dz)


def demo():
    seated_bottom = TOP + PANEL_TOP / 1000
    aim = (PANEL_LINED[0] - PANEL_LANDING[0] / 1000, PANEL_LINED[1] - PANEL_LANDING[1] / 1000)
    lip0, lip1 = lip_tcp(PANEL_START), lip_tcp(aim)
    right = [
        Waypoint(2.0, _above(lip0, 0.04), OPEN_LIP, "over the panel's lip"),
        Waypoint(1.5, lip0, OPEN_LIP, "down beside the lip"),
        Waypoint(0.8, lip0, PINCH_LIP, "pinch the lip"),
        Waypoint(SLIDE_S, lip1, PINCH_LIP, "slide the panel into line"),
        Waypoint(HOLD_PAD_S, lip1, PINCH_LIP, "hold it"),
    ]
    ready = sum(w.duration for w in right)
    head0 = head_tcp(TRAY_XY, CLIP_START[2])
    over = head_tcp(HOLE_W, seated_bottom + ENGAGE + 0.012)
    engage = head_tcp(HOLE_W, seated_bottom + ENGAGE)
    head_top = seated_bottom + HEAD_T / 1000
    press = tuple(np.array([*HOLE_W, head_top - PRESS + PAD_TIP_PAST_TCP]) + _shift())
    left = [
        Waypoint(2.0, _above(head0, 0.04), OPEN_HEAD, "over the clip", LEFT_YAW),
        Waypoint(1.5, head0, OPEN_HEAD, "down round its head", LEFT_YAW),
        Waypoint(0.8, head0, GRIP_HEAD, "grip the head", LEFT_YAW),
        Waypoint(1.0, _above(head0, 0.03), GRIP_HEAD, "lift it out of the tray", LEFT_YAW),
    ]
    high = _above(over, CARRY_HIGH)
    wait = max(ready - sum(w.duration for w in left) - 1.5, 0.3)
    left += [
        Waypoint(wait, _above(head0, 0.03), GRIP_HEAD, "wait for the panel", LEFT_YAW),
        Waypoint(1.5, high, GRIP_HEAD, "high over the lined-up holes", LEFT_YAW),
        Waypoint(1.2, over, GRIP_HEAD, "straight down over them", LEFT_YAW),
        Waypoint(1.5, engage, GRIP_HEAD, "into the holes, collar on the catch", LEFT_YAW),
        Waypoint(0.6, engage, OPEN_HEAD, "let go", LEFT_YAW),
        Waypoint(0.8, _above(engage, 0.012), OPEN_HEAD, "up", LEFT_YAW),
        Waypoint(0.6, _above(engage, 0.012), CLOSED, "close the fingers", LEFT_YAW),
        Waypoint(1.0, _above(press, 0.004), CLOSED, "over the head", LEFT_YAW),
        Waypoint(1.5, press, CLOSED, "push it home", LEFT_YAW),
        Waypoint(0.5, press, CLOSED, "hold", LEFT_YAW),
        Waypoint(1.0, _above(press, 0.04), CLOSED, "up", LEFT_YAW),
        Waypoint(1.5, (0.20, 0.30, TOP + 0.15), FINGER_OPEN, "out of the way", LEFT_YAW),
    ]
    done = sum(w.duration for w in left)
    right += [
        Waypoint(max(done - ready - 1.0, 0.5), lip1, PINCH_LIP, "keep holding"),
        Waypoint(0.6, lip1, OPEN_LIP, "let go of the lip"),
        Waypoint(1.0, _above(lip1, 0.04), OPEN_LIP, "up"),
        Waypoint(1.5, (0.20, -0.30, TOP + 0.15), FINGER_OPEN, "out of the way"),
    ]
    return {"left": left, "right": right}
