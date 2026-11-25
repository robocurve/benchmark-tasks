"""Clip a book light onto an open hardback on a book stand, with no pages caught, then bend its neck to aim the light at the page.

A 6 x 9 in trade hardback (boards 152 x 229 x 2.5 mm, each half's page block 11 mm) stands open at its middle on a bamboo book stand, leaning back 25 degrees, its head edge
standing free above the stand's back. The right half's leaves are one rigid body on a hinge at the gutter, lying back under gravity on the free endpaper, one 0.3 mm leaf on its own hinge, which lies flat on the board: no gap between board and endpaper. A Gritin 9 LED clip-on book light, switched on, lies loose on its side on the table in front of the book, to the right; its lamp is a warm-white cone light at the head. Its clip is a sprung jaw on a hinge
(torsion spring, about 0.31 N m at the jaws shut on the board, 0.48 N m with them 20 degrees open); its gooseneck is a collar on a friction joint at the clip and a neck on a second friction
joint (breakaway 0.40 and 0.10 N m, about ten and four times the weight moments they carry), with the head at its end. The light is hard plastic (friction 0.5), with a rubber pad (0.8) on the back jaw.

The right hand pinches the clip's tails, which opens the jaws, lifts it off the table and turns it upright, eases its squeeze until the front jaw stands about 2 degrees open, squares the clip over the board's face, slides the jaws down over the right board's head edge at the marked spot (the front jaw's thin tapered tip between the board
and the endpaper, lifting the endpaper and the leaves off the board), and lets go so the jaws shut on the board, and moves clear. The left hand then takes the head from the left, level, bends the neck down until the light points at the page, lets go and backs off.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,noaim,held,beside,slack,endpaper,overpages,reversed
Final:  python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import dataclasses
import math
import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Clip the book light onto the top edge of the open book's right cover at the mark, no pages in the clip, then bend its neck to light the right page."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself
SCORE_OVER = "end"
TOP = d.TABLE_TOP
HANG = (0.0, -68.0)  # the collar and neck angles (deg) that hang the head in front of the page, the LED face on the page centre
TILT = 25.0  # the book stand's lean back from vertical, degrees
HOLD_S = 2.0
TAILS_W = 0.0225  # the tails' span across the pinch with the jaws shut (front jaw at 0), m; each 0.026 m of squeeze per radian opens the front jaw
OPEN_DEG = 2.0  # how far the hand lets the front jaw stand open as it goes down onto the board
SEAT_D = 0.019  # how far the board's edge goes into the jaws in the demo
# Range modes: each sets one estimate to an end of the range the README accepts in a real trial, then runs as full. Run each alone (--mode mu_lo), since it changes the scene built at the start
RANGES = {"tip05": {"TIP": 0.0005}, "tip10": {"TIP": 0.0010}, "spring_lo": {"JAW_K": 0.45}, "spring_hi": {"JAW_K": 0.55},
          "mu_lo": {"BOOK_MU": (0.3, 0.2)}, "mu_hi": {"BOOK_MU": (0.7, 0.6)},
          "neck_lo": {"NECK_CAP": 0.07}, "neck_hi": {"NECK_CAP": 0.13}, "collar_lo": {"COLLAR_CAP": 0.35}, "collar_hi": {"COLLAR_CAP": 0.50}}
_RANGE = RANGES.get(os.environ.get("TASK_DEMO", "full"), {})
MU = (0.5, 0.4)  # the light's hard plastic, and the book's cloth and paper (static, dynamic): estimates. PhysX averages the two sides of a contact, so the range modes change the book's side alone: clip on book 0.4 to 0.6, the fingers' grip on the clip unchanged
JAW_K = _RANGE.get("JAW_K", 0.5)  # the clip's spring, N m per radian (preloaded 35 degrees past shut): an estimate
NECK_CAP = _RANGE.get("NECK_CAP", 0.10)  # the neck joint's breakaway torque, N m: an estimate
COLLAR_CAP = _RANGE.get("COLLAR_CAP", 0.40)  # the collar joint's breakaway torque, N m: an estimate
LEAF_GAP = 0.0  # the gap between the board and the first leaf at the mark, at rest: none, the endpaper lies on the board

LOOKS = {"wood": m.Look(m.rgb(0.78, 0.58, 0.34), roughness=0.6), "cloth": m.Look(m.rgb(0.10, 0.20, 0.42), roughness=0.8), "paper": m.Look(m.rgb(0.96, 0.94, 0.88), roughness=0.9),
         "ink": m.Look(m.rgb(0.30, 0.30, 0.32), roughness=0.9), "black": m.Look(m.rgb(0.06, 0.06, 0.07), roughness=0.5), "rubber": m.Look(m.rgb(0.15, 0.15, 0.15), roughness=0.9),
         "steel": m.Look(m.rgb(0.55, 0.57, 0.60), metallic=0.8, roughness=0.35), "white": m.Look(m.rgb(0.93, 0.93, 0.93), roughness=0.4), "led": m.Look(m.rgb(1.0, 0.92, 0.75), roughness=0.2, emissive=(2.0, 1.75, 1.35)),
         "tray": m.Look(m.rgb(0.25, 0.27, 0.30), roughness=0.6), "mark": m.Look(m.rgb(0.85, 0.12, 0.10), roughness=0.6)}

# -- the book, in its own frame: x into the stand (normal to the boards), y across (gutter at 0, right half at -y), z up the spine ------------------------------
BW, BH, BT = 0.152, 0.229, 0.0025  # board width, height, thickness
PT = 0.011  # each half's page block
SQ = 0.003  # the squares: boards stand 3 mm past the pages
MARK_Y = -0.076  # the clip mark: the middle of the right board's head edge
LEAF_OFF = math.degrees(math.atan(LEAF_GAP / -MARK_Y))  # 0: the endpaper lies on the board, held at its stop, and the leaves lie on the endpaper under their weight
EP = 0.0003  # the free endpaper, the first leaf on the board: one thin leaf on its own hinge
PAD_MU = (0.8, 0.7)  # the back jaw's rubber pad, on the cover (static, dynamic): an estimate; with the cover's 0.5 the contact is 0.65
BOOK_MU = _RANGE.get("BOOK_MU", MU)  # cloth-covered board and paper (static, dynamic)
BOOK_AT = (0.40, 0.10, TOP + 0.014)
H = math.radians(TILT) / 2
BOOK_Q = (0.0, math.sin(H), 0.0, math.cos(H))


def _page_lines(sign: float) -> list:
    out = []
    for k in range(14):
        z = 0.03 + k * 0.012
        out.append((f"line{k}", m.box((0.0004, 0.11, 0.0015), (-PT - 0.0002, sign * 0.076, z)), "ink"))
    return out


_left_lines = _page_lines(1.0)
EP_GAP = 0.0  # the page block lies on the endpaper
EP_OFF = 0.00005  # the endpaper lies on the board: 0.05 mm of solver clearance between them, as near flat as the contact allows
_leaf_block = m.box((PT - EP - EP_GAP - EP_OFF, BW - 2 * SQ, BH - 2 * SQ), (-EP_OFF - EP - EP_GAP - (PT - EP - EP_GAP - EP_OFF) / 2, -(BW - SQ) / 2 - 0.002, BH / 2))  # from 2 mm off the gutter: the joint's groove
_endpaper = m.box((EP, BW - 2 * SQ, BH - 2 * SQ), (-EP_OFF - EP / 2, -(BW - SQ) / 2 - 0.002, BH / 2))
Y_LO = -(BW - SQ) / 2 - 0.002 - (BW - 2 * SQ) / 2  # the leaves' fore-edge side; they span BW - 2 * SQ from here toward the gutter


def _round_top(x_board: float, x_far: float, r: float, z_lo: float = SQ, y0: float = Y_LO, y1: float = Y_LO + BW - 2 * SQ) -> "m.Mesh":
    """A leaf's collider: x from x_far to x_board (the board side), z from z_lo to the top, y from y0 to y1, its top edge on the board side rounded to radius r,
    as a paper edge gives under a thin tip (a square box corner would stop the tip dead in the solver)."""
    top = BH - SQ
    arc = [(x_board - r + r * math.cos(a), top - r + r * math.sin(a)) for a in np.linspace(0.0, math.pi / 2, 7)]
    return m.prism([(x_far, z_lo), (x_board, z_lo)] + arc + [(x_far, top)], -y1, -y0).turned(90.0, "x")


STRIP = 0.004  # the endpaper's rounded top edge: a strip this tall, cut in pieces short enough for the GPU's convex hulls
_ep_strip = [m.col_hull(f"endpaper_t{k}", _round_top(-EP_OFF, -EP_OFF - EP, EP, BH - SQ - STRIP, Y_LO + k * (BW - 2 * SQ) / 16, Y_LO + (k + 1) * (BW - 2 * SQ) / 16))
             for k in range(16)]


ENDPAPER = m.hinge("endpaper", "Z", (0.0, 0.0, 0.0), -30.0, 0.0, [("endpaper", _endpaper.turned(-LEAF_OFF, "z"), "paper")],
                   [m.col_box("endpaper_c", (EP, BW - 2 * SQ, BH - 2 * SQ - STRIP), (-EP_OFF - EP / 2, Y_LO + (BW - 2 * SQ) / 2, (BH - STRIP) / 2))] + _ep_strip, mass=0.005, spring=0.002, damping=0.002, friction=0.0, gravity=True)
_right_lines = [(n, mesh, k) for n, mesh, k in _page_lines(-1.0)]
LEAVES = m.hinge("leaves", "Z", (0.0, 0.0, 0.0), -30.0, 0.1,  # a stop 0.1 degrees past the endpaper's plane (0.26 mm at the fore-edge, under the endpaper's 0.3 mm): the leaves' weight rests on the endpaper, and they cannot pass through it
                 [("block", _leaf_block.turned(-LEAF_OFF, "z"), "paper")] + [(n, mesh.turned(-LEAF_OFF, "z"), k) for n, mesh, k in _right_lines],
                 [m.col_hull("leaves_c", _round_top(-EP_OFF - EP - EP_GAP, -PT, 0.001).turned(-LEAF_OFF, "z"))], mass=0.20, spring=0.05, damping=0.05, friction=0.0, gravity=True)
BOOK = m.mechanism(
    "book", BOOK_AT, [ENDPAPER, LEAVES], LOOKS, start_quat=BOOK_Q, frame_inertia=(0.003, 0.002, 0.002),  # fixed in the stand
    frame_parts=[("rboard", m.box((BT, BW, BH), (BT / 2, -BW / 2, BH / 2)), "cloth"), ("lboard", m.box((BT, BW, BH), (BT / 2, BW / 2, BH / 2)), "cloth"),
                 ("spine", m.box((0.0004, 0.024, BH), (BT + 0.0002, 0.0, BH / 2)), "black"),
                 ("lpages", m.box((PT, BW - 2 * SQ, BH - 2 * SQ), (-PT / 2, (BW) / 2, BH / 2)), "paper"),
                 ("mark", m.box((0.0006, 0.006, 0.004), (-0.0003, MARK_Y, BH - 0.002)), "mark")] + _left_lines,
    frame_colliders=[m.col_box("boards_c", (BT, 2 * BW, BH), (BT / 2, 0.0, BH / 2)), m.col_box("lpages_c", (PT, BW - 2 * SQ, BH - 2 * SQ), (-PT / 2, BW / 2, BH / 2))],
    friction=BOOK_MU)
_book_usd = BOOK.build
# the book never sleeps, so its leaves answer the lightest touch of the clip
BOOK = dataclasses.replace(BOOK, build=lambda: _book_usd().replace("int physxArticulation:solverVelocityIterationCount = 1",
                                                                  "int physxArticulation:solverVelocityIterationCount = 1\n    float physxArticulation:sleepThreshold = 0\n    float physxArticulation:stabilizationThreshold = 0"))

# -- the stand: a back plate shorter than the book, a ledge, a base (world-aligned fixed part at the book's origin, meshes tilted with the book) ----------------
# a wishacc bamboo book stand, 11.0 x 8.1 in size: a 279 x 206 mm bamboo plate, the full-width ledge (about 2 in deep) hinged at its foot, a wire prop to a slotted rack
_back = m.box((0.008, 0.279, 0.206), (BT + 0.0045, 0.0, 0.206 / 2 - 0.0125)).turned(TILT, "y")
_ledge = m.box((0.05, 0.279, 0.012), (-0.020, 0.0, -0.0065)).turned(TILT, "y")
_strut = m.box((0.006, 0.04, 0.12), (0.075, 0.0, 0.035 - 0.014))
_base = m.box((0.14, 0.152, 0.010), (0.01, 0.0, -0.014 + 0.005))  # flat on the table under the ledge: nothing in the act reaches it, so it carries no collider
STAND = m.fixed("stand", BOOK_AT, [("back", _back, "wood"), ("ledge", _ledge, "wood"), ("strut", _strut, "wood"), ("base", _base, "wood")],
                [m.col_hull("back_c", _back), m.col_hull("ledge_c", _ledge), m.col_hull("strut_c", _strut)], LOOKS)

# -- the light, in the clip's frame: x across the jaws (board side at x in [-WALL, 0]), z up from the jaw tips -------------------------------------------------
WALL = 0.0025  # the gap between the closed jaws: the board
JAW_D = 0.022  # jaw depth: the bridge's underside, where a board's edge stops
HINGE = (-WALL, 0.0, 0.024)
TAIL = (0.026, 0.055)
E = math.radians(-5.0)  # the folded neck's rise from the clip's -x, in the clip's frame: with the clip leaning back 25 degrees on the book, it rises 20 degrees over the page
DIR = np.array([-math.cos(E), 0.0, math.sin(E)])
NRM = np.array([-math.sin(E), 0.0, -math.cos(E)])  # the LED face's normal when folded
P1 = np.array([0.005, 0.0, 0.080])  # the collar leaves the clip above the tails, clear of the fingers that pinch them
COLLAR_L, NECK_L, HEAD_L = 0.075, 0.12, 0.0827  # the gooseneck is the Gritin's 19.5 cm; head 82.7 x 40.3 mm
P2 = P1 + COLLAR_L * DIR
P3 = P2 + NECK_L * DIR
HEAD_C = P3 + 0.5 * HEAD_L * DIR
LED_C = HEAD_C + 0.006 * NRM


BEAM = 3.0e7  # the lamp's intensity, in the renderer's units, set so the lit pool on the page reads in the cameras (3e6 barely shows); on from the start
BEAM_DEG = 45.0  # half-angle of the cone; the Gritin's nine LEDs throw a wide, soft spot


def _beam() -> str:
    """The lamp: a small warm-white sphere light just under the LED face, its cone along the face's normal. It rides on the neck link."""
    c = LED_C + 0.004 * NRM
    return f"""
        def SphereLight "beam" (
            prepend apiSchemas = ["ShapingAPI", "ShadowAPI"]
        )
        {{
            float inputs:intensity = {BEAM}
            float inputs:radius = 0.008
            color3f inputs:color = (1, 0.84, 0.66)
            float inputs:shaping:cone:angle = {BEAM_DEG}
            float inputs:shaping:cone:softness = 0.4
            bool inputs:shadow:enable = 1
            double3 xformOp:translate = ({c[0]:.5f}, {c[1]:.5f}, {c[2]:.5f})
            float3 xformOp:rotateXYZ = (0, {math.degrees(E):.3f}, 0)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]
        }}"""


def _rod(a, b, r):
    return m.tube([a, 0.5 * (a + b), b], r, sides=14)


_head = m.box((HEAD_L, 0.0403, 0.012)).turned(-(180.0 - math.degrees(E)), "y").moved(HEAD_C)
_ledface = m.box((0.07, 0.03, 0.001)).turned(-(180.0 - math.degrees(E)), "y").moved(HEAD_C + 0.0062 * NRM)
_back_jaw = m.prism([(0.002, 0.0), (0.004, 0.0), (0.004, JAW_D), (0.0, JAW_D), (0.0, 0.003)], -0.014, 0.014).turned(90.0, "x")  # its tip rounded off on the inside, as on real clips
_bridge = m.box((WALL + 0.004, 0.028, 0.004), ((0.004 - WALL) / 2, 0.0, JAW_D + 0.002))
_back_tail = m.box((0.006, 0.028, TAIL[1] - JAW_D), (0.007, 0.0, (JAW_D + TAIL[1]) / 2))
_post = m.box((0.006, 0.028, P1[2] - TAIL[1]), (0.007, 0.0, (TAIL[1] + P1[2]) / 2))  # the housing the gooseneck leaves from, above the tails
# the front jaw's tip is thin and tapered, as on real clips, so it goes between the board and the leaves that lie on it
TIP = _RANGE.get("TIP", 0.0006)  # the front jaw's tip thickness; its end bevelled 45 degrees on the outside, its inside edge square, so it runs down the board's face
_front_jaw = m.prism([(-WALL, 0.024), (-WALL - 0.004, 0.024), (-WALL - 0.004, 0.016), (-WALL - TIP, TIP), (-WALL, 0.0)], -0.014, 0.014).turned(90.0, "x")
_front_tail = m.box((0.006, 0.028, TAIL[1] - TAIL[0]), (-WALL - 0.007, 0.0, (TAIL[0] + TAIL[1]) / 2))
_front_knee = m.box((0.010, 0.028, 0.004), (-WALL - 0.005, 0.0, 0.025))
_collar = _rod(P1, P2, 0.0065)
_neck = _rod(P2, P3, 0.004)
JAW = m.hinge("jaw", "Y", HINGE, -8.0, 50.0, [("jaw", _front_jaw, "black"), ("knee", _front_knee, "black"), ("tail", _front_tail, "black"),
                                               ("pad", m.box((0.0008, 0.024, 0.018), (-WALL - 0.0004, 0.0, 0.011)), "rubber")],
              [m.col_hull("jaw_c", _front_jaw), m.col_hull("knee_c", _front_knee), m.col_hull("tail_c", _front_tail)], mass=0.008, spring=JAW_K, damping=0.02, friction=0.0)
COLLAR = m.hinge("collar", "Y", tuple(P1), -120.0, 120.0, [("collar", _collar, "black")], [m.col_hull("collar_c", _collar)], mass=0.008, damping=0.05, friction=0.4, gravity=True)
NECK = m.hinge("neck", "Y", tuple(P2), -150.0, 150.0, [("neck", _neck, "steel"), ("head", _head, "white"), ("ledface", _ledface, "led")],
               [m.col_hull("neck_c", _neck), m.col_hull("head_c", _head), _beam()], mass=0.016, damping=0.05, friction=0.3, gravity=True, parent="collar")

LIGHT_AT = (0.25, -0.02, TOP + 0.0155)  # lying on its side on the table, to the right of the stand: the clip's y axis down, its neck pointing to -x
LIGHT_Q = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))  # xyzw: -90 degrees about x
LIGHT = m.mechanism("light", LIGHT_AT, [JAW, COLLAR, NECK], LOOKS, free_mass=0.031, start_quat=LIGHT_Q, frame_inertia=(2e-5, 2e-5, 2e-5),
                    frame_parts=[("back_jaw", _back_jaw, "black"), ("bridge", _bridge, "black"), ("back_tail", _back_tail, "black"), ("post", _post, "black"),
                                 ("pad", m.box((0.0008, 0.024, 0.018), (0.0004, 0.0, 0.011)), "rubber")],
                    frame_colliders=[m.col_hull("back_jaw_c", _back_jaw), m.col_hull("bridge_c", _bridge), m.col_hull("back_tail_c", _back_tail), m.col_hull("post_c", _post)], friction=(1.2, 1.0))

# the light is hard plastic (ABS on paper and book cloth, about 0.5), but for the back jaw's rubber pad on the cover; the head alone keeps the mechanism's grippy material, standing in for the fingers' rubber pads
_light_usd = LIGHT.build


def _light_build() -> str:
    t = _light_usd().replace('def Scope "Looks"', 'def Material "PlasticMaterial" (\n        prepend apiSchemas = ["PhysicsMaterialAPI"]\n    )\n    {\n        float physics:staticFriction = ' + str(MU[0]) + '\n        float physics:dynamicFriction = ' + str(MU[1]) + '\n        float physics:restitution = 0\n    }\n\n    def Material "PadMaterial" (\n        prepend apiSchemas = ["PhysicsMaterialAPI"]\n    )\n    {\n        float physics:staticFriction = ' + str(PAD_MU[0]) + '\n        float physics:dynamicFriction = ' + str(PAD_MU[1]) + '\n        float physics:restitution = 0\n    }\n\n    def Scope "Looks"', 1)
    i = t.index('"back_jaw_c"')
    j = t.index("PhysicsMaterial>", i)
    t = t[:j] + "PadMaterial>" + t[j + len("PhysicsMaterial>"):]
    for name in ("bridge_c", "back_tail_c", "post_c", "jaw_c", "knee_c", "tail_c", "collar_c", "neck_c"):
        i = t.index(f'"{name}"')
        j = t.index("PhysicsMaterial>", i)
        t = t[:j] + "PlasticMaterial>" + t[j + len("PhysicsMaterial>"):]
    return t


LIGHT = dataclasses.replace(LIGHT, build=_light_build)

OBJECTS = [STAND, BOOK, LIGHT]


# -- geometry the scorer and the demo share ----------------------------------------------------------------------------------------------------------------------

def _ry(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def light_points(light) -> dict:
    """World points of the light from its frame pose and joints: the neck joints, the LED's centre and normal."""
    q1, q2 = light.joint("collar"), light.joint("neck")
    R = light.rotation()
    R1, R2 = _ry(q1), _ry(q2)

    def place(p):
        return light.to_world(R1 @ (R2 @ (np.asarray(p) - P2) + P2 - P1) + P1)[0]

    return {"p1": light.to_world(P1)[0], "p2": light.to_world(R1 @ (P2 - P1) + P1)[0], "led": place(LED_C), "head": place(HEAD_C),
            "normal": R @ R1 @ R2 @ NRM}


def page_centre(book) -> np.ndarray:
    q = book.joint("leaves")
    p = _rz(q) @ (_rz(math.radians(-LEAF_OFF)) @ np.array([-PT, -(BW - SQ) / 2, BH / 2]))
    return book.to_world(p)[0]


def aim_error(light, book) -> tuple[float, float]:
    """Degrees between the LED's normal and the line to the right page's centre, and that line's length."""
    lp = light_points(light)
    v = page_centre(book) - lp["led"]
    dist = float(np.linalg.norm(v))
    return float(np.degrees(np.arccos(np.clip(lp["normal"] @ v / max(dist, 1e-9), -1.0, 1.0)))), dist


def clip_on_board(light, book) -> dict:
    """Where the clip sits on the right board: the board's edge depth in the jaws, the clip's offset from the mark, its tilt from the board, and whether leaves are inside the jaws."""
    Rb, Rl = book.rotation(), light.rotation()
    o = book.to_local(light.pos)[0]  # the clip's jaw tips in the book's frame
    depth = BH - o[2]  # how far the board's edge reaches up into the jaws, from the tips
    tilt = float(np.degrees(np.arccos(np.clip((Rb[:, 2] @ Rl[:, 2]), -1.0, 1.0))))
    facing = float(Rb[:, 0] @ Rl[:, 0])  # 1 when the back jaw is on the board's outer face, -1 for a clip put on backwards
    across = o[0] - (BT - 0.0)  # the clip's back-pad face against the board's back face: about 0 when seated
    jaw = light.deg("jaw")
    # the leaves' back face at the clip, at half the jaw depth, in the clip's frame: inside the jaws if behind the front pad's face
    # each leaf on its own: the endpaper, and the page block behind it
    # read at the jaws' middle and 12 mm to each side of it, across their 28 mm width
    front_pad = -WALL - (HINGE[2] - 0.011) * math.sin(math.radians(jaw))
    lxs = []
    for dy in (-0.012, 0.0, 0.012):
        for leaf, x0 in (("endpaper", -EP_OFF), ("leaves", -EP_OFF - EP - EP_GAP)):
            back = book.to_world(_rz(book.joint(leaf)) @ (_rz(math.radians(-LEAF_OFF)) @ np.array([x0, o[1] + dy, BH - 0.011])))[0]
            lxs.append(float(light.to_local(back)[0][0]))
    lx = max(lxs)
    caught = lx > front_pad - 0.0005
    return {"depth": float(depth), "off": float(math.hypot(o[1] - MARK_Y, 0.0)), "tilt": tilt, "across": float(across), "jaw": jaw, "facing": facing, "caught": bool(caught), "leaf_x": lx}


def gripper_points(env) -> np.ndarray:
    """Points over the whole of both grippers (fingers, palm, housing, and the wrist link's end), station frame: a grid on each, at the current opening."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        pts = []
        for sx in (-1.0, 1.0):  # each finger: a 5 x 3 x 2 grid over its plate
            for dy in np.linspace(-0.02, 0.02, 5):
                for dz in np.linspace(0.095, 0.152, 4):
                    for t in (0.0, 0.009):
                        pts.append((sx * (opening + 0.0045 + t), dy, dz))
        for x in np.linspace(-0.05, 0.05, 6):  # the palm and housing: a box about 10 x 6 x 10 cm behind the fingers
            for y in np.linspace(-0.03, 0.03, 4):
                for z in np.linspace(-0.01, 0.095, 6):
                    pts.append((x, y, z))
        out.append(pose[:3] - origin + np.asarray(pts) @ R.T)
    return np.vstack(out)


def _light_box_gap(light, pts) -> float:
    """The least distance from any point to the light's bounding boxes: the clip, and each neck part from its joints."""
    lp = light_points(light)
    loc = light.to_local(pts)
    lo, hi = np.array([-0.017, -0.016, -0.002]), np.array([0.012, 0.016, P1[2] + 0.003])
    g_clip = np.linalg.norm(np.maximum(0.0, np.maximum(lo - loc, loc - hi)), axis=1).min()

    def seg(a, b, r):
        ab = b - a
        t = np.clip(((pts - a) @ ab) / max(ab @ ab, 1e-12), 0.0, 1.0)
        return float((np.linalg.norm(pts - (a + t[:, None] * ab), axis=1) - r).min())

    tip = lp["head"] + (lp["head"] - lp["p2"]) / max(np.linalg.norm(lp["head"] - lp["p2"]), 1e-9) * (HEAD_L / 2)
    return float(min(g_clip, seg(lp["p1"], lp["p2"], 0.0065), seg(lp["p2"], tip, 0.021)))


def _book_gap(book, pts) -> float:
    loc = book.to_local(pts)
    lo, hi = np.array([-PT, -BW, 0.0]), np.array([BT, BW, BH])  # the closed book: both boards and page blocks
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(lo - loc, loc - hi)), axis=1).min())


class Trial:
    """What the scorer holds across a trial. Every tier is read from the state now; only the 2 s hold counts steps."""

    def reset(self, env=None) -> None:
        self.n = 0
        self.set_pt = {}
        self.held = 0
        self.score = 0.0
        self.info = {}
        if env is not None:
            self.step(env)

    def step(self, env) -> None:
        if self.n == 0:
            m.set_target(env, "light", "jaw", math.radians(-35.0))  # the clip's torsion spring is preloaded past closed: about 0.31 N m on the board
            self.set_pt = {}
        light_now = m.state(env, "light")
        for link, cap in (("collar", COLLAR_CAP), ("neck", NECK_CAP)):  # gooseneck friction: a stiff drive capped at the breakaway torque, its set point following the joint once slipped 2 degrees
            q = light_now.joint(link)
            if link not in self.set_pt or abs(q - self.set_pt[link]) > math.radians(2.0):
                m.lock(env, "light", link, stiffness=200.0, effort=cap)
                self.set_pt[link] = q
        book, light = m.state(env, "book"), m.state(env, "light")
        c = clip_on_board(light, book)
        clipped = bool(0.010 <= c["depth"] <= JAW_D + 0.002 and c["off"] <= 0.03 and c["tilt"] <= 15.0 and c["facing"] > 0.9 and abs(c["across"]) <= 0.003 and -3.0 <= c["jaw"] <= 3.0 and not c["caught"])
        ang, dist = aim_error(light, book)
        aimed = bool(clipped and ang <= 15.0 and dist >= 0.06)
        pts = gripper_points(env)
        clear = bool(_light_box_gap(light, pts) >= 0.05 and _book_gap(book, pts) >= 0.05)
        full = aimed and clear
        self.held = self.held + 1 if full else 0
        if full and self.held * d.DT >= HOLD_S:
            self.score = 1.0
        elif aimed:
            self.score = 0.67
        elif clipped:
            self.score = 0.33
        else:
            self.score = 0.0
        self.info = dict(c, aim=ang, dist=dist, clipped=clipped, aimed=aimed, clear=clear)
        self.n += 1
        if self.n % 20 == 0 and os.environ.get("BOOK_TRACE", "1") == "1":
            print(f"[book] step {self.n}: score {self.score} jaw {c['jaw']:.1f} depth {c['depth'] * 1000:.1f}mm off {c['off'] * 1000:.0f}mm tilt {c['tilt']:.1f} across {c['across'] * 1000:.1f}mm "
                  f"caught {c['caught']} leaf_x {c['leaf_x'] * 1000:.1f} aim {ang:.1f} dist {dist:.3f} collar {light.deg('collar'):.1f} neck {light.deg('neck'):.1f} "
                  f"clear {clear} endpaper {book.deg('endpaper'):.1f} leaves {book.deg('leaves'):.1f}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


# -- the demo ------------------------------------------------------------------------------------------------------------------------------------------------------

def _grip(rot_y_deg: float, base: d.Grip) -> d.Grip:
    return d._as_grip(_ry(math.radians(rot_y_deg)) @ base.rot)


def demo_script(ctx):
    """Modes: full; none, the arms wait; noaim, the clip goes on and the neck is not bent; held, the left keeps hold of the head; beside, the clip goes on 45 mm from the mark; slack, the clip goes on only 6 mm deep; endpaper, the front jaw goes between the endpaper and the page block, so the endpaper is in the jaws; overpages, the front jaw goes in front of the leaves, so the leaves are in the jaws; reversed, the light is turned round so its back jaw goes between board and leaves."""
    if ctx.mode in RANGES:
        ctx.mode = "full"
    if ctx.mode not in ("full", "none", "noaim", "held", "beside", "slack", "endpaper", "overpages", "reversed"):
        raise ValueError(f"unknown mode {ctx.mode!r}")
    l, r = ctx.left, ctx.right
    if ctx.mode == "none":
        yield from l.wait(3.0)
        return
    light = lambda: ctx.state("light")  # noqa: E731
    book = lambda: ctx.state("book")  # noqa: E731
    tails = lambda: light().to_world(np.array([-0.00125, -0.004, 0.050]))[0]  # high on the tails, 4 mm above the clip's middle as it lies: the fingers stay above the leaves when seated, and off the table  # noqa: E731
    pinch = d.level(90.0)
    if ctx.mode == "reversed":
        pinch = d._as_grip(np.diag([-1.0, -1.0, 1.0]) @ pinch.rot)  # the light is turned half a turn about the vertical
    # 1. the right hand comes down on the tails of the light lying on the table, squeezes them (the jaws open), lifts it and turns it upright, jaws down
    yield from r.move(tails() + [0.0, 0.0, 0.07], grip=d.down(90.0), label="over the tails")
    yield from r.move(tails(), t=1.2, label="down on the tails")
    yield from r.servo(tails(), timeout=1.0, label="down on the tails")
    yield from r.close(0.010, force=30.0, t=0.8, label="squeeze")
    print(f"[book] jaw open {ctx.deg('light', 'jaw'):.1f} deg", flush=True)
    yield from r.move(r.at() + [0.0, 0.0, 0.12], t=1.2, label="off the table")
    yield from r.move(r.at(), grip=pinch, t=1.5, label="upright")
    # 2. over the mark, the clip leaning with the board, jaws above the head edge; then down along the board
    B = book()
    zb, xb = B.rotation()[:, 2], B.rotation()[:, 0]
    lean = _grip(TILT, pinch)
    off = lambda: r.tcp() - light().pos  # noqa: E731
    my = MARK_Y - (0.045 if ctx.mode == "beside" else 0.0)
    if ctx.mode in ("overpages", "reversed"):
        seat = B.to_world(np.array([BT + 0.0015, my, BH - JAW_D]))[0]  # jaws wide open, the back jaw 1.5 mm clear of the board's back face
        if ctx.mode == "overpages":
            seat = seat - (PT + 0.004) * xb  # the front jaw goes down in front of the leaves, which then lie inside the jaws
    else:
        # the hand eases the squeeze until the front jaw stands only about a degree open, so its thin tip slides down the board's inner face, and its bevel lifts the leaves off the board
        opening = 4.0 if ctx.mode == "endpaper" else OPEN_DEG
        w = TAILS_W - 0.026 * math.sin(math.radians(opening))
        for _ in range(4):
            yield from r.close(w, force=13.0, t=0.6, label="ease the squeeze")
            yield from r.wait(0.3)
            err = math.sin(math.radians(ctx.deg("light", "jaw"))) - math.sin(math.radians(opening))
            if abs(math.degrees(err)) < 0.5:
                break
            w += 0.026 * err
        j = math.radians(ctx.deg("light", "jaw"))
        # where the front jaw's inner face goes down: 0.6 mm into the board's inner face, so the tip stays pressed on the board's face under the endpaper, and the spring keeps it running down
        # the board's face; for endpaper, 0.5 mm into the page block's edge, past the endpaper
        xk = -(EP_OFF + EP + 0.0005) if ctx.mode == "endpaper" else 0.0006
        c = max(0.0, xk + 0.024 * math.sin(j))  # the back pad's offset from the board's outer face
        print(f"[book] jaw eased to {math.degrees(j):.1f} deg; back pad {c * 1000:.2f} mm off the board", flush=True)
        seat = B.to_world(np.array([BT + c, my, BH - (0.006 if ctx.mode == "slack" else SEAT_D)]))[0]  # the clip's jaw tips when seated
    above = seat + 0.035 * zb
    yield from r.move(above + off() + [-0.05, 0.0, 0.04], grip=lean, t=2.5, label="over the book")
    yield from r.move(above + off(), t=1.0, label="over the mark")
    yield from r.servo(above, read=lambda: light().pos, timeout=2.0, tol=0.0004, label="over the mark")
    # the light turns a little in the pinch: lean the hand until the clip leans with the board, then the jaws go down square
    lean_of = lambda R: math.degrees(math.atan2(R[0, 2], R[2, 2]))  # noqa: E731
    g = TILT
    for _ in range(6):
        err = lean_of(book().rotation()) - lean_of(light().rotation())
        if abs(err) < 0.5:
            break
        g += err
        yield from r.move(r.at(), grip=_grip(g, pinch), t=0.6, label="lean with the board")
        yield from r.servo(above, read=lambda: light().pos, timeout=1.5, tol=0.0004, label="over the mark")
    # square the clip over the board's face to a tenth of a millimetre: the front jaw's tip must meet the board's edge on its chamfer
    p0 = r.at()
    for _ in range(6):
        e = book().to_local(light().pos)[0][0] - book().to_local(above)[0][0]
        if abs(e) < 0.0001:
            break
        p0 = p0 - e * xb
        yield from r.move(p0, t=0.4, label="square over the board")
        yield from r.wait(0.2)
    print(f"[book] clip from its set point, book frame (mm): {np.round((book().to_local(light().pos)[0] - book().to_local(above)[0]) * 1000, 2)}", flush=True)
    print(f"[book] lean: hand {g:.1f} deg, clip off the board's lean by {lean_of(book().rotation()) - lean_of(light().rotation()):.1f} deg", flush=True)
    yield from r.servo(lambda t: seat + max(0.0, 0.035 - 0.008 * t) * zb, read=lambda: light().pos, timeout=8.0, tol=0.0008, label="down onto the board")
    print(f"[book] down: {clip_on_board(light(), book())}", flush=True)
    # 3. the right hand lets go, so the spring shuts the jaws on the board, and moves clear; the left takes the head and bends the neck down to the page
    yield from r.open(t=0.6, label="let the jaws shut")
    yield from r.wait(0.5)
    yield from r.move(r.at() + [0.0, -0.08, 0.0], t=1.0, label="right out")  # sideways along the hinge, clear of the collar above the fingers
    yield from r.move(r.at() + [-0.03, 0.0, 0.05], t=0.8, label="right up")
    c = clip_on_board(light(), book())
    print(f"[book] clipped: {c}", flush=True)
    yield from r.move(r.at() + [-0.08, -0.06, 0.04], t=1.5, label="right back")
    if ctx.mode == "noaim":
        yield from r.wait(3.0)
        return
    head = lambda: light_points(light())["head"]  # noqa: E731
    tg = d.level(-90.0, flip=True)  # from the left, level, along the neck's hinge axis: the head stands about 42 cm above the desk, and the bend is then a turn of the wrist
    ax = l.approach_axis(tg)
    yield from l.move(head() - 0.07 * ax + [0.0, 0.0, 0.03], grip=tg, t=2.5, label="over the head")
    yield from l.move(head() - 0.005 * ax, t=1.0, label="round the head")
    yield from l.servo(head() - 0.005 * ax, timeout=1.0, label="round the head")
    yield from l.close(0.040, force=30.0, label="take the head")
    # the hand carries the head to where the neck hangs it in front of the page: the collar and neck step toward HANG (0, -68 degrees) and stop once the light is aimed within 8 degrees; the joints slip to follow
    tq1, tq2 = math.radians(HANG[0]), math.radians(HANG[1])
    for i, k in enumerate(list(range(8, 0, -1)) + [3, 2, 1]):
        L = light()
        q1, q2 = L.joint("collar"), L.joint("neck")
        if i >= 8:  # the collar slipped on the way: leave it, and bend the neck on until the head's direction (the two angles' sum) is HANG's
            tq1, tq2 = q1, math.radians(HANG[0] + HANG[1]) - q1
        n1, n2 = q1 + (tq1 - q1) / k, q2 + (tq2 - q2) / k
        Rl = L.rotation()
        delta = Rl @ _ry(n1) @ _ry(n2) @ (Rl @ _ry(q1) @ _ry(q2)).T
        hp = light_points(L)["head"]
        a2 = _ry(n1) @ (_ry(n2) @ (HEAD_C - P2) + P2 - P1) + P1
        newhead = L.to_world(a2)[0]
        yield from l.move(newhead + delta @ (l.at() - hp), grip=d._as_grip(delta @ l.grip.rot), t=0.6, label="hang the head")
        if aim_error(light(), book())[0] < 8.0:  # the light faces the page: stop bending
            break
    print(f"[book] aim {aim_error(light(), book())} neck {ctx.deg('light', 'neck'):.1f}", flush=True)
    if ctx.mode == "held":
        yield from l.wait(4.0)
        return
    yield from l.wait(1.0)
    l.grip = d._as_grip(d.ik.tcp_pose(l.q, l.base)[1])  # back out with the wrist as it stands, not as last asked (the turn stops a few degrees short at the wrist's limit)
    yield from l.open(t=0.6, label="let go of the head")
    yield from l.wait(1.0)
    yield from l.move(l.at() + [-0.08, 0.0, 0.0], t=1.2, label="left out")  # back off the head along its length, toward the reader
    print(f"[book] left out: aim {aim_error(light(), book())} neck {ctx.deg('light', 'neck'):.1f}", flush=True)
    yield from l.move(l.at() + [-0.04, 0.08, 0.04], t=1.5, label="left back")
    yield from r.wait(3.0)
