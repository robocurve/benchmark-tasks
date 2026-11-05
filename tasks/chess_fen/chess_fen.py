"""On the station table: a chessboard between the arms and ten chess pieces lined up beside it. The robot sets up the
position given in Forsyth-Edwards Notation (FEN) in the instruction, white at the near side, and leaves the pieces
that are not in the position off the board.

Board (fixed, taped to the table): 8 x 8 squares of 2.8 cm, buff and green, in a 1.3 cm border; 25 x 25 x 1 cm,
centred at (24.5, 0) cm. Rank 1 is nearest the arms, the a-file on the left (as seen from the arms), so a1 is a dark
square and h1 a light one, as on every chessboard.
Pieces (Staunton style, wood, painted ivory or black, 2.2 cm bases, 3.8 cm (pawn) to 7.1 cm (king) tall): beside the
board on the left, white bishop, white pawn, black queen and white knight; on the right, white king, white rook, black
king, black knight, black pawn and black rook.

The position: 6k1/8/1q3n2/4p3/3P4/2B5/8/4K2R w - - 0 1 (white: king e1, rook h1, bishop c3, pawn d4; black: king g8,
queen b6, knight f6, pawn e5). The white knight and the black rook are not in it.

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/chess_fen/chess_fen.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
    --instruction "Set up this chess position, white at the near side: 6k1/8/1q3n2/4p3/3P4/2B5/8/4K2R w - - 0 1" \\
    --scorer task_score --max-steps 2400 --store-frames --log-dir runs/chess-fen-demo
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

# Load yampatch.py from this folder (task files are imported by path, so their folder is not on sys.path).
if "yampatch" not in sys.modules:
    _spec = importlib.util.spec_from_file_location("yampatch", Path(__file__).with_name("yampatch.py"))
    sys.modules["yampatch"] = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(sys.modules["yampatch"])
yampatch = sys.modules["yampatch"]

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the board ---------------------------------------------------------------------------------------------------------
SQUARE, BORDER, THICK = 0.028, 0.013, 0.010
BOARD_XY = (0.245, 0.0)
BOARD_SIZE = 8 * SQUARE + 2 * BORDER
BOARD_TOP = TOP + THICK
LIGHT, DARK, EDGE = (0.93, 0.89, 0.76), (0.32, 0.52, 0.34), (0.85, 0.85, 0.82)
FILES = "abcdefgh"


def square_xy(sq: str) -> np.ndarray:
    """A square's centre (world x, y): rank 1 nearest the arms, the a-file on the left (+y)."""
    f, r = FILES.index(sq[0]), int(sq[1])
    return np.array([BOARD_XY[0] + (r - 4.5) * SQUARE, BOARD_XY[1] + (3.5 - f) * SQUARE])


def _stage(name: str):
    """A new USD stage (z up, metres, kg) with a root prim ``/name`` and a rubber-like physics material."""
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    material = UsdShade.Material.Define(stage, f"/{name}/physics_material")
    phys = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    phys.CreateStaticFrictionAttr(1.2)  # the adapter's friction for graspable objects
    phys.CreateDynamicFrictionAttr(1.0)
    return stage, root, material


def _shape(stage, path, kind, size, at, color, collide, material):
    """A cube (size: x, y, z), cylinder (size: radius, height) or sphere (size: radius) centred at ``at``."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    if kind == "cube":
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(1.0)
        geom.AddTranslateOp().Set(Gf.Vec3d(*at))
        geom.AddScaleOp().Set(Gf.Vec3f(*size))
    elif kind == "cylinder":
        geom = UsdGeom.Cylinder.Define(stage, path)
        geom.CreateRadiusAttr(size[0])
        geom.CreateHeightAttr(size[1])
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(*at))
    else:
        geom = UsdGeom.Sphere.Define(stage, path)
        geom.CreateRadiusAttr(size[0])
        geom.AddTranslateOp().Set(Gf.Vec3d(*at))
    geom.CreateDisplayColorAttr([Gf.Vec3f(*_linear(color))])
    if collide:
        prim = geom.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        prim.AddAppliedSchema("PhysxCollisionAPI")  # contacts start 5 mm out, as for the adapter's own objects
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.005)
        prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")


def _linear(srgb):
    """sRGB colour to the linear values USD display colours hold."""
    return [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in srgb]


def _out(name: str) -> Path:
    import tempfile

    path = Path(tempfile.gettempdir()) / "yam_tasks" / f"{name}.usda"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def build_board() -> str:
    """A fixed board: one collider slab (its bottom at the origin) and 64 thin visual squares on top."""
    stage, _, mat = _stage("board")
    _shape(stage, "/board/slab", "cube", (BOARD_SIZE, BOARD_SIZE, THICK), (0, 0, THICK / 2), EDGE, True, mat)
    for f in range(8):
        for r in range(8):
            color = DARK if (f + r) % 2 == 0 else LIGHT  # a1 (f 0, r 0) is dark
            at = ((r - 3.5) * SQUARE, (3.5 - f) * SQUARE, THICK - 0.0003)
            _shape(stage, f"/board/{FILES[f]}{r + 1}", "cube", (SQUARE, SQUARE, 0.001), at, color, False, mat)
    path = _out("board")
    stage.GetRootLayer().Export(str(path))
    return str(path)


# -- the pieces: compound rigid bodies, their origin at the centre of the base's bottom (m) ---------------------------
def _base():
    return [("cylinder", (0.011, 0.006), (0, 0, 0.003))]


CRENELS = ((1, 0), (-1, 0), (0, 1), (0, -1))
SHAPES = {
    "pawn": _base() + [
        ("cylinder", (0.006, 0.016), (0, 0, 0.014)),
        ("cylinder", (0.008, 0.002), (0, 0, 0.023)),
        ("sphere", (0.0075,), (0, 0, 0.0305)),
    ],
    "rook": _base() + [
        ("cylinder", (0.0085, 0.028), (0, 0, 0.020)),
        ("cylinder", (0.0105, 0.008), (0, 0, 0.038)),
        *[("cube", (0.005, 0.005, 0.004), (0.0065 * c, 0.0065 * s, 0.044)) for c, s in CRENELS],
    ],
    "knight": _base() + [
        ("cylinder", (0.0075, 0.020), (0, 0, 0.016)),
        ("cube", (0.012, 0.010, 0.022), (-0.002, 0, 0.034)),  # neck
        ("cube", (0.022, 0.010, 0.010), (0.005, 0, 0.042)),  # head, its muzzle forward (+x)
        ("cube", (0.004, 0.006, 0.006), (-0.005, 0, 0.049)),  # ears
    ],
    "bishop": _base() + [
        ("cylinder", (0.0075, 0.030), (0, 0, 0.021)),
        ("cylinder", (0.0095, 0.002), (0, 0, 0.037)),
        ("sphere", (0.0085,), (0, 0, 0.046)),
        ("sphere", (0.003,), (0, 0, 0.0565)),
    ],
    "queen": _base() + [
        ("cylinder", (0.008, 0.036), (0, 0, 0.024)),
        ("cylinder", (0.0105, 0.003), (0, 0, 0.0435)),
        ("cylinder", (0.0095, 0.008), (0, 0, 0.049)),
        ("sphere", (0.0045,), (0, 0, 0.0575)),
    ],
    "king": _base() + [
        ("cylinder", (0.0085, 0.040), (0, 0, 0.026)),
        ("cylinder", (0.011, 0.003), (0, 0, 0.0475)),
        ("cylinder", (0.0095, 0.010), (0, 0, 0.054)),
        ("cube", (0.003, 0.003, 0.012), (0, 0, 0.065)),  # cross
        ("cube", (0.003, 0.009, 0.003), (0, 0, 0.066)),
    ],
}
MASS = {"pawn": 0.020, "rook": 0.030, "knight": 0.030, "bishop": 0.030, "queen": 0.035, "king": 0.040}
IVORY, BLACK = (0.92, 0.87, 0.74), (0.10, 0.10, 0.10)


def build_piece(name: str, kind: str, color) -> str:
    from pxr import Gf, UsdPhysics

    stage, root, mat = _stage(name)
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.CreateMassAttr(MASS[kind])
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.010))  # weighted base, as in real sets
    for i, (shape, size, at) in enumerate(SHAPES[kind]):
        _shape(stage, f"/{name}/part{i}", shape, size, at, color, True, mat)
    path = _out(name)
    stage.GetRootLayer().Export(str(path))
    return str(path)


# name: (kind, colour, start (x, y)). White knights face away from the arms, black ones towards them.
PIECES = {
    "white_bishop": ("bishop", "white", (0.15, 0.17)),
    "black_queen": ("queen", "black", (0.20, 0.17)),
    "white_knight": ("knight", "white", (0.25, 0.17)),
    "white_pawn": ("pawn", "white", (0.30, 0.17)),
    "black_knight": ("knight", "black", (0.13, -0.17)),
    "white_king": ("king", "white", (0.17, -0.17)),
    "black_rook": ("rook", "black", (0.21, -0.17)),
    "black_pawn": ("pawn", "black", (0.25, -0.17)),
    "white_rook": ("rook", "white", (0.29, -0.17)),
    "black_king": ("king", "black", (0.33, -0.17)),
}
FEN = "6k1/8/1q3n2/4p3/3P4/2B5/8/4K2R w - - 0 1"
GOALS = {  # where each piece of the position stands
    "white_king": "e1", "white_rook": "h1", "white_bishop": "c3", "white_pawn": "d4",
    "black_king": "g8", "black_queen": "b6", "black_knight": "f6", "black_pawn": "e5",
}  # fmt: skip
EXTRA = [n for n in PIECES if n not in GOALS]  # the white knight and the black rook


def _piece(name, kind, color, xy):
    rot = (0.0, 0.0, 1.0, 0.0) if color == "black" else (0.0, 0.0, 0.0, 1.0)  # black faces the arms
    rgb = IVORY if color == "white" else BLACK
    return yampatch.usd_object(
        name, (xy[0], xy[1], TOP), kind="rigid", mass=MASS[kind], rot_xyzw=rot, extent=0.07,
        build=lambda: build_piece(name, kind, rgb),
    )  # fmt: skip


BOARD = yampatch.usd_object("board", (*BOARD_XY, TOP), kind="fixed", build=build_board, extent=BOARD_SIZE)
OBJECTS = [BOARD, *(_piece(n, *spec) for n, spec in PIECES.items())]
yampatch.install()

# -- scoring -------------------------------------------------------------------------------------------------------
XY_TOL = 0.010  # a piece's centre within 1 cm of its square's centre (squares are 2.8 cm)
Z_TOL = 0.005  # standing on the board: its bottom within 5 mm of the board's top
MAX_TILT = np.radians(20)


def upright(s) -> bool:
    x, y, _, _ = s.quat_xyzw
    return bool(1.0 - 2.0 * (x * x + y * y) > np.cos(MAX_TILT))  # the piece's z axis against the vertical


def over_board(s) -> bool:
    return bool(np.all(np.abs(s.pos[:2] - np.array(BOARD_XY)) < 4 * SQUARE))


def on_square(s, sq: str) -> bool:
    return (
        np.linalg.norm(s.pos[:2] - square_xy(sq)) < XY_TOL and abs(s.low[2] - BOARD_TOP) < Z_TOL and upright(s)
    )  # fmt: skip


def counts(objects):
    """(pieces of the position standing on their square, other pieces over the board's 64 squares)."""
    right = {n for n, sq in GOALS.items() if on_square(objects[n], sq)}
    wrong = [n for n in PIECES if n not in right and over_board(objects[n])]
    return len(right), len(wrong)


def success(objects):
    right, wrong = counts(objects)
    return right == len(GOALS) and wrong == 0 and all(objects[n].resting() for n in PIECES)


def score(objects):
    """1/8 per piece of the position on its square, minus 1/8 per other piece over the board (at least 0)."""
    right, wrong = counts(objects)
    return max(0, right - wrong) / len(GOALS)


def distance(objects):
    """Summed distance (m) of the position's pieces from their squares."""
    return float(sum(np.linalg.norm(objects[n].pos[:2] - square_xy(sq)) for n, sq in GOALS.items()))


# -- demo: each arm sets up its half of the board (files a-d left, e-h right) ---------------------------------------
GRASP = 0.028  # TCP height above a piece's bottom: the finger pads (13 mm below the TCP) hold its column
TRAVEL_Z = BOARD_TOP + 0.075 + GRASP + 0.015  # a carried piece clears the tallest standing one by 1.5 cm
NARROW = -0.020  # finger target when open around pieces: a 4 cm gap, clear of the neighbouring squares
SPEED = 1.3


def move(name):
    (px, py), (qx, qy) = PIECES[name][2], square_xy(GOALS[name])
    pz, qz = TOP + GRASP, BOARD_TOP + GRASP + 0.004
    k = 1.0 / SPEED
    return [
        Waypoint(1.5 * k, (px, py, TRAVEL_Z), NARROW, f"{name}: to it"),
        Waypoint(1.5 * k, (px, py, pz), NARROW, "descend"),
        Waypoint(0.8 * k, (px, py, pz), 0.0, "close"),
        Waypoint(1.2 * k, (px, py, TRAVEL_Z), 0.0, "lift"),
        Waypoint(1.8 * k, (qx, qy, TRAVEL_Z), 0.0, "carry"),
        Waypoint(1.4 * k, (qx, qy, qz), 0.0, "lower"),
        Waypoint(0.6 * k, (qx, qy, qz), NARROW, "let go"),
        Waypoint(1.0 * k, (qx, qy, TRAVEL_Z), NARROW, "up"),
    ]


def rest(arm, seconds):
    return [Waypoint(seconds, (0.20, 0.30 if arm == "left" else -0.30, TRAVEL_Z), NARROW, "rest")]


def _duration(waypoints):
    return sum(w.duration for w in waypoints)


def demo():
    """Both arms work on the outer files at once (b, c and g, h), then one after the other near the middle."""
    left1 = move("black_queen") + move("white_bishop") + rest("left", 2.0)
    right1 = move("white_rook") + move("black_king") + rest("right", 2.0)
    t1 = max(_duration(left1), _duration(right1))
    left2 = move("white_pawn") + rest("left", 2.0)
    right2 = move("white_king") + move("black_pawn") + move("black_knight") + rest("right", 2.0)
    left = left1 + rest("left", t1 - _duration(left1)) + left2
    right = right1 + rest("right", t1 - _duration(right1) + _duration(left2)) + right2
    return {"left": left, "right": right}
