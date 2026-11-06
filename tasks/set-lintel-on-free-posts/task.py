"""On the station table, on the midline in front of the arms: two loose square wooden posts stand upright,
side by side, 100 mm apart. Nothing holds them; a light push near the top tips one over. A wooden lintel
(a flat beam) lies flat on the table on the right arm's side, nearer the robot than the posts. Lay the lintel
flat across the tops of both posts, centred on them, and let go, without knocking over or moving either post.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
POST = (0.040, 0.040, 0.110)  # square post, standing on end
POST_MASS = 0.125
POST_X = 0.250
POST_Y = {"left": 0.070, "right": -0.070}  # centres 140 mm apart: a 100 mm clear gap between them
POST_TOP = TOP + POST[2]

LINTEL = (0.040, 0.220, 0.030)  # lying flat, long side along y
LINTEL_MASS = 0.185
LINTEL_START = (0.180, -0.200)

OAK, PALE = (0.55, 0.36, 0.20), (0.86, 0.74, 0.55)

POSTS = {side: Box(f"post_{side}", pos=(POST_X, y, TOP + POST[2] / 2), size=POST, mass=POST_MASS, color=PALE)
         for side, y in POST_Y.items()}
LINTEL_BOX = Box("lintel", pos=(*LINTEL_START, TOP + LINTEL[2] / 2), size=LINTEL, mass=LINTEL_MASS, color=OAK)
OBJECTS = [*POSTS.values(), LINTEL_BOX]

POST_TILT_MAX = np.radians(3.0)  # upright: post axis within 3 degrees of vertical at the end
POST_MOVE_OK = 0.010  # full credit: no post ever moved more than 10 mm from its start
POST_MOVE_MAX = 0.030  # any credit: no post ever moved more than 30 mm
POST_TURN_MAX = np.radians(15.0)  # any credit: no post ever turned more than 15 degrees from its start pose
LINTEL_TILT_MAX = np.radians(5.0)  # flat: a 40 x 220 face down, within 5 degrees of level
SEAT_TOL = 0.001  # lintel underside within 1 mm of the post tops: resting on them, not held above
OVERHANG_TOL = 0.010  # centred: the two end overhangs differ by at most 10 mm
SIDE_TOL = 0.010  # centred: each post's top centre within 10 mm of the lintel's centre line, sideways
RESET_TOL = 0.0005  # a scene reset puts every object within 0.5 mm of its start, at rest
RESET_JUMP = 0.020  # ... in one jump: some object moved over 20 mm since the previous step (a teleport)

_START = {o.name: np.asarray(o.pos, float) for o in OBJECTS}
# Worst displacement of each post's centre from its start, over the whole trial (m). Cleared only by a scene
# reset: every object within 0.5 mm of its start and at rest, reached by a jump of over 20 mm in one control step,
# which no push or carry can do. A robot standing a toppled post back on its mark moves it there step by step,
# so the record stays.
_WORST = {name: 0.0 for name in ("post_left", "post_right")}
_TURN = {name: 0.0 for name in _WORST}  # worst rotation of each post from its start pose (rad), same record
_PREV: dict = {}  # each object's centre at the previous call


def _rot(state) -> np.ndarray:
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    """Angle between the object's own vertical axis and the world vertical (rad); upside down counts as level."""
    return float(np.arccos(np.clip(abs(_rot(state)[2, 2]), 0.0, 1.0)))


def _track(objects) -> None:
    pos = {n: np.asarray(objects[n].pos, float) for n in _START}
    at_start = all(np.linalg.norm(pos[n] - p) < RESET_TOL and objects[n].resting() for n, p in _START.items())
    jumped = not _PREV or max(np.linalg.norm(pos[n] - _PREV[n]) for n in _START) > RESET_JUMP
    _PREV.update(pos)
    if at_start and jumped:
        for n in _WORST:
            _WORST[n] = _TURN[n] = 0.0
        return
    for n in _WORST:
        _WORST[n] = max(_WORST[n], float(np.linalg.norm(objects[n].pos - _START[n])))
        w = abs(float(np.asarray(objects[n].quat_xyzw, float)[3]) / np.linalg.norm(objects[n].quat_xyzw))
        _TURN[n] = max(_TURN[n], 2 * float(np.arccos(min(w, 1.0))))  # posts start unrotated


def _posts_upright(objects) -> bool:
    return all(_tilt(objects[n]) <= POST_TILT_MAX and objects[n].resting() for n in _WORST)


def _lintel_set(objects) -> bool:
    """Lying flat and still on both posts: its underside within 1 mm of the post tops, and the centre of each
    post's top face under the lintel (so it carries at least about half of each post's top)."""
    lin = objects["lintel"]
    if not (_tilt(lin) <= LINTEL_TILT_MAX and lin.resting()):
        return False
    for local in _post_tops_local(objects):  # each post's top centre in the lintel's frame
        if abs(abs(local[2]) - LINTEL[2] / 2) > SEAT_TOL:  # on the lintel's underside, within 1 mm
            return False
        if abs(local[0]) > LINTEL[0] / 2 or abs(local[1]) > LINTEL[1] / 2:
            return False
    return True


def _post_tops_local(objects) -> list[np.ndarray]:
    """Each post's top-face centre in the lintel's frame (x across the lintel, y along it, z up from its centre)."""
    lin = objects["lintel"]
    r = _rot(lin)
    out = []
    for n in _WORST:
        post = objects[n]
        top = post.pos + _rot(post)[:, 2] * POST[2] / 2
        out.append(r.T @ (top - lin.pos))
    return out


def _centring(objects) -> tuple[float, float]:
    """(difference between the two end overhangs past the posts, largest sideways distance of a post's top
    centre from the lintel's centre line), m. Each end's overhang is measured along the lintel, from the end to
    the post under that end; for posts of equal width the difference is |u_left + u_right|, where u is each post
    top centre's position along the lintel from its middle."""
    a, b = _post_tops_local(objects)
    return abs(float(a[1] + b[1])), max(abs(float(a[0])), abs(float(b[0])))


def _centred(objects) -> bool:
    diff, side = _centring(objects)
    return diff <= OVERHANG_TOL and side <= SIDE_TOL


def success(objects):
    """Lintel set and centred, both posts upright and still, and neither post ever moved more than 10 mm or turned
    more than 15 degrees."""
    _track(objects)
    return bool(
        _lintel_set(objects)
        and _posts_upright(objects)
        and max(_WORST.values()) <= POST_MOVE_OK
        and max(_TURN.values()) <= POST_TURN_MAX
        and _centred(objects)
    )


def distance(objects):
    lin = objects["lintel"].pos
    goal = np.array((POST_X, 0.0, POST_TOP + LINTEL[2] / 2))
    return float(np.linalg.norm(lin - goal))


def score(objects):
    """1.0: success. 0.7: lintel set, posts upright and never moved over 10 mm, but not centred. 0.5: lintel set
    and posts upright, but a post moved 10 to 30 mm at some point. 0 otherwise (a post tilted over 3 degrees or
    moving, a post ever moved over 30 mm or turned over 15 degrees, or the lintel not lying flat and still on both
    posts)."""
    _track(objects)
    worst = max(_WORST.values())
    if worst > POST_MOVE_MAX or max(_TURN.values()) > POST_TURN_MAX or not _posts_upright(objects) or not _lintel_set(objects):
        return 0.0
    if worst > POST_MOVE_OK:
        return 0.5
    return 1.0 if _centred(objects) else 0.7


# Right arm. The grip is turned 90 degrees so the fingers close across the lintel's 40 mm width, at its middle.
YAW = np.pi / 2
TIP_PAST_TCP = 0.013
GRIP_ABOVE_BOTTOM = 0.010  # finger tips 10 mm above the lintel's underside
PICK_Z = TOP + GRIP_ABOVE_BOTTOM + TIP_PAST_TCP
# Commanded with the underside 8 mm above the post tops: carrying the lintel the arm settles a few mm below its
# target, and the lintel must still be clear of the tops when the fingers open, so that it drops onto them and the
# 1 mm seat test can only hold after release.
PLACE_Z = POST_TOP + 0.008 + GRIP_ABOVE_BOTTOM + TIP_PAST_TCP
CARRY_Z = POST_TOP + 0.020 + GRIP_ABOVE_BOTTOM + TIP_PAST_TCP  # underside 20 mm above the post tops


def demo():
    lx, ly = LINTEL_START
    w = [
        Waypoint(1.5, (lx, ly, CARRY_Z), FINGER_OPEN, "over the lintel", YAW),
        Waypoint(1.5, (lx, ly, PICK_Z), FINGER_OPEN, "descend", YAW),
        Waypoint(0.8, (lx, ly, PICK_Z), FINGER_CLOSED, "grip", YAW),
        Waypoint(1.5, (lx, ly, CARRY_Z), FINGER_CLOSED, "lift", YAW),
        Waypoint(2.0, (POST_X, 0.0, CARRY_Z), FINGER_CLOSED, "carry over the posts", YAW),
        Waypoint(2.0, (POST_X, 0.0, PLACE_Z), FINGER_CLOSED, "lower", YAW),
        Waypoint(1.2, (POST_X, 0.0, PLACE_Z), FINGER_CLOSED, "settle", YAW),
        Waypoint(0.8, (POST_X, 0.0, PLACE_Z), FINGER_OPEN, "release", YAW),
        Waypoint(1.5, (POST_X, 0.0, CARRY_Z + 0.02), FINGER_OPEN, "up", YAW),
    ]
    return {"right": w}
