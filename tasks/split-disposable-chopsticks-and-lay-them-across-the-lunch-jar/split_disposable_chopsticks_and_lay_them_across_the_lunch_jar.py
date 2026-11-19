"""Split a joined pair of disposable chopsticks and lay the two sticks side by side across a lunch jar.

Scene (metres; x forward, y to the robot's left, z above the floor; the table top is at TOP). A grey cotton place
mat lies on the table. A closed 16 oz insulated food jar, 14.2 cm high, stands on it. A 24 cm pair of bamboo twin
chopsticks lies level across the jar's lid, long axis along x, eating tips toward the robot, joined head away. The
two sticks (about 4.8 x 4.8 mm each, 2 mm apart) are one piece over the last 3.5 cm at the head.

Model. Each stick is its own 3.9 g rigid body. One fixed joint at the root of the slit holds the two together and
PhysX breaks it for good when the moment through it passes 0.8 N m (about 4 N pulling the two tips apart). Nothing
is spawned, moved or welded during an episode. Section, slit, joined length, mass and split moment are estimates.

Scoring (judged at the end). 0: the pair is not split, or a stick has fallen off the table. Split cleanly: 1.0 when
both sticks lie across the jar's lid, tips together, let go and still; 0.75 otherwise. Split unevenly: 0.5 when
laid like that, 0.25 otherwise. The rigid sticks cannot splinter, so "uneven" is a rule: the joint let go while the
sticks were being scissored or slid past each other rather than opened across the slit (see `parting`).

Demonstration. A pinch on a 5 mm stick cannot resist turning about the pads' own axis, so the two pulls must act
in one line: both hands come level from their own sides and pinch ONE stick each at the same place, 1.5 cm from the
tips (the two fingertips are then about 5 mm apart), pull straight apart until the joint lets go, lift, bring the
sticks back side by side over the lid, lower them, let go and withdraw.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl  # noqa: E402
sys.modules.pop("twin_kit", None)  # bind the bond module to the parts module loaded with this task file
import twin_kit as tk  # noqa: E402
from twin_kit import TwinLeg  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
RUN_LIMIT_STEPS = 1600
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Split the chopsticks and lay them ready across the lunch jar."
PRESSED_PARTS = ("stick_left", "stick_right")  # the pads meet the sticks from the side on purpose
TEST_GRIPPERS = None  # offline layouts: {"left": (tool point xyz, pads' opening m), "right": (...)}
TEST_UNEVEN = None  # offline layouts: True when the split is to count as uneven
DIAG = False  # GPU diagnostic copies print CHOP_ lines
DEMO_WRONG = ""  # wrong-way copy: "stagger" puts the left hand 30 mm further along the pair than the right

# ---- the pair: bamboo twin chopsticks, 24 cm (section, slit, joined length, mass and split moment are estimates)
LEG = 0.0048  # each stick's section
GAP = 0.002  # slit between the sticks
LEN = 0.24
JOINED = 0.035  # the two sticks are one piece over this length at the head
HALF = (LEG + GAP) / 2  # a stick's centre line off the pair's centre line
MASS = 0.0039  # per stick
SPLIT_TORQUE = 0.8  # N m at the root of the slit
SPLIT_FORCE = 100.0  # N
BAMBOO_A, BAMBOO_B = (0.90, 0.78, 0.52), (0.84, 0.70, 0.44)
# ---- the place mat and the food jar
MAT_SIZE, MAT_T = (0.349, 0.451), 0.002
MAT_XY = (0.262, 0.0)
MAT_GREY = (0.30, 0.31, 0.33)
JAR_H, JAR_R, JAR_MASS = 0.142, 0.047, 0.363  # maker's listing: 5.6 in high, 3.7 in wide, 0.8 lb
LID_H, LID_R = 0.034, 0.044  # the steel lid and its flat top (estimates)
JAR_XY = (0.255, 0.0)
JAR_BLUE, STEEL = (0.09, 0.13, 0.28), (0.56, 0.58, 0.61)
PAIR_POS = (JAR_XY[0], JAR_XY[1], TOP + MAT_T + JAR_H + LEG / 2)  # the pair's middle over the jar's middle


def _stick(name, side, color):
    """One stick: the long leg plus its half of the joined web at the head (side +1: left, toward +y)."""
    return TwinLeg(name, PAIR_POS,
                   [box((0, side * HALF, 0), (LEN, LEG, LEG), color),
                    box((LEN / 2 - JOINED / 2, side * GAP / 4, 0), (JOINED, GAP / 2, LEG), color)],
                   mass=MASS, color=color, friction=(0.8, 0.7), contact_offset=0.001)


MAT = Part("place_mat", (MAT_XY[0], MAT_XY[1], TOP),
           [box((0, 0, MAT_T / 2), (MAT_SIZE[0], MAT_SIZE[1], MAT_T), MAT_GREY)], mass=None, color=MAT_GREY)
JAR = Part("food_jar", (JAR_XY[0], JAR_XY[1], TOP + MAT_T),
           [cyl((0, 0, (JAR_H - LID_H) / 2), JAR_R, JAR_H - LID_H, color=JAR_BLUE),
            cyl((0, 0, JAR_H - LID_H / 2), LID_R, LID_H, color=STEEL)],
           mass=JAR_MASS, color=JAR_BLUE, friction=(0.4, 0.3), contact_offset=0.001)
LEFT = _stick("stick_left", +1, BAMBOO_A)
RIGHT = _stick("stick_right", -1, BAMBOO_B)
BOND = tk.bond("joint", LEFT, RIGHT, (LEN / 2 - JOINED, 0.0, 0.0), SPLIT_TORQUE, SPLIT_FORCE)
OBJECTS = [MAT, JAR, LEFT, RIGHT]
STICK = {"left": ("stick_left", +1), "right": ("stick_right", -1)}

# ---- scoring thresholds (each has a ruler or photo check in the README)
TIPS_TOGETHER = 0.010 + LEG  # the two tips' centres this close: a gap of 10 mm or less between them
PARALLEL_DEG = 10.0
ON_LID_Z = 0.004  # a laid stick's centre is within this of lying flat on the lid
LAID_TILT_DEG = 8.0
RELEASE_CLEAR = 0.030  # both tool points at least this far from both sticks
SPEED = 0.01  # m/s
OFF_TABLE = 0.05  # anything this far below the table top has fallen off it
APART_MM, APART_DEG = 2.5, 3.0  # offline layouts: sticks this far off their joined pose are two pieces
PART_MIN = 0.001  # the way the sticks parted is read once their tips are this far off the joined pose


# ---------------------------------------------------------------- run context (reads only)
def _context():
    d = {"env": None, "t": None, "episode": 0, "step": 0}
    try:
        from yam_arena import inspect_embodiment as ie
    except ImportError:
        return d
    cls = ie.YamArenaEmbodiment
    if hasattr(cls, "_chop_run"):
        return cls._chop_run
    old_step, old_reset = cls._task_step, cls.reset

    def step(self, *a, **kw):
        d["env"], d["t"] = self._env, self._t
        d["step"] += 1
        return old_step(self, *a, **kw)

    def reset(self, *a, **kw):
        r = old_reset(self, *a, **kw)
        d["env"], d["t"] = self._env, 0.0
        d["episode"] += 1
        return r

    cls._chop_run = d
    cls._task_step, cls.reset = step, reset
    return d


_RUN = _context()
_MEM = {"episode": None}
_C = {"episode": None}


def _episode():
    if _MEM["episode"] != _RUN["episode"]:
        _MEM.clear()
        _MEM.update(episode=_RUN["episode"], split=False, uneven=None, diag=-1.0, tare=None)
    if _C["episode"] != _RUN["episode"]:
        _C.clear()
        _C.update(episode=_RUN["episode"], ref={}, trim={}, trim_step={}, cache={})


def _bases():
    from yam_arena import station
    return station.BASES


def hands():
    """Per arm: (tool point, rotation, pads' opening m), from the measured joints."""
    env = _RUN["env"]
    if env is None:
        return {}
    from yam_arena import ik
    out = {}
    for arm in ("left", "right"):
        q, _ = tk.arm_joints(env, arm)
        p, r = ik.tcp_pose(q, _bases()[arm])
        out[arm] = (p, r, tk.finger_state(env, arm)[0])
    return out


def grippers():
    """Per arm: (tool point, pads' opening m); None when nothing is known (then the sticks count as let go)."""
    if TEST_GRIPPERS is not None:
        return {k: (np.asarray(v[0], float), float(v[1])) for k, v in TEST_GRIPPERS.items()}
    hs = hands()
    return {k: (v[0], v[2]) for k, v in hs.items()} if hs else None


# ---------------------------------------------------------------- geometry of the sticks
def axes(st):
    """(tip-to-head unit, right-to-left unit), both level, of a stick's frame."""
    r = st.rotation()
    u, n = r @ np.array([1.0, 0.0, 0.0]), r @ np.array([0.0, 1.0, 0.0])
    u, n = np.array([u[0], u[1], 0.0]), np.array([n[0], n[1], 0.0])
    return u / max(np.linalg.norm(u), 1e-9), n / max(np.linalg.norm(n), 1e-9)


def station_point(st, side, s):
    """World point on a stick's centre line at distance s from the head end (side +1 left, -1 right)."""
    return np.asarray(st.origin, float) + st.rotation() @ np.array([LEN / 2 - s, side * HALF, 0.0])


def _segment_distance(p, a, b):
    ab = b - a
    k = float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + k * ab)))


def parting(left, right):
    """How the two tips have moved off their joined pose, in the left stick's frame: (opening across the slit,
    sliding along the sticks, scissoring across their thickness), metres. All zero while the pair is whole."""
    r = left.rotation()
    d = r.T @ (station_point(right, -1, LEN) - station_point(left, +1, LEN)) - np.array([0.0, -2 * HALF, 0.0])
    return -float(d[1]), float(d[0]), float(d[2])


def laid(stick, side, jar):
    """The stick lies flat across the jar's lid: on it, level, over it, and overhanging it at both ends."""
    if jar.tilt_deg() > 5.0:
        return False
    axis = np.asarray(jar.origin, float)
    mid, tip, head = (station_point(stick, side, s) for s in (LEN / 2, LEN, 0.0))
    flat = abs(mid[2] - (axis[2] + JAR_H + LEG / 2)) <= ON_LID_Z and stick.tilt_deg() <= LAID_TILT_DEG
    u = (head - tip)[:2] / max(np.linalg.norm((head - tip)[:2]), 1e-9)
    rel = (axis - mid)[:2]
    along, across = float(rel @ u), float(abs(rel[0] * u[1] - rel[1] * u[0]))
    return bool(flat and across <= LID_R and abs(along) <= LEN / 2 - LID_R)


# ---------------------------------------------------------------- scoring
def score_terms(objects):
    _episode()
    m = _MEM
    left, right, jar = objects["stick_left"], objects["stick_right"], objects["food_jar"]
    live = _RUN["t"] is not None and TEST_GRIPPERS is None
    if live:
        tk.release_broken(_RUN["env"], objects)  # a dead joint is switched off so the two sticks can touch again
    if live:  # in a run the pair is split once its joint has broken
        split = m["split"] = bool(m["split"] or tk.broken("joint"))
    else:  # from poses alone: the two sticks are off their joined pose (a whole pair holds them within 1 mm)
        mm, deg = tk.bond_error(left, right)
        split = bool(mm > APART_MM or deg > APART_DEG)
    opening, slide, scissor = parting(left, right)
    if TEST_UNEVEN is not None or not live:
        uneven = bool(TEST_UNEVEN)
    else:
        if split and m["uneven"] is None and max(abs(opening), abs(slide), abs(scissor)) >= PART_MIN:
            m["uneven"] = bool(opening < max(abs(slide), abs(scissor)))
            if DIAG:
                print("CHOP_PARTING t", _RUN["t"], "opening_slide_scissor_mm",
                      [round(v * 1000, 2) for v in (opening, slide, scissor)], "uneven", m["uneven"], flush=True)
        uneven = bool(m["uneven"])
    fallen = [n for n in ("stick_left", "stick_right") if np.asarray(objects[n].low)[2] < TOP - OFF_TABLE]
    tips = {arm: station_point(objects[n], side, LEN) for arm, (n, side) in STICK.items()}
    heads = {arm: station_point(objects[n], side, 0.0) for arm, (n, side) in STICK.items()}
    ul, ur = axes(left)[0], axes(right)[0]
    angle = math.degrees(math.acos(float(np.clip(ul @ ur, -1.0, 1.0))))
    tip_gap = float(np.linalg.norm(tips["left"] - tips["right"]))
    together = tip_gap <= TIPS_TOGETHER and angle <= PARALLEL_DEG
    on_lid = {arm: laid(objects[n], side, jar) for arm, (n, side) in STICK.items()}
    g = grippers()
    clear = True if g is None else all(_segment_distance(p, tips[arm], heads[arm]) >= RELEASE_CLEAR
                                       for p, _ in g.values() for arm in tips)
    still = all(float(np.linalg.norm(objects[n].lin_vel)) < SPEED for n, _ in STICK.values())
    t = dict(split=split, uneven=uneven, fallen=fallen, laid=on_lid, together=bool(together), released=bool(clear),
             still=bool(still), tip_gap=tip_gap, angle=angle)
    if DIAG:
        _diag(objects, t)
    return t


def score(objects):
    t = score_terms(objects)
    if not t["split"] or t["fallen"]:
        return 0.0  # the pair is still one piece (wherever it is), or a stick has fallen off the table
    ready = all(t["laid"].values()) and t["together"] and t["released"] and t["still"]
    if t["uneven"]:
        return 0.5 if ready else 0.25
    return 1.0 if ready else 0.75


def success(objects):
    return score(objects) >= 1.0


def _diag(objects, t):
    m, now = _MEM, _RUN["t"]
    if now is None or now - m["diag"] < 0.25:
        return
    m["diag"] = now
    f, info = {}, {}
    try:
        hs = hands()
        for arm, (name, side) in STICK.items():
            f[arm] = tk.hand_force(_RUN["env"], arm, _bases()[arm])
            p, r, g = hs[arm]
            info[arm] = (round(float(np.linalg.norm(p + TIP * r[:, 2] - station_point(objects[name], side, S_G))) * 1000, 1),
                         round(g * 1000, 1))
        if m["tare"] is None and now >= 6.0:
            m["tare"] = {k: v.copy() for k, v in f.items()}
        f = {k: np.round(v - (m["tare"][k] if m["tare"] else 0.0), 1).tolist() for k, v in f.items()}
    except Exception as e:  # noqa: BLE001
        f = repr(e)
    print("CHOP_DIAG", round(now, 2), {k: (round(v, 4) if isinstance(v, float) else v) for k, v in t.items()},
          "parting_mm", [round(v * 1000, 2) for v in parting(objects["stick_left"], objects["stick_right"])],
          "pads", info, "L", np.round(station_point(objects["stick_left"], +1, S_G), 4).tolist(),
          "R", np.round(station_point(objects["stick_right"], -1, S_G), 4).tolist(),
          "jar", np.round(np.asarray(objects["food_jar"].origin), 4).tolist(),
          "tilt", round(objects["stick_left"].tilt_deg(), 1), round(objects["stick_right"].tilt_deg(), 1), "F", f,
          flush=True)


# ---------------------------------------------------------------- demonstration (closed loop on live poses; reads only)
S_G = 0.225  # both pads' middle: this far from the head end, 1.5 cm from the tips
S_ARM = {"left": S_G - (0.030 if DEMO_WRONG == "stagger" else 0.0), "right": S_G}
D_IN = 0.0035  # pad tips this far past the stick's outer face
TIP = 0.013  # tool point to pad tips
J_OPEN = -0.006  # pads 12 mm apart
J_CLOSE = -0.0009  # pads commanded 1.8 mm apart: 3 mm inside the 4.8 mm stick, about 7.5 N
PULL, LIFT = 0.030, 0.020
LAY_OUT = 0.002  # each stick is laid this far outside its joined place: 6 mm between the two
PARK = {"left": np.array([0.150, 0.200, TOP + 0.200]), "right": np.array([0.150, -0.200, TOP + 0.200])}
Z = np.array([0.0, 0.0, 1.0])


class Grip(float):
    """A yaw that carries a full tool rotation (for grips that are not straight down)."""

    def __new__(cls, matrix):
        o = super().__new__(cls, 0.0)
        o.grip_matrix = np.asarray(matrix, float)
        return o


def _install_rotation():
    from yam_arena import demo as ad
    old = ad.grip_rotation
    if getattr(old, "_grip_matrix", False):
        return

    def rotation(yaw=0.0):
        r = getattr(yaw, "grip_matrix", None)
        return r.copy() if r is not None else old(float(yaw))

    rotation._grip_matrix = True
    rotation.__wrapped__ = old
    ad.grip_rotation = rotation


_install_rotation()


def state_now(name):
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import scene
    try:
        return scene.object_states(env, [name])[name]
    except Exception as e:  # noqa: BLE001
        print("CHOP_READ failed", name, repr(e), flush=True)
        return None


def pinch_pose(st, arm, out=0.0, pull=0.0, lift=0.0):
    """(tool point, rotation) of a hand on its own stick in pose st: level, fingers pointing across the pair from
    the stick's own side, pads above and below it, pad tips D_IN past its outer face; backed off `out` along the
    fingers, moved `pull` away from the other stick on an arc about the slit root, raised `lift`."""
    side, s = STICK[arm][1], S_ARM[arm]
    u, n = axes(st)
    n_out = side * n
    a = -n_out
    tip = (station_point(st, side, s) + n_out * (LEG / 2 - D_IN)
           + n_out * pull + u * (pull * pull / (2 * (s - JOINED))) + Z * lift)
    c = np.cross(a, u)
    return tip - (TIP + out) * a, np.column_stack([c, np.cross(a, c), a])


class CL(Waypoint):
    """Closed-loop waypoint. `where(ref)` gives the ideal tool pose from a stick's pose; until the phase ends the
    pose is read live (or from the reference frozen under `frozen`), afterwards the last target is kept. With
    `trim` the residual between the ideal and the measured tool point is integrated (0.1 per step, 15 mm at most)."""

    def __init__(self, t0, dur, arm, label, finger, where, stick, trim=False, frozen=None):
        self.t0, self.duration, self.arm, self.label, self.finger = t0, dur, arm, label, finger
        self.where, self.stick, self.trim, self.frozen = where, stick, trim, frozen

    def _ref(self):
        st = state_now(self.stick)
        if st is None:
            return tk.Pose(PAIR_POS)
        if self.frozen is None:
            return st
        return _C["ref"].setdefault((self.arm, self.frozen), tk.Pose(st.origin, st.quat_xyzw))

    def _target(self):
        _episode()
        t = _RUN["t"] if _RUN["t"] is not None else 0.0
        key = (self.arm, self.label, self.t0)
        if key in _C["cache"]:
            return _C["cache"][key]
        pos, rot = self.where(self._ref())
        pos = np.asarray(pos, float).copy()
        step = _RUN["step"]
        hs = hands()
        tr = _C["trim"].setdefault(self.arm, np.zeros(3))
        if self.trim and hs and _C["trim_step"].get(self.arm) != step:
            _C["trim_step"][self.arm] = step
            e = pos - hs[self.arm][0]
            tr += 0.1 * e
            np.clip(tr, -0.015, 0.015, out=tr)
            if DIAG and step % 10 == 0:
                print("CHOP_TRIM", self.arm, self.label, "resid_mm", np.round(e * 1000, 2).tolist(),
                      "trim_mm", np.round(tr * 1000, 2).tolist(), flush=True)
        out = (pos + tr, rot)
        if t >= self.t0 + self.duration:
            _C["cache"][key] = out
        return out

    @property
    def pos(self):
        return tuple(float(v) for v in self._target()[0])

    @property
    def yaw(self):
        r = self._target()[1]
        return Grip(r) if isinstance(r, np.ndarray) else float(r)


def demo():
    plan = {}
    for arm in ("left", "right"):
        name = STICK[arm][0]
        ways, t = [], 0.0

        def add(dur, label, finger, trim=False, frozen=None, _ways=ways, _arm=arm, _name=name, **kw):
            nonlocal t
            _ways.append(CL(t, dur, _arm, label, finger, lambda st, kw=kw, a=_arm: pinch_pose(st, a, **kw), _name,
                            trim, frozen))
            t += dur

        add(2.5, "out to the side of the tips", J_OPEN, out=0.080)
        add(1.5, "closer, level with the stick", J_OPEN, out=0.030)
        add(1.5, "wait and line up", J_OPEN, trim=True, out=0.030)
        add(2.0, "pad tips over this stick only", J_OPEN)
        add(1.5, "settle at depth", J_OPEN, trim=True)
        add(1.2, "close on the stick", J_CLOSE, frozen="clamp")
        add(0.6, "hold the clamp", J_CLOSE, frozen="clamp")
        add(4.0, "pull away from the other stick", J_CLOSE, frozen="clamp", pull=PULL)
        add(1.0, "hold", J_CLOSE, frozen="clamp", pull=PULL)
        add(2.0, "lift the stick", J_CLOSE, frozen="clamp", pull=PULL, lift=LIFT)
        add(2.5, "bring it back beside the other", J_CLOSE, frozen="clamp", pull=LAY_OUT, lift=LIFT)
        add(2.0, "lower it onto the lid", J_CLOSE, frozen="clamp", pull=LAY_OUT)
        add(0.6, "rest", J_CLOSE, frozen="clamp", pull=LAY_OUT)
        add(1.0, "let go", J_OPEN, frozen="clamp", pull=LAY_OUT)
        add(2.0, "draw the hand back", J_OPEN, frozen="clamp", pull=LAY_OUT, out=0.080)
        plan[arm] = ways
    return plan
