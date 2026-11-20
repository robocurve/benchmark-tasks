"""Braid a practice mannequin head's long hair: three sections, six crossings, an outer one over the middle each time.

A hairdressing training head with long dark synthetic hair lies on its back on a folded hand towel at the far side of
the table, crown toward the robot. The hair is combed out over the crown onto the bare table and gathered there with
a plain elastic that lies against the crown (x 0.54, the start of the braid). From the elastic the hair is split into
three equal sections of 450 mm, each a round bundle about 10 mm thick whose tip is bound with a small elastic (red on
the section to the robot's right, white in the middle, blue to its left). The sections lie flat and straight on the
table, fanned toward the robot: red 20 degrees to the robot's right, white straight toward the robot, blue 20
degrees to its left. Braid them: lay an outer section over the middle one, then the other outer section over the
new middle one, and keep alternating until there are six crossings; then let go.
Metres; x forward from the arm bases, y to the robot's left, z up.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
for _helper in ("rope_strand", "plait_word"):
    sys.modules.pop(_helper, None)
import plait_word  # noqa: E402
import rope_strand  # noqa: E402
from rope_strand import Piece  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 5.0, "end"  # success: six crossings photographed for 5 s in a row
RUN_LIMIT_STEPS = 3600  # 180 s at 20 Hz
DEMO_WAY = "plait"  # "plait": sides alternate; "twist" (wrong way): the right section goes over the middle every time
TEST_ROPE = None  # offline checks only: {section name: (N, 3) centre-line points} instead of the objects' states
TEST_GRIPPERS = None  # offline checks only: {"left"/"right": (TCP position, TCP rotation)}

# ---------------------------------------------------------------- the hair sections (all figures are estimates)
D = 0.010  # a third of the hair held as one round bundle (a ponytail of about 5.5 cm circumference, split in three)
PITCH = 0.015  # link length of the simulated section
LINKS = 30  # 450 mm from the gathering elastic to the end; link 0 is the hair inside the elastic
KG_PER_M = 0.040  # a 10 mm bundle of synthetic fibre (about 1.3 g/cm3) packed about 40%
EI = 3.0e-5  # N m^2: ~7,000 fibres of ~75 um bending side by side; held level, a 10 cm overhang hangs ~7.6 cm down
HAIR_FRICTION = (0.8, 0.8)  # fibre bundle on fibre and on bare wood
ELASTIC_X = 0.55  # the sections' starts, side by side in the gathering elastic
SPACING = 0.011  # section centres where they leave the elastic
FAN = math.radians(20.0)  # the outer sections' angle from straight toward the robot
HAIR = (0.13, 0.09, 0.06)  # darkest brown synthetic hair
BAND_LINK = LINKS - 2  # each section's tip elastic sits 22.5 mm from its end


def _section(band):
    return rope_strand.Rope(links=LINKS, pitch=PITCH, radius=D / 2, link_mass=KG_PER_M * PITCH, bend_k=EI / PITCH,
                            bend_d=0.1 * EI / PITCH, friction=HAIR_FRICTION, color=HAIR, roughness=0.55,
                            band_link=BAND_LINK, band_color=band, band_width=0.008)


# name, place at the elastic (y), heading toward the robot (world yaw), tip-elastic colour; from the robot's right
_SECTIONS = [("section_red", -SPACING, math.pi + FAN, (0.86, 0.10, 0.10)),
             ("section_white", 0.0, math.pi, (0.96, 0.96, 0.96)),
             ("section_blue", SPACING, math.pi - FAN, (0.12, 0.32, 0.86))]
NAMES = tuple(n for n, _, _, _ in _SECTIONS)
SECTIONS = [rope_strand.strand(n, (ELASTIC_X, y), yaw, _section(c), TOP) for n, y, yaw, c in _SECTIONS]

# ---------------------------------------------------------------- the training head, lying on its back
# Head frame: origin on the table top below the crown (vertex), x from the crown toward the chin, z up. True size of
# an adult female training head (estimates: crown to chin 225 mm, 145 mm ear to ear, 185 mm back to brow).
TOWEL_H = 0.020  # a hand towel folded to about 300 x 250 mm, 20 mm thick
CROWN_X = 0.590
SKIN, BROW, LIPS, ELASTIC = (0.93, 0.79, 0.68), (0.30, 0.20, 0.14), (0.78, 0.42, 0.42), (0.55, 0.56, 0.60)


def head_pieces():
    b = TOWEL_H  # the back of the head rests on the towel
    cz = b + 0.0925  # cranium centre height
    return [
        Piece("ellipsoid", (0.095, 0.0, cz), (0.095, 0.0725, 0.0925), SKIN),  # cranium
        Piece("ellipsoid", (0.160, 0.0, cz + 0.032), (0.068, 0.056, 0.060), SKIN),  # face and jaw
        Piece("ellipsoid", (0.135, 0.0, cz + 0.092), (0.024, 0.012, 0.014), SKIN),  # nose
        Piece("ellipsoid", (0.135, -0.074, cz - 0.012), (0.031, 0.009, 0.018), SKIN),  # ears
        Piece("ellipsoid", (0.135, 0.074, cz - 0.012), (0.031, 0.009, 0.018), SKIN),
        Piece("ellipsoid", (0.118, -0.031, cz + 0.0796), (0.006, 0.012, 0.003), BROW),  # painted eyes
        Piece("ellipsoid", (0.118, 0.031, cz + 0.0796), (0.006, 0.012, 0.003), BROW),
        Piece("ellipsoid", (0.103, -0.031, cz + 0.0823), (0.004, 0.017, 0.003), BROW),  # brows
        Piece("ellipsoid", (0.103, 0.031, cz + 0.0823), (0.004, 0.017, 0.003), BROW),
        Piece("ellipsoid", (0.180, 0.0, cz + 0.0883), (0.006, 0.018, 0.003), LIPS),
        Piece("cylinder", (0.225, 0.0, b + 0.052), (0.052, 0.120), SKIN),  # neck, to its flat base
        Piece("ellipsoid", (0.090, 0.0, cz - 0.015), (0.105, 0.079, 0.105), HAIR),  # hair over the crown and back
        Piece("ellipsoid", (0.000, 0.0, b + 0.012), (0.050, 0.050, 0.028), HAIR),  # hair combed off the crown
        Piece("ellipsoid", (-0.022, 0.0, 0.010), (0.022, 0.030, 0.010), HAIR),  # ... down to the elastic
        Piece("ellipsoid", (ELASTIC_X - 0.004 - CROWN_X, 0.0, 0.009), (0.004, 0.023, 0.009), ELASTIC),
        Piece("ellipsoid", (0.095, 0.0, cz), (0.070, 0.070, 0.070), SKIN, collide=True),  # invisible colliders
        Piece("ellipsoid", (0.165, 0.0, cz + 0.020), (0.054, 0.054, 0.054), SKIN, collide=True),
        Piece("cylinder", (0.225, 0.0, b + 0.052), (0.050, 0.120), SKIN, collide=True),
    ]


TOWEL = Box(name="towel", pos=(CROWN_X + 0.150, 0.0, TOP + TOWEL_H / 2), size=(0.300, 0.250, TOWEL_H),
            color=(0.62, 0.74, 0.84))
HEAD = rope_strand.figure("training_head", (CROWN_X, 0.0, TOP), 0.0, head_pieces(), SKIN)
OBJECTS = [*SECTIONS, TOWEL, HEAD]
rope_strand.install()

# ---------------------------------------------------------------- scoring: a top photo with the hands clear
NEEDED = 6  # crossings for full score
HAND_CLEAR = 0.030  # both grippers at least this far from the hair for a photo
STILL = 0.010  # m/s: every link of every section slower than this for a photo
GRIP_REACH = 0.050  # gripper envelope: a cylinder of 50 mm radius (open fingers) from the pad tips 120 mm up the hand
GRIP_TIP, GRIP_BACK = 0.013, 0.120
PLAIT_START = np.array((ELASTIC_X - 0.010, 0.0))  # where the braid starts (the gathering elastic); it grows along -x
AXIS = np.array((-1.0, 0.0))
GRIP_DOWN = ((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0))  # TCP rotation of a hand pointing straight down


def centre_lines(objects):
    """Each section's centre-line, elastic end first."""
    if TEST_ROPE is not None:
        return {n: np.asarray(TEST_ROPE[n], float) for n in NAMES}
    out = {}
    for n in NAMES:
        o = objects[n]
        if hasattr(o, "polyline"):
            out[n] = o.polyline()
        else:  # a plain object state (offline checks): the straight section it stands for
            x, y, z, w = o.quat_xyzw
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            d = np.array((math.cos(yaw), math.sin(yaw), 0.0)) * PITCH
            p0 = np.asarray(o.pos, float) - d * LINKS / 2
            out[n] = np.array([p0 + d * i for i in range(LINKS + 1)])
    return out


def read_plait(objects):
    """{'count', 'word', 'along', 'dz', 'skipped'}: see plait_word.read."""
    return plait_word.read(centre_lines(objects), PLAIT_START, AXIS, NAMES)


def hair_still(objects):
    for n in NAMES:
        o = objects[n]
        v = o.speeds if hasattr(o, "speeds") else [np.linalg.norm(o.lin_vel)]
        if not np.all(np.isfinite(v)) or float(np.max(v)) >= STILL:
            return False
    return True


def hands_clear(objects):
    """Both grippers at least HAND_CLEAR from the hair's surface (offline checks without grippers: clear)."""
    grips = grippers()
    if grips is None:
        return True
    pts = np.vstack(list(centre_lines(objects).values()))
    for tcp, rot in grips.values():
        up = -np.asarray(rot, float)[:, 2]  # from the fingertips toward the wrist
        tips = np.asarray(tcp, float) - up * GRIP_TIP
        along = (pts - tips) @ up
        radial = np.linalg.norm(pts - tips - along[:, None] * up, axis=1)
        gap = np.hypot(np.maximum(radial - GRIP_REACH, 0.0), np.maximum(np.maximum(-along, along - GRIP_BACK), 0.0))
        if float(gap.min()) - D / 2 < HAND_CLEAR:
            return False
    return True


def photo(objects):
    """The braid count a top photo would show now, or None if no photo may be taken (a hand near the hair, or the
    hair still moving)."""
    if not (hair_still(objects) and hands_clear(objects)):
        return None
    return read_plait(objects)["count"]


_PHOTO = {"episode": None, "count": 0}


def score(objects):
    """Crossings in the last photo / 6 (at most 1.0). In a run the last photo is kept while a hand is near the hair;
    before any photo the score is 0."""
    count = photo(objects)
    if _RUN["env"] is None:  # offline checks: judge the given state alone
        return 0.0 if count is None else min(count, NEEDED) / NEEDED
    if _PHOTO["episode"] != _RUN["episode"]:
        _PHOTO.update(episode=_RUN["episode"], count=0)
    if count is not None:
        _PHOTO["count"] = count
    return min(_PHOTO["count"], NEEDED) / NEEDED


def success(objects):
    """Six crossings in the photo; the harness ends the run once this holds for SUCCESS_HOLD_S."""
    count = photo(objects)
    return count is not None and count >= NEEDED


# ---------------------------------------------------------------- run context (simulator state for the demo)
def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()


def _np(x):
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy().astype(float) if hasattr(x, "detach") else np.asarray(x, float)


def grippers():
    """{"left"/"right": (TCP position, TCP rotation)} from the arms' joint angles; None offline."""
    if TEST_GRIPPERS is not None:
        return {k: (np.asarray(p, float), np.asarray(r, float)) for k, (p, r) in TEST_GRIPPERS.items()}
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS
    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        out[arm] = ik.tcp_pose(_np(art.data.joint_pos)[0][ids], station.BASES[arm])
    return out


def strands_now():
    """The sections' live states (None offline)."""
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import scene as arena_scene
    states = arena_scene.object_states(env, list(NAMES))
    for n in NAMES:
        if not (np.isfinite(states[n].centres).all() and np.isfinite(states[n].axes).all()):
            raise ValueError(f"non-finite hair state {n}")
    return states


# ---------------------------------------------------------------- scripted demonstration
# Each crossing is one cycle by one hand: the right hand does the right section's crossings, the left hand the left's.
# CROSS: pinch the outer section at its tip elastic, 22.5 mm from its end, lift it so the chord from its root (its
# furthest crossing, else the gathering elastic) to the hand is 5 mm short of the section's length, swing it over the
# middle section to 65% of the way from the middle section's end toward the far section's, lower it along that arc
# about the root so it lands straight (laid slack, a section bows near the braid and the crossing forms far out),
# pull it 1.5 mm past its length (light tension), let go. SPREAD: pinch the old middle section, now outer, near its
# end and slide it out to 30 degrees about its previous root, which draws the new crossing toward the elastic. The
# demo reads the sections' link poses (simulator state) to pick grasp points and roots; a model under test sees only
# the cameras.
GRASP_LINK = BAND_LINK  # pinch at the tip elastic, 22.5 mm from the end
FRAC = 0.65
CARRY_SLACK, LOWER_SLACK, PULL = 0.005, 0.003, 0.0015
SPREAD_ANGLE = math.radians(30.0)
SPREAD_SLACK = 0.003
PRE = -0.016  # finger target before a pinch (~32 mm between the pads)
HOVER_Z = TOP + 0.070
CARRY_Z = (0.075, 0.130)  # carry height range above the table (0.4 x the section's free length, clipped)
TCP_OVER_ROPE = 0.0095  # pinch height: TCP this far above the section's centre-line (pad tips 1.5 mm above table)
PARK = {"left": np.array((0.15, 0.29, TOP + 0.19)), "right": np.array((0.15, -0.29, TOP + 0.19))}
START = [("to park", 1.5)]
CROSS = [("hover", 1.0), ("descend", 0.6), ("close", 0.45), ("lift", 0.6), ("carry1", 0.45), ("carry2", 0.45),
         ("carry3", 0.45), ("lower1", 0.4), ("lower2", 0.4), ("pull", 0.35), ("open", 0.4), ("rise", 0.4)]
SPREAD = [("s_hover", 0.8), ("s_descend", 0.5), ("s_close", 0.45), ("s_slide1", 0.5), ("s_slide2", 0.5),
          ("s_open", 0.35), ("s_rise", 0.35), ("retreat", 0.8)]
CYCLE = CROSS + SPREAD
FINAL_PARK_S = 3.5  # the hands are clear from ~59.5 s, success holds by ~65 s; the policy stops 3 s after the plan


def cycles():
    return "RLRLRL" if DEMO_WAY == "plait" else "RRRRRR"


def lane_dir(a):
    """Unit xy direction at angle a from straight toward the robot (positive to the robot's left)."""
    return np.array((-math.cos(a), math.sin(a)))


def angle_from(origin, p):
    v = np.asarray(p)[:2] - origin
    return math.atan2(v[1], -v[0]), float(np.linalg.norm(v))


def point_at(poly, s):
    i = int(min(max(s, 0.0), len(poly) - 1.001))
    return poly[i] + (s - i) * (poly[i + 1] - poly[i])


def root_of(states, name, skip_with=None):
    """Where a section leaves the braid: its crossing furthest along it (arc in links from the elastic end), else the
    elastic (arc 1.0). skip_with: ignore its newest crossing with that section (the one just made)."""
    found = plait_word.read({n: states[n].polyline() for n in NAMES}, PLAIT_START, AXIS, NAMES)["crossings"]
    found = [c for c in found if name in c["arc"]]
    if skip_with is not None:
        mine = [c for c in found if skip_with in c["arc"]]
        if mine:
            newest = max(mine, key=lambda c: c["arc"][name])
            found = [c for c in found if c is not newest]
    s = max([c["arc"][name] for c in found] + [1.0])
    return s, point_at(states[name].polyline(), s)


def grasp_pose(state):
    c, a = state.centres[GRASP_LINK], state.axes[GRASP_LINK]
    yaw = -math.atan2(a[1], -a[0])
    while yaw > math.pi / 2:
        yaw -= math.pi
    while yaw <= -math.pi / 2:
        yaw += math.pi
    return np.array((c[0], c[1], max(TOP + 0.0145, c[2] + TCP_OVER_ROPE))), yaw


def plan_cross(states, side):
    """Waypoints {phase: (TCP position, yaw, finger)} laying the outer section on `side` over the middle one."""
    ends = {n: states[n].centres[GRASP_LINK][:2] for n in NAMES}
    order = sorted(NAMES, key=lambda n: ends[n][1])  # robot's right .. left
    S, mid, far = (order[0], order[1], order[2]) if side == "R" else (order[2], order[1], order[0])
    s_root, root = root_of(states, S)
    free = (GRASP_LINK + 0.5 - s_root) * PITCH  # section length from its root to the pinch
    g, yaw_g = grasp_pose(states[S])
    th_g, d_g = angle_from(root[:2], g)
    th_t, _ = angle_from(root[:2], ends[mid] + FRAC * (ends[far] - ends[mid]))
    z_c = TOP + float(np.clip(0.40 * free, *CARRY_Z))

    def radius(z_tcp, chord):  # horizontal distance at which the root -> pinch chord is `chord` long
        h = (z_tcp - TCP_OVER_ROPE) - root[2]
        return math.sqrt(max(chord ** 2 - h ** 2, (0.5 * chord) ** 2))

    r_c = radius(z_c, free - CARRY_SLACK)
    wp = {"hover": (np.array((g[0], g[1], HOVER_Z)), yaw_g, PRE), "descend": (g, yaw_g, PRE),
          "close": (g, yaw_g, FINGER_CLOSED)}
    xy = root[:2] + lane_dir(th_g) * min(r_c, d_g)
    wp["lift"] = (np.array((xy[0], xy[1], z_c)), yaw_g, FINGER_CLOSED)
    for i, f in enumerate((1 / 3, 2 / 3, 1.0)):
        xy = root[:2] + lane_dir(th_g + (th_t - th_g) * f) * r_c
        wp[f"carry{i + 1}"] = (np.array((xy[0], xy[1], z_c)), yaw_g + (-th_t - yaw_g) * f, FINGER_CLOSED)
    for name, z in (("lower1", 0.5 * (z_c + g[2])), ("lower2", g[2])):
        xy = root[:2] + lane_dir(th_t) * radius(z, free - LOWER_SLACK)
        wp[name] = (np.array((xy[0], xy[1], z)), -th_t, FINGER_CLOSED)
    xy = root[:2] + lane_dir(th_t) * (free + PULL)
    low = np.array((xy[0], xy[1], g[2]))
    wp["pull"] = (low, -th_t, FINGER_CLOSED)
    wp["open"] = (low, -th_t, PRE)
    wp["rise"] = (np.array((low[0], low[1], HOVER_Z)), -th_t, PRE)
    return wp, S, mid


def plan_spread(states, side, S, T, hand):
    """Waypoints sliding the old middle section T, now outer on `side`, out to the spread angle about its previous
    root; None if it has under 50 mm free (nothing to open)."""
    _, pivot = root_of(states, T, skip_with=S)
    g, yaw_g = grasp_pose(states[T])
    th_T, d_T = angle_from(pivot[:2], g)
    if d_T < 0.05:
        return None
    th_o = -SPREAD_ANGLE if side == "R" else SPREAD_ANGLE
    wp = {"s_hover": (np.array((g[0], g[1], HOVER_Z)), yaw_g, PRE), "s_descend": (g, yaw_g, PRE),
          "s_close": (g, yaw_g, FINGER_CLOSED)}
    for i, f in enumerate((0.5, 1.0)):
        xy = pivot[:2] + lane_dir(th_T + (th_o - th_T) * f) * (d_T - SPREAD_SLACK)
        wp[f"s_slide{i + 1}"] = (np.array((xy[0], xy[1], g[2] + 0.003)), yaw_g + (-th_o - yaw_g) * f, FINGER_CLOSED)
    end = wp["s_slide2"][0]
    wp["s_open"] = (end, -th_o, PRE)
    wp["s_rise"] = (np.array((end[0], end[1], HOVER_Z)), -th_o, PRE)
    wp["retreat"] = (PARK[hand], 0.0, FINGER_OPEN)
    return wp


class Plan:
    """Each crossing and spread is planned from the sections as they lie when its hand starts it."""

    def __init__(self):
        self.episode, self.cache = None, {}

    def part(self, kind, k):
        if self.episode != _RUN["episode"]:
            self.episode, self.cache = _RUN["episode"], {}
        if (kind, k) not in self.cache:
            try:
                self.cache[(kind, k)] = self._resolve(kind, k)
            except Exception:  # noqa: BLE001  no live state (offline) or a bad read: that hand parks
                self.cache[(kind, k)] = None
        return self.cache[(kind, k)]

    def _resolve(self, kind, k):
        side = cycles()[k]
        hand = "right" if side == "R" else "left"
        if kind == "cross":
            wp, S, mid = plan_cross(strands_now(), side)
            return dict(hand=hand, wp=wp, S=S, mid=mid)
        cross = self.part("cross", k)
        if cross is None or k == len(cycles()) - 1:  # nothing to open after the last crossing
            return None
        wp = plan_spread(strands_now(), side, cross["S"], cross["mid"], hand)
        return None if wp is None else dict(hand=hand, wp=wp)

    def target(self, arm, k, phase):
        part = self.part("cross" if phase in dict(CROSS) else "spread", k) if k >= 0 else None
        if part is None or part["hand"] != arm:
            return PARK[arm], 0.0, FINGER_OPEN
        return part["wp"][phase]


PLAN = Plan()


class SlotWaypoint(Waypoint):
    """A waypoint whose target is planned when the arm reaches it."""

    def __init__(self, arm, k, phase, duration):
        self.arm, self.k, self.phase, self.duration, self.label = arm, k, phase, duration, f"c{k} {phase}"

    def _get(self):
        return PLAN.target(self.arm, self.k, self.phase)

    @property
    def pos(self):
        return tuple(float(v) for v in self._get()[0])

    @property
    def yaw(self):
        return float(self._get()[1])

    @property
    def finger(self):
        return float(self._get()[2])


def demo():
    plans = {"left": [], "right": []}
    for arm in plans:
        plans[arm] += [SlotWaypoint(arm, -1, name, d) for name, d in START]
        for k in range(len(cycles())):
            plans[arm] += [SlotWaypoint(arm, k, name, d) for name, d in CYCLE]
        plans[arm].append(SlotWaypoint(arm, -1, "final park", FINAL_PARK_S))
    return plans
