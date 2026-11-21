"""On the station table: a two-cell pool water tester standing on its closed kit box, both cells filled to the line
with the water sample; the tester's loose cover lying beside it; and two squeeze dropper bottles of reagent lying on
their sides with their caps off: the chlorine reagent (yellow) to the robots' left, the pH reagent (red) to their right.

The task: five drops of the chlorine reagent into the tester's left cell and five drops of the pH reagent into its
right cell, each with the bottle held nose-down; then the cover over both cells, and the tester turned over and back
three times with the cover held on, to mix; then the tester stood back on the box.

Simulation notes
- A bottle is one part with two sprung wall pads (slides). A light grip holds it; a firmer one squeezes its walls.
- The liquid is a scored model, not a fluid (class Drops). With the nose down and the squeeze above the opening force,
  liquid gathers at the tip at a rate set by the squeeze; a drop falls when one drop's volume has gathered; a partly
  formed drop is drawn back when the squeeze is let off. So one short squeeze gives one drop. A drop falls straight
  down: it lands in a cell if it is let go above that cell's opening from no more than 5 cm, else on the tester's
  top or on the table.
- The cover is a loose plate with a sprung lip at each end, standing 2 mm proud of the tester's ends. Fingers closing
  on the tester's ends at its top press the lips in first, so they hold the cover and the tester together.
- Mixing is state too (class Mix): a turn counts when the covered tester is tipped at least 105 degrees from upright (15
  degrees past level, so the sample runs to the cover) and brought back upright. Tipping an uncovered tester past 45
  degrees loses the sample.
- Thin display-only shapes tint a cell's water once drops have gone in (pale until mixed), and mark spilled drops.
- A step hook prints lines starting "WT|" (state every 5 steps) and "WT!" (events) for tuning.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HZ = 20.0  # control steps per second

CASE_BLUE = (0.10, 0.24, 0.58)
CASE_DARK = (0.06, 0.15, 0.40)
CLEAR = (0.86, 0.90, 0.94)
WATER = (0.74, 0.86, 0.97)
WHITE = (0.96, 0.96, 0.95)
LDPE = (0.92, 0.92, 0.88)
YELLOW = (0.95, 0.78, 0.08)
RED = (0.80, 0.10, 0.12)

# -- the kit case, used as a stand ---------------------------------------------------------------------------------------
CASE_SIZE = (0.083, 0.102, 0.044)  # the kit's closed box
CASE_XY = (0.40, 0.0)
CASE_POS = (CASE_XY[0], CASE_XY[1], TOP + CASE_SIZE[2] / 2 + 0.0005)
CASE = part(
    "case", pos=CASE_POS, mass=0.06, friction=0.8,
    boxes=[((0.0, 0.0, 0.0), CASE_SIZE, CASE_BLUE),
           ((0.0, 0.0, 0.004), (CASE_SIZE[0] + 0.0006, CASE_SIZE[1] + 0.0006, 0.002), CASE_DARK)],  # the lid's seam
)

# -- the tester: two cells either side of a panel of printed colour standards ----------------------------------------------
# Tester frame: origin at its centre, x away from the robots, y to their left, z up. Left cell (+y): chlorine.
T_D, T_W, T_H = 0.022, 0.066, 0.060  # depth, width, height
FLOOR_T = 0.004
CELL_Y = 0.022  # the cells' centres either side of the middle
CELL_HALF = 0.007  # half the cell's 14 mm square opening
WATER_H = 0.042  # the sample fills each cell to the line
RIM_Z = T_H / 2  # the cells' rims, in the tester frame
CELLS = {"left": +1, "right": -1}
REAGENT_OF_CELL = {"left": "cl", "right": "ph"}
TESTER_POS = (CASE_XY[0], CASE_XY[1], TOP + CASE_SIZE[2] + T_H / 2 + 0.001)
_WALL_H = T_H - FLOOR_T
_WATER_C = (-0.002, -T_H / 2 + FLOOR_T + WATER_H / 2)  # the water's centre: x and z (it shows at the open front)
_WATER_SIZE = (0.018, 2 * CELL_HALF, WATER_H)


def _standards():
    """Five printed colour standards per test, on the panel's front face."""
    out = []
    for s, lo, hi in ((+1, (0.99, 0.97, 0.75), (0.93, 0.70, 0.05)), (-1, (0.98, 0.85, 0.55), (0.80, 0.15, 0.35))):
        for i in range(5):
            k = i / 4
            colour = tuple(a + k * (b - a) for a, b in zip(lo, hi))
            out.append(((-(T_D / 2 + 0.0002), s * 0.008, -0.016 + 0.008 * i), (0.0006, 0.010, 0.006), colour))
    return out


def _cell_boxes(s):
    return [
        ((0.009, s * CELL_Y, FLOOR_T / 2), (0.004, 2 * CELL_HALF, _WALL_H), CLEAR),  # back wall
        ((0.0, s * 0.031, FLOOR_T / 2), (T_D, 0.004, _WALL_H), CLEAR),  # outer wall
        ((_WATER_C[0], s * CELL_Y, _WATER_C[1]), _WATER_SIZE, WATER),  # the water sample
    ]


# Display-only shapes. On the body: per cell a pale tint (1 to 4 drops of its own reagent), a full tint (exactly 5) and a
# murky one (the wrong reagent, or more than 5). At the root: marks for spilled drops, three per reagent.
_TINTS = {"left": ((0.98, 0.95, 0.62), (0.97, 0.84, 0.12), (0.62, 0.48, 0.22)),
          "right": ((0.98, 0.82, 0.80), (0.90, 0.28, 0.40), (0.55, 0.36, 0.30))}
_TINT_SIZE = tuple(v + 0.0005 for v in _WATER_SIZE)
SPILL_MARKS = 3  # per reagent
labkit.DISPLAY["tester"] = {
    "body": [((_WATER_C[0], s * CELL_Y, _WATER_C[1]), _TINT_SIZE, colour) for cell, s in CELLS.items() for colour in _TINTS[cell]],
    "root": [((0.0, 0.0, -1.0), (0.005, 0.005, 0.0006), colour) for colour in (YELLOW, RED) for _ in range(SPILL_MARKS)],
}
TESTER = part(
    "tester", pos=TESTER_POS, mass=0.045, friction=0.8,
    boxes=[((0.0, 0.0, -T_H / 2 + FLOOR_T / 2), (T_D, T_W, FLOOR_T), CLEAR),
           ((0.0, 0.0, FLOOR_T / 2), (T_D, 0.030, _WALL_H), WHITE),  # the panel of colour standards
           *_cell_boxes(+1), *_cell_boxes(-1), *_standards()],
)

# -- the cover: a loose plate with a sprung lip at each end -----------------------------------------------------------------
# Cover frame: origin at its centre, x across its depth, y along its length, z up. On the tester it rests on the rims.
COVER_D, COVER_W, COVER_T = T_D, 0.060, 0.012  # depth, the rigid plate's length, thickness
LIP_T, LIP_PROUD, LIP_TRAVEL, LIP_K = 0.002, 0.002, 0.003, 500.0  # each lip: thickness, how far it stands past the tester's end, travel, N/m
COVER_BLUE = (0.42, 0.60, 0.88)
COVER_XY = (0.37, 0.125)
COVER_POS = (COVER_XY[0], COVER_XY[1], TOP + COVER_T / 2 + 0.0005)
COVER_SEAT = np.array([0.0, 0.0, RIM_Z + COVER_T / 2])  # the cover's centre in the tester frame when it is on
LIPS = (("a", +1), ("b", -1))


def _lip(tag, s):
    y = s * (T_W / 2 + LIP_PROUD - LIP_T / 2)
    lo, hi = (-LIP_TRAVEL, 0.0) if s > 0 else (0.0, LIP_TRAVEL)
    return Joint(f"lip_{tag}", kind="slide", axis="Y", anchor=(0.0, y, 0.0), lower=lo, upper=hi, initial=0.0,
                 stiffness=LIP_K, damping=1.0, target=0.0, mass=0.0005,
                 boxes=[((0.0, y, 0.0), (COVER_D, LIP_T, COVER_T), COVER_BLUE)])


COVER = part("cover", pos=COVER_POS, mass=0.004, friction=0.8,
             boxes=[((0.0, 0.0, 0.0), (COVER_D, COVER_W, COVER_T), COVER_BLUE)], joints=[_lip(tag, s) for tag, s in LIPS])

# -- the reagent bottles ---------------------------------------------------------------------------------------------------
# Bottle frame: origin mid-body, x toward the tip, the two wall pads on its +y and -y sides. It lies with its tip toward
# the robots (-x in the world), so a straight-down grasp across the pads, then turning the hand level, leaves it nose-down.
B_R = 0.0135  # body radius (27 mm across)
B_BASE, B_SHOULDER, B_NECK, TIP_X = -0.030, 0.022, 0.030, 0.040  # along the bottle: base, shoulder, neck, tip (70 mm long)
PAD_X = (-0.026, 0.014)  # the pads run along the body
PAD_T, PAD_H, PAD_TRAVEL, PAD_PROUD = 0.003, 0.016, 0.005, 0.001
GRIP_X = (PAD_X[0] + PAD_X[1]) / 2  # where the demo holds it
HALF_GAP = B_R + PAD_PROUD  # from the axis to each pad's outer face at rest: 14.5 mm
WALL_K = 2000.0  # N/m per wall
BOTTLE_X = 0.30
BOTTLE_Y = {"cl": +0.14, "ph": -0.14}
BOTTLE_LOOK = {"cl": YELLOW, "ph": RED}
BOTTLE_Z = TOP + B_R + 0.0005
REAGENTS = ("cl", "ph")
PADS = (("a", +1), ("b", -1))


def _pad(tag, s, colour):
    y = s * (HALF_GAP - PAD_T / 2)
    lo, hi = (-PAD_TRAVEL, 0.0) if s > 0 else (0.0, PAD_TRAVEL)
    return Joint(f"pad_{tag}", kind="slide", axis="Y", anchor=(GRIP_X, y, 0.0), lower=lo, upper=hi, initial=0.0,
                 stiffness=WALL_K, damping=2.0, target=0.0, mass=0.002,
                 boxes=[((GRIP_X, y, 0.0), (PAD_X[1] - PAD_X[0], PAD_T, PAD_H), colour)])


def _bottle(reagent):
    colour = BOTTLE_LOOK[reagent]
    core_half = HALF_GAP - PAD_T - PAD_TRAVEL  # the rigid core's half-width under the pads: 6.5 mm
    return part(
        f"bottle_{reagent}", pos=(BOTTLE_X, BOTTLE_Y[reagent], BOTTLE_Z), mass=0.026, friction=0.8, yaw=math.pi,
        boxes=[((GRIP_X, 0.0, 0.0), (PAD_X[1] - PAD_X[0], 2 * core_half, 2 * B_R), LDPE),
               ((GRIP_X, 0.0, B_R + 0.0002), (0.030, 2 * core_half, 0.0004), colour),  # its label, on the two flat faces
               ((GRIP_X, 0.0, -B_R - 0.0002), (0.030, 2 * core_half, 0.0004), colour)],
        cylinders=[((B_BASE + 0.002, 0.0, 0.0), B_R, 0.004, LDPE, "X"),
                   (((PAD_X[1] + B_SHOULDER) / 2, 0.0, 0.0), B_R, B_SHOULDER - PAD_X[1], LDPE, "X"),
                   (((B_SHOULDER + B_NECK) / 2, 0.0, 0.0), 0.006, B_NECK - B_SHOULDER, LDPE, "X"),
                   (((B_NECK + TIP_X) / 2, 0.0, 0.0), 0.0025, TIP_X - B_NECK, WHITE, "X")],
        joints=[_pad(tag, s, colour) for tag, s in PADS],
    )


BOTTLES = {r: _bottle(r) for r in REAGENTS}
OBJECTS = [CASE, TESTER, COVER, *BOTTLES.values()]

# -- the liquid ------------------------------------------------------------------------------------------------------------
OPEN_N = 3.0  # N: below this squeeze nothing leaves the tip
FLOW_UL = 12.5  # uL per second for each newton of squeeze above OPEN_N
DROP_UL = 50.0  # one drop (20 drops to the millilitre)
DROP_R = 0.0023  # m: a 50 uL drop's radius
NOSE_DOWN_DEG = 20.0  # drops form only with the bottle this close to straight down
DROP_FALL_MAX = 0.050  # m: a drop let go from higher than this above the rims splashes and does not count for a cell
DOSE = 5  # drops of its own reagent a cell needs


class Drops:
    """The liquid as state: what has gathered at each bottle's tip, and where every released drop landed."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.pending = {r: 0.0 for r in REAGENTS}
        self.cells = {c: {r: 0 for r in REAGENTS} for c in CELLS}
        self.spills = []  # (reagent, zone, world point)
        self.events = []

    @staticmethod
    def landing(tip, tester_pos, tester_rot, covered=False):
        """Where a drop leaving ``tip`` lands, falling straight down: ("left" | "right" | "tester" | "table", offset).
        The offset is the tip's distance from the nearer cell's centre, across and along the tester (m). With the
        cover on, a drop over the tester lands on the cover."""
        upright = tester_rot[2, 2] > math.cos(math.radians(10.0))
        local = tester_rot.T @ (np.asarray(tip, float) - np.asarray(tester_pos, float))
        cell = "left" if local[1] >= 0 else "right"
        off = np.array([local[0], local[1] - CELLS[cell] * CELL_Y])
        inside = CELL_HALF - DROP_R
        near = RIM_Z - 0.002 <= local[2] <= RIM_Z + DROP_FALL_MAX  # above the rim, not dipped in, and not from too high
        if upright and near and not covered and abs(off[0]) <= inside and abs(off[1]) <= inside:
            return cell, off
        if upright and local[2] >= 0 and abs(local[0]) <= T_D / 2 and abs(local[1]) <= T_W / 2:
            return "tester", off
        return "table", off

    def step(self, reagent, force, nose_down_deg, tip, tester_pos, tester_rot, covered=False, dt=1.0 / HZ):
        if force < OPEN_N or nose_down_deg > NOSE_DOWN_DEG:
            self.pending[reagent] = 0.0  # a partly formed drop is drawn back in
            return
        self.pending[reagent] += FLOW_UL * (force - OPEN_N) * dt
        while self.pending[reagent] >= DROP_UL:
            self.pending[reagent] -= DROP_UL
            zone, off = self.landing(tip, tester_pos, tester_rot, covered)
            if zone in CELLS:
                self.cells[zone][reagent] += 1
            else:
                self.spills.append((reagent, zone, np.asarray(tip, float).copy()))
            local = tester_rot.T @ (np.asarray(tip, float) - np.asarray(tester_pos, float))
            self.events.append(
                f"drop of {reagent} -> {zone}: tip {1000 * off[0]:+.1f} {1000 * off[1]:+.1f} mm from the nearer cell's centre, "
                f"{1000 * (local[2] - RIM_Z):+.0f} mm above the rims, {nose_down_deg:.0f} deg off straight down, "
                f"squeeze {force:.1f} N | {self.summary()}")

    def dosed(self, cell):
        """Exactly DOSE drops of the cell's own reagent and none of the other."""
        own = REAGENT_OF_CELL[cell]
        return self.cells[cell][own] == DOSE and all(n == 0 for r, n in self.cells[cell].items() if r != own)

    def summary(self):
        return " ".join(f"{c}:" + "/".join(f"{r}{n}" for r, n in self.cells[c].items()) for c in CELLS) + f" spilled:{len(self.spills)}"


TURN_OVER_DEG = 105.0  # a turn counts once the tester's tilt from upright passes this: 15 degrees past level
TURN_BACK_DEG = 20.0  # ... and it is brought back to within this of upright
TURNS = 3  # turns each dosed cell needs to be mixed
SPILL_TILT_DEG = 45.0  # tipping an uncovered tester past this loses the sample
COVER_TOL_XY, COVER_TOL_Z, COVER_TOL_DEG = 0.004, 0.002, 8.0  # how closely the cover must sit to count as on


class Mix:
    """Mixing as state: how many times each dosed cell has been turned over and back under its cover."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.turns = {c: 0 for c in CELLS}
        self.over = False
        self.peak = 0.0
        self.lost = False
        self.events = []

    def step(self, tilt_deg, covered, dosed_cells):
        if tilt_deg > SPILL_TILT_DEG and not covered and not self.lost:
            self.lost = True
            self.events.append(f"sample lost: the tester was tipped {tilt_deg:.0f} deg with its cover off")
        if covered and tilt_deg >= TURN_OVER_DEG and not self.over:
            self.over, self.peak = True, tilt_deg
        if self.over:
            self.peak = max(self.peak, tilt_deg)
            if tilt_deg <= TURN_BACK_DEG:
                self.over = False
                for c in dosed_cells:
                    self.turns[c] += 1
                self.events.append(f"turned over to {self.peak:.0f} deg and back, cover on | turns " + self.summary())

    def mixed(self, cell):
        return self.turns[cell] >= TURNS and not self.lost

    def summary(self):
        return " ".join(f"{c}:{n}" for c, n in self.turns.items()) + (" SAMPLE LOST" if self.lost else "")


_drops = Drops()
_mix = Mix()
_state = {"step": 0, "hist": [], "still": False, "free": False, "down": False, "placed": True, "covered": False, "show": None, "show_failed": False}

LOG_EVERY = 5  # control steps
STILL_STEPS, STILL_TOL = 20, 0.001  # at rest: nothing moved 1 mm over the last 1 s
RELEASE_DIST = 0.030  # m: every fingertip this far from each bottle and from the tester
PLACE_TOL = 0.010  # m: the tester still stands within this of where it started


def _np(x):
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def squeeze_of(bottle):
    """(squeeze force in N, each pad's inward travel in m) from a bottle's state."""
    travel = [max(0.0, -s * bottle.joints[f"pad_{tag}"]) for tag, s in PADS]
    return WALL_K * sum(travel) / len(travel), travel


def nose_of(bottle):
    """(tip position, degrees off straight down) from a bottle's state."""
    axis = bottle.rotation()[:, 0]
    return bottle.to_world((TIP_X, 0.0, 0.0)), math.degrees(math.acos(max(-1.0, min(1.0, -axis[2]))))


def cover_on(cover, tester):
    """The cover lies over both cells: flat on the rims, within 4 mm of centred."""
    off = tester.to_local(cover.pos) - COVER_SEAT
    lean = math.degrees(math.acos(max(-1.0, min(1.0, float(cover.rotation()[:, 2] @ tester.rotation()[:, 2])))))
    return abs(off[0]) <= COVER_TOL_XY and abs(off[1]) <= COVER_TOL_XY and abs(off[2]) <= COVER_TOL_Z and lean <= COVER_TOL_DEG


def _put_down(bottle):
    """Resting on the table, lying or standing (not on the case or the tester, and not in a hand)."""
    return float(bottle.pos[2]) - TOP < 0.045


def _read(env):
    """Everything the hooks need, from the simulator: poses in the table frame, joints, fingertips."""
    from yam_arena import runtime as _rt

    b = env.unwrapped
    origin = _np(b.scene.env_origins[0])
    out = {name: labkit._part_state(env, labkit._SPECS[name]) for name in ("case", "tester", "cover")}
    out["bottles"] = {r: labkit._part_state(env, labkit._SPECS[f"bottle_{r}"]) for r in REAGENTS}
    out["tips"], out["arms"] = [], {}
    for arm in ("left", "right"):
        robot = b.scene[f"{arm}_robot"]
        g = _rt.link_poses(robot)["gripper"]
        tcp = g[:3, :3] @ np.array(TCP_OFFSET) + g[:3, 3] - origin
        ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        fingers = _np(robot.data.joint_pos)[0][list(ids)]
        mid = tcp + g[:3, 2] * PAD_TIP_PAST_TCP
        out["tips"] += [mid + abs(fingers[0]) * g[:3, 0], mid - abs(fingers[1]) * g[:3, 0]]
        out["arms"][arm] = (tcp, g[:3, :3], fingers)
    return out


def _box_distance(p, centre, size):
    d = np.abs(np.asarray(p) - np.asarray(centre)) - np.asarray(size) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)) + min(float(d.max()), 0.0))


_BOTTLE_BOX = (((B_BASE + TIP_X) / 2, 0.0, 0.0), (TIP_X - B_BASE, 2 * HALF_GAP, 2 * B_R))
_TESTER_BOX = ((0.0, 0.0, 0.0), (T_D, T_W, T_H))
_COVER_BOX = ((0.0, 0.0, 0.0), (COVER_D, T_W + 2 * LIP_PROUD, COVER_T))


def _update(read):
    """One control step of the liquid and mixing models and of the at-rest and let-go checks."""
    tester, cover = read["tester"], read["cover"]
    t_pos, t_rot = np.asarray(tester.pos, float), tester.rotation()
    covered = cover_on(cover, tester)
    for r, bottle in read["bottles"].items():
        force, _ = squeeze_of(bottle)
        tip, off_down = nose_of(bottle)
        _drops.step(r, force, off_down, tip, t_pos, t_rot, covered)
    _mix.step(tester.tilt_deg(), covered, dosed())
    free = True
    for tip in read["tips"]:
        free = free and _box_distance(tester.to_local(tip), *_TESTER_BOX) >= RELEASE_DIST
        free = free and _box_distance(cover.to_local(tip), *_COVER_BOX) >= RELEASE_DIST
        for bottle in read["bottles"].values():
            free = free and _box_distance(bottle.to_local(tip), *_BOTTLE_BOX) >= RELEASE_DIST
    _state["free"] = free
    _state["covered"] = covered
    _state["down"] = all(_put_down(b) for b in read["bottles"].values())
    _state["placed"] = tester.tilt_deg() < 10.0 and float(np.linalg.norm(t_pos - np.array(TESTER_POS))) < PLACE_TOL
    h = _state["hist"]
    h.append(np.concatenate([t_pos, np.asarray(cover.pos, float)] + [np.asarray(b.pos, float) for b in read["bottles"].values()]))
    del h[:-STILL_STEPS]
    _state["still"] = len(h) == STILL_STEPS and max(float(np.abs(p - h[-1]).max()) for p in h) < STILL_TOL


# -- drawing ---------------------------------------------------------------------------------------------------------------
def _find_shows():
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    want = {f"tint:{i}": f"/tester/body/show_{i}" for i in range(3 * len(CELLS))}
    want.update({f"spill:{i}": f"/tester/show_{i}" for i in range(SPILL_MARKS * len(REAGENTS))})
    found = {}
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        for key, tail in want.items():
            if path.endswith(tail):
                found.setdefault(key, []).append(prim)
    missing = sorted(set(want) - set(found))
    if missing:
        raise RuntimeError(f"display shapes not found on the stage: {missing}")
    return found


def _place(prims, pos):
    from pxr import Gf, UsdGeom

    for prim in prims:
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            attr = op.GetAttr()
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                attr.Set(Gf.Vec3f(*map(float, pos)) if attr.GetTypeName() == "float3" else Gf.Vec3d(*map(float, pos)))
        UsdGeom.Imageable(prim).MakeVisible()


def _draw():
    """Show the liquid as it stands. Drawing never stops a run: a failure is reported once and the task carries on."""
    if _state["show_failed"]:
        return
    try:
        from pxr import UsdGeom

        if _state["show"] is None:
            _state["show"] = _find_shows()
        shows = _state["show"]
        used = set()
        for ci, cell in enumerate(CELLS):
            own = REAGENT_OF_CELL[cell]
            n_own = _drops.cells[cell][own]
            n_other = sum(n for r, n in _drops.cells[cell].items() if r != own)
            which = None
            if n_other or n_own > DOSE or (_mix.lost and n_own):
                which = 2  # murky: a wrong or extra drop, or the sample lost
            elif n_own == DOSE and _mix.mixed(cell):
                which = 1  # the full test colour, once dosed and mixed
            elif n_own:
                which = 0  # pale and streaky until mixed
            if which is not None:
                used.add(f"tint:{3 * ci + which}")
        seen = {r: 0 for r in REAGENTS}
        for reagent, zone, tip in _drops.spills:
            i = seen[reagent]
            seen[reagent] += 1
            if i >= SPILL_MARKS:
                continue
            key = f"spill:{SPILL_MARKS * REAGENTS.index(reagent) + i}"
            z = TESTER_POS[2] + RIM_Z if zone == "tester" else TOP
            inside = abs(tip[0] - CASE_POS[0]) <= CASE_SIZE[0] / 2 and abs(tip[1] - CASE_POS[1]) <= CASE_SIZE[1] / 2
            if zone == "table" and inside:
                z = TOP + CASE_SIZE[2]
            _place(shows[key], (tip[0] - TESTER_POS[0], tip[1] - TESTER_POS[1], z - TESTER_POS[2] + 0.0006))
            used.add(key)
        for key, prims in shows.items():
            for prim in prims:
                if key in used:
                    UsdGeom.Imageable(prim).MakeVisible()
                else:
                    UsdGeom.Imageable(prim).MakeInvisible()
    except Exception as e:  # noqa: BLE001
        _state["show_failed"] = True
        print(f"WT! drawing the liquid failed, carrying on without: {type(e).__name__}: {e}", flush=True)


@on_reset
def _reset(env):
    _drops.reset()
    _mix.reset()
    _state.update(step=0, hist=[], still=False, free=False, down=False, placed=True, covered=False)
    _draw()
    print(
        f"WT! reset: pads {1000 * 2 * HALF_GAP:.1f} mm apart at rest, hold at finger {1000 * FINGER_HOLD:.2f} mm, squeeze at finger "
        f"{1000 * FINGER_SQUEEZE:.2f} mm (aiming at {SQUEEZE_N:.1f} N), opens at {OPEN_N:.1f} N, {FLOW_UL:.1f} uL/s per N, "
        f"drop {DROP_UL:.0f} uL, nose within {NOSE_DOWN_DEG:.0f} deg of straight down",
        flush=True,
    )


@on_step
def _watch(env):
    _state["step"] += 1
    read = _read(env)
    seen, seen_mix = len(_drops.events), len(_mix.events)
    _update(read)
    for text in _drops.events[seen:] + _mix.events[seen_mix:]:
        print(f"WT! step {_state['step']:4d} {text}", flush=True)
    if len(_drops.events) > seen or len(_mix.events) > seen_mix:
        _draw()
    if _state["step"] % LOG_EVERY:
        return
    tester = read["tester"]
    parts = []
    for r, bottle in read["bottles"].items():
        force, travel = squeeze_of(bottle)
        tip, off_down = nose_of(bottle)
        zone, off = Drops.landing(tip, np.asarray(tester.pos, float), tester.rotation(), _state["covered"])
        parts.append(
            f"{r} F {force:4.1f} pads {1000 * travel[0]:.2f} {1000 * travel[1]:.2f} tip {tip[0]:+.3f} {tip[1]:+.3f} {tip[2] - TOP:+.3f} "
            f"off_down {off_down:5.1f} over {zone} {1000 * off[0]:+.1f} {1000 * off[1]:+.1f} pend {_drops.pending[r]:4.1f}")
    rf, lf = read["arms"]["right"][2], read["arms"]["left"][2]
    moved = 1000 * float(np.linalg.norm(np.asarray(tester.pos, float) - np.array(TESTER_POS)))
    cover = read["cover"]
    c_off = 1000 * (tester.to_local(cover.pos) - COVER_SEAT)
    lips = [1000 * max(0.0, -sg * cover.joints[f"lip_{tag}"]) for tag, sg in LIPS]
    rtcp, rrot, _ = read["arms"]["right"]
    in_grip = 1000 * (rrot.T @ (tester.to_world((0.0, 0.0, RIM_Z)) - rtcp))  # the tester's top centre seen from the right gripper
    print(
        f"WT| step {_state['step']:4d} " + " | ".join(parts) + f" | R {1000 * rf[0]:+.2f} {1000 * rf[1]:+.2f} L {1000 * lf[0]:+.2f} {1000 * lf[1]:+.2f} "
        f"| tester moved {moved:.1f} mm tilt {tester.tilt_deg():5.1f} in_R {in_grip[0]:+.1f} {in_grip[1]:+.1f} {in_grip[2]:+.1f} | cover off_seat {c_off[0]:+.1f} {c_off[1]:+.1f} {c_off[2]:+.1f} "
        f"lips {lips[0]:.2f} {lips[1]:.2f} on {int(_state['covered'])} | {_drops.summary()} | turns {_mix.summary()} | free {int(_state['free'])} "
        f"down {int(_state['down'])} placed {int(_state['placed'])} still {int(_state['still'])} | score {score({}):.2f}",
        flush=True,
    )


# -- scoring ---------------------------------------------------------------------------------------------------------------
def dosed():
    """The cells holding exactly five drops of their own reagent and nothing else."""
    return [c for c in CELLS if _drops.dosed(c)]


def score(objects):
    """0 if the sample was lost. 0.25: one cell holds exactly five drops of its own reagent and nothing else. 0.5: both
    cells do, and no drop landed anywhere else. 0.75: also both cells are mixed: the covered tester turned over and
    back three times. 1.0: also the tester stands upright where it was with its cover on, both bottles rest on the
    table, everything is let go with every fingertip 3 cm or more away, and nothing has moved for 1 s."""
    if _mix.lost:
        return 0.0
    n = len(dosed())
    if n < len(CELLS) or _drops.spills:
        return 0.25 * min(n, 1)
    if not all(_mix.mixed(c) for c in CELLS):
        return 0.5
    if _state["covered"] and _state["down"] and _state["placed"] and _state["free"] and _state["still"]:
        return 1.0
    return 0.75


def success(objects):
    return score(objects) == 1.0


# -- scripted demo ---------------------------------------------------------------------------------------------------------
class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


_down_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)  # yam_arena's own: straight down, turned by yaw
if not hasattr(_demo.grip_rotation, "_plain"):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)  # any waypoint yaw that carries a rotation, whichever module made it
        return _down_grip(yaw) if rot is None else rot

    _grip_rotation._plain = _down_grip
    _demo.grip_rotation = _grip_rotation


def _turn(axis, deg):
    return axis_angle_matrix(np.array(axis, float), math.radians(deg))


DOWN = _down_grip(0.0)  # straight down, fingers closing along world y: one on each wall pad of a lying bottle
LEVEL = _turn((0, 1, 0), -90.0) @ DOWN  # level, pointing forward (+x): the bottle hangs nose-down under the hand

# The grip. The pads sit HALF_GAP from the axis; a finger commanded d inside a pad moves it d / (1 + WALL_K / 5000).
FINGER_K = 5000.0  # N/m: each finger's position drive
HOLD_N, SQUEEZE_N = 1.5, 7.0  # N per wall: holding the 30 g bottle, and giving a drop
_GAIN = 1.0 + WALL_K / FINGER_K
FINGER_HOLD = -(HALF_GAP - _GAIN * HOLD_N / WALL_K)
FINGER_SQUEEZE = -(HALF_GAP - _GAIN * SQUEEZE_N / WALL_K)
# The cover is taken by its two ends with the hand pitched 20 degrees forward of straight down: that grip reaches both
# the table and the tester's top (the straight-down one does not reach over the tester).
COVER_GRIP = _turn((0, 1, 0), -20.0) @ DOWN
COVER_N = 3.0  # N per finger on the cover's ends; its lips are pressed right in
FINGER_COVER = -(T_W / 2 + LIP_PROUD - LIP_TRAVEL - COVER_N / FINGER_K)
PAD_HALF_ACROSS = 0.0137  # m: half the finger pad's width


def _cover_grasp_offset():
    """The TCP relative to the cover's centre for that grip: the pads' lowest corners 2 mm above the cover's underside,
    and the patch where each pad meets the cover's end face centred on it."""
    rot = COVER_GRIP
    corners = [rot @ np.array([0.0, a, PAD_TIP_PAST_TCP]) for a in (-PAD_HALF_ACROSS, PAD_HALF_ACROSS)]
    dz = -COVER_T / 2 + 0.002 - min(c[2] for c in corners)
    dx = 0.0
    for _ in range(4):
        pts = np.array([rot @ np.array([0.0, a, b]) for a in np.linspace(-PAD_HALF_ACROSS, PAD_HALF_ACROSS, 28) for b in np.linspace(-0.030, PAD_TIP_PAST_TCP, 44)])
        pts = pts + np.array([dx, 0.0, dz])
        on = pts[(np.abs(pts[:, 0]) <= COVER_D / 2) & (np.abs(pts[:, 2]) <= COVER_T / 2)]
        dx -= float(on[:, 0].mean())
    return np.array([dx, 0.0, dz])


COVER_GRASP_OFF = _cover_grasp_offset()
TESTER_N = 5.0  # N per finger closing on the tester's ends at its top: about 1 N presses each lip in, the rest holds the tester
FINGER_TESTER = -(T_W / 2 - TESTER_N / FINGER_K)
MIX_ROLL = 114.0  # degrees the wrist rolls each way to turn the tester over (the joint's limit is 120)
MIX_LIFT = 0.075  # m the tester is lifted off the box to be turned
PAD_CLEAR = 0.0035  # m: the finger pads' tips above the table when they close on a lying bottle
AXIS_PAST_TCP = PAD_CLEAR + PAD_TIP_PAST_TCP - (B_R + 0.0005)  # the bottle's axis lies this far beyond the TCP
TIP_CLEAR = 0.015  # m: the tip above the cells' rims while dosing
RIM_WORLD = TESTER_POS[2] + RIM_Z


def grasp_tcp(reagent):
    """The TCP for the straight-down grasp of a lying bottle at its start."""
    return np.array([BOTTLE_X - GRIP_X, BOTTLE_Y[reagent], TOP + PAD_CLEAR + PAD_TIP_PAST_TCP])


def tip_from_tcp(rot):
    """The bottle's tip relative to the TCP, in the world, for a bottle grasped lying and now held with TCP rotation
    ``rot``."""
    at_grasp = np.array([-(TIP_X - GRIP_X), 0.0, -AXIS_PAST_TCP])  # in the world, with the DOWN grip
    return np.asarray(rot, float) @ (DOWN.T @ at_grasp)


def dose_tcp(cell):
    """The TCP that holds the tip TIP_CLEAR above the centre of a cell, in the level grip."""
    centre = np.array([TESTER_POS[0], TESTER_POS[1] + CELLS[cell] * CELL_Y, RIM_WORLD + TIP_CLEAR])
    return centre - tip_from_tcp(LEVEL)


SQUEEZE_S = (0.5, 1.15, 0.5, 0.3)  # one squeeze: close, hold, ease off, wait (s). About 75 uL: one drop, the rest drawn back
ARM_OF = {"ph": "r", "cl": "l"}  # each arm works the bottle and the cell on its own side
CELL_OF = {"ph": "right", "cl": "left"}


def roll(deg):
    """The level grip rolled about its approach axis: what turns a tester held by its ends over sideways."""
    return _turn((1, 0, 0), deg) @ LEVEL


def demo():
    """Both arms pick the bottle on their own side up from above by its wall pads, lift it, and turn the hand level so
    the bottle hangs nose-down. The right arm carries its tip over the right cell and gives five short squeezes, then
    draws back; the left arm does the same over the left cell. Both lay their bottles back where they were. The left
    arm then picks the cover up from above and sets it over both cells. The right arm comes down level over the
    tester, closes on its ends at the top (the cover's lips first), lifts it, rolls it over to one side and back three
    times, stands it back on the box and lets go."""
    right, left = [], []
    cur = {"r": None, "l": None}

    def step(t, label, r=None, l=None):
        if r is not None:
            cur["r"] = r
        if l is not None:
            cur["l"] = l
        for arm, plan in (("r", right), ("l", left)):
            pos, finger, rot = cur[arm]
            plan.append(Waypoint(float(t), tuple(float(v) for v in pos), float(finger), label, rot))

    lv = Orient(LEVEL)
    grasp = {r: grasp_tcp(r) for r in REAGENTS}
    above = {r: grasp[r] + np.array([0.0, 0.0, 0.10]) for r in REAGENTS}
    lifted = {r: grasp[r] + np.array([0.0, 0.0, 0.12]) for r in REAGENTS}  # the straight-down grip runs out of reach above about 16 cm
    side = {r: math.copysign(1.0, BOTTLE_Y[r]) for r in REAGENTS}
    turn = {r: np.array([0.30, side[r] * 0.17, TOP + 0.19]) for r in REAGENTS}  # where each hand turns level and waits, clear of the other arm
    over = {r: dose_tcp(CELL_OF[r]) for r in REAGENTS}
    high = {r: over[r] + np.array([-0.03, 0.0, 0.04]) for r in REAGENTS}

    def both(t, label, at, finger, rot):
        step(t, label, **{ARM_OF[r]: (at[r], finger, rot) for r in REAGENTS})

    both(2.5, "over the bottles", above, FINGER_OPEN, 0.0)
    both(1.5, "fingers either side of each bottle", grasp, FINGER_OPEN, 0.0)
    both(1.0, "close on their walls, lightly", grasp, FINGER_HOLD, 0.0)
    both(2.0, "lift them", lifted, FINGER_HOLD, 0.0)
    both(3.0, "turn the hands level: the bottles hang nose-down", turn, FINGER_HOLD, lv)
    for r in ("ph", "cl"):
        a, name = ARM_OF[r], ("pH" if r == "ph" else "chlorine")
        step(2.0, f"carry the {name} bottle toward the {CELL_OF[r]} cell", **{a: (high[r], FINGER_HOLD, lv)})
        step(1.5, "lower the tip over the cell", **{a: (over[r], FINGER_HOLD, lv)})
        step(0.8, "steady", **{a: (over[r], FINGER_HOLD, lv)})
        for i in range(DOSE):
            step(SQUEEZE_S[0], f"squeeze: {name} drop {i + 1}", **{a: (over[r], FINGER_SQUEEZE, lv)})
            step(SQUEEZE_S[1], "hold the squeeze", **{a: (over[r], FINGER_SQUEEZE, lv)})
            step(SQUEEZE_S[2], "ease off", **{a: (over[r], FINGER_HOLD, lv)})
            step(SQUEEZE_S[3], "wait", **{a: (over[r], FINGER_HOLD, lv)})
        step(1.5, "lift the tip away", **{a: (high[r], FINGER_HOLD, lv)})
        step(2.0, "draw the bottle back", **{a: (turn[r], FINGER_HOLD, lv)})
    both(3.0, "turn the hands down: the bottles lie level", lifted, FINGER_HOLD, 0.0)
    down = {r: grasp[r] + np.array([0.0, 0.0, 0.001]) for r in REAGENTS}
    both(2.0, "lower them", down, FINGER_HOLD, 0.0)
    both(0.8, "let go", down, FINGER_OPEN, 0.0)
    both(1.5, "clear", above, FINGER_OPEN, 0.0)

    # the cover, with the left arm
    cg = Orient(COVER_GRIP)
    cover_at = np.array(COVER_POS) + COVER_GRASP_OFF
    seat = np.array([TESTER_POS[0], TESTER_POS[1], RIM_WORLD + COVER_T / 2 + 0.0005]) + COVER_GRASP_OFF
    park = {r: np.array([0.25, side[r] * 0.28, TOP + 0.16]) for r in REAGENTS}  # each hand's place well out of the other's way
    step(2.5, "left hand over the cover; right hand out of the way", l=(cover_at + np.array([0.0, 0.0, 0.08]), FINGER_OPEN, cg),
         r=(park["ph"], FINGER_OPEN, 0.0))
    step(1.5, "fingers at the cover's two ends", l=(cover_at, FINGER_OPEN, cg))
    step(0.8, "close on the cover", l=(cover_at, FINGER_COVER, cg))
    step(1.5, "lift it", l=(cover_at + np.array([0.0, 0.0, 0.08]), FINGER_COVER, cg))
    step(2.5, "carry it over the tester", l=(seat + np.array([0.0, 0.0, 0.03]), FINGER_COVER, cg))
    step(1.5, "lower it onto the cells", l=(seat, FINGER_COVER, cg))
    step(0.8, "let go of the cover", l=(seat, FINGER_OPEN, cg))
    step(1.5, "lift the left hand clear", l=(seat + np.array([0.0, 0.0, 0.04]), FINGER_OPEN, cg))
    step(2.5, "left hand out of the way; right hand turns level", l=(park["cl"], FINGER_OPEN, 0.0), r=(turn["ph"], FINGER_OPEN, lv))

    # mixing, with the right arm
    hold_at = np.array([TESTER_POS[0] - 0.002, TESTER_POS[1], RIM_WORLD])  # the fingers span the tester's top and the cover
    pre = hold_at + np.array([0.0, 0.0, 0.05])
    mix_at = hold_at + np.array([0.0, 0.0, MIX_LIFT])
    step(2.5, "right hand level, above the tester", r=(pre, FINGER_OPEN, lv))
    step(1.5, "fingers down either side of the tester's top", r=(hold_at, FINGER_OPEN, lv))
    step(1.0, "close on the cover's lips and the tester's ends", r=(hold_at, FINGER_TESTER, lv))
    step(1.5, "lift the tester", r=(mix_at, FINGER_TESTER, lv))
    for i, sign in enumerate((+1, -1, +1)):
        turned = Orient(roll(sign * MIX_ROLL))
        step(2.2, f"turn it over: {i + 1}", r=(mix_at, FINGER_TESTER, turned))
        step(0.4, "let the sample run to the cover", r=(mix_at, FINGER_TESTER, turned))
        step(2.2, "and back upright", r=(mix_at, FINGER_TESTER, lv))
        step(0.3, "pause", r=(mix_at, FINGER_TESTER, lv))
    back = hold_at + np.array([0.0, 0.0, 0.0005])
    step(1.5, "stand it back on the box", r=(back, FINGER_TESTER, lv))
    step(0.8, "let go of the tester", r=(back, FINGER_OPEN, lv))
    step(1.5, "lift the right hand clear", r=(pre, FINGER_OPEN, lv))
    step(2.5, "right hand back to its side", r=(turn["ph"], FINGER_OPEN, lv))
    step(1.5, "rest", r=(turn["ph"], FINGER_OPEN, lv))
    return {"left": left, "right": right}
