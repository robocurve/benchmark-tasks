"""Close a roll-end tuck-front mailer (FEFCO 0427, at Uline S-16934's outside size) from its erected start: lock both roll ends, then close
the lid and seat its tuck. At each end one hand holds the sprung end wall upright while the other rolls the wall's inner panel over its
double crease and down inside the box, until both its tabs snap into the holes in the base. Then the lid, standing up in line with the
back wall, is pushed over and pressed shut, and its tuck flap is folded down and pushed inside the front wall. Then both hands come away.

The box stands with its ends to the robot's left and right: end 1 at +y, end 2 at -y, the front wall toward the robot, the lid hinged on the
back wall's top. It is one free articulation, its links colliding with one another (only a link and its own parent do not):
  frame: base (with the four tab holes, two along each end's crease) and the front and back walls, folded up;
  outer, outer2: each end's outer panel, hinged on the base crease. Its crease springs it back to 80 degrees above the table;
  roll, roll2: each end's inner panel, hinged on its outer panel's top. The double crease is one hinge here, placed so that the rolled panel
         lies 17.5 mm inside the outer face, as Uline's outside and inside sizes give, and its spring resists the roll;
  tab_l, tab_r, tab2_l, tab2_r: the locking tabs on each inner panel's free edge. Each rides a short sprung slide, standing for the board's
         give as the tab is pressed over the base: it springs into its hole when it comes over it;
  ear_l, ear_r, ear2_l, ear2_r: the dust flaps on the front and back walls' ends, folded in across each end before the task starts;
  lid: hinged on the back wall's top, standing up in line with the back wall (its crease not yet folded), sprung back to there;
  tuck: the lid's tuck flap, hinged on the lid's front edge, in line with the lid until it is folded.
A tab latches in its hole once it has been pressed at least 2 mm into the base board inside the hole's outline for 3 steps, with its inner
panel folded to 150 degrees or more. With both latched, the end is held locked, as the seated tabs and the flaps trapped inside the rolled
wall hold a real one, but only when it reads locked: its wall within 5 degrees of upright, its panel within 2 degrees of flat and its dust
flaps within 10 degrees of folded in. The tuck latches once it is folded down at least
60 degrees with its face on the front wall's inside (a gap of 0.6 mm at most) and its edge 2 cm down inside it with the lid shut: the lid is then held shut, as the tuck's friction in the
front wall holds a real one.

Debug:  python demokit.py task.py OUT --no-record --mode full,unheld,oneend,laid,lidonly,lidover,shut,undo,push,hover,lean
Final:  python demokit.py task.py OUT
Model:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 3200 --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import collections
import dataclasses
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "The box's front and back walls are already folded up: lock both roll ends with every tab in its hole, then shut the lid with its tuck inside the front wall."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
DT = d.DT
HOLD_STEPS = 20  # 1 s at 20 Hz

LOOKS = {
    "kraft": m.Look(m.rgb(0.66, 0.49, 0.31), roughness=0.92),
    "kraft_in": m.Look(m.rgb(0.74, 0.58, 0.40), roughness=0.95),
    "edge": m.Look(m.rgb(0.55, 0.40, 0.25), roughness=0.95),
    "hole": m.Look(m.rgb(0.16, 0.12, 0.08), roughness=0.95),
}

# -- the box, in its own frame: x along the box from end 1's crease to end 2's, y from the front wall to the back wall, z up; origin on the
# table under end 1's crease. The box is turned -90 degrees on the table: its x runs toward the robot's right (-y), its y away (+x).
YAW = float(os.environ.get("MAILER_YAW", "-90"))  # dev: another start turn, for a lid-only check of the turned box
_MID = (float(os.environ.get("MAILER_MX", "0.34")), 0.0)  # the box's middle on the table
BOX_AT = (float(os.environ.get("MAILER_X0", _MID[0] - 0.170 * math.cos(math.radians(YAW)))),
          float(os.environ.get("MAILER_Y0", _MID[1] - 0.170 * math.sin(math.radians(YAW)))))  # end 1's crease along x at y 17 cm: the box centred between the arms, its middle at x 34 cm
T = 0.0032  # B-flute board, 1/8 in
LX, WY, H = 0.340, 0.314, 0.159  # outside 13 3/8 x 12 3/8 x 6 1/4 in (Uline S-16934); the roll ends on the 12 3/8 in sides
ROLLED = 0.0175  # a rolled end's thickness: (13 3/8 - 12) / 2 in, outside less inside
S = ROLLED - T  # outer panel's middle to the rolled inner panel's middle
XA, ZA = (T + S) / 2, H  # the roll's hinge: half way across the rolled top, at the outer panel's top
GAP = 0.002  # the unrolled inner panel starts this far above the outer's top: rolled, its top sits this far below
PANEL_L = H - GAP - T - 0.002  # rolled, its free edge stands 2 mm over the base board
PANEL_W = WY - 2 * T - 0.003  # it rolls down between the front and back walls
P_TOP = H + GAP + PANEL_L  # the unrolled panel's free edge
TAB_W, TAB_OUT, TAB_IN = 0.025, 0.005, 0.004  # a tab: width, how far it stands out of the panel's edge, how far into it it overlaps
TAB_Y = 0.080
HOLE_X = 2 * XA - T / 2  # a hole's middle across the crease, under the rolled panel
HOLE_DX, HOLE_DY = 0.006, 0.030
EAR_W, EAR_Z0, EAR_Z1 = 0.140, 0.005, 0.145  # a dust flap, folded in across the end behind the outer panel
START_DEG = -10.0  # an outer panel's rest: 80 degrees above the table, leaning out from the box (clear of the grippers at the ready pose)
LID_L = WY - 2 * T - 0.001  # the lid, from its hinge on the back wall's outer top edge to the tuck's crease, just inside the front wall
TUCK_H = 0.045  # the tuck flap's depth (estimate: mailer tuck flaps run 1 1/2 to 2 in)
TUCK_X0, TUCK_X1 = 0.030, LX - 0.030  # 280 mm wide: inside the 305 mm between the rolled ends, 12.5 mm clear each side
TUCK_RC = 0.015  # its free edge's corners rounded to this radius, as a mailer's die cuts them (estimate, from photos of tuck-front mailers)

K_OUTER = 3.0 * H / math.radians(-START_DEG)  # N m/rad: 3 N at its top edge holds it upright
K_ROLL = float(os.environ.get("MAILER_KROLL", "0.01"))  # N m/rad: 0.2 N at its free edge (1.6 N 2 cm out from the crease) holds it rolled flat inside
K_TAB = float(os.environ.get("MAILER_KTAB", "150"))  # N/m: a tab pressed back 3 mm by the base bears on it with 0.45 N
K_EAR = 0.05
K_LID = float(os.environ.get("MAILER_KLID", "0.10"))  # N m/rad: shut, the lid's crease pushes it open with 0.5 N at its front edge
K_TUCK = float(os.environ.get("MAILER_KTUCK", "0.01"))  # N m/rad: folded down 90 degrees, the tuck pushes back with 0.35 N at its edge
F_TUCK = float(os.environ.get("MAILER_FTUCK", "0.012"))  # N m: the folded crease's own friction, less than its spring past 70 degrees: let go, an
# overfolded tuck springs back until it bears on the front wall's inside, as a scored board crease does (estimate)
MASS = 0.45  # Uline: 0.99 lb
LID_HOLD_NM = 2.0 * 0.30  # N m: a seated tuck holds the lid shut against 2 N lifting its front edge, 30 cm from the hinge (estimate)
M_OUTER, M_ROLL, M_TAB, M_EAR, M_LID, M_TUCK = 0.035, 0.033, 0.001, 0.012, 0.080, 0.010


def _far_pt(p) -> tuple:
    """End 2 is end 1 turned half round about the box's upright middle line."""
    return (LX - p[0], -p[1], p[2])


def _far_mesh(mesh: m.Mesh) -> m.Mesh:
    return mesh.moved((-LX / 2, 0.0, 0.0)).turned(180.0, "z").moved((LX / 2, 0.0, 0.0))


def _box_mesh(size, c, far: bool) -> m.Mesh:
    return m.box(size, _far_pt(c) if far else c)


def _col(name: str, size, c, far: bool) -> str:
    return m.col_box(name, size, _far_pt(c) if far else c)


def _outer(far: bool) -> m.Link:
    n = "outer2" if far else "outer"
    panel = _box_mesh((T, WY, H), (T / 2, 0.0, H / 2), far)
    col = _col(f"{n}_c", (T, WY, H - 0.0015), (T / 2, 0.0, (H + 0.0015) / 2), far)  # 1.5 mm clear of the table at the crease
    lo, hi = START_DEG - 60.0, 0.0
    return m.hinge(n, "Y", _far_pt((0.0, 0.0, 0.0)) if far else (0.0, 0.0, 0.0), *((-hi, -lo) if far else (lo, hi)), [(f"{n}_panel", panel, "kraft")], [col],
                   mass=M_OUTER, spring=0.0, damping=0.03, friction=0.0, armature=0.0)  # its spring is set at the first step (README 1)


def _roll(far: bool) -> m.Link:
    n, o = ("roll2", "outer2") if far else ("roll", "outer")
    zc = H + GAP + PANEL_L / 2
    panel = _box_mesh((T, PANEL_W, PANEL_L), (T / 2, 0.0, zc), far)
    bead = m.cylinder(XA, PANEL_W + 0.004, (XA, 0.0, ZA), axis="y", segments=20)  # the rolled top's round, the same from either side
    bead = _far_mesh(bead) if far else bead
    face = _box_mesh((0.0004, PANEL_W - 0.01, PANEL_L - 0.01), (T + 0.0002, 0.0, zc), far)  # the board's paler inside face
    lo, hi = -10.0, 180.0
    return m.hinge(n, "Y", _far_pt((XA, 0.0, ZA)) if far else (XA, 0.0, ZA), *((-hi, -lo) if far else (lo, hi)),
                   [(f"{n}_panel", panel, "kraft"), (f"{n}_bead", bead, "kraft"), (f"{n}_face", face, "kraft_in")],
                   [_col(f"{n}_c", (T, PANEL_W, PANEL_L), (T / 2, 0.0, zc), far)], parent=o, mass=M_ROLL, spring=K_ROLL, damping=0.01, friction=0.0, armature=0.0)


def _tab(name: str, y: float, far: bool) -> m.Link:
    size, c = (T, TAB_W, TAB_OUT + TAB_IN), (T / 2, y, P_TOP + (TAB_OUT - TAB_IN) / 2)
    return m.slider(name, "Z", -TAB_OUT, 0.0, [(f"{name}_v", _box_mesh(size, c, far), "edge")], [_col(f"{name}_c", size, c, far)], parent="roll2" if far else "roll",
                    mass=M_TAB, spring=K_TAB, damping=0.5, friction=0.0, armature=0.0)


def _ear(name: str, side: float, far: bool) -> m.Link:
    y0 = side * (WY / 2 - T)
    size, c = (T, EAR_W, EAR_Z1 - EAR_Z0), (1.5 * T + 0.0003, y0 - side * EAR_W / 2, (EAR_Z0 + EAR_Z1) / 2)
    lo, hi = (-80.0, 0.0) if side > 0 else (0.0, 80.0)  # 0: folded square across the end; opening swings its free edge out of the end
    at = (T + 0.0003, y0, 0.0)
    return m.hinge(name, "Z", _far_pt(at) if far else at, lo, hi, [(f"{name}_v", _box_mesh(size, c, far), "kraft")], [_col(f"{name}_c", size, c, far)],
                   mass=M_EAR, spring=K_EAR, damping=0.01, friction=0.0, armature=0.0)


def _lid() -> m.Link:
    """At joint zero it stands straight up from the back wall's outer face, as the unfolded blank has it; +90 lays it shut over the box."""
    size, c = (LX - 2 * T, T, LID_L), (LX / 2, WY / 2 + T / 2, H + LID_L / 2)
    face = m.box((LX - 2 * T - 0.01, 0.0004, LID_L - 0.01), (LX / 2, WY / 2 - 0.0002, H + LID_L / 2))
    return m.hinge("lid", "X", (0.0, WY / 2, H), -5.0, 95.0, [("lid_panel", m.box(size, c), "kraft"), ("lid_face", face, "kraft_in")], [m.col_box("lid_c", size, c)],
                   mass=M_LID, spring=K_LID, damping=0.05, friction=0.0, armature=0.0)


def _tuck_outline(n: int = 6) -> np.ndarray:
    """The tuck flap's outline, (x, distance from its crease), counter-clockwise: square at the crease, its free edge's two corners
    rounded to TUCK_RC as a mailer's die cuts them, the lead-in that lets the edge find its way inside the front wall."""
    pts = [(TUCK_X0, 0.0), (TUCK_X1, 0.0)]
    for cx, a0 in ((TUCK_X1 - TUCK_RC, 0.0), (TUCK_X0 + TUCK_RC, 90.0)):
        for i in range(n + 1):
            a = math.radians(a0 + 90.0 * i / n)
            pts.append((cx + TUCK_RC * math.cos(a), TUCK_H - TUCK_RC + TUCK_RC * math.sin(a)))
    return np.array(pts)


def _tuck() -> m.Link:
    # the flap as a prism, its outline in the lid's plane and its thickness T on the lid's outside, a convex hull for its collider
    panel = m.prism(_tuck_outline(), -T, 0.0).turned(90.0, "x").moved((0.0, WY / 2, H + LID_L))
    return m.hinge("tuck", "X", (0.0, WY / 2, H + LID_L), -5.0, 150.0, [("tuck_panel", panel, "kraft")], [m.col_hull("tuck_c", panel)], parent="lid",
                   mass=M_TUCK, spring=K_TUCK, damping=0.01, friction=F_TUCK, armature=0.0)


def _base_pieces() -> list[tuple[str, tuple, tuple]]:
    """The base board as boxes (name, size, centre), leaving the four tab holes open."""
    xs = [0.0, HOLE_X - HOLE_DX / 2, HOLE_X + HOLE_DX / 2, LX - HOLE_X - HOLE_DX / 2, LX - HOLE_X + HOLE_DX / 2, LX]
    out = [(f"base_{k}", (xs[2 * k + 1] - xs[2 * k], WY, T), ((xs[2 * k] + xs[2 * k + 1]) / 2, 0.0, T / 2)) for k in range(3)]
    ys = [-WY / 2, -TAB_Y - HOLE_DY / 2, -TAB_Y + HOLE_DY / 2, TAB_Y - HOLE_DY / 2, TAB_Y + HOLE_DY / 2, WY / 2]
    for e, xc in enumerate((HOLE_X, LX - HOLE_X)):
        for k, (a, b) in enumerate([(ys[0], ys[1]), (ys[2], ys[3]), (ys[4], ys[5])]):
            out.append((f"base_h{e}{k}", (HOLE_DX, b - a, T), (xc, (a + b) / 2, T / 2)))
    return out


def _mailer() -> m.Part:
    frame, cols = [], []
    for name, size, c in _base_pieces():
        frame.append((name, m.box(size, c), "kraft_in"))
        cols.append(m.col_box(f"{name}_c", size, c))
    for e, xc in enumerate((HOLE_X, LX - HOLE_X)):
        for k, (a, b) in enumerate([(-TAB_Y - HOLE_DY / 2, -TAB_Y + HOLE_DY / 2), (TAB_Y - HOLE_DY / 2, TAB_Y + HOLE_DY / 2)]):
            frame.append((f"hole_{e}{k}", m.box((HOLE_DX, b - a, 0.0004), (xc, (a + b) / 2, 0.0002)), "hole"))
    for name, side in (("wall_back", 1.0), ("wall_front", -1.0)):
        size, c = (LX - 2 * T, T, H), (LX / 2, side * (WY / 2 - T / 2), H / 2)  # between the two ends' outer panels
        frame.append((name, m.box(size, c), "kraft"))
        cols.append(m.col_box(f"{name}_c", size, c))
    links = [_outer(False), _roll(False), _tab("tab_l", TAB_Y, False), _tab("tab_r", -TAB_Y, False), _ear("ear_l", 1.0, False), _ear("ear_r", -1.0, False),
             _outer(True), _roll(True), _tab("tab2_l", TAB_Y, True), _tab("tab2_r", -TAB_Y, True), _ear("ear2_l", 1.0, True), _ear("ear2_r", -1.0, True),
             _lid(), _tuck()]
    inertia = (MASS / 12 * (WY ** 2 + H ** 2), MASS / 12 * (LX ** 2 + H ** 2), MASS / 12 * (LX ** 2 + WY ** 2))
    # The box is a free body: the frame carries its whole 0.45 kg (the links have no gravity here), so the table bears the real weight.
    # Table (Isaac Lab default 0.5) and board average to MU, the combined static and dynamic friction of corrugated board on a laminate
    # or wood bench: Park et al. 2019 give board on board 0.38 static; 0.3 to 0.5 is assumed for the bench, 0.4 used.
    mu = float(os.environ.get("MAILER_MU", "0.4")) * 2 - 0.5
    part = m.mechanism("mailer", (*BOX_AT, TOP), links, LOOKS, frame_parts=frame, frame_colliders=cols, friction=(mu, mu), free_mass=MASS,
                       frame_inertia=inertia, yaw_deg=YAW, start=(("outer", math.radians(START_DEG)), ("outer2", math.radians(-START_DEG))))
    build0 = part.build

    def build() -> str:
        txt = build0()
        # the articulation's root is the frame, so that the reset writes the frame's pose and not a link's (kit README 26)
        api = ' (\n    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n)'
        attrs = txt[txt.index("    bool physxArticulation:enabledSelfCollisions"):txt.index("    def Material")]
        txt = txt.replace(api, "", 1).replace(attrs, "", 1)
        fh = 'def Xform "frame" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]\n    )\n    {\n'
        txt = txt.replace(fh, fh.replace('["PhysicsRigidBodyAPI", "PhysicsMassAPI"]', '["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysicsArticulationRootAPI", "PhysxArticulationAPI"]') + attrs.replace("    bool", "        bool").replace("\n    int", "\n        int"), 1)
        txt = txt.replace("bool physxArticulation:enabledSelfCollisions = 0", "bool physxArticulation:enabledSelfCollisions = 1")
        for joint, deg in (("outer_joint", START_DEG), ("outer2_joint", -START_DEG)):  # the outer panels start at their rest, as joint states
            head = f'def PhysicsRevoluteJoint "{joint}" (\n        prepend apiSchemas = ["PhysicsDriveAPI:angular"]'
            k = txt.index(head)
            txt = txt[:k] + head.replace('["PhysicsDriveAPI:angular"]', '["PhysicsDriveAPI:angular", "PhysicsJointStateAPI:angular"]') + txt[k + len(head):]
            anchor = 'uniform token drive:angular:physics:type = "force"'
            j = txt.index(anchor, k)
            txt = txt[:j] + f"float state:angular:physics:position = {deg:.6f}\n        float state:angular:physics:velocity = 0\n        " + txt[j:]
        return txt

    return dataclasses.replace(part, build=build)


OBJECTS = [_mailer()]


def size_check() -> list[tuple[str, float, float, str]]:
    """The scene's key sizes against the published ones (README, size table): (what, scene mm, published mm, source)."""
    return [("outside length, end to end", LX * 1000, 13.375 * 25.4, "Uline S-16934 outside 13 3/8 in"),
            ("outside width, front to back", WY * 1000, 12.375 * 25.4, "Uline 12 3/8 in"),
            ("outside height", H * 1000, 6.25 * 25.4, "Uline 6 1/4 in"),
            ("board", T * 1000, 0.125 * 25.4, "B flute 1/8 in"),
            ("rolled end", ROLLED * 1000, (13.375 - 12.0) / 2 * 25.4, "Uline outside less inside length, halved"),
            ("lid, hinge to tuck crease", LID_L * 1000, (12.375 - 2 * 0.125) * 25.4, "inside width, estimate"),
            ("tuck flap depth", TUCK_H * 1000, 45.0, "estimate: mailer tuck flaps 1 1/2 to 2 in"),
            ("tuck flap width", (TUCK_X1 - TUCK_X0) * 1000, 12.0 * 25.4 - 25.0, "inside length 12 in, less 12.5 mm each side, estimate"),
            ("tuck flap corner radius (lead-in)", TUCK_RC * 1000, 15.0, "estimate, from photos of tuck-front mailers"),
            ("tuck crease, hold at 90 deg fold, N at the edge", K_TUCK * math.pi / 2 / TUCK_H, 0.35, "estimate: a scored B-flute crease, a light finger push"),
            ("box mass, g", MASS * 1000, 0.99 * 453.6, "Uline 0.99 lb")]


# -- reading the box ---------------------------------------------------------------------------------------------------------------


def link_pose(env, body: str) -> tuple[np.ndarray, np.ndarray]:
    """A mailer link's pose (station frame): where the part's own origin has gone with that link (README 14)."""
    base = env.unwrapped
    asset = base.scene["mailer"]
    i = list(asset.body_names).index(body)
    pose = m._np(asset.data.body_link_pose_w)[0, i]
    return pose[:3] - m._np(base.scene.env_origins[0]), m.quat_matrix(pose[3:7])


def on_link(env, body: str, p) -> np.ndarray:
    pos, R = link_pose(env, body)
    return R @ np.asarray(p, float) + pos


E2 = np.diag([-1.0, -1.0, 1.0])  # end 2's frame from end 1's: turned half round


def end_local(k: int, p) -> np.ndarray:
    """A point in end k's own frame (x into the box from its crease, y along the crease, z up) in the box's frame."""
    p = np.asarray(p, float)
    return p if k == 0 else np.array([LX - p[0], -p[1], p[2]])


def to_end(k: int, local) -> np.ndarray:
    """A point in the box's frame in end k's own frame (the same map: it is its own inverse)."""
    return end_local(k, local)


def end_rot(box, k: int) -> np.ndarray:
    """End k's frame's axes in the station frame."""
    return box.rotation() @ (np.eye(3) if k == 0 else E2)


def end_world(box, k: int, p) -> np.ndarray:
    return box.to_world(end_local(k, p))[0]


END_LINKS = (("outer", "roll", "tab_l", "tab_r", "ear_l", "ear_r"), ("outer2", "roll2", "tab2_l", "tab2_r", "ear2_l", "ear2_r"))


def roll_axis(env, k: int) -> np.ndarray:
    """End k's roll hinge line's middle, in the station frame, wherever its outer panel stands."""
    return on_link(env, END_LINKS[k][0], end_local(k, (XA, 0.0, ZA)))


class EndSeen:
    """One end, in its own frame. outer_deg: 0 upright, negative leaning out. roll_deg: 0 unrolled, 180 rolled flat inside. edge: the inner
    panel's free edge, middle. tips: each tab's tip. ears_deg: each dust flap's opening, 0 folded in."""

    def __init__(self, outer_deg, roll_deg, edge, tips, ears_deg):
        self.outer_deg, self.roll_deg, self.edge, self.tips = float(outer_deg), float(roll_deg), np.asarray(edge, float), tuple(tips)
        self.ears_deg = tuple(ears_deg)


class Seen:
    """What the scorer reads at one step: plain numbers, so check_score.py can build them by hand. ends: two EndSeen. lid_deg: 0 standing
    up, 90 shut. tuck_deg: 0 in line with the lid, 90 folded square under it. tuck_edge: the tuck's free edge, middle, in the box's frame.
    clear: the nearest fingertip to the box. off_bench: the box's middle is past the bench top's edge, or has dropped below it."""

    def __init__(self, ends, lid_deg, tuck_deg, tuck_edge, clear, off_bench=False):
        self.ends, self.lid_deg, self.tuck_deg, self.tuck_edge = tuple(ends), float(lid_deg), float(tuck_deg), np.asarray(tuck_edge, float)
        self.clear, self.off_bench = float(clear), bool(off_bench)


def in_hole(tip, y: float) -> bool:
    """A tab's tip (end frame) inside its hole's outline, at least 2 mm down into the base board."""
    return bool(abs(tip[0] - HOLE_X) <= (HOLE_DX - T) / 2 + 0.0005 and abs(tip[1] - y) <= (HOLE_DY - TAB_W) / 2 + 0.0005 and tip[2] <= T - 0.002)


def end_tier(e: EndSeen) -> float:
    """An end's rung, read from its state now: 0.25 rolled over the top, 0.5 down inside, 0.75 one tab in, 1.0 locked."""
    rolled = e.roll_deg >= 100.0 and e.edge[0] > T + 0.002 and e.edge[2] < H - 0.005
    down = rolled and e.roll_deg >= 170.0 and e.edge[2] <= T + 0.010
    tabs = [in_hole(e.tips[0], TAB_Y), in_hole(e.tips[1], -TAB_Y)]
    locked = down and all(tabs) and e.roll_deg >= 178.0 and e.outer_deg >= -5.0 and max(abs(a) for a in e.ears_deg) <= 10.0
    return 1.0 if locked else 0.75 if down and any(tabs) else 0.5 if down else 0.25 if rolled else 0.0


def tuck_in(s: Seen) -> bool:
    """The tuck folded down at least 60 degrees, its edge bearing on the front wall's inside (within TUCK_BEAR of its inner face) and at
    least 2 cm below its top: the tuck's friction on the wall is what holds a real lid shut."""
    e = s.tuck_edge
    return bool(s.tuck_deg >= 60.0 and -WY / 2 + T - 0.001 <= e[1] <= -WY / 2 + T + TUCK_BEAR and e[2] <= H - 0.02)


def lid_tier(s: Seen, ends_locked: bool) -> float:
    """The lid's rung, only once both ends are locked (the real order): 0.5 shut (within 5 degrees), 1.0 shut with its tuck in."""
    if not ends_locked or s.lid_deg < 85.0:
        return 0.0
    return 1.0 if tuck_in(s) else 0.5


W_END, W_LID = 0.4, 0.2
TUCK_BEAR = T / 2 + 0.0006  # m: the tuck's edge middle (mid-board: T/2 = 1.6 mm in when its face lies on the wall; the GPU demo reads 1.8) at most 0.6 mm off that: face on the wall, a gap of 0.6 mm at most


def failed(s: Seen) -> bool:
    """The box leaves the bench (its middle past the bench top's edge, or dropped below it), or a dust flap swings out of an end (damage: a
    flap forced past 30 degrees). Sliding, turning and tipping the box are not failures. Latched by the trial: once true, the score stays 0."""
    return bool(s.off_bench or max(abs(a) for e in s.ends for a in e.ears_deg) > 30.0)


def tiers(s: Seen) -> tuple[float, float, float]:
    t1, t2 = end_tier(s.ends[0]), end_tier(s.ends[1])
    return t1, t2, lid_tier(s, t1 >= 1.0 and t2 >= 1.0)


def step_score(s: Seen, still: bool = True, dead: bool = False) -> float:
    """The score this step's state is worth: 0.4 for each end's rung and 0.2 for the lid's; all three done with the hands 3 cm clear give
    1.0, all three done without that 0.95. The trial takes the lowest over the last 1 s, and 1.0 only if the box held still over that 1 s."""
    if dead or failed(s):
        return 0.0
    t1, t2, tl = tiers(s)
    total = W_END * t1 + W_END * t2 + W_LID * tl
    if total >= 1.0 - 1e-9:
        return 1.0 if (s.clear >= 0.03 and still) else 0.95
    return round(total, 4)


class Trial:
    def reset(self, env=None) -> None:
        self.steps = 0
        self.latched = [[False, False], [False, False]]
        self.near = [[0, 0], [0, 0]]
        self.locked = [False, False]
        self.tuck_near, self.lid_locked = 0, False
        self.score, self.full = 0.0, False
        self.failed = False
        self.trail = collections.deque(maxlen=HOLD_STEPS + 1)
        self.window = collections.deque(maxlen=HOLD_STEPS)
        self.peaks = {"outer_Nm": 0.0, "roll_Nm": 0.0, "tab_N": 0.0, "lid_Nm": 0.0}
        self.seen = None
        if env is not None and os.environ.get("MAILER_LOG"):
            box = m.state(env, "mailer")
            print(f"[mailer] at reset: box at {np.round((box.pos - [BOX_AT[0], BOX_AT[1], TOP]) * 1000, 1)} mm, x axis {np.round(box.rotation()[:, 0], 3)}, joints "
                  f"{ {k: round(v, 3) for k, v in box.joints.items()} }", flush=True)

    def _spring(self, env, link: str, k: float, rest_deg: float, effort: float) -> None:
        asset, j = m._joint(env, "mailer", link)
        m._TOUCHED.add(("mailer", link))
        asset.write_joint_effort_limit_to_sim(m._tensor(asset, effort), joint_ids=[j])
        asset.write_joint_stiffness_to_sim(m._tensor(asset, k), joint_ids=[j])
        asset.set_joint_position_target(m._tensor(asset, math.radians(rest_deg)), joint_ids=[j])

    def read(self, env) -> Seen:
        box = m.state(env, "mailer")
        tips = m.fingertips(env)
        local = box.to_local(tips)
        lo, hi = np.array([0.0, -WY / 2, 0.0]), np.array([LX, WY / 2 + T, H + T])  # the closed box's outside: ends upright, the lid shut on its walls
        gap = float(np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0.0), axis=1).min())
        ends = []
        for k, (outer, roll, tl, tr, el, er) in enumerate(END_LINKS):
            sgn = 1.0 if k == 0 else -1.0
            edge = to_end(k, box.to_local(on_link(env, roll, end_local(k, (T / 2, 0.0, P_TOP))))[0])
            tip_l = to_end(k, box.to_local(on_link(env, tl, end_local(k, (T / 2, TAB_Y, P_TOP + TAB_OUT))))[0])
            tip_r = to_end(k, box.to_local(on_link(env, tr, end_local(k, (T / 2, -TAB_Y, P_TOP + TAB_OUT))))[0])
            ends.append(EndSeen(sgn * box.deg(outer), sgn * box.deg(roll), edge, (tip_l, tip_r), (box.deg(el), box.deg(er))))
        tuck_edge = box.to_local(on_link(env, "tuck", (LX / 2, WY / 2 + T / 2, H + LID_L + TUCK_H)))[0]
        mid = box.pos + box.rotation() @ np.array([LX / 2, 0.0, H / 2])  # the box's middle, in the station frame
        off = bool(not (BENCH_X[0] < mid[0] < BENCH_X[1] and abs(mid[1]) < BENCH_Y) or mid[2] < TOP - 0.03)
        return Seen(ends, box.deg("lid"), box.deg("tuck"), tuck_edge, gap, off)

    def step(self, env) -> None:
        self.steps += 1
        if self.steps == 1:
            self._spring(env, "outer", K_OUTER, START_DEG, 20.0)
            self._spring(env, "outer2", K_OUTER, -START_DEG, 20.0)
        box = m.state(env, "mailer")
        s = self.seen = self.read(env)
        self.failed = self.failed or failed(s)
        for k in range(2):
            e = s.ends[k]
            for j, y in enumerate((TAB_Y, -TAB_Y)):
                self.near[k][j] = self.near[k][j] + 1 if in_hole(e.tips[j], y) else 0
                if self.near[k][j] >= 3 and not self.latched[k][j] and not self.failed and e.roll_deg >= 150.0:
                    self.latched[k][j] = True
                    m.lock(env, "mailer", END_LINKS[k][2 + j])
                    print(f"[mailer] end {k + 1} {'left' if j == 0 else 'right'} tab latched in its hole at step {self.steps}: tip {np.round(e.tips[j] * 1000, 1)} mm, "
                          f"outer {e.outer_deg:.1f} deg, roll {e.roll_deg:.1f} deg", flush=True)
            if all(self.latched[k]) and not self.locked[k] and end_tier(e) >= 1.0:  # the end locks only when it reads locked: wall upright within 5 degrees, flaps in
                for link in END_LINKS[k][:2]:
                    m.lock(env, "mailer", link)
                self.locked[k] = True
                print(f"[mailer] end {k + 1}: both tabs in, the end is locked (step {self.steps})", flush=True)
        self.tuck_near = self.tuck_near + 1 if (tuck_in(s) and s.lid_deg >= 85.0) else 0
        if self.tuck_near >= 3 and not self.lid_locked and not self.failed and all(self.locked):  # a lid can only be tucked over locked ends
            m.lock(env, "mailer", "lid", effort=LID_HOLD_NM)  # the tuck's friction in the wall: up to 2 N lifting the lid's front edge
            m.lock(env, "mailer", "tuck")
            self.lid_locked = True
            print(f"[mailer] tuck seated inside the front wall at step {self.steps}: lid {s.lid_deg:.1f} deg, tuck {s.tuck_deg:.1f} deg, edge {np.round(s.tuck_edge * 1000, 1)} mm", flush=True)
        elif self.lid_locked and s.lid_deg < 85.0:  # pulled open harder than the tuck's friction holds: the tuck comes out, the lid springs open
            for link in ("lid", "tuck"):
                m.unlock(env, "mailer", link)
            self.lid_locked, self.tuck_near = False, 0
            print(f"[mailer] tuck pulled out at step {self.steps}: lid {s.lid_deg:.1f} deg", flush=True)
        q = box.joints
        for e in s.ends:
            self.peaks["outer_Nm"] = max(self.peaks["outer_Nm"], K_OUTER * math.radians(e.outer_deg - START_DEG))
            self.peaks["roll_Nm"] = max(self.peaks["roll_Nm"], K_ROLL * math.radians(max(0.0, e.roll_deg)))
        self.peaks["tab_N"] = max(self.peaks["tab_N"], K_TAB * max(-min(q[f"{n}_joint"] for n in ("tab_l", "tab_r", "tab2_l", "tab2_r")), 0.0))
        self.peaks["lid_Nm"] = max(self.peaks["lid_Nm"], K_LID * math.radians(max(0.0, s.lid_deg)))
        self.trail.append(np.r_[box.pos, 0.40 * box.rotation().ravel(), [H * math.radians(e.outer_deg) for e in s.ends], [0.15 * math.radians(e.roll_deg) for e in s.ends],
                                0.3 * math.radians(s.lid_deg), 0.045 * math.radians(s.tuck_deg)])
        arr = np.array(self.trail)
        still = len(arr) > HOLD_STEPS - 1 and float(np.abs(arr[-HOLD_STEPS:] - arr[-1]).max()) <= 0.001  # over the same last 1 s
        self.window.append(step_score(s, True, self.failed))
        # the score is the state's worth held over the last 1 s: the lowest step score in it (0 until 1 s has passed); 1.0 also needs the
        # box (every point of it within 40 cm of its frame, so its turn counts at 40 cm) and its panels still over that 1 s
        low = 0.0 if self.failed or len(self.window) < HOLD_STEPS else float(min(self.window))
        self.score = 0.95 if (low >= 1.0 and not still) else low
        self.full = self.score >= 1.0
        if os.environ.get("MAILER_LOG") and self.steps % int(os.environ["MAILER_LOG"]) == 0:
            yaw = math.degrees(math.atan2(box.rotation()[1, 0], box.rotation()[0, 0]))
            ends = " | ".join(f"end {k + 1}: outer {e.outer_deg:.1f} roll {e.roll_deg:.1f} edge {np.round(e.edge * 1000, 0)} tips z {e.tips[0][2] * 1000:.1f} {e.tips[1][2] * 1000:.1f} "
                              f"ears {e.ears_deg[0]:.1f} {e.ears_deg[1]:.1f} tier {end_tier(e)}" for k, e in enumerate(s.ends))
            print(f"[mailer] step {self.steps}: box at {np.round((box.pos - [BOX_AT[0], BOX_AT[1], TOP]) * 1000, 1)} mm, yaw {yaw:.1f}; {ends}; lid {s.lid_deg:.1f} tuck {s.tuck_deg:.1f} "
                  f"edge {np.round(s.tuck_edge * 1000, 0)}; clear {s.clear * 1000:.0f} mm, step score {self.window[-1]}, score {self.score}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Both ends locked, the lid shut with its tuck inside the front wall, the hands 3 cm clear, nothing moving, held 1 s."""
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


BENCH_X, BENCH_Y = (0.0, 0.90), 0.60  # the 120 x 90 cm bench top from the arm bases: its near edge at x 0, 90 cm deep, 60 cm each side

# -- the demo's paths, in an end's own frame (x into the box from the crease, y along it, z up), mapped through the box's pose -------------

TIP = 0.016  # the closed fingers' ends stand this far past the TCP along the fingers
SIDE = 0.020  # and their broad side this far beside it
PRESS = 0.003  # how far into a face the fingers aim, so that they push
HOLD_PRESS = 0.015  # the holding hand aims this far past the upright wall's face, so that it holds the wall hard on its stop
DOWN_PRESS, FLAT_PRESS = 0.012, float(os.environ.get("MAILER_FLAT", "0.04"))
LEAD = float(os.environ.get("MAILER_LEAD", "30"))  # degrees: how far ahead of the panel the rolling hand aims while it presses it down inside
OVER = 2.0  # and past flat at the end, so that it holds the panel on its stop
HOLD_Z = float(os.environ.get("MAILER_HOLD_Z", "0.09"))  # where the holding hand presses the outer panel: height
LEAN_DEG = float(os.environ.get("MAILER_LEAN", "-7"))  # the lean mode's wall angle while the tabs go in
HOLD_Y = (0.06, -0.06)  # and across, in each end's frame: at world x 40 cm at both ends
HOLD_TILT = float(os.environ.get("MAILER_HOLD_TILT", "45"))  # the holding fingers point into the box, tilted this far down from level
ROLL_R = float(os.environ.get("MAILER_ROLL_R", "0.08"))  # where the rolling hand pushes the inner panel: out along it from the hinge
ROLL_Y = (-0.04, 0.04)  # where the rolling hand pushes, across: at world x 30 cm at both ends
TILT_C = float(os.environ.get("MAILER_TILT_C", "30"))  # down inside, the rolling fingers tilt back toward the wall by this much at flat
SLIDE_TO = float(os.environ.get("MAILER_SLIDE", "0.065"))  # end 2's crease is pushed to this far right of the middle (world y -6.5 cm)


def rot_y(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def nominal_rot(k: int) -> np.ndarray:
    """End k's frame's axes in the station frame with the box where it starts (turned YAW on the table)."""
    c, s = math.cos(math.radians(YAW)), math.sin(math.radians(YAW))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ (np.eye(3) if k == 0 else E2)


def in_end(k: int, world_grip) -> np.ndarray:
    """A grip given in the station's terms, in end k's frame (so that the box's own turn on the table carries it)."""
    return nominal_rot(k).T @ world_grip.rot


HOLD_FLIP = os.environ.get("MAILER_HFLIP", "0") == "1"  # end 2's holding hand turned over about its fingers: its wrist camera then sits below the leaning inner panel's line, not in it


def hold_grip_end(k: int) -> np.ndarray:
    """The holding fingers point into the box from outside the end, tilted HOLD_TILT down from level."""
    return in_end(k, d.tilted(90.0 - HOLD_TILT, -90.0 if k == 0 else 90.0, flip=bool(k == 1 and HOLD_FLIP)))


def hold_tcp_end(k: int, outer_deg: float, press: float = PRESS, across: float | None = None) -> np.ndarray:
    """The holding TCP (end frame) with its fingers' ends on the outer panel's outside face, the panel at outer_deg, `across` along it
    (default HOLD_Y)."""
    face = rot_y(outer_deg) @ np.array([0.0, HOLD_Y[k] if across is None else across, HOLD_Z])
    normal = rot_y(outer_deg) @ np.array([-1.0, 0.0, 0.0])
    a = hold_grip_end(k)[:, 2]
    return face - normal * press - a * TIP


SWITCH = float(os.environ.get("MAILER_SWITCH", "25"))  # roll angle (theta) where the rolling grip starts turning from side-on to fingers down
BLEND = 45.0  # over this many degrees of theta


def roll_tilt(theta: float) -> float:
    """Down inside, the rolling fingers' tilt from straight down back toward the wall (end frame -x): growing to TILT_C at flat."""
    return 0.0 if theta >= 0.0 else TILT_C * min(1.0, -theta / 90.0)


def roll_grip_end(k: int, theta: float) -> np.ndarray:
    """The rolling grip at roll angle theta. While the panel is up and going over (theta >= SWITCH) the fingers point forward, 30 degrees
    below level (side-on: their broad side on the panel's outside face), turning with the panel about its crease; then they turn to
    straight down (their ends on the face) over BLEND degrees, and down inside they tilt back toward the wall."""
    side_on = lambda th: rot_y(90.0 - th) @ in_end(k, d.tilted(60.0, 0.0))  # noqa: E731
    down = rot_y(roll_tilt(theta)) @ in_end(k, d.down(0.0))
    if theta >= SWITCH:
        return side_on(theta)
    if theta <= SWITCH - BLEND:
        return down
    return d.ik.slerp_matrix(side_on(theta), down, d.ik.smoothstep((SWITCH - theta) / BLEND))


def roll_tcp_end(k: int, theta: float, press: float = PRESS, axis=None) -> np.ndarray:
    """The rolling TCP (end frame) at roll angle theta (deg; 90: the inner panel straight up, 0: flat over the box, -90: down inside), the
    closed fingers on the panel's outside face ROLL_R out from the hinge. The TCP stands off the face by the fingers' extent toward it:
    TIP along the fingers, SIDE across them."""
    t = math.radians(theta)
    u = np.array([math.cos(t), 0.0, math.sin(t)])
    n = np.array([-math.sin(t), 0.0, math.cos(t)])  # the outside face's normal
    ax = np.array([XA, 0.0, ZA]) if axis is None else np.asarray(axis, float)
    face = ax + np.array([0.0, ROLL_Y[k], 0.0]) + ROLL_R * u + XA * n
    a = roll_grip_end(k, theta)[:, 2]  # where the fingers point
    across = n - float(n @ a) * a
    reach = TIP * max(0.0, float(-n @ a)) + SIDE * float(np.linalg.norm(across))
    return face + n * (reach - press)


# -- demo ---------------------------------------------------------------------------------------------------------------------------


def _w(box, k, p) -> np.ndarray:
    return end_world(box, k, p)


def _wr(box, k, R) -> np.ndarray:
    return end_rot(box, k) @ R


def axis_end(env, box, k) -> np.ndarray:
    """End k's roll hinge in its own frame, from the live outer panel."""
    return to_end(k, box.to_local(roll_axis(env, k))[0])


def work_end(ctx, k: int, mode: str):
    """Lock end k: the hand on its side holds its wall upright from outside, the other hand rolls its inner panel over and down inside."""
    holder, roller = (ctx.left, ctx.right) if k == 0 else (ctx.right, ctx.left)
    env = ctx.emb._env
    shut = d.FINGER_CLOSED
    out = 1.0 if k == 0 else -1.0  # end k's outside, in station y
    box = ctx.state("mailer")
    G = hold_grip_end(k)
    # 1. The holding hand comes outside the leaning wall, stands it upright onto its stop, pressing it against the dust flaps, and goes on
    # pushing until the box has slid on the table to put that end where both hands reach it.
    start = _w(box, k, hold_tcp_end(k, START_DEG, press=-0.025, across=0.0))
    R0 = end_rot(box, k)
    high = start + R0 @ np.array([-0.06, 0.0, 0.0]) + [0.0, 0.0, P_TOP + 0.06 - start[2] + TOP]  # come down from above, outside the end: not
    # through the leaning inner panel
    yield from holder.move(high, grip=d._as_grip(_wr(box, k, G)), finger=shut, t=1.4, label="above, outside the wall")
    yield from holder.move(start + R0 @ np.array([-0.04, 0.0, 0.0]), t=1.0, label="down outside the wall")
    yield from holder.move(start, t=0.5, label="outside the wall")
    yield from holder.move(_w(box, k, hold_tcp_end(k, START_DEG, press=-0.002, across=0.0)), t=0.6, label="to the wall")
    steps = 70
    box = ctx.state("mailer")
    crease0 = box.to_world(end_local(k, (0.0, 0.0, 0.0)))[0]
    goal_y = SLIDE_TO * out
    lean = mode == "lean" and k == 0
    a_end = LEAN_DEG if lean else 0.0  # lean: the wall is held leaning at first, stood upright once the tabs are in
    for j in range(1, steps + 1):  # stand it up and slide the box, pushing at the wall's middle so that the box does not turn
        s = d.ik.smoothstep(min(1.0, j / 24))
        a = START_DEG + (a_end - START_DEG) * s
        box = ctx.state("mailer")
        R = end_rot(box, k)
        crease = crease0.copy()
        crease[1] = crease0[1] + (goal_y - crease0[1]) * d.ik.smoothstep(j / steps)
        # pushed at the wall's middle, or off it toward the side that turns the box back square (feedback on its observed turn)
        across = float(np.clip(SQUARE_GAIN * (box_yaw(box) - YAW), -0.08, 0.08))
        mm, deg = holder._solve(crease + R @ hold_tcp_end(k, a, press=HOLD_PRESS * s, across=across), R @ G)
        holder._note(mm, deg, "stand the wall up and slide the box")
        yield
    for j in range(1, 17):  # then the fingers glide along the wall to their holding place, still pressing; aimed from where the slide
        # ended, not from the box as it moves, so that the hold does not chase the box across the table
        mm, deg = holder._solve(crease + R @ hold_tcp_end(k, a_end, press=HOLD_PRESS, across=across + (HOLD_Y[k] - across) * d.ik.smoothstep(j / 16)), R @ G)
        holder._note(mm, deg, "to the holding place")
        yield
    yield from holder.wait(0.5)
    box = ctx.state("mailer")
    print(f"[mailer] end {k + 1}: wall held at {out * ctx.deg('mailer', END_LINKS[k][0]):.1f} deg, crease at y {box.to_world(end_local(k, (0, 0, 0)))[0][1] * 100:.1f} cm", flush=True)
    if mode == "unheld" and k == 0:
        yield from holder.move(holder.at() + [0.0, 0.06 * out, 0.0], t=0.4, label="let go")
        for _ in range(3):
            yield from holder.wait(0.5)
            print(f"[mailer] let go: wall at {ctx.deg('mailer', 'outer'):.1f} deg", flush=True)
        return
    # 2. The rolling hand comes down outside the inner panel's top, its fingers' broad side on the panel's outside face, and rolls it over
    # the top: an arc about the hinge, following the hinge as the box moves.
    box = ctx.state("mailer")
    ax = axis_end(env, box, k)
    over = roll_tcp_end(k, 90.0, press=-0.04, axis=ax)
    top = P_TOP + 0.03  # over the unrolled panel's free edge first, so that the hand does not strike it on its way out past it
    yield from roller.move(_w(box, k, np.r_[over[0] + 0.03, over[1], top]), grip=d._as_grip(_wr(box, k, roll_grip_end(k, 90.0))), finger=shut, t=1.2, label="above the panel")
    yield from roller.move(_w(box, k, np.r_[over[0] - 0.02, over[1], top]), t=0.6, label="past the panel")
    yield from roller.move(_w(box, k, over + [-0.02, 0.0, 0.03]), t=0.8, label="down outside the panel")
    yield from roller.move(_w(box, k, roll_tcp_end(k, 90.0, press=0.0, axis=ax)), t=0.8, label="outside the panel")
    for j, th in enumerate(np.linspace(90.0, -30.0, 90)):
        box = ctx.state("mailer")
        ax = axis_end(env, box, k)
        press = PRESS if th >= 0.0 else PRESS + (DOWN_PRESS - PRESS) * (-th / 90.0)
        mm, deg = roller._solve(_w(box, k, roll_tcp_end(k, th, press=press, axis=ax)), _wr(box, k, roll_grip_end(k, th)))
        roller._note(mm, deg, "roll the panel")
        yield
    # 3. Down inside, the hand follows the panel as it is, aiming LEAD degrees ahead of it and pressing into its face, then holds it flat
    # against the wall, its fingers tilted back toward the wall, until both tabs have snapped into their holes.
    aim = -30.0
    if mode == "laid":
        stop = -62.0  # the tabs laid on the base's edge, not pressed in
    else:
        stop = -90.0 - OVER
    for j in range(round(6.0 / DT)):
        box = ctx.state("mailer")
        ax = axis_end(env, box, k)
        now = 90.0 - out * ctx.deg("mailer", END_LINKS[k][1])
        if not TRIAL.locked[k]:
            aim = max(now - LEAD, aim - 2.0, stop)
        press = DOWN_PRESS + (FLAT_PRESS - DOWN_PRESS) * max(0.0, (-aim - 80.0) / (10.0 + OVER))
        mm, deg = roller._solve(_w(box, k, roll_tcp_end(k, aim, press=press, axis=ax)), _wr(box, k, roll_grip_end(k, max(aim, -90.0))))
        roller._note(mm, deg, "press the panel down inside")
        if lean:  # lean: the holder keeps the wall leaning until both tabs are in, then stands it up
            a_end = min(0.0, a_end + 0.5) if all(TRIAL.latched[k]) else a_end
            Rk = end_rot(box, k)
            mm, deg = holder._solve(box.to_world(end_local(k, (0.0, 0.0, 0.0)))[0] + Rk @ hold_tcp_end(k, a_end, press=HOLD_PRESS, across=HOLD_Y[k]), Rk @ G)
            holder._note(mm, deg, "hold the wall leaning, then stand it up")
        yield
        if os.environ.get("MAILER_LOG") and j % 10 == 0:
            print(f"[mailer] end {k + 1} press: aim {aim:.0f}, panel at {out * ctx.deg('mailer', END_LINKS[k][1]):.1f} deg, latched {TRIAL.latched[k]}", flush=True)
        if (TRIAL.locked[k] and j >= 10) or (mode == "laid" and j >= 60):
            for _ in range(5):
                yield
            break
    print(f"[mailer] end {k + 1} pressed: roll {out * ctx.deg('mailer', END_LINKS[k][1]):.1f} deg, outer {out * ctx.deg('mailer', END_LINKS[k][0]):.1f} deg, "
          f"tabs latched {TRIAL.latched[k]}; spring peaks {TRIAL.peaks}", flush=True)
    # 4. Both hands leave: the rolling hand up out of the box, the holding hand back off the wall.
    box = ctx.state("mailer")
    R = end_rot(box, k)
    yield from roller.move(roller.at() + R @ np.array([0.04, 0.0, 0.0]), t=0.4, label="off the panel")
    yield from ctx.together(roller.move(roller.at() + R @ np.array([0.03, 0.0, 0.16]), t=0.9, label="up out of the box"),
                            holder.move(holder.at() + R @ np.array([-0.06, 0.0, 0.03]), t=0.6, label="off the wall"))


SQUARE_GAIN = float(os.environ.get("MAILER_SQUARE", "0.004"))  # m of push offset per degree the box has turned
SQUARE_IN = 0.05  # the squaring push on the front wall: this far in from the end that has come toward the robot
SQUARE_Z = 0.10  # at this height
SQUARE_TILT = float(os.environ.get("MAILER_SQ_TILT", "20"))  # its fingers this far from straight down, toward the wall


def box_yaw(box) -> float:
    R = box.rotation()
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def square_box(ctx, tol: float = 2.0):
    """After end 2: read the box's turn on the table and push it back square, by feedback, with the fingers' side on the front wall near the
    end that has come toward the robot (the hand on that side, fingers down beside the wall), until it is within `tol`. The box stays on
    the table."""
    for attempt in range(4):
        box = ctx.state("mailer")
        err = box_yaw(box) - YAW
        print(f"[mailer] box turned {err:.1f} deg from square", flush=True)
        if abs(err) <= tol:
            return
        hand, x_at = (ctx.right, LX - SQUARE_IN) if err < 0 else (ctx.left, SQUARE_IN)
        G = d.tilted(SQUARE_TILT, box_yaw(box) - YAW)  # the fingers down beside the front wall, square to it, their narrow side on it
        R = box.rotation()
        normal = R @ np.array([0.0, -1.0, 0.0])  # the front wall's outside normal
        face = lambda b: b.to_world(np.array([x_at, -WY / 2, SQUARE_Z]))[0]  # noqa: E731
        yield from hand.move(face(box) + normal * (EDGE + 0.03) + [0.0, 0.0, 0.10], grip=G, finger=d.FINGER_CLOSED, t=1.4, label="above, before the front wall")
        yield from hand.move(face(box) + normal * (EDGE + 0.01), t=0.8, label="down before the front wall")
        p0 = face(box) + normal * EDGE
        for j in range(round(4.0 / DT)):
            box = ctx.state("mailer")
            err = box_yaw(box) - YAW
            if abs(err) <= tol * 0.4:
                break
            p0 = p0 + (-normal) * 0.0015  # 3 cm a second, into the wall
            mm, deg = hand._solve(p0, G.rot)
            hand._note(mm, deg, "square the box")
            yield
        yield from hand.move(hand.at() + normal * 0.05 + [0.0, 0.0, 0.05], t=0.6, label="off the front wall")
        yield from hand.move(RIGHT_AWAY if hand is ctx.right else LEFT_AWAY, grip=AWAY_GRIP, t=1.0, label="away")


DRAW = float(os.environ.get("MAILER_DRAW", "-0.10"))  # the box is drawn toward the robot until it stands this far nearer than at the start (m)


def draw_box(ctx, tol: float = 3.0):
    """Before the lid: the left hand's fingers go down inside the open box behind the front wall's middle and draw the box toward the robot
    along the table, by feedback on where it stands, until it stands DRAW nearer than at the start; off the middle by the observed turn, so
    that it stays square."""
    l = ctx.left
    G = d.down(0.0)  # the jaws along the wall: the fingers' narrow side on its inside face
    box = ctx.state("mailer")
    goal = BOX_AT[0] + DRAW
    if box.pos[0] - goal <= 0.01:
        return
    hook = lambda b, xm: b.to_world(np.array([xm, -WY / 2 + T + EDGE + 0.004, H - 0.03]))[0]  # noqa: E731
    xm = LX / 2
    yield from l.move(hook(box, xm) + [0.0, 0.0, 0.10], grip=d._as_grip(_rz(box_yaw(box) - YAW) @ G.rot), finger=d.FINGER_CLOSED, t=1.4, label="above the box, behind the front wall")
    yield from l.move(hook(box, xm), t=0.8, label="down inside the front wall")
    p = l.at().copy()
    for j in range(round(6.0 / DT)):
        box = ctx.state("mailer")
        if box.pos[0] - goal <= 0.003:
            break
        toward = box.rotation() @ np.array([0.0, -1.0, 0.0])  # toward the robot, square to the front wall
        p = p + toward * 0.0015  # 3 cm a second
        # pulled off the middle toward the end that has come nearer, so that the box turns back square
        xm = LX / 2 + float(np.clip(SQUARE_GAIN * 2.0 * (box_yaw(box) - YAW), -0.10, 0.10))
        q = hook(box, xm)
        target = np.r_[p[0], q[1], q[2]] if abs(toward[0]) > 0.7 else p
        mm, deg = l._solve(target, _rz(box_yaw(box) - YAW) @ G.rot)
        l._note(mm, deg, "draw the box toward the robot")
        yield
    box = ctx.state("mailer")
    print(f"[mailer] box drawn to {np.round((box.pos - [BOX_AT[0], BOX_AT[1], TOP]) * 1000, 0)} mm, turned {box_yaw(box) - YAW:.1f} deg", flush=True)
    yield from l.move(l.at() + box.rotation() @ np.array([0.0, 0.01, 0.0]), t=0.3, label="off the front wall")
    yield from l.move(l.at() + [0.0, 0.0, 0.12], t=0.7, label="up out of the box")
    yield from l.move(LEFT_AWAY, grip=AWAY_GRIP, t=1.0, label="away")
    yield from square_box(ctx, tol=tol)


def _rz(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


LEFT_AWAY, RIGHT_AWAY =(0.26, 0.22, TOP + 0.26), (0.26, -0.22, TOP + 0.26)
AWAY_GRIP = d.tilted(30.0, 0.0)
LEFT_PARK, RIGHT_PARK = (0.24, 0.30, TOP + 0.36), (0.24, -0.30, TOP + 0.36)  # clear of the far end's leaning panel while the box slides


def demo_script(ctx):
    """Modes: full. look: the arms wait, for a still. unheld: end 1's wall stood up, held 1 s and let go (it springs back). oneend: end 1
    only. laid: end 1 pressed short of flat (about 150 degrees), its tabs on the base's edge, then let go. lidonly: the lid pushed shut with the ends open. undo: both
    ends locked, the lid swung shut with its tuck unfolded, held 1.5 s and let go, so that it springs open before the end. push: nothing folded, the box only pushed, slid
    and turned a quarter on the table. lidover: end 1 locked, end 2 left open, then the box turned and the lid shut and tucked over it.
    shut: both ends locked, the lid pressed shut with its tuck unfolded, and the hand still holding it down at the end. hover: both ends locked, the
    tuck folded by a hand and not seated, the lid pressed on it (it must not count as tucked). lean: end 1's wall held leaning 7 degrees while its tabs
    go in, stood upright after (the order of the two does not matter: it must still lock and score 1.0)."""
    mode = ctx.mode
    l, r = ctx.left, ctx.right
    if mode == "look":
        yield from l.wait(3.0)
        return
    if mode not in ("full", "unheld", "oneend", "laid", "lidonly", "undo", "push", "lidover", "shut", "hover", "lean"):
        raise ValueError(f"unknown mode {mode!r}")
    if mode == "push":
        yield from turn_box(ctx)
        yield from ctx.together(l.move(LEFT_AWAY, grip=AWAY_GRIP, t=1.2, label="away"), r.move(RIGHT_AWAY, grip=AWAY_GRIP, t=1.2, label="away"))
        yield from l.wait(1.6)
        print(f"[mailer] done ({mode}): box at {box_yaw(ctx.state('mailer')):.1f} deg, score {TRIAL.score}", flush=True)
        return
    if mode != "lidonly":
        yield from r.move(RIGHT_PARK, grip=AWAY_GRIP, finger=d.FINGER_CLOSED, t=1.0, label="out of the box's way")  # end 2's leaning panel comes past READY as the box slides
        yield from work_end(ctx, 0, mode)
        if mode == "lidover":
            yield from ctx.together(l.move(LEFT_AWAY, grip=AWAY_GRIP, t=1.2, label="away"), r.move(RIGHT_AWAY, grip=AWAY_GRIP, t=1.2, label="away"))
    if mode not in ("lidonly", "lidover"):
        if mode in ("unheld", "laid", "oneend"):  # end 1 only
            yield from ctx.together(l.move(LEFT_AWAY, grip=AWAY_GRIP, t=1.2, label="away"), r.move(RIGHT_AWAY, grip=AWAY_GRIP, t=1.2, label="away"))
            yield from l.wait(2.0)
            print(f"[mailer] done ({mode}): score {TRIAL.score}", flush=True)
            return
        yield from ctx.together(l.move(LEFT_PARK, grip=AWAY_GRIP, finger=d.FINGER_CLOSED, t=1.2, label="out of the box's way"), r.move(RIGHT_AWAY, grip=AWAY_GRIP, t=1.2, label="away"))
        yield from work_end(ctx, 1, mode)
        yield from ctx.together(l.move(LEFT_AWAY, grip=AWAY_GRIP, t=1.2, label="away"), r.move(RIGHT_AWAY, grip=AWAY_GRIP, t=1.2, label="away"))
    if mode != "lidonly" and os.environ.get("MAILER_TURN", "1") == "1":
        yield from turn_box(ctx)  # turned a quarter on the table, sliding, so that the lid's hinge is to the left
    elif mode != "lidonly":
        yield from recentre(ctx)
        yield from square_box(ctx, tol=3.0)  # by feedback on the box's observed turn, until it is within 3 degrees of square
        yield from draw_box(ctx)  # then drawn toward the robot along the table, square, so that the lid's arc is in reach
    if abs(math.sin(math.radians(box_yaw(ctx.state("mailer"))))) < 0.5:  # the hinge to one side
        yield from close_lid_side(ctx, mode)
    else:
        yield from close_lid(ctx, mode)
    if mode == "shut":  # the hand stays on the shut lid, pressing it down, to the end
        lh = hinge_hand(ctx, ctx.state("mailer"))[0]
        for _ in range(round(2.0 / DT)):
            _hold_lid(ctx, lh, ctx.state("mailer"), 90.0 + OVER, press=PRESS + 0.012)
            yield
        print(f"[mailer] done ({mode}): lid {ctx.deg('mailer', 'lid'):.1f} tuck {ctx.deg('mailer', 'tuck'):.1f}, held, score {TRIAL.score}", flush=True)
        return
    yield from ctx.together(l.move(LEFT_AWAY, grip=AWAY_GRIP, t=1.2, label="away"), r.move(RIGHT_AWAY, grip=AWAY_GRIP, t=1.2, label="away"))
    print(f"[mailer] done ({mode}): latched {TRIAL.latched}, lid {ctx.deg('mailer', 'lid'):.1f} tuck {ctx.deg('mailer', 'tuck'):.1f}, score {TRIAL.score}", flush=True)
    yield from l.wait(1.6)


LID_R = float(os.environ.get("MAILER_LID_R", "0.15"))  # the lid-closing hand pushes the lid's outside face this far from its hinge
LID_X = LX - 0.075  # and this far along the box (box frame x: 7.5 cm in from end 2's crease), its fingers pointing along the lid toward end 1
LID_HOLD = 70.0  # degrees: the lid is held here while the tuck is folded
TUCK_S = 0.025  # the folding hand pushes the tuck's outside face this far from its crease
TUCK_X = float(os.environ.get("MAILER_TUCK_X", "0.24"))  # and this far along the box (box frame x)
TUCK_GRIP_MAX = float(os.environ.get("MAILER_TUCK_GRIP", "130"))  # the folding grip turns with the tuck up to this angle, then its fingers' corner carries on
TUCK_TO = float(os.environ.get("MAILER_TUCK_TO", "110"))  # degrees the tuck is folded to, relative to the lid


def rot_x(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def lid_tcp(phi: float, press: float = PRESS) -> np.ndarray:
    """The lid-closing TCP (box frame) at lid angle phi, side-on to the lid's outside face LID_R from its hinge."""
    p = math.radians(phi)
    u, n = np.array([0.0, -math.sin(p), math.cos(p)]), np.array([0.0, math.cos(p), math.sin(p)])
    return np.array([LID_X, WY / 2, H]) + LID_R * u + T * n + n * (SIDE - press)


def tuck_tcp(phi: float, psi: float, press: float = PRESS) -> np.ndarray:
    """The tuck-folding TCP (box frame) with the lid at phi and the tuck at psi, side-on to the tuck's outside face TUCK_S from its crease."""
    p, a = math.radians(phi), math.radians(phi + psi)
    crease = np.array([TUCK_X, WY / 2, H]) + LID_L * np.array([0.0, -math.sin(p), math.cos(p)])
    u, n = np.array([0.0, -math.sin(a), math.cos(a)]), np.array([0.0, math.cos(a), math.sin(a)])
    return crease + TUCK_S * u + T * n + n * (SIDE - press)


LID_PR = float(os.environ.get("MAILER_LID_PR", "0.20"))  # the lid hand's fingers lie on the lid's outside face this far out from the hinge
LID_PX = float(os.environ.get("MAILER_LID_PX", "0.30"))  # and this far along the box (box frame x), 4 cm in from end 2's crease
EDGE = 0.012  # the closed fingers' narrow side, this far beside the TCP


LID_BETA = float(os.environ.get("MAILER_LID_BETA", "90"))  # the closed fingers' direction in the lid's plane: 0 along the lid toward the hinge, 90 along the hinge line toward end 1
LID_FLIP = os.environ.get("MAILER_LID_FLIP", "0") == "1"  # the hand turned over about its fingers


def lid_push(box, phi: float, r: float | None = None, xc: float | None = None, beta: float | None = None, press: float = PRESS):
    """The lid hand's TCP (world) and rotation with the lid at phi: the closed fingers lie in the lid's plane (along the hinge line at
    beta 90), their broad side on the lid's outside face r out from the hinge, so that the hand swings the lid over along its own arc,
    turning with it, as a door is swung by a hand flat on it."""
    r, xc, beta = LID_PR if r is None else r, LID_PX if xc is None else xc, LID_BETA if beta is None else beta
    p, b = math.radians(phi), math.radians(beta)
    u, n = np.array([0.0, -math.sin(p), math.cos(p)]), np.array([0.0, math.cos(p), math.sin(p)])
    a = math.cos(b) * (-u) + math.sin(b) * np.array([-1.0, 0.0, 0.0])  # where the fingers point (box frame)
    Rl = np.stack([np.cross(n, a), n, a], axis=1)  # the jaws across the face: the fingers' broad side on it
    if LID_FLIP:
        Rl = Rl @ np.diag([-1.0, -1.0, 1.0])
    local = np.array([xc, WY / 2, H]) + r * u + T * n + n * (SIDE - press)
    return box.to_world(local)[0], box.rotation() @ Rl


def lid_grip(box, k_side: str, angle: float) -> np.ndarray:
    """A level grip along the lid (right hand: fingers toward end 1, +y; left: toward end 2, -y), turned with the lid or tuck about the
    hinge line by `angle`, so that the fingers' broad side stays on the face."""
    R = box.rotation()
    g = d.level(90.0 if k_side == "right" else -90.0).rot
    return R @ rot_x(angle) @ nominal_rot(0).T @ g


def recentre(ctx, goal_y: float = 0.17):
    """After end 2: the left hand pushes on end 1's locked wall from outside, sliding the box back toward the middle and, off the wall's
    middle by the observed turn, back square, as at the ends' slides."""
    l = ctx.left
    k = 0
    G = hold_grip_end(k)
    box = ctx.state("mailer")
    R0 = end_rot(box, k)
    start = _w(box, k, hold_tcp_end(k, 0.0, press=-0.02, across=0.0))
    yield from l.move(start + R0 @ np.array([-0.06, 0.0, 0.0]) + [0.0, 0.0, 0.12], grip=d._as_grip(_wr(box, k, G)), finger=d.FINGER_CLOSED, t=1.4, label="above, outside end 1")
    yield from l.move(start, t=0.8, label="outside end 1")
    crease0 = box.to_world(end_local(k, (0.0, 0.0, 0.0)))[0]
    steps = 60
    for j in range(1, steps + 1):
        box = ctx.state("mailer")
        R = end_rot(box, k)
        crease = crease0.copy()
        crease[1] = crease0[1] + (goal_y - crease0[1]) * d.ik.smoothstep(j / steps)
        across = float(np.clip(SQUARE_GAIN * (box_yaw(box) - YAW), -0.08, 0.08))
        mm, deg = l._solve(crease + R @ hold_tcp_end(k, 0.0, press=HOLD_PRESS, across=across), R @ G)
        l._note(mm, deg, "slide the box back to the middle")
        yield
    box = ctx.state("mailer")
    print(f"[mailer] box back at {np.round((box.pos - [BOX_AT[0], BOX_AT[1], TOP]) * 1000, 0)} mm, turned {box_yaw(box) - YAW:.1f} deg", flush=True)
    yield from l.move(l.at() + end_rot(box, k) @ np.array([-0.05, 0.0, 0.05]), t=0.6, label="off end 1")
    yield from l.move(LEFT_AWAY, grip=AWAY_GRIP, t=1.0, label="away")


LID_LEAD = float(os.environ.get("MAILER_LID_LEAD", "10"))  # degrees: how far ahead of the lid the hand aims while it swings it
LID_RATE = 1.2  # degrees a step (24 a second) at most


def _swing(ctx, stop: float, label: str, until=None, max_s: float = 6.0):
    """The right hand swings the lid along its arc to `stop`, aiming LID_LEAD ahead of the lid as it is read each step (its joint angle),
    and following the box as it is. Returns False when the box stands out of reach."""
    r = ctx.right
    aim = ctx.deg("mailer", "lid")
    for j in range(round(max_s / DT)):
        box = ctx.state("mailer")
        now = ctx.deg("mailer", "lid")
        aim = min(now + LID_LEAD, aim + LID_RATE, stop)
        press = PRESS + 0.012 * max(0.0, aim - 80.0) / 12.0  # pressing harder over the last degrees, against the spring and the tuck
        P, R = lid_push(box, aim, press=press)
        mm, deg = r._solve(P, R)
        r._note(mm, deg, label)
        yield
        if mm > 40.0:
            print(f"[mailer] lid: out of reach ({mm:.0f} mm) at lid {now:.1f} deg", flush=True)
            return False
        if until is not None and until() and j > 10:
            return True
        if until is None and aim >= stop and now >= stop - 3.0 and j > 10:
            return True
    return True


def close_lid(ctx, mode: str):
    """The right hand comes in from beyond end 2 behind the standing lid, its closed fingers along the hinge line and their broad side on
    the lid's outside face, and swings the lid over along its arc to LID_HOLD, where it holds it; the left hand folds the
    tuck down past square; the right swings the lid on down shut, the folded tuck going down inside the front wall."""
    l, r = ctx.left, ctx.right
    shut = d.FINGER_CLOSED
    box = ctx.state("mailer")
    side, R0 = lid_push(box, 0.0, xc=LID_PX + 0.10, press=-0.03)
    yield from r.move(side + [0.0, 0.0, 0.04], grip=d._as_grip(R0), finger=shut, t=1.6, label="beyond end 2, behind the lid's line")
    yield from r.move(side, t=0.5, label="down beside the lid")
    yield from r.move(lid_push(box, 0.0, press=-0.03)[0], t=0.8, label="in behind the lid")
    yield from r.move(lid_push(box, 0.0, press=0.0)[0], t=0.5, label="on the lid")
    ok = yield from _swing(ctx, LID_HOLD, "swing the lid over")
    print(f"[mailer] lid held at {ctx.deg('mailer', 'lid'):.1f} deg", flush=True)
    if not ok or ctx.deg("mailer", "lid") < LID_HOLD - 10.0:  # the swing did not carry the lid over: leave it rather than work a lid out of place
        print("[mailer] lid: not carried over; the hands leave", flush=True)
        box = ctx.state("mailer")
        yield from r.move(r.at() + box.rotation() @ np.array([0.0, 0.06, 0.0]) + [0.0, 0.0, 0.10], t=0.8, label="off the lid")
        return
    if mode == "undo":  # the lid is swung shut with its tuck unfolded, held 1.5 s, and let go: it springs open before the end
        yield from _swing(ctx, 90.0 + OVER, "swing the lid shut", until=lambda: ctx.deg("mailer", "lid") >= 88.0)
        for _ in range(30):
            yield
        yield from _off_lid(ctx)
        return
    # The left hand comes above the tuck, side-on, and folds it down past square, following it, while the right holds the lid.
    box = ctx.state("mailer")
    phi = ctx.deg("mailer", "lid")
    yield from l.move(box.to_world(tuck_tcp(phi, 0.0, press=-0.04))[0] + [0.0, 0.0, 0.03], grip=d._as_grip(lid_grip(box, "left", phi)), finger=shut, t=1.4, label="above the tuck")
    yield from l.move(box.to_world(tuck_tcp(phi, 0.0, press=-0.005))[0], t=0.6, label="on the tuck")
    aim = 0.0
    for j in range(round(5.0 / DT)):
        box = ctx.state("mailer")
        phi, now = ctx.deg("mailer", "lid"), ctx.deg("mailer", "tuck")
        aim = min(now + 25.0, aim + 3.0, TUCK_TO + 5.0)
        mm, deg = l._solve(box.to_world(tuck_tcp(phi, aim))[0], lid_grip(box, "left", min(phi + aim, TUCK_GRIP_MAX)))
        l._note(mm, deg, "fold the tuck")
        P, R = lid_push(box, LID_HOLD)  # the right hand keeps the lid where it is
        mm, deg = r._solve(P, R)
        r._note(mm, deg, "hold the lid")
        yield
        if now >= TUCK_TO and j > 10:
            break
    print(f"[mailer] tuck folded to {ctx.deg('mailer', 'tuck'):.1f} deg, lid at {ctx.deg('mailer', 'lid'):.1f}", flush=True)
    box = ctx.state("mailer")
    yield from l.move(l.at() + box.rotation() @ np.array([0.0, -0.04, 0.05]), t=0.6, label="off the tuck")
    yield from l.move(LEFT_AWAY, grip=AWAY_GRIP, t=1.0, label="away")
    # The right hand swings the lid on down shut; the tuck goes down inside the front wall and is held there.
    yield from _swing(ctx, 90.0 + OVER, "swing the lid shut", until=lambda: TRIAL.lid_locked)
    for _ in range(5):
        yield
    print(f"[mailer] lid {ctx.deg('mailer', 'lid'):.1f} deg, tuck {ctx.deg('mailer', 'tuck'):.1f} deg, seated {TRIAL.lid_locked}", flush=True)
    yield from _off_lid(ctx)


def _off_lid(ctx):
    r = ctx.right
    box = ctx.state("mailer")
    yield from r.move(r.at() + box.rotation() @ np.array([0.0, 0.0, 0.06]), t=0.6, label="up off the lid")
    yield from r.move(RIGHT_AWAY, grip=AWAY_GRIP, t=1.0, label="away")


# -- the lid with the box turned a quarter, its hinge to one side ------------------------------------------------------------------------

S_FAM = os.environ.get("MAILER_SL_FAM", "side")  # how the lid hand meets the lid's outside face (face_pose)
S_XC = float(os.environ.get("MAILER_SL_XC", "0.30"))  # where along the hinge (box frame x)
S_R = float(os.environ.get("MAILER_SL_R", "0.20"))  # and how far out from it
S_FLIP = os.environ.get("MAILER_SL_FLIP", "1") == "1"
T_FAM = os.environ.get("MAILER_ST_FAM", "side")  # the same for the tuck hand on the tuck's outside face
T_XC = float(os.environ.get("MAILER_ST_XC", "0.10"))
T_FLIP = os.environ.get("MAILER_ST_FLIP", "1") == "1"
S_HOLD = float(os.environ.get("MAILER_SL_HOLD", "65"))  # degrees: the lid is held here while the tuck is folded
LID_OBS = os.environ.get("MAILER_LID_OBS", "1") == "1"  # the lid hand comes to the lid at its observed angle (it leans out to -5 after the turn)


def face_pose(box, crease, a_deg: float, r: float, fam: str, flip: bool, press: float = PRESS):
    """The TCP (world) and rotation for a hand on a panel's outside face (the lid, or the tuck), the panel standing at a_deg about its
    crease line (box frame x through `crease`; 0 straight up, 90 lying over the box), r out from the crease. tip: the closed fingers
    point into the face, their ends on it, the jaws along the crease; tipr: the jaws across it; side: the fingers lie along the crease,
    pointing forward, their broad side on the face; hook: they lie along the panel toward its crease, broad side on the face."""
    a = math.radians(a_deg)
    u, n = np.array([0.0, -math.sin(a), math.cos(a)]), np.array([0.0, math.cos(a), math.sin(a)])
    face = np.asarray(crease, float) + r * u + T * n
    fwd = box.rotation().T @ np.array([1.0, 0.0, 0.0])  # world forward, in the box frame
    xb = np.array([1.0 if fwd[0] >= 0 else -1.0, 0.0, 0.0])
    if fam in ("tip", "tipr", "tipf"):  # tipf: as tip, the fingers turned 22 degrees forward (off the level -y wrist singularity, README 21)
        az = -n if fam != "tipf" else (-n + 0.4 * fwd) / np.linalg.norm(-n + 0.4 * fwd)
        ay = xb if fam != "tipr" else u
        ay = (ay - (ay @ az) * az) / np.linalg.norm(ay - (ay @ az) * az)
        P = face + n * (TIP - press)
    elif fam == "side":
        az, ay, P = xb, n, face + n * (SIDE - press)
    else:
        az, ay, P = -u, n, face + n * (SIDE - press)
    Rl = np.stack([np.cross(ay, az), ay, az], axis=1)
    if flip:
        Rl = Rl @ np.diag([-1.0, -1.0, 1.0])
    return box.to_world(P)[0], box.rotation() @ Rl, box.rotation() @ n


def lid_crease(xc: float) -> np.ndarray:
    return np.array([xc, WY / 2, H])


def tuck_crease(xc: float, phi: float) -> np.ndarray:
    p = math.radians(phi)
    return lid_crease(xc) + LID_L * np.array([0.0, -math.sin(p), math.cos(p)])


def hinge_hand(ctx, box):
    """The hand on the lid hinge's side of the box, then the other."""
    left = (box.rotation() @ np.array([0.0, 1.0, 0.0]))[1] > 0
    return (ctx.left, ctx.right) if left else (ctx.right, ctx.left)


def _swing_side(ctx, hand, stop: float, label: str, until=None, max_s: float = 6.0):
    aim = ctx.deg("mailer", "lid")
    for j in range(round(max_s / DT)):
        box = ctx.state("mailer")
        now = ctx.deg("mailer", "lid")
        aim = min(now + LID_LEAD, aim + LID_RATE, stop)
        press = PRESS + 0.012 * max(0.0, aim - 80.0) / 12.0  # harder over the last degrees, against the spring and the tuck
        P, R, _ = face_pose(box, lid_crease(S_XC), aim, S_R, S_FAM, S_FLIP, press=press)
        mm, deg = hand._solve(P, R)
        hand._note(mm, deg, label)
        if os.environ.get("MAILER_LOG") and (j % 10 == 0 or mm > 10.0):
            print(f"[mailer] {label} j {j}: lid {now:.1f} aim {aim:.1f}, target {np.round(P * 100, 1)} cm, hand {np.round(hand.at() * 100, 1)}, miss {mm:.0f} mm {deg:.0f} deg, q {np.round(np.degrees(hand.q), 0)}", flush=True)
        yield
        if mm > 40.0:
            print(f"[mailer] lid: out of reach ({mm:.0f} mm) at lid {now:.1f} deg", flush=True)
            return False
        if until is not None and until() and j > 10:
            return True
        if until is None and aim >= stop and now >= stop - 3.0 and j > 10:
            return True
    return True


def to_ready(arm, t: float = 1.2):
    """The arm to its ready joint angles along a smooth joint-space path (the lid-only check starts there)."""
    q0, q1 = arm.q.copy(), np.array(d.READY, float)
    n = max(1, round(t / DT))
    for k in range(1, n + 1):
        s = k / n
        arm.q = q0 + (q1 - q0) * s * s * (3.0 - 2.0 * s)
        arm.cmd = np.r_[arm.q, float(np.clip(arm.finger / d.FINGER_OPEN, 0.0, 1.0))]
        yield
    arm.grip = d._as_grip(d.ik.tcp_pose(q1, arm.base)[1])


def close_lid_side(ctx, mode: str):
    """The box turned with its hinge to one side: the hand on that side comes to the lid's outside face and swings the lid over along
    its arc to S_HOLD, where it holds it; the other hand folds the tuck down past square; the first presses the lid on down shut from
    above, the folded tuck going down inside the far wall."""
    box = ctx.state("mailer")
    lh, th = hinge_hand(ctx, box)
    shut = d.FINGER_CLOSED
    zb = box.rotation() @ np.array([0.0, 0.0, 1.0])
    print(f"[mailer] lid start: box at {box_yaw(box):.1f} deg, middle {np.round(box.to_world(np.array([LX / 2, 0.0, 0.0]))[0] * 100, 1)} cm, "
          f"tilt {math.degrees(math.acos(min(1.0, zb[2]))):.1f} deg, lid {ctx.deg('mailer', 'lid'):.1f}, tuck {ctx.deg('mailer', 'tuck'):.1f}", flush=True)
    if mode != "lidonly":  # both hands back to the ready pose first, in joint space, so that their IK runs on the branch that reaches the lid's whole arc
        yield from ctx.together(to_ready(lh), to_ready(th))
    box = ctx.state("mailer")  # the hand comes to the lid where it is seen now: the box as the turn left it, the lid at its observed angle
    a0 = min(0.0, ctx.deg("mailer", "lid")) if LID_OBS else 0.0
    P, R, n = face_pose(box, lid_crease(S_XC), a0, S_R, S_FAM, S_FLIP, press=-0.03)
    yield from lh.move(P + n * 0.05 + [0.0, 0.0, 0.03], grip=d._as_grip(R), finger=shut, t=1.6, label="before the lid")
    yield from lh.move(P, t=0.8, label="to the lid")
    yield from lh.move(face_pose(box, lid_crease(S_XC), a0, S_R, S_FAM, S_FLIP, press=0.0)[0], t=0.5, label="on the lid")
    seat = SEAT and mode != "hover"
    bare = mode in ("undo", "shut")  # the lid alone, its tuck left unfolded
    hold = SEAT_HOLD if (seat and not bare) else S_HOLD
    if a0 < -2.0:  # leaning out: first stood up square by feedback on its read angle, then swung over from there
        yield from _swing_side(ctx, lh, 0.0, "stand the lid up", until=lambda: ctx.deg("mailer", "lid") >= -1.0, max_s=2.0)
        print(f"[mailer] lid stood up to {ctx.deg('mailer', 'lid'):.1f} deg", flush=True)
    ok = yield from _swing_side(ctx, lh, hold, "swing the lid over")
    print(f"[mailer] lid held at {ctx.deg('mailer', 'lid'):.1f} deg", flush=True)
    if not ok or ctx.deg("mailer", "lid") < hold - 10.0:
        print("[mailer] lid: not carried over; the hands leave", flush=True)
        yield from lh.move(lh.at() + [0.0, 0.0, 0.10], t=0.8, label="off the lid")
        return
    if seat and not bare:
        yield from seat_tuck(ctx, lh, th)
        return
    if not bare:
        box = ctx.state("mailer")
        phi = ctx.deg("mailer", "lid")
        P, R, n = face_pose(box, tuck_crease(T_XC, phi), phi, TUCK_S, T_FAM, T_FLIP, press=-0.04)
        yield from th.move(P + n * 0.03, grip=d._as_grip(R), finger=shut, t=1.4, label="above the tuck")
        yield from th.move(face_pose(box, tuck_crease(T_XC, phi), phi, TUCK_S, T_FAM, T_FLIP, press=-0.005)[0], t=0.6, label="on the tuck")
        aim = 0.0
        for j in range(round(5.0 / DT)):
            box = ctx.state("mailer")
            phi, now = ctx.deg("mailer", "lid"), ctx.deg("mailer", "tuck")
            aim = min(now + 25.0, aim + 3.0, TUCK_TO + 5.0)
            P, R, _ = face_pose(box, tuck_crease(T_XC, phi), phi + aim, TUCK_S, T_FAM, T_FLIP)
            mm, deg = th._solve(P, R)
            th._note(mm, deg, "fold the tuck")
            P, R, _ = face_pose(box, lid_crease(S_XC), S_HOLD, S_R, S_FAM, S_FLIP)  # the lid hand keeps the lid where it is
            mm, deg = lh._solve(P, R)
            lh._note(mm, deg, "hold the lid")
            yield
            if now >= TUCK_TO and j > 10:
                break
        print(f"[mailer] tuck folded to {ctx.deg('mailer', 'tuck'):.1f} deg, lid at {ctx.deg('mailer', 'lid'):.1f}", flush=True)
        box = ctx.state("mailer")
        n = face_pose(box, tuck_crease(T_XC, ctx.deg("mailer", "lid")), ctx.deg("mailer", "lid") + ctx.deg("mailer", "tuck"), TUCK_S, T_FAM, T_FLIP)[2]
        yield from th.move(th.at() + n * 0.04 + [0.0, 0.0, 0.04], t=0.6, label="off the tuck")
        yield from th.move(RIGHT_AWAY if th is ctx.right else LEFT_AWAY, grip=AWAY_GRIP, t=1.0, label="away")
    yield from _swing_side(ctx, lh, 90.0 + OVER, "press the lid shut", until=(lambda: ctx.deg("mailer", "lid") >= 88.0) if bare else (lambda: TRIAL.lid_locked))
    for _ in range(30 if bare else 5):
        yield
    print(f"[mailer] lid {ctx.deg('mailer', 'lid'):.1f} deg, tuck {ctx.deg('mailer', 'tuck'):.1f} deg, seated {TRIAL.lid_locked}", flush=True)
    if mode == "shut":  # the hand stays on the lid (demo_script)
        return
    yield from lh.move(lh.at() + [0.0, 0.0, 0.06], t=0.6, label="up off the lid")
    yield from lh.move(RIGHT_AWAY if lh is ctx.right else LEFT_AWAY, grip=AWAY_GRIP, t=1.0, label="away")


# -- seating the tuck as a person does: the lid 35 degrees open, the tuck curled in by a fingertip, the lid pressed down over it ----------

SEAT = os.environ.get("MAILER_SEAT", "1") == "1"
SEAT_HOLD = float(os.environ.get("MAILER_SEAT_HOLD", "55"))  # degrees: the lid is held here (35 open) while the tuck is curled
SEAT_PSI = float(os.environ.get("MAILER_SEAT_PSI", "125"))  # the tuck curled this far under the lid, its edge pointing down and in
SEAT_R = float(os.environ.get("MAILER_SEAT_R", "0.030"))  # the fingertip on the tuck's outside face this far from its crease
SEAT_XC = float(os.environ.get("MAILER_SEAT_XC", "0.10"))  # and this far along the box (box frame x)
SEAT_REF, SEAT_K = float(os.environ.get("MAILER_SEAT_REF", "130")), float(os.environ.get("MAILER_SEAT_K", "0"))  # 0: the fingers keep one direction (plan_seat.py: reaches the whole curl and close, 0 mm)
SEAT_FLIP = os.environ.get("MAILER_SEAT_FLIP", "1") == "1"
SEAT_TILT = float(os.environ.get("MAILER_SEAT_TILT", "0.6"))
SEAT_REL = float(os.environ.get("MAILER_SEAT_REL", "0.004"))  # the fingertip lets go once the tuck's edge is inside the far wall, this high over its top
FRONT_R, FRONT_MAX = LID_L - 0.020, 0.012  # the closing press: a fingertip on the lid's top 2 cm in from its front edge, at most this far in


def tip_on_tuck(box, phi: float, psi: float, press: float = PRESS):
    """The tuck hand's TCP (world), rotation and the tuck's outside normal (world), its closed fingers' ends on the tuck's outside face
    SEAT_R from its crease, the lid at phi and the tuck folded psi under it. The fingers turn with the face at SEAT_K of its rate, about
    SEAT_REF, so that a fingertip pushes it round without winding the wrist."""
    a = math.radians(phi + psi)
    u, n = np.array([0.0, -math.sin(a), math.cos(a)]), np.array([0.0, math.cos(a), math.sin(a)])
    tip = tuck_crease(SEAT_XC, phi) + SEAT_R * u + (T - press) * n
    h = math.radians(SEAT_REF + SEAT_K * (phi + psi - SEAT_REF))
    az = -np.array([0.0, math.cos(h), math.sin(h)])
    fwd = box.rotation().T @ np.array([1.0, 0.0, 0.0])
    if SEAT_TILT:  # the fingers turned toward the robot's forward, off the wrist's singular line (README 21)
        az = (az + SEAT_TILT * fwd) / np.linalg.norm(az + SEAT_TILT * fwd)
    xb = np.array([1.0 if fwd[0] >= 0 else -1.0, 0.0, 0.0])
    ay = (xb - (xb @ az) * az) / np.linalg.norm(xb - (xb @ az) * az)
    Rl = np.stack([np.cross(ay, az), ay, az], axis=1)
    if SEAT_FLIP:
        Rl = Rl @ np.diag([-1.0, -1.0, 1.0])
    return box.to_world(tip - TIP * az)[0], box.rotation() @ Rl, box.rotation() @ n


def _hold_lid(ctx, lh, box, phi: float, press: float = PRESS) -> None:
    P, R, _ = face_pose(box, lid_crease(S_XC), phi, S_R, S_FAM, S_FLIP, press=press)
    lh._note(*lh._solve(P, R), "hold the lid")


def seat_tuck(ctx, lh, th):
    """With the lid held SEAT_HOLD, the other hand's fingertip curls the tuck in under it to SEAT_PSI, its edge pointing down and in; the
    lid hand swings the lid on down while the fingertip keeps the curl, until the tuck's edge is inside the far wall at its top; the
    fingertip lets go and the lid goes on down, the tuck sliding down inside the wall; a fingertip presses the lid's front edge flat, by
    feedback on the lid's angle, until the tuck holds it shut."""
    lid = lambda: ctx.deg("mailer", "lid")  # noqa: E731
    tuck = lambda: ctx.deg("mailer", "tuck")  # noqa: E731
    box = ctx.state("mailer")
    P, R, n = tip_on_tuck(box, lid(), 0.0, press=-0.03)
    yield from th.move(P + n * 0.03, grip=d._as_grip(R), finger=d.FINGER_CLOSED, t=1.4, label="above the tuck")
    yield from th.move(tip_on_tuck(box, lid(), 0.0, press=-0.004)[0], t=0.6, label="fingertip on the tuck")
    aim = max(0.0, tuck())
    for j in range(round(6.0 / DT)):
        box, now = ctx.state("mailer"), tuck()
        aim = min(now + 20.0, aim + 2.5, SEAT_PSI + 5.0)
        P, R, _ = tip_on_tuck(box, lid(), aim)
        th._note(*th._solve(P, R), "curl the tuck")
        _hold_lid(ctx, lh, box, SEAT_HOLD)
        yield
        if now >= SEAT_PSI and j > 10:
            break
    print(f"[mailer] tuck curled to {tuck():.1f} deg, lid at {lid():.1f}", flush=True)
    aim = lid()
    for j in range(round(8.0 / DT)):
        box, phi, psi = ctx.state("mailer"), lid(), tuck()
        aim = min(phi + LID_LEAD, aim + 0.8, 90.0 + OVER)
        _hold_lid(ctx, lh, box, aim)
        e = TRIAL.seen.tuck_edge
        if (e[1] > -WY / 2 + T + 0.002 and e[2] < H + SEAT_REL) or phi > 86.0:
            print(f"[mailer] tuck's edge inside the far wall at lid {phi:.1f}, tuck {psi:.1f}: edge {np.round(e * 1000, 1)} mm; the fingertip lets go", flush=True)
            break
        P, R, _ = tip_on_tuck(box, phi, max(SEAT_PSI, psi))
        th._note(*th._solve(P, R), "keep the curl")
        yield
    yield from th.move(th.at() + [0.0, 0.0, 0.05], t=0.5, label="fingertip up off the tuck")
    yield from _swing_side(ctx, lh, 90.0 + OVER, "press the lid down", until=lambda: lid() >= 88.0 or TRIAL.lid_locked)
    box = ctx.state("mailer")
    R0 = th.grip.rot  # the fingertip keeps the curl's grip: it presses the lid's top 2 cm in from its front edge, straight down
    az = R0[:, 2]

    def front(press: float) -> np.ndarray:
        b = ctx.state("mailer")
        return b.to_world(np.array([SEAT_XC, WY / 2 - FRONT_R, H + T - press]))[0] - TIP * az

    yield from th.move(front(-0.03), t=1.0, label="above the lid's front edge")
    press, good = -0.01, 0
    for j in range(round(5.0 / DT)):
        box = ctx.state("mailer")
        flat = lid() >= 89.0 and TRIAL.lid_locked
        good = good + 1 if flat else 0
        press = min(press + 0.0008, FRONT_MAX) if not flat else press
        th._note(*th._solve(front(press), R0), "press the front edge flat")
        _hold_lid(ctx, lh, box, 90.0 + OVER)
        yield
        if good >= 10:
            break
    print(f"[mailer] lid {lid():.1f} deg, tuck {tuck():.1f} deg, seated {TRIAL.lid_locked}, front press {press * 1000:.1f} mm", flush=True)
    yield from lh.move(lh.at() + [0.0, 0.0, 0.06], t=0.6, label="up off the lid")
    yield from lh.move(RIGHT_AWAY if lh is ctx.right else LEFT_AWAY, grip=AWAY_GRIP, t=1.0, label="away")
    yield from th.move(th.at() + [0.0, 0.0, 0.06], t=0.6, label="up off the lid's front edge")


# -- turning the box a quarter on the table, after the ends ---------------------------------------------------------------------------

TURN_TO = float(os.environ.get("MAILER_TURN_TO", "0"))  # the box is turned until its x axis has this heading: 0 puts the lid's hinge to the left
TURN_LEAD, TURN_RATE, TURN_PASS = 6.0, 0.8, float(os.environ.get("MAILER_TURN_PASS", "28"))  # degrees ahead of the box, a step at most, a pass at most (28: the end wall takes over at about -42, f2; from -27 its corner is out of reach, f1)
TURN_IN = float(os.environ.get("MAILER_TURN_IN", "0.07"))  # the contacts this far in from the wall's ends
TURN_MID = np.array([float(os.environ.get("MAILER_TURN_MX", "0.32")), float(os.environ.get("MAILER_TURN_MY", "0.0"))])  # where the box's middle is slid to as it turns
TURN_SLIDE = float(os.environ.get("MAILER_TURN_SLIDE", "0.02"))  # m: how far ahead of the box's middle the hands lead it, along the wall's normal
WALLS = {"front": ((0.0, -1.0, 0.0), (LX / 2, -WY / 2), (1.0, 0.0, 0.0), LX / 2, T),
         "end1": ((-1.0, 0.0, 0.0), (0.0, 0.0), (0.0, 1.0, 0.0), WY / 2, ROLLED),
         "end2": ((1.0, 0.0, 0.0), (LX, 0.0), (0.0, 1.0, 0.0), WY / 2, ROLLED)}  # outward normal, middle, along, half length, thickness (box frame)


def _heading(v) -> float:
    return math.degrees(math.atan2(v[1], v[0]))


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def turn_box(ctx):
    """After the ends: turn the box on the table, sliding, until the lid's hinge is to one side (TURN_TO), in passes of up to TURN_PASS
    degrees. Each pass takes the wall that faces the robot most nearly: one hand pushes its outside face near one end, away from the
    robot, and the other hooks its inside face near the other end and pulls toward the robot, so that the two make a turn and little
    slide. Both hands follow their contacts round the box's middle a few degrees ahead of the box's turn as it is read each step. Then
    both hands leave round the box's near end, clear of the standing lid, and the box's turn is read again: a turn knocked off on the
    way out takes more passes."""
    for attempt in range(2):
        yield from _turn_passes(ctx)
        yield from centre_box(ctx)
        yield from clear_box(ctx)
        box = ctx.state("mailer")
        err = _wrap(TURN_TO - box_yaw(box))
        print(f"[mailer] turn, hands clear: box at {box_yaw(box):.1f} deg, middle {np.round(box.to_world(np.array([LX / 2, 0.0, 0.0]))[0][:2] * 100, 1)} cm, "
              f"lid {ctx.deg('mailer', 'lid'):.1f}", flush=True)
        if abs(err) <= 4.0:
            break


CENTRE_Y = float(os.environ.get("MAILER_CENTRE_Y", "-0.015"))  # m: after the turn, the box's middle is slid across the table to this y, where the
# way out past its near end and the lid's whole arc solve exactly (plan_join.py, plan_exit_ik.py)
CENTRE_TOL, CENTRE_SQ = 0.010, 2.0  # m and degrees: closer than this to CENTRE_Y and to square, the box is left as it is


def _down_along(arm, P, box):
    """A straight-down grip with the jaws along the box's x axis (the front wall), whichever of its two turns the arm solves better at P."""
    best = None
    for k in (90.0, -90.0):  # down(0) has its jaws along the station's y: turned 90 degrees more than the box, they lie along its x
        R = _rz(box_yaw(box) + k) @ d.down(0.0).rot
        _, pe, re_ = d.ik.solve(P, d.ik.matrix_to_quat_xyzw(R), np.asarray(arm.q, float), base_pos=arm.base, iters=150)
        if best is None or pe + 0.05 * re_ < best[0]:
            best = (pe + 0.05 * re_, R)
    return best[1]


def _joint_or_move(arm, P, R, label: str, t: float = 0.8):
    """To P along a joint-space path when a multi-seed solve finds it (a hand near its wrist's limit), else a plain move."""
    q = ik_near(arm, np.asarray(P, float), R)
    if q is None:
        print(f"[mailer] no exact solve for {label}", flush=True)
        yield from arm.move(P, grip=d._as_grip(R), t=t, label=label)
    else:
        yield from joint_to(arm, q, t=t)


def centre_box(ctx):
    """After the turn passes, before the hands leave: read the box's pose; if its middle is more than CENTRE_TOL off CENTRE_Y across the
    table, or the box is more than CENTRE_SQ degrees off square, the right hand slides it back across the table by feedback on its observed
    pose. Its fingers go down inside the front wall and draw the box toward the right hand's side, or down outside it and push the box the
    other way, off the wall's middle by the observed turn so that it comes back square, until the middle is within 3 mm. The left hand
    first moves in over the box's middle, clear of the standing lid, and leaves from there."""
    l, r = ctx.left, ctx.right
    box = ctx.state("mailer")
    mid = box.to_world(np.array([LX / 2, 0.0, 0.0]))[0]
    turn = _wrap(box_yaw(box) - TURN_TO)
    off = mid[1] - CENTRE_Y
    print(f"[mailer] centre: box at {box_yaw(box):.1f} deg, middle {np.round(mid[:2] * 100, 1)} cm, {off * 100:.1f} cm off across", flush=True)
    if (abs(off) <= CENTRE_TOL and abs(turn) <= CENTRE_SQ) or abs(turn) > 10.0:  # well off its turn: the turn's next passes come first
        return
    toward = box.rotation() @ np.array([0.0, -1.0, 0.0])  # the front wall's outside normal: toward the right hand's side once turned
    pull = -off * toward[1] >= 0.0  # the box must go toward the front wall's outside: drawn from inside; otherwise pushed from outside
    yb = (-WY / 2 + T + EDGE + 0.004) if pull else (-WY / 2 - EDGE - 0.004)
    sgn = 1.0 if pull else -1.0  # moving along the outside normal, off the middle by +turn, turns the box back; the other way, by -turn
    yaw0 = box_yaw(box)
    lb, lg = box.to_local(l.at())[0], l.grip.rot.copy()
    yield from _joint_or_move(l, box.to_world(lb + np.array([0.0, -0.08, 0.03]))[0], lg, "in over the box, clear of the lid")
    yield from r.move(r.at() + [0.0, 0.0, 0.08], t=0.6, label="up")
    yield from to_ready(r)  # in joint space, so that its IK starts on the branch that reaches over the front wall
    hook = lambda b, xm: b.to_world(np.array([xm, yb, H - 0.03]))[0]  # noqa: E731
    G = _down_along(r, hook(box, LX / 2), box)
    yield from r.move(hook(box, LX / 2) + [0.0, 0.0, 0.05], grip=d._as_grip(G), finger=d.FINGER_CLOSED, t=1.2, label="above the front wall")  # 10 cm up is out of reach there (plan_centre_ik.py)
    yield from r.move(hook(box, LX / 2), t=0.8, label="down inside the front wall" if pull else "down outside the front wall")
    p = r.at().copy()
    for j in range(round(5.0 / DT)):
        box = ctx.state("mailer")
        mid = box.to_world(np.array([LX / 2, 0.0, 0.0]))[0]
        rem = -(mid[1] - CENTRE_Y) * toward[1] * sgn  # how far the box has still to go, along the slide
        turn = _wrap(box_yaw(box) - TURN_TO)
        if (rem <= 0.003 and abs(turn) <= CENTRE_SQ) or rem <= -CENTRE_TOL:
            break
        n = box.rotation() @ np.array([0.0, -1.0, 0.0])
        p = p + n * 0.0015 * sgn  # 3 cm a second
        xm = LX / 2 + float(np.clip(sgn * SQUARE_GAIN * 2.0 * turn, -0.10, 0.10))
        q = hook(box, xm)
        target = q + n * float((p - q) @ n)  # along the wall's normal from the push, along the wall at the contact as the box now stands
        mm, deg = r._solve(target, _rz(box_yaw(box) - yaw0) @ G)
        r._note(mm, deg, "slide the box across")
        yield
        if mm > 20.0:  # the hand has lost the wall: stop rather than drag the box askew; the way out copes with the box where it is
            print(f"[mailer] centre: slide stopped, the hand {mm:.0f} mm off its aim", flush=True)
            break
    box = ctx.state("mailer")
    mid = box.to_world(np.array([LX / 2, 0.0, 0.0]))[0]
    print(f"[mailer] centred: box at {box_yaw(box):.1f} deg, middle {np.round(mid[:2] * 100, 1)} cm", flush=True)
    n = box.rotation() @ np.array([0.0, -1.0, 0.0])
    yield from r.move(r.at() - n * 0.01 * sgn, t=0.3, label="off the front wall")
    yield from r.move(r.at() + [0.0, 0.0, 0.05], t=0.5, label="up from the front wall")
    yield from r.move(RIGHT_AWAY, grip=AWAY_GRIP, t=1.0, label="away")  # the left hand stays in over the box, clear of the lid, for the way out


CLEAR_X = float(os.environ.get("MAILER_CLEAR_X", "0.10"))  # m: the hands leave the box this far nearer the robot than its near end
CLEAR_Y = float(os.environ.get("MAILER_CLEAR_Y", "0.24"))  # and then go out to this far to their own side


CLEAR_G = os.environ.get("MAILER_CLEAR_G", "keep")  # the hands' grip on the way out: keep, or turned to the away grip / straight down


def clear_path(box, l0, r0, lg, rg):
    """clear_box's waypoints: (label, left TCP, left rotation, right TCP, right rotation). Only the left hand (the hinge side's) is inside
    the box at the last pass; it leaves past the near end with the away grip, its wrist camera then clear of the lid (plan_join.py)."""
    corners = [box.to_world(np.array([x, y, 0.0]))[0] for x in (0.0, LX) for y in (-WY / 2, WY / 2)]
    xc = min(c[0] for c in corners) - CLEAR_X
    yl = max(c[1] for c in corners) + 0.07  # 7 cm outside the hinge side's wall, where the lid stands
    z = TOP + H + 0.05  # 5 cm over the walls: the left arm reaches the near-end corner at this height, not at 8 cm or more (plan_exit_ik.py)
    g = {"keep": None, "away": AWAY_GRIP.rot, "down0": d.down(0.0).rot, "down90": d.down(90.0).rot}[CLEAR_G]
    lg2, rg2 = (lg, rg) if g is None else (g, g)
    lz, rz = z, max(r0[2], z)
    return [("up clear of the walls", [l0[0], l0[1], lz], lg, [r0[0], r0[1], rz], rg),
            ("toward the robot, past the box's end", [xc, l0[1], lz], lg2, [r0[0], r0[1], rz], rg2),  # the right hand is already outside the near wall
            ("out to its side", [xc, min(CLEAR_Y, yl), lz], lg2, [r0[0], r0[1], rz], rg2)]  # left: just past the lid's line; right: stays


def ik_near(arm, P, R):
    """Joint angles for the TCP at P, solved from several seeds, with the rotation R or, where R has no exact solve, another of a few
    plain grips (the way out only needs the hand's place): of the exact solutions, the one nearest the arm's joints now."""
    best = None
    grips = [R] + [g.rot for g in (AWAY_GRIP, d.tilted(30.0, 90.0), d.tilted(30.0, -90.0), d.down(0.0), d.down(90.0), d.tilted(45.0, 0.0))]
    seeds = [arm.q] + [np.array(d.READY, float) + np.radians([a, 0, 0, w, 0, 0]) for a in (-40, -20, 0, 20, 40) for w in (-60, 0, 60)]
    for gi, Rg in enumerate(grips):
        for seed in seeds:
            q, pe, re_ = d.ik.solve(P, d.ik.matrix_to_quat_xyzw(Rg), np.asarray(seed, float), base_pos=arm.base, iters=150)
            if pe > 0.003 or re_ > math.radians(2.0):
                continue
            cost = float(np.abs(q - arm.q).max())
            if best is None or cost < best[1]:
                best = (q, cost)
        if best is not None and gi == 0:
            break
    return None if best is None else best[0]


def joint_to(arm, q1, t: float = 1.0):
    """The arm to joint angles q1 along a smooth joint-space path."""
    q0, q1 = arm.q.copy(), np.asarray(q1, float)
    n = max(1, round(t / DT))
    for k in range(1, n + 1):
        s = k / n
        arm.q = q0 + (q1 - q0) * s * s * (3.0 - 2.0 * s)
        arm.cmd = np.r_[arm.q, float(np.clip(arm.finger / d.FINGER_OPEN, 0.0, 1.0))]
        yield
    arm.grip = d._as_grip(d.ik.tcp_pose(q1, arm.base)[1])


def clear_box(ctx):
    """Both hands up over the walls, then toward the robot until they are clear of the box's near end, then out to their own side and
    back to the ready joint angles: never across the lid, which stands 47 cm over the table on the hinge side. Each waypoint is solved
    from several seeds and reached in joint space, since the hand that hooked inside the wall is near its wrist's limit there."""
    box = ctx.state("mailer")
    l, r = ctx.left, ctx.right
    for label, lp, lg, rp, rg in clear_path(box, l.at(), r.at(), l.grip.rot, r.grip.rot):
        moves = []
        for arm, P, R in ((l, lp, lg), (r, rp, rg)):
            q = ik_near(arm, np.asarray(P, float), R)
            moves.append(joint_to(arm, q, t=0.8) if q is not None else arm.move(P, grip=d._as_grip(R), t=0.8, label=label))
            if q is None:
                print(f"[mailer] clear: no exact solve for {label}", flush=True)
        yield from ctx.together(*moves)
    yield from ctx.together(to_ready(l), to_ready(r))


def _turn_passes(ctx):
    for p in range(5):
        box = ctx.state("mailer")
        err = _wrap(TURN_TO - box_yaw(box))
        print(f"[mailer] turn: box at {box_yaw(box):.1f} deg, {err:.1f} to go", flush=True)
        if abs(err) <= 4.0:
            break
        sgn = 1.0 if err > 0 else -1.0
        R = box.rotation()
        name = min((k for k in WALLS), key=lambda k: (R @ np.array(WALLS[k][0]))[0])
        n_b, c_b, t_b, half, thick = WALLS[name]
        n_b, t_b = np.array(n_b), np.array(t_b)
        ends = [np.r_[c_b, 0.0] + t_b * s * (half - TURN_IN) for s in (-1.0, 1.0)]
        ys = [box.to_world(e)[0][1] for e in ends]
        # turning up (counterclockwise from above), the push is at the wall's right-hand end and the hook at its left; down, the other way
        right_end, left_end = (ends[0], ends[1]) if ys[0] < ys[1] else (ends[1], ends[0])
        push_b, hook_b = (right_end, left_end) if sgn > 0 else (left_end, right_end)
        pusher, hooker = (ctx.right, ctx.left) if sgn > 0 else (ctx.left, ctx.right)
        push_b = push_b + n_b * EDGE + np.array([0.0, 0.0, SQUARE_Z])
        hook_b = hook_b - n_b * (thick + EDGE + 0.004) + np.array([0.0, 0.0, H - 0.03])
        mid = box.to_world(np.array([LX / 2, 0.0, 0.0]))[0]

        def grips(Rb):
            nw, tw = Rb @ n_b, Rb @ t_b
            return d.tilted(SQUARE_TILT, _heading(-nw)).rot, d.down(_heading(tw) - 90.0).rot, nw

        Gp, Gh, nw = grips(R)
        hp, pp = box.to_world(hook_b)[0], box.to_world(push_b)[0]
        yield from ctx.together(hooker.move(hp + [0.0, 0.0, 0.12], grip=d._as_grip(Gh), finger=d.FINGER_CLOSED, t=1.4, label="above the box, inside a wall"),
                                pusher.move(pp + nw * 0.03 + [0.0, 0.0, 0.10], grip=d._as_grip(Gp), finger=d.FINGER_CLOSED, t=1.4, label="above, outside the wall"))
        yield from ctx.together(hooker.move(hp, t=0.8, label="down inside the wall"), pusher.move(pp + nw * 0.01, t=0.8, label="down outside the wall"))
        y0 = box_yaw(box)
        goal = y0 + sgn * min(abs(err), TURN_PASS)
        aim = y0
        for j in range(round(6.0 / DT)):
            box = ctx.state("mailer")
            now = y0 + _wrap(box_yaw(box) - y0)  # unwrapped from the pass's start
            if sgn * (now - goal) >= -1.0:
                break
            aim = aim + sgn * TURN_RATE
            if sgn * (aim - now) > TURN_LEAD:
                aim = now + sgn * TURN_LEAD
            if sgn * (aim - goal) > 2.0:
                aim = goal + sgn * 2.0
            Rz = _rz(aim - now)
            Rb = Rz @ box.rotation()
            c = box.to_world(np.array([LX / 2, 0.0, 0.0]))[0]
            # and slide it along the wall's normal toward TURN_MID, by feedback on its middle as read each step, at most TURN_SLIDE ahead
            inward = -(box.rotation() @ n_b)
            s = float(np.clip((TURN_MID - c[:2]) @ inward[:2], -TURN_SLIDE, TURN_SLIDE))
            at = lambda q: c + Rz @ (box.to_world(q)[0] - c) + inward * s  # noqa: E731
            Gp, Gh, nw = grips(Rb)
            mm, deg = pusher._solve(at(push_b) - nw * 0.004, Gp)
            pusher._note(mm, deg, "turn the box: push")
            mm2, deg2 = hooker._solve(at(hook_b) + nw * 0.004, Gh)
            hooker._note(mm2, deg2, "turn the box: pull")
            yield
            if max(mm, mm2) > 40.0:
                print(f"[mailer] turn: out of reach ({mm:.0f}, {mm2:.0f} mm) at {now:.1f} deg", flush=True)
                break
        box = ctx.state("mailer")
        print(f"[mailer] turn pass {p + 1} ({name}): box at {box_yaw(box):.1f} deg, middle {np.round(box.to_world(np.array([LX / 2, 0.0, 0.0]))[0][:2] * 100, 1)} cm "
              f"(was {np.round(mid[:2] * 100, 1)})", flush=True)
        nw = box.rotation() @ n_b
        yield from ctx.together(hooker.move(hooker.at() + [0.0, 0.0, 0.13], t=0.8, label="up out of the box"),
                                pusher.move(pusher.at() + nw * 0.05 + [0.0, 0.0, 0.05], t=0.6, label="off the wall"))
