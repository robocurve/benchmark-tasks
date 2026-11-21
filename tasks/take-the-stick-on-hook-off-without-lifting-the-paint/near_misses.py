"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before each
case this file writes into the task's per-trial record what the step callback would have measured (which bond cells
let go, whether paint came away, the grippers' distance, the 1 s hold). _run() then drives the step-side functions
directly: the bond's release and tearing rules at and either side of each limit, the release check against gripper
faces (not only corners), the hold and the missing-measurement rule, in both lots, and raises if any is wrong.
"""

import math

import numpy as np

import lots
import task
from task import E_REL, N_CELLS, PANEL_PART, RIP_E, TOP

DEG = math.radians
CELLS = [f"s{i + 1}" for i in range(N_CELLS)]


def record(released=True, torn=(), held=True, gap=0.05, blind=False):
    task.fresh()
    task._rec.update(released=[bool(released)] * N_CELLS, torn=[i in torn for i in range(N_CELLS)],
                     freed=bool(released), blind=blind, gap=gap, t=30.0, since=29.0 if held else None,
                     held=held, hook_v=0.0,released_held=bool(held and gap is not None and np.isfinite(gap) and gap>task.CLEAR and not blind),released_since=29.0 if held else None)


def in_tray(dx=0.0, dy=0.0, dz=0.0, lot="A"):
    """Joint values that put the hook's plate in the middle of the tray, resting on its floor."""
    lots.use(lot)
    p0 = PANEL_PART.state(pos=lots.current().pos("panel"), quat_xyzw=_yaw_q(lots.current().yaw_of("panel")))
    hook = [b for b in task._hook_boxes(p0, with_strip=False) if b[0] == "bz"]
    corners = np.concatenate([c + (np.abs(r) @ (s / 2)) * np.array([[-1.0], [1.0]]) for _, c, r, s in hook])
    plate = [b for b in hook if abs(b[3][2] - task.PLATE[2]) < 1e-9][0]
    want = np.array([task.TRAY_XY[0] + dx, task.TRAY_XY[1] + dy, TOP + task.TRAY_FLOOR + dz])
    move = want - np.array([plate[1][0], plate[1][1], corners[:, 2].min()])
    a = lots.current().yaw_of("panel")
    c, s = math.cos(a), math.sin(a)
    local = (c * move[0] + s * move[1], -s * move[0] + c * move[1], move[2])
    return {"bx": local[0], "by": local[1], "bz": local[2], "tab": 0.0}


def _yaw_q(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def panel(joints=None, lot="A"):
    lots.use(lot)
    st = PANEL_PART.state(joints=joints or {}, pos=lots.current().pos("panel"),
                          quat_xyzw=_yaw_q(lots.current().yaw_of("panel")))
    return {"panel": st}


record()  # the checker scores every case below with this record: strip let go clean, grippers 50 mm off, held 1 s
CASES = [
    ("strip fully let go clean, hook in the tray, let go, still 1 s", panel(in_tray()), 1.0, True),
    ("same, plate's edge 0 mm from the tray's wall", panel(in_tray(dx=0.05 - 0.012)), 1.0, True),
    ("plate overlapping the tray's wall line by 1 mm", panel(in_tray(dx=0.05 - 0.011)), 0.0, False),
    ("hook's upper end 0 mm from the tray's end wall", panel(in_tray(dy=0.075 - 0.031)), 1.0, True),
    ("hook's upper end 1 mm over the tray's end wall line", panel(in_tray(dy=0.075 - 0.030)), 0.0, False),
    ("plate's lowest point exactly 2 mm above the tray floor", panel(in_tray(dz=0.002)), 1.0, True),
    ("hook held 3 mm above the tray floor", panel(in_tray(dz=0.003)), 0.0, False),
    ("strip let go clean, hook left lying on the panel", panel({"bz": 0.0}), 0.5, False),
    ("strip let go clean, hook dropped on the table beside the tray", panel(in_tray(dy=0.20, dz=-0.003)), 0.5, False),
    ("lot B layout: hook in the tray, let go, still", panel(in_tray(lot="B"), lot="B"), 1.0, True),
]
lots.use(None)
# Record-dependent outcomes (paint torn, the grippers near or unmeasured, no 1 s hold, the bond not let go) are
# checked in _run() below, since the checker's cases share one record.


def _box_hull(center, half):
    from yam_arena.analysis import Hull

    half = np.asarray(half, float)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    verts = corners + np.asarray(center, float)
    planes = []
    for i in range(3):
        for sign in (-1.0, 1.0):
            n = np.zeros(3)
            n[i] = sign
            planes.append([*n, float(n @ (np.asarray(center, float) + n * half))])
    return Hull(vertices=verts, planes=np.asarray(planes, float))


def _stretch(state_joints, upto, e, lift_deg):
    j = dict(state_joints)
    for i in range(upto):
        j[CELLS[i]] = E_REL
    j[CELLS[upto]] = e
    j["tab"] = DEG(lift_deg)
    return j


def _run():
    bad = []
    for lot in ("A", "B"):
        lots.use(lot)
        # A clean slow pull: every cell lets go at its own stretch, none torn, the hook freed.
        task.fresh()
        for k in range(N_CELLS):
            for e in (E_REL - 0.001, E_REL):
                went = task.judge(panel(_stretch({}, k, e, 0.0), lot)["panel"])
                if (e < E_REL) == bool(went):
                    bad.append(f"lot {lot}: cell {k} at stretch {e * 1000:.0f} mm: let go {went}")
        if not task._rec["freed"] or any(task._rec["torn"]):
            bad.append(f"lot {lot}: clean pull did not free the hook cleanly")
        # Lift limits at the moment a cell lets go.
        for deg, torn in ((30.0, False), (30.5, True), (-60.0, False)):
            task.fresh()
            task.judge(panel(_stretch({}, 0, E_REL, deg), lot)["panel"])
            if task._rec["torn"][0] != torn:
                bad.append(f"lot {lot}: cell let go at {deg} deg lift: torn {task._rec['torn'][0]}")
        # Peeling up: over 45 degrees under half the release pull peels the front with its paint.
        for deg, e, torn, went in ((45.5, RIP_E, True, True), (44.5, E_REL - 0.001, False, False),
                                   (60.0, RIP_E - 0.001, False, False)):
            task.fresh()
            g = task.judge(panel(_stretch({}, 0, e, deg), lot)["panel"])
            if bool(g) != went or task._rec["torn"][0] != torn:
                bad.append(f"lot {lot}: {deg} deg at {e * 1000:.0f} mm: went {g}, torn {task._rec['torn'][0]}")
        # A peel keeps tearing cell after cell while it is held up.
        task.fresh()
        task.judge(panel(_stretch({}, 0, RIP_E, 50.0), lot)["panel"])
        task.judge(panel(_stretch({}, 1, RIP_E, 50.0), lot)["panel"])
        if task._rec["torn"][:2] != [True, True]:
            bad.append(f"lot {lot}: a held peel did not tear the next cell")
        # Pulling the hook itself: off the face, or sideways.
        for j, torn in (({"bz": 0.002}, False), ({"bz": 0.0021}, True), ({"bx": 0.004}, False),
                        ({"bx": 0.003, "by": 0.0028}, True), ({"by": -0.0041}, True)):
            task.fresh()
            task.judge(panel(j, lot)["panel"])
            if all(task._rec["torn"]) != torn:
                bad.append(f"lot {lot}: hook moved {j}: torn {task._rec['torn']}")
        # Pulling the hook off half way through: the cells still bonded tear, those let go do not.
        task.fresh()
        for k in range(2):
            task.judge(panel(_stretch({}, k, E_REL, 0.0), lot)["panel"])
        task.judge(panel({**_stretch({}, 2, 0.01, 0.0), "bz": 0.005}, lot)["panel"])
        if task._rec["torn"] != [False, False, True, True, True]:
            bad.append(f"lot {lot}: pulled off after two cells: torn {task._rec['torn']}")
        # Once freed, lifting the hook away tears nothing.
        task.fresh()
        for k in range(N_CELLS):
            task.judge(panel(_stretch({}, k, E_REL, 0.0), lot)["panel"])
        task.judge(panel({"bz": 0.05, "bx": 0.1}, lot)["panel"])
        if any(task._rec["torn"]):
            bad.append(f"lot {lot}: lifting the freed hook tore paint")
        # Release check: a gripper face 0.5 mm over the hook's stem top, its corners far off to the sides, counts
        # as touching; 6 mm up it is clear.
        st = panel(in_tray(lot=lot), lot)["panel"]
        hook = [b for b in task._hook_boxes(st, with_strip=False) if b[0] == "bz"]
        top = max(c[2] + s[2] / 2 for _, c, _, s in hook)
        cx = np.mean([c for _, c, _, _ in hook], axis=0)
        for dz, want_clear in ((0.0005, False), (0.006, True)):
            plate = _box_hull(np.array([cx[0], cx[1], top + dz + 0.002]), (0.060, 0.060, 0.002))
            wh = task._WorldHull(plate, task._face_points(plate), np.eye(4))
            g_faces, g_corners = task.gap(st, [wh]), task.gap(st, [plate])
            if (g_faces > task.CLEAR) != want_clear or g_corners <= task.CLEAR:
                bad.append(f"lot {lot}: face {dz * 1000:.1f} mm above the hook: gap {g_faces * 1000:.1f} mm from "
                           f"faces, {g_corners * 1000:.1f} mm from corners")
        # The hold: needs 1 s let go, freed, untorn, still, in the tray; a gripper coming back resets it.
        record(held=False)
        task._rec.update(since=None)
        for t, gp in ((0.0, 0.05), (1.0, 0.05)):
            task._rec["gap"] = gp
            task._hold(st, t)
        if not task._rec["held"]:
            bad.append(f"lot {lot}: 1 s let go in the tray did not count")
        task._rec["gap"] = 0.004
        task._hold(st, 1.05)
        if task._rec["held"] or task.score({"panel": st}) != 0.0:
            bad.append(f"lot {lot}: gripper back within 5 mm still scored 1")
        record(held=False)
        task._rec.update(since=None, hook_v=0.02)
        for t in (0.0, 1.2):
            task._hold(st, t)
        if task._rec["held"]:
            bad.append(f"lot {lot}: a moving hook counted as still")
        record(held=False)
        task._rec.update(since=None, hook_v=None)
        task._hold(st, 0.0)
        task._hold(st, 1.2)
        if task._rec["held"]:
            bad.append(f"lot {lot}: unmeasured hook speed counted as still")
        # Whole-trial record outcomes, with the hook lying in the tray.
        tray = panel(in_tray(lot=lot), lot)
        for label, rec, want in (
            ("paint torn under one cell", dict(torn=(0,)), 0.0),
            ("hook pulled off (all cells torn)", dict(torn=tuple(range(N_CELLS))), 0.0),
            ("grippers could not be measured", dict(blind=True), 0.0),
            ("a gripper face within 5 mm", dict(gap=0.004), 0.0),
            ("a gripper exactly 5 mm away", dict(gap=0.005), 0.0),
            ("not yet let go and still for 1 s", dict(held=False), 0.0),
            ("gripper distance never measured", dict(gap=None), 0.0),
            ("bond not let go", dict(released=False), 0.0),
        ):
            record(**rec)
            got = task.score(tray)
            if got != want:
                bad.append(f"lot {lot}: {label}: score {got}, want {want}")
        record(released=False)
        if task.score(panel(lot=lot)) != 0.0:
            bad.append(f"lot {lot}: do nothing scored")
    lots.use(None)
    record()
    if bad:
        raise AssertionError("; ".join(bad))


_run()

if __name__ == "__main__":
    for label, states, want, _ in CASES:
        print(f"{task.score(states):.2f} (want {want:.2f})  {label}")
    print("rule checks passed")

# Revision regression: the tab is the strip's own unbonded end (same width, same 1/20 in thickness, no grip
# adapter), it lies flat on an uninterrupted face throughout release, and the whole of it shows below the plate.
assert abs(task.N_CELLS*task.CELL+task.TAB_LEN-.070)<1e-9, "strip must total 70 mm"
assert task.TAB_T == task.STRIP_T == 0.00127, "tab is the strip's own thickness, 1/20 in"
assert task.TAB.shapes[0].size[0] == task.STRIP_W == 0.019, "tab is the strip's own width"
assert task.TAB.lower == 0.0 and task.TAB.spring == 0.0, "tab starts flat with nothing holding it up"
assert task.PLATE_Y0 == task.BOND_Y0 == task.TAB_PIVOT_Y, "plate ends where the bond ends; the tab is not under it"
assert task.PANEL_Y / 2 - (abs(task.TAB_PIVOT_Y) + task.TAB_LEN + task.N_CELLS * task.E_REL) > 0.100, \
    "the face continues at least 100 mm past the tab's end at full stretch"
for lot in ("A","B"):
    for stretched in (False,True):
        j={f"s{i+1}":task.E_REL if stretched else 0. for i in range(task.N_CELLS)}
        j["tab"]=0.
        p=panel(j,lot)["panel"]
        tab=[b for b in task._hook_boxes(p) if b[0]=="tab"]
        assert len(tab)==1
        _,c,r,size=tab[0];half=np.abs(r)@(size/2)
        lo,hi=c-half,c+half
        assert abs(lo[2]-task.FACE_Z)<1e-8, "reset tab must touch the face"
        assert lo[0]>=task.PANEL_XY[0]-task.PANEL/2 and hi[0]<=task.PANEL_XY[0]+task.PANEL/2
        assert lo[1]>=-task.PANEL_Y/2 and hi[1]<=task.PANEL_Y/2, "release path must remain backed"
        assert abs(task.tab_lift(p))<1e-12
lots.use(None)
record()
print("continuous-face/flat-tab/both-lot/full-stroke regressions passed")

# Unknown hand data and below-floor states cannot certify any credit.
from types import SimpleNamespace
class MockScene(dict):
    env_origins=np.zeros((1,3))
pose=np.tile(np.array([10.,10.,10.,0.,0.,0.,1.]),(1,3,1))
def asset(names):return SimpleNamespace(body_names=names,data=SimpleNamespace(body_link_pose_w=pose.copy()))
names=list(task._HAND_LINKS)
scene=MockScene(left_robot=asset(names),right_robot=asset(names))
sim=SimpleNamespace(env=SimpleNamespace(unwrapped=SimpleNamespace(scene=scene)))
original=task.parts._np
try:
    task.parts._np=lambda x:np.asarray(x)
    assert task._hands(sim) is not None
    scene["right_robot"].data.body_link_pose_w[:]=np.nan
    assert task._hands(sim) is None
    scene["right_robot"]=asset(names[:-1])
    assert task._hands(sim) is None
    del scene["right_robot"]
    assert task._hands(sim) is None
finally:task.parts._np=original
record()
below=panel(in_tray(dz=-.015))
assert task.score(below)==0.0
record(gap=None)
assert task.score(panel(in_tray()))==0.0
record(held=False)
assert task.score(panel(in_tray()))==0.0
record()
print("missing/nonfinite hands, below-floor and unmeasured/held-credit regressions passed")

# Finite cached links do not certify a panel with unknown root or joint state.
import copy
for corrupt in ("root-position", "root-quaternion", "joint", "missing-joint", "missing-link"):
    record()
    states=panel(in_tray());p=copy.deepcopy(states["panel"])
    if corrupt=="root-position": p.pos[0]=np.nan
    elif corrupt=="root-quaternion": p.quat_xyzw[0]=np.nan
    elif corrupt=="joint": p.joints["bz"]=np.nan
    elif corrupt=="missing-joint": del p.joints["bz"]
    else: del p.links["tab"]
    states["panel"]=p
    assert task.score(states)==0., corrupt
    assert not task.hook_supported(p) and not task.hook_in_tray(p), corrupt
    task.judge(p)
    assert task._rec["blind"], corrupt
record()
print("unknown root/joint/link state fails closed despite finite cached hook geometry")
