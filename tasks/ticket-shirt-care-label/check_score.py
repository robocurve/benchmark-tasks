"""The scoring table on hand-built records, no simulator: python check_score.py"""
import importlib.util
import pathlib
import sys
import types

# task.py imports the simulator's modules at import time; the scoring table (_tier) needs none of them
for name in ("demokit", "mechkit", "yam_arena", "yam_arena.task_file"):
    mod = types.ModuleType(name)
    sys.modules[name] = mod
sys.modules["yam_arena.task_file"].TABLE_TOP = {"station": 0.0}
mk = sys.modules["mechkit"]
mk.Look = lambda **kw: kw
mk.rgb = lambda *a: a
mk.ENV_PATH = "/World/envs/env_0"
mk.set_visible = lambda *a, **kw: None
mk.Mesh = object
for fn in ("box", "cylinder", "col_box", "hinge", "slider", "mechanism", "on_step", "on_reset", "Part"):
    setattr(mk, fn, lambda *a, **kw: None)
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
task = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task)

base = {"pierced": False, "torn": False, "gun_fell": False, "fastened": False, "out": False, "clear": False, "in_label": False, "edge_mm": 0.0}
cases = [
    ("the full solution: fastened through the label alone, needle out, label let go", dict(fastened=True, out=True, clear=True, edge_mm=5.0), 1.0),
    ("fastened, but the needle still in the label", dict(fastened=True, out=False, clear=True, in_label=True, edge_mm=5.0), 0.5),
    ("fastened and out, but the label still held up", dict(fastened=True, out=True, clear=False, edge_mm=45.0), 0.5),
    ("needle through the label now, never fired", dict(in_label=True, edge_mm=45.0), 0.5),
    ("through the label once, drawn back unfired, label let down (final state: nothing in it)", dict(out=True, clear=True, edge_mm=5.0), 0.0),
    ("through the label once, drawn back unfired, label still held 45 mm up", dict(out=True, edge_mm=45.0), 0.25),
    ("label lifted 2 cm now, nothing more", dict(edge_mm=21.0), 0.25),
    ("label lifted under 2 cm (its curl alone)", dict(edge_mm=8.3), 0.0),
    ("fired with the needle through the label, but the shirt touched by the needle on the way", dict(fastened=True, out=True, clear=True, edge_mm=5.0, pierced=True), 0.0),
    ("label folded back to its seam's stop (torn)", dict(in_label=True, edge_mm=60.0, torn=True), 0.0),
    ("gun knocked off the table", dict(edge_mm=52.0, gun_fell=True), 0.0),
]
bad = 0
# the firing gate on hand-built needle poses: the label hinged at the origin along x, lifted 40 degrees (seam height 0)
import math
import numpy as np
a = math.radians(40.0)
Rl = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])
hinge = np.zeros(3)
nrm = Rl[:, 2]  # the label's face normal
def at(u):
    return Rl @ np.array([0.0, u, 0.0])
gate = [
    ("needle through the label's face 38 mm out (2.4 cm clear)", at(0.038) - 0.004 * nrm, at(0.038) + 0.021 * nrm, True),
    ("the same needle entering from the label's underside", at(0.038) + 0.004 * nrm, at(0.038) - 0.021 * nrm, True),
    ("needle through the label 20 mm out (1.3 cm over the shirt: through; the shirt is guarded by contact, not height)", at(0.020) - 0.004 * nrm, at(0.020) + 0.021 * nrm, True),
    ("needle slipped under the label from its free edge, tip under it", at(0.040) - 0.004 * nrm, at(0.065) - 0.004 * nrm, False),
    ("needle passing beside the label (outside its 30 mm width)", at(0.038) + np.array([0.02, 0.0, 0.0]) - 0.004 * nrm, at(0.038) + np.array([0.02, 0.0, 0.0]) + 0.021 * nrm, False),
    ("needle under the label, not through it (both ends below its face)", at(0.038) - 0.004 * nrm, at(0.038) - 0.010 * nrm - Rl @ np.array([0.0, 0.02, 0.0]), False),
]
for name, tip, base_pt, want in gate:
    got, clear = task.needle_through(tip, base_pt, hinge, Rl, 0.0)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} gate {got!s:5} (want {want!s:5}, label {clear * 1000:.0f} mm clear)  {name}")
# the needle in the hole it pierced, step by step: the label flat at the origin, the hole 40 mm out on the free segment, the needle coming down from above
flat = [(np.zeros(3), np.eye(3)) for _ in range(task.NSEG)]
def hole_path(points, entered=True, far=True):
    """points: (tip x, tip u, tip height over the face) per step, the needle upright; returns (through at the end, torn at any step)."""
    st = {"k": task.NSEG - 1, "x": 0.0, "u": 0.040, "side": 1.0, "entered": entered, "far": far}
    thr, torn = False, False
    for x, u, z in points:
        tip = np.array([x, u, task.LABEL_T / 2 + z])
        thr, ripped, _ = task.hole_step(st, tip, tip + np.array([0.0, 0.0, task.NEEDLE_L]), flat)
        torn = torn or ripped
    return thr, torn
holes = [
    ("pierced at the hole, tip pushed 4 mm past", [(0.0, 0.040, -0.002), (0.0, 0.040, -0.004)], {}, (True, False)),
    ("pierced, then dragged 8 mm sideways while through", [(0.0, 0.040, -0.004), (0.004, 0.040, -0.004), (0.008, 0.040, -0.004)], {}, (False, True)),
    ("pierced, then drawn back out", [(0.0, 0.040, -0.004), (0.0, 0.040, 0.002), (0.0, 0.040, 0.010)], {}, (False, False)),
    ("drawn back, then pushed in again through the same hole", [(0.0, 0.040, 0.004), (0.0, 0.041, -0.003)], {}, (True, False)),
    ("drawn back, then slid round to the far side 20 mm off the hole and along to it", [(0.0, 0.040, 0.004), (0.020, 0.040, -0.003), (0.010, 0.040, -0.003), (0.001, 0.040, -0.003)], {}, (False, False)),
    ("at the pierce the tip not yet past the face, then through at the hole", [(0.0, 0.040, 0.0005), (0.0, 0.040, -0.003)], {"far": False, "entered": True}, (True, False)),
]
for name, pts, kw, want in holes:
    got = hole_path(pts, **kw)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} hole (through, torn) {got} (want {want})  {name}")
# the pierce rule on hand-built pushes: the needle's collider tip pressing on the free segment's face, 40 mm out, the label flat (segment frames at u = 0)
segs = [(np.array([0.0, 0.0, 0.0]), np.eye(3)) for _ in range(task.NSEG)]
axis = np.array([0.0, 0.0, -1.0])  # pressing straight down onto the face
def press(forces, depths, at_u=0.040, x=0.0):
    st, got = {}, None
    for f, z in zip(forces, depths):
        k = task.press_step(st, f, np.array([x, at_u, task.LABEL_T / 2 + 0.001 - z]), axis, segs)
        got = got if got is not None else k
    return got
prs = [
    ("4.5 N held while the tip tents the label 1.5 mm", [4.5] * 4, [0.0, 0.0005, 0.001, 0.0015], 2),
    ("a light touch: 2 N over 3 mm", [2.0] * 4, [0.0, 0.001, 0.002, 0.003], None),
    ("4.5 N but no advance (0.5 mm)", [4.5] * 4, [0.0, 0.0002, 0.0004, 0.0005], None),
    ("4.5 N, the push let off halfway (force under 4 N), then 0.6 mm more", [4.5, 4.5, 3.0, 4.5, 4.5], [0.0, 0.0006, 0.0008, 0.0009, 0.0015], None),
    ("4.5 N over 1.5 mm beside the label (x 20 mm)", [4.5] * 4, [0.0, 0.0005, 0.001, 0.0015], None),
]
for name, fs, zs, want in prs:
    kw = {"x": 0.02} if "beside" in name else {}
    got = press(fs, zs, **kw)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} pierce {got!s:5} (want {want!s:5})  {name}")
# the pierce rule with the gun turning in the hand during the push (the advance is read along the axis saved when the force first reached 4 N, from that tip)
def press_turn(adv_mm, turn_deg, at_u=0.040):
    st, got = {}, None
    for j in range(5):
        th = math.radians(turn_deg * j / 4)
        ax = np.array([0.0, math.sin(th), -math.cos(th)])
        tip = np.array([0.0, at_u, task.LABEL_T / 2 + 0.001]) + np.array([0.0, 0.0, -adv_mm * 1e-3 * j / 4])
        tip_w = tip + np.array([0.0, 0.0, 0.0])
        k = task.press_step(st, 4.5, tip_w + 0.0 * ax, ax, segs)
        got = got if got is not None else k
    return got
turns = [("4.5 N over 1.5 mm while the gun turns 4 degrees in the hand", 1.5, 4.0, 2), ("4.5 N, no advance, the gun turning 4 degrees", 0.0, 4.0, None)]
for name, adv, turn, want in turns:
    got = press_turn(adv, turn)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} pierce {got!s:5} (want {want!s:5})  {name}")
# a turning gun far from the origin: the old reading (the tip's world position along the axis now) jumped by hundreds of mm; this one does not
st = {}
for j in range(5):
    th = math.radians(4.0 * j / 4)
    ax = np.array([0.0, math.sin(th), -math.cos(th)])
    task.press_step(st, 4.5, np.array([0.30, 0.040, task.LABEL_T / 2 + 0.001]) + np.array([0.0, 0.0, 0.0]), ax, [(np.array([0.30, 0.0, 0.0]), np.eye(3))] * task.NSEG)
ok = abs(st["adv"]) < 1e-4
bad += not ok
print(f"{'ok ' if ok else 'BAD'} pierce advance {st['adv'] * 1000:.2f} mm (want 0.00)  a still tip 30 cm from the origin while the gun turns 4 degrees")
# contact with the shirt's simulated surface on a hand-built patch of cloth: two 1 cm triangles at z 0, nodes at their corners
cloth = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.01, 0.01, 0.0], [0.0, 0.01, 0.0]])
faces = np.array([[0, 1, 2], [0, 2, 3]])
level = np.array([0.0, 1.0, 0.0])
down = np.array([0.0, 0.0, -1.0])
cont = [
    ("level needle 17 mm over the cloth", np.array([0.005, 0.03, 0.017]), level, False),
    ("level needle 4 mm over the cloth", np.array([0.005, 0.03, 0.004]), level, False),
    ("needle pressed down onto the cloth, tip 1.5 mm over it", np.array([0.006, 0.004, 0.0015]), down, True),
    ("needle through the cloth between its nodes, tip 2 mm under it", np.array([0.006, 0.004, -0.002]), down, True),
    ("level needle, its tip 20 mm past the cloth patch and its shaft lying on it 2 mm over (the shaft on the cloth is no hole)", np.array([0.005, 0.03, 0.002]), level, False),
    ("needle pitched up 12 degrees, tip 6.5 mm over the cloth, its shaft's base 1.3 mm over it", np.array([0.005, 0.03, 0.0065]), np.array([0.0, math.cos(math.radians(12)), math.sin(math.radians(12))]), False),
]
flat = np.array([[x, y, 0.0025] for x in np.arange(0.0, 0.1, 0.01) for y in np.arange(0.0, 0.1, 0.01)])
for name, z0, want in [("shirt flat, label's edge 25 mm over it", 0.0, True), ("whole shirt lifted 40 mm with the label on it, edge 25 mm over the cloth beside it still", 0.04, True),
                       ("whole shirt lifted 40 mm, label lying on it (edge 3 mm over the cloth beside it)", 0.04, False)]:
    edge = np.array([0.05, 0.05, 0.0025 + z0 + (0.025 if want else 0.003)])
    up = task.free_edge_up(edge, flat + np.array([0.0, 0.0, z0]), 0.0035)
    got = up >= 0.02
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} free edge {up * 1000:.1f} mm over the shirt beside it: counts as lifted {got!s:5} (want {want!s:5})  {name}")
for name, tip, ax, want in cont:
    d = task.needle_to_shirt(tip, ax, cloth, faces)
    got = d < task.BODY_CONTACT
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} shirt contact {got!s:5} (want {want!s:5}, {d * 1000:.1f} mm)  {name}")
# the shirt check at 240 Hz on the path since the last check (shirt_touch): the same patch of cloth, 1 cm triangles at z 0
P = lambda *v: np.array(v, float)
def touch(prev_tip, tip, prev_base, base, faces_=faces, cloth_=cloth):
    return task.shirt_touch(prev_tip, tip, prev_base, base, cloth_, faces_)
chk = [
    ("tip falls through the cloth in one step (20 mm path, 8 mm over to 12 mm under), between two checks", touch(P(0.005, 0.004, 0.008), P(0.005, 0.004, -0.012), P(0.005, 0.004, 0.033), P(0.005, 0.004, 0.013)) < task.BODY_CONTACT, True),
    ("needle driven on through the cloth, the tip past it 12 mm and the shaft across it", touch(P(0.005, 0.004, -0.012), P(0.005, 0.004, -0.012), P(0.005, 0.004, 0.013), P(0.005, 0.004, 0.013)) < task.BODY_CONTACT, True),
    ("needle swept sideways through the cloth, tip and base each side of it (shaft crossing)", touch(P(0.005, 0.004, 0.02), P(0.005, 0.004, -0.02), P(0.005, 0.004, 0.045), P(0.005, 0.004, 0.005)) < task.BODY_CONTACT, True),
    ("shaft lying on the cloth, tip 6.5 mm over it, nothing crossed", touch(P(0.005, 0.03, 0.0065), P(0.005, 0.03, 0.0065), P(0.005, 0.005, 0.0013), P(0.005, 0.005, 0.0013)) < task.BODY_CONTACT, False),
    ("level needle moving along 17 mm over the cloth", touch(P(0.004, 0.02, 0.017), P(0.006, 0.02, 0.017), P(0.004, 0.0, 0.017), P(0.006, 0.0, 0.017)) < task.BODY_CONTACT, False),
    ("no triangles, nodes only: tip between four nodes 1 cm apart, 4 mm over the cloth", touch(P(0.005, 0.005, 0.004), P(0.005, 0.005, 0.004), P(0.005, 0.005, 0.029), P(0.005, 0.005, 0.029), None) < task.BODY_CONTACT, True),
    ("no triangles, nodes only: the tip 30 mm over the cloth", touch(P(0.005, 0.005, 0.03), P(0.005, 0.005, 0.03), P(0.005, 0.005, 0.055), P(0.005, 0.005, 0.055), None) < task.BODY_CONTACT, False),
    ("no triangles, nodes only: the tip path crossing the cloth between two checks, 20 mm each side", touch(P(0.005, 0.005, 0.02), P(0.005, 0.005, -0.02), P(0.005, 0.005, 0.045), P(0.005, 0.005, 0.005), None) < task.BODY_CONTACT, True),
]
for name, got, want in chk:
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} shirt check per step: hole {got!s:5} (want {want!s:5})  {name}")
# the let-go check on hand-built fingertips (12 per hand, left then right), the label's middle at 30 mm out
mid = at(0.030)
away = np.tile(mid + np.array([0.0, -0.12, 0.10]), (24, 1))
pinned = away.copy()
pinned[18] = mid + 0.003 * nrm  # one right fingertip on the label
lgo = [
    ("both hands 12 cm off, label let down to 20 degrees", away, math.radians(20.0), True),
    ("left hand off, right hand pinning the label down at 20 degrees", pinned, math.radians(20.0), False),
    ("both hands off, label still standing at 40 degrees", away, math.radians(40.0), False),
]
for name, tips, ang, want in lgo:
    got = task.let_go(tips, mid, ang)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} let go {got!s:5} (want {want!s:5})  {name}")
for name, over, want in cases:
    got = task._tier({**base, **over})
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.2f} (want {want:.2f})  {name}")
# the per-physics-step reads in Tag.substep, on a stub scene: the seam tear (130 degrees, however briefly) and the trigger (the first squeeze fastens only if the needle is through then)
class _J:
    def __init__(self, **q):
        self.q = q
    def joint(self, n):
        return self.q[n]
def seam_run(degs):
    t = task.Tag()
    t._shirt_check = lambda env, cloth=None: (1.0, None)
    for dg in degs:
        task.m.state = lambda env, name, dg=dg: _J(seg1=math.radians(dg))
        t.substep(None)
    return t.torn
for name, degs, want in [("seam folded to 129 degrees: not torn", [20, 129, 20], False),
                         ("seam folded to 131 degrees for one step between control steps, then back: torn", [20, 131, 20], True)]:
    got = seam_run(degs)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} seam per step: torn {got!s:5} (want {want!s:5})  {name}")
def fire_run(through_pierced, travel):
    t = task.Tag()
    t.pierced = through_pierced[1]
    t._fire(_J(trigger=-travel), through_pierced[0])
    return t.fired, t.fastened
for name, args, want in [("squeezed through the label alone: fastened", ((True, False), 0.003), (True, True)),
                         ("squeezed with the needle not through: fired, not fastened", ((False, False), 0.003), (True, False)),
                         ("squeezed through the label but the shirt pierced: not fastened", ((True, True), 0.003), (True, False)),
                         ("trigger under its travel: not fired", ((True, False), 0.001), (False, False))]:
    got = fire_run(*args)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} trigger per step {got} (want {want})  {name}")
task.size_check()
sys.exit(1 if bad else 0)
