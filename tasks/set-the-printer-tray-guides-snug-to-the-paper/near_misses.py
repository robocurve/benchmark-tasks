"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so each case also
writes into the task's per-trial record what the step callback would have measured: the closest gripper face to the
paper and guides, whether everything was still for 1 s with both latches let go, and whether the arms were readable.
A case is a function of that record, so the checker's single scoring call per case sees the right one.
"""

import math
from dataclasses import replace

import numpy as np

import lots
import task
from task import A4, BUNDLES, CASSETTE, FLOOR, LEGAL, T, TOP, WIDE, XC, YF

lots.use("A")
W_SNUG = (WIDE - A4[0]) / 2  # each width guide's travel to touch an A4 stack centred in the tray
L_SNUG = LEGAL - A4[1]


def _q(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


class Case(dict):
    """A state dict that loads its record into the task when the checker reads it."""

    def __init__(self, states, rec):
        super().__init__(states)
        self.rec = rec

    def items(self):
        _load(self.rec)
        return super().items()

    def __iter__(self):
        _load(self.rec)
        return super().__iter__()


def _load(rec):
    task.fresh()
    task._rec.update(t=1.0)
    task._rec.update(rec)


def state(width=0.0, length=0.0, press=(0.0005, 0.0005, 0.0005), latches=(0.0, 0.0), bundles=None, gap=0.05,
          held=True, blind=False):
    """Guides set so each pad face is `width`/`length` m off an A4 stack squared in the feed corner, pads pressed
    `press` (near, far, length); bundle overrides as {index: (dx, dy, dz, yaw_deg, tilt_deg)}."""
    pn, pf, pl = press
    joints = {"width_near": W_SNUG - width + pn, "width_far": W_SNUG - width + pf, "length": L_SNUG - length + pl,
              "pad_near": pn, "pad_far": pf, "pad_length": pl, "latch_width": latches[0], "latch_length": latches[1]}
    out = {"cassette": CASSETTE.state(joints=joints)}
    for i, b in enumerate(BUNDLES):
        dx, dy, dz, yaw, tilt = (bundles or {}).get(i, (0, 0, 0, 0, 0))
        q = np.array(_q(math.radians(yaw)))
        if tilt:
            s = math.sin(math.radians(tilt) / 2)
            qt = np.array((s, 0.0, 0.0, math.cos(math.radians(tilt) / 2)))
            x1, y1, z1, w1 = q
            x2, y2, z2, w2 = qt
            q = np.array((w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                          w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2))
        pos = (XC + dx, YF + A4[1] / 2 + dy, FLOOR + T / 2 + i * T + dz)
        out[b.name] = b.state(pos=pos, quat_xyzw=tuple(q))
    rec = dict(gap=None if blind else gap, held=held and not blind, blind=blind, since=0.0 if held else None)
    return Case(out, rec)


START = Case({"cassette": CASSETTE.state(), **{b.name: b.state() for b in BUNDLES}},
             dict(gap=0.05, held=True, blind=False, since=0.0))

CASES = [
    ("every guide touching, stack square in the feed corner, let go, still 1 s", state(), 1.0, True),
    ("each guide 0.9 mm off the stack", state(width=0.0009, length=0.0009), 1.0, True),
    ("pads pressed 1.9 mm (2.85 N)", state(press=(0.0019, 0.0019, 0.0019)), 1.0, True),
    ("length guide 1.2 mm off: width guides only", state(length=0.0012), 0.5, False),
    ("width guides 1.2 mm off a side: length guide only", state(width=0.0012), 0.5, False),
    ("width guides left at the Letter setting (3 mm a side)", state(width=0.004), 0.5, False),
    ("length guide pressed in 2.1 mm: stack squeezed", state(press=(0.0005, 0.0005, 0.0021)), 0.0, False),
    ("width guides pressed in 2.1 mm: stack squeezed", state(press=(0.0021, 0.0021, 0.0005)), 0.0, False),
    ("guides shoved home: every pad bottomed", state(press=(0.005, 0.005, 0.005)), 0.0, False),
    ("a gripper face 4 mm from a latch", state(gap=0.004), 0.0, False),
    ("a gripper face 5.5 mm away", state(gap=0.0055), 1.0, True),
    ("not yet still for 1 s", state(held=False), 0.0, False),
    ("arms could not be read: no credit", state(blind=True), 0.0, False),
    ("width latch still squeezed", state(latches=(0.12, 0.0), gap=0.05), 0.0, False),
    ("top bundle 1.5 mm short of the feed edge, length guide on it", state(
        bundles={3: (0, 0.0015, 0, 0, 0)}, length=0.0015), 0.5, False),
    ("every skewed layer touches width-guide spans, but not square at feed edge", state(
        bundles={i: (0, 0.004, 0, 1.5, 0) for i in range(4)}, length=0.007, width=0.002), 0.5, False),
    ("remote skewed corners do not certify 4 mm width gaps as snug", state(
        bundles={2: (0, 0.004, 0, 1.5, 0)}, length=0.007, width=0.004), 0.0, False),
    ("opposite layers fanned 8 mm touch untouched width guides: rear tier only", state(
        width=.004, press=(0,0,0), bundles={0:(-.004,0,0,0,0),3:(.004,0,0,0,0)}), .5, False),
    ("one layer more than 1 mm from either width guide: rear tier only", state(
        width=.001, press=(0,0,0), bundles={0:(-.001,0,0,0,0),3:(.001,0,0,0,0)}), .5, False),
    ("all layer side gaps exactly 1 mm remain snug", state(
        width=.0005, press=(0,0,0), bundles={0:(-.0005,0,0,0,0),3:(.0005,0,0,0,0)}), 1., True),
    ("top bundle tilted 3 degrees (lifted edge)", state(bundles={3: (0, 0, 0.004, 0, 3.0)}), 0.0, False),
    ("top bundle riding 4 mm up on a guide", state(bundles={3: (0, 0, 0.004, 0, 0)}), 0.0, False),
    ("start layout, untouched, hands clear and still", START, 0.0, False),
    ("top bundle on tabletop outside cassette", state(
        bundles={3: (0.27, 0, -(FLOOR - TOP) - 3 * T, 0, 0)}), 0.0, False),
    ("all bundles below tray floor", state(bundles={i: (0, 0, -0.1, 0, 0) for i in range(4)}), 0.0, False),
    ("four bundles coincident at bottom layer", state(bundles={i: (0, 0, -i * T, 0, 0) for i in range(4)}), 0.0, False),
    ("unsupported top bundle 1.2 mm above its layer", state(bundles={3: (0, 0, 0.0012, 0, 0)}), 0.0, False),
    ("paper through feed panel", state(bundles={3: (0, -0.005, 0, 0, 0)}), 0.0, False),
    ("guide pads 10 mm through paper", state(width=-0.010), 0.0, False),
    ("bundle identity does not fix layer order", state(
        bundles={0: (0, 0, 3 * T, 0, 0), 3: (0, 0, -3 * T, 0, 0)}), 1.0, True),
    ("middle layers overlap by 2 mm", state(
        bundles={1: (0, 0, 0.001, 0, 0), 2: (0, 0, -0.001, 0, 0)}), 0.0, False),
    ("middle layers separated by 2 mm", state(
        bundles={1: (0, 0, -0.001, 0, 0), 2: (0, 0, 0.001, 0, 0)}), 0.0, False),
    ("top two layers separated by 2 mm", state(
        bundles={2: (0, 0, -0.001, 0, 0), 3: (0, 0, 0.001, 0, 0)}), 0.0, False),
    ("top support exactly 1 mm above lower surface", state(
        bundles={3: (0, 0, 0.001, 0, 0)}), 1.0, True),
    ("top support gap exceeds tolerance", state(
        bundles={3: (0, 0, 0.001002, 0, 0)}), 0.0, False),
    ("top numerical surface overlap exactly 0.25 mm", state(
        bundles={3: (0, 0, -0.00025, 0, 0)}), 1.0, True),
    ("top surface overlap exceeds numerical tolerance", state(
        bundles={3: (0, 0, -0.000252, 0, 0)}), 0.0, False),
]

unseated = state()
unseated.rec["free"] = {"width_near": True, "length": False}
CASES.append(("released lever but square pawl still on a tooth crest", unseated, 0.0, False))


def _invalid(label, change=None, record=None):
    st = state()
    raw = dict(st)
    if change is not None:
        change(raw)
    return (label, Case(raw, dict(st.rec) | (record or {})), 0.0, False)


for joint in ("latch_width", "latch_length", "pad_near", "pad_far", "pad_length"):
    def bad_joint(raw, key=joint):
        p = raw["cassette"]
        raw["cassette"] = replace(p, joints=dict(p.joints) | {key: float("nan")})
    CASES.append(_invalid(f"unknown {joint} cannot earn credit", bad_joint))

CASES += [
    _invalid("NaN cached hand clearance", record={"gap": float("nan")}),
    _invalid("NaN hold timestamp", record={"t": float("nan")}),
    _invalid("NaN hold start", record={"since": float("nan")}),
    _invalid("zero paper quaternion", lambda raw: raw.update(paper_1=replace(raw["paper_1"], quat_xyzw=np.zeros(4)))),
    _invalid("unknown paper position", lambda raw: raw.update(paper_1=replace(raw["paper_1"], pos=np.full(3, np.nan)))),
    _invalid("unknown paper velocity", lambda raw: raw.update(paper_1=replace(raw["paper_1"], ang_vel=np.full(3, np.nan)))),
    _invalid("zero cassette quaternion", lambda raw: raw.update(cassette=replace(raw["cassette"], quat_xyzw=np.zeros(4)))),
    _invalid("unknown guide pose", lambda raw: raw.update(cassette=replace(
        raw["cassette"], links=dict(raw["cassette"].links) | {"pad_far": (np.full(3, np.nan), np.array([0, 0, 0, 1]))}))),
    _invalid("missing paper", lambda raw: raw.pop("paper_4")),
    _invalid("missing guide link", lambda raw: raw.update(cassette=replace(
        raw["cassette"], links={k: v for k, v in raw["cassette"].links.items() if k != "pad_far"}))),
]


def _run():
    for label, st, want, _ in CASES:
        _load(st.rec)
        got = task.score(st)
        assert abs(got - want) < 1e-9, (label, got, want)
    # lot B's start scores 0 too
    lots.use("B")
    lot = lots.current()
    st = {"cassette": CASSETTE.state()}
    for b in BUNDLES:
        st[b.name] = b.state(pos=lot.pos(b.name), quat_xyzw=_q(lot.yaw_of(b.name)))
    _load(dict(gap=0.05, held=True, blind=False, since=0.0))
    assert task.score(st) == 0.0
    lots.use("A")
    # Invalid latch data must also fail in the actual step callback, before it
    # can turn a NaN comparison into a released latch and certify a stale hold.
    valid = dict(state())
    bad = dict(valid)
    bad["cassette"] = replace(valid["cassette"], joints=dict(valid["cassette"].joints) | {"latch_length": float("nan")})
    class Sim:
        t = 0.0
        def states(self):
            return bad
        def set_target(self, *args):
            raise AssertionError("invalid measurements must not drive a joint")
    sim = Sim()
    task.fresh()
    for sim.t in (0.0, 0.1, 1.2):
        task._step(sim)
        assert task._rec["blind"] and not task._rec["held"] and task.score(bad) == 0.0
    # Square teeth do not self-centre. Exercise the real callback with a pawl
    # previously released, first on a crest and then slid into its phase window.
    assert math.isclose(task._tooth_target(.0045), .0045, abs_tol=1e-12)
    assert math.isclose(task._tooth_target(.00455), .0045, abs_tol=1e-12)
    assert task._tooth_target(.004552) is None
    assert task._tooth_target(.00475) is None
    class PhaseSim:
        t = 0.0
        current = dict(state(width=-.00025))
        def states(self):
            return self.current
        def set_target(self, *args):
            pass
    old_hands, old_lock = task._hand_points, task._write_lock
    calls = []
    try:
        task._hand_points = lambda sim: np.array([[2.,2.,2.]])
        task._write_lock = lambda sim,g,locked,at=None: calls.append((g,locked,at))
        phase = PhaseSim()
        task.fresh()
        task._rec["free"]["width_near"] = True
        for phase.t in (0.,.1,1.2):
            task._step(phase)
            assert task._rec["free"]["width_near"] and task.score(phase.current) == 0.
        phase.current = dict(state())
        for phase.t in (1.3,1.4,2.5):
            task._step(phase)
        assert not task._rec["free"]["width_near"] and task.score(phase.current) == 1.
        assert len(calls)==1 and calls[0][0:2]==("width_near",True)
        # A free pawl that rides from one crest past a tooth between two 20 Hz samples catches that tooth (and is
        # held there); one that only moves along a crest without passing a tooth stays free and earns nothing.
        assert task._crossed_tooth(-.00025, -.00065) == -.0005 and task._crossed_tooth(-.00025, -.0003) is None
        assert task._crossed_tooth(.0032, .0039) == .0035 and task._crossed_tooth(None, .0039) is None
        calls.clear()
        phase.current = dict(state(width=-.00025))
        task.fresh()
        task._rec["free"]["width_near"] = True
        for phase.t in (0.,.05):
            task._step(phase)
        phase.current = dict(state(width=-.0003))
        task._step(phase)
        assert task._rec["free"]["width_near"] and not calls
        before = phase.current["cassette"].joints["width_near"]
        phase.current = dict(state(width=-.00068))
        after = phase.current["cassette"].joints["width_near"]
        tooth = task._crossed_tooth(before, after)
        assert tooth is not None and task._tooth_target(after) is None and min(before, after) < tooth < max(before, after)
        task._step(phase)
        assert not task._rec["free"]["width_near"] and len(calls) == 1 and calls[0][:2] == ("width_near", True)
        assert math.isclose(calls[0][2], tooth, abs_tol=1e-12)
    finally:
        task._hand_points, task._write_lock = old_hands, old_lock
    # Release check: a 120 mm wide gripper face standing 0.5 mm off the width latch's post, its corners 45 mm past
    # the post's ends, must count as touching; 6 mm off, as clear. Measuring from hull corners alone would miss it.
    st = START
    post = task.WN0 - 0.003  # the post's inner face (x), at the start setting
    for dx, want_clear in ((0.0005, False), (0.006, True)):
        hull = _box_hull((post + dx + 0.005, task.TAB_YC, task.DECK + 0.0525), (0.005, 0.06, 0.0075))
        g_faces = task.hand_gap(task._face_points(hull), st)
        g_corners = task.hand_gap(hull.vertices, st)
        assert (g_faces > task.CLEAR) == want_clear, (dx, g_faces)
        assert g_corners > task.CLEAR, g_corners
    print("near_misses: all", len(CASES), "cases, lot B's start and the face-sampled release check as declared")


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


if __name__ == "__main__":
    _run()
