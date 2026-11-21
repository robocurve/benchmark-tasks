"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so each case first
replays the step-side function observe() for 1.2 s of control steps with the given end state and the hands 50 mm away
(unless the case says otherwise), and then scores it. _run() afterwards drives observe() directly to check the 1 s
hold, the drop rule over the whole trial, unknown states, face sampling and the other lots, and raises if anything
scores wrongly. It also checks that the real-trial reading of a latch (the distance from the tab's outer face to the
jamb's face, 22.5 mm or less) agrees with the scorer's bolt depth, and that the fabrication drawings' dimensions
(fabrication/dimensions.json) are the model's.
"""

import json
import math
import pathlib

import numpy as np

import lots
import parts
import task
from task import SCREEN, SEAT, TRAVEL

lots.use("A")
OUT = 0.0  # latch fully out (bolt 9.5 mm proud)
SHUT = TRAVEL  # tab squeezed to the lug (bolt 2 mm inside the edge)


def screen(dx=0.0, dy=0.0, dz=0.0, ql=OUT, qr=OUT, rot=None, vz=0.0):
    """The screen at its seat moved by dx (toward the window), dy, dz, with the latches at ql, qr."""
    r = np.eye(3) if rot is None else rot
    return SCREEN.state(pos=tuple(SEAT + np.array([dx, dy, dz])), quat_xyzw=tuple(parts._matrix_to_quat(r)),
                        joints={"latch_l": ql, "latch_r": qr}, lin_vel=(0.0, 0.0, vz))


def replay(objects, gap=0.05, seconds=1.2):
    """Fill the trial record as the simulator would after ``seconds`` at 20 Hz with these states."""
    task.reset_record()
    for k in range(int(round(seconds * 20)) + 1):
        task.observe(objects, gap, k / 20.0)
    return task.score(objects)


def task_start():
    return {o.name: o.state() for o in task.OBJECTS}


def case(label, st, want, gap=0.05):
    objs = {"screen": st}
    replay({**task_start(), **objs}, gap=gap)
    return (label, objs, want, want >= 1.0)


class _Lazy(list):
    """The checker scores each case right after building it, so each case is replayed just before it is checked."""

    def __init__(self, makers):
        super().__init__(range(len(makers)))
        self._makers = makers

    def __iter__(self):
        for m in self._makers:
            yield m()

    def __getitem__(self, i):
        return self._makers[i]()


def tipped(deg, axis=(0.0, 1.0, 0.0)):
    return parts._rot(axis, math.radians(deg))


def tab_to_jamb(st, joint) -> float:
    """The real-trial reading of one latch: distance from the tab's outer face to its jamb's inner face (m). Both
    faces are seen from the room; nothing inside the jamb is read."""
    side = 1 if joint == "latch_l" else -1
    q = float(st.joints[joint])
    face = task._local(st, (task.TAB_X, side * (task.Y_TAB + task.TAB[1] / 2 - q), task.B_Z))
    return task.Y_JAMB - side * float(face[1])


MAKERS = [
    lambda: case("seated on all four sides, both latches out in their holes, hands clear, 1 s still", screen(), 1.0),
    lambda: case("seated, both bolts 3.0 mm into their holes (latches 3.5 mm in: exact limit)",
                 screen(ql=0.0035, qr=0.0035), 1.0),
    lambda: case("seated, left bolt only 2.5 mm into its hole", screen(ql=0.0040), 0.5),
    lambda: case("seated, left latch held squeezed shut", screen(ql=SHUT), 0.5),
    lambda: case("seated, both latches squeezed shut (nothing holds it)", screen(ql=SHUT, qr=SHUT), 0.0),
    lambda: case("2.0 mm off the stop (exact limit)", screen(dx=-0.0020), 1.0),
    lambda: case("2.5 mm off the stop, standing in the opening", screen(dx=-0.0025), 0.0),
    lambda: case("bottom 1.5 mm above the sill (exact limit)", screen(dz=0.0015), 1.0),
    lambda: case("bottom 2.0 mm above the sill (hung on the bolts)", screen(dz=0.0020), 0.0),
    lambda: case("leaning 2 degrees in from the stop at the top (top 12 mm off the stop)",
                 screen(rot=tipped(-2.0)), 0.0),
    lambda: case("shifted 2.9 mm toward the left jamb, both latched", screen(dy=0.0029), 1.0),
    lambda: case("shifted 2 mm toward the right jamb, left tab 1.5 mm in: its outer face 22.5 mm from the left jamb "
                 "(exact limit of the ruler reading), bolt 3.0 mm in", screen(dy=-0.0020, ql=0.0015), 1.0),
    lambda: case("shifted 2 mm toward the right jamb, left tab 2.0 mm in: its outer face 23.0 mm from the left jamb, "
                 "bolt 2.5 mm in", screen(dy=-0.0020, ql=0.0020), 0.5),
    lambda: case("racked 1 degree in the window's plane", screen(rot=tipped(1.0, (1.0, 0.0, 0.0))), 0.0),
    lambda: case("still leaning on the stool where it started", SCREEN.state(), 0.0),
    lambda: case("seated and latched, a fingertip 4 mm from a tab (still held)", screen(), 0.0, gap=0.004),
    lambda: case("seated and latched, nearest hand point 5.5 mm away", screen(), 1.0, gap=0.0055),
    lambda: case("upside down in the opening (latches at the top)", screen(dz=task.H, rot=tipped(180.0, (1, 0, 0))),
                 0.0),
    lambda: case("turned round, mesh face to the window (housings behind)",
                 screen(dx=-0.0, rot=tipped(180.0, (0, 0, 1))), 0.0),
]
CASES = _Lazy(MAKERS)


def _run():
    start = task_start()
    good = {**start, "screen": screen()}
    assert replay(good, seconds=0.9) == 0.0, "0.9 s of stillness must not score"
    assert replay(good, seconds=1.0) == 1.0, "1.0 s of stillness must score"
    # a drop at any step ends credit for the trial, even if the screen is put back
    task.reset_record()
    for k in range(40):
        st = screen(dz=0.10 - (0.04 if k == 4 else 0.0)) if k in (3, 4) else screen()
        task.observe({**good, "screen": st}, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a 4 cm fall in one step with the hands away must end credit"
    task.reset_record()
    held = {3: 0.10, 4: 0.06, 5: 0.03, 6: 0.0}
    for k in range(40):
        st = screen(dz=held[k]) if k in held else screen()
        task.observe({**good, "screen": st}, 0.01 if k in held else 0.05, k / 20.0)
    assert task.score(good) == 1.0, "lowering it fast while holding it is not a drop"
    # movement during the hold restarts it
    task.reset_record()
    for k in range(25):
        task.observe({**good, "screen": screen(dx=-0.0015 if k == 15 else 0.0)}, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a 1.5 mm movement 0.45 s before the end must restart the hold"
    # unknown hands or screen at any step: no credit
    task.reset_record()
    for k in range(30):
        task.observe(good, None if k == 3 else 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown hand state must give no credit"
    task.reset_record()
    bad = {**good, "screen": SCREEN.state(pos=(float("nan"), 0.0, 0.0))}
    for k in range(30):
        task.observe(bad if k == 2 else good, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown screen state must give no credit"
    # the hand gap: a point 0.5 mm off a tab's outer face is caught; a flat hull face is sampled
    st = screen()
    tab = SEAT + np.array([task.TAB_X, task.Y_TAB + task.TAB[1] / 2 + 0.0005, task.B_Z])
    g = task.hand_gap(good, np.array([tab]))
    assert 0.0004 < g < 0.0006, g
    from yam_arena.analysis import Hull
    cube = np.array([[x, y, z] for x in (0, 0.03) for y in (0, 0.03) for z in (0, 0.03)], float)
    planes = np.array([[1, 0, 0, 0.03], [-1, 0, 0, 0], [0, 1, 0, 0.03], [0, -1, 0, 0], [0, 0, 1, 0.03], [0, 0, -1, 0]],
                      float)
    face = task._hull_face_points(Hull(vertices=cube, planes=planes))
    assert len(face) > 80, "hull faces must be sampled"
    assert np.min(np.linalg.norm(face - np.array([0.015, 0.015, 0.03]), axis=1)) < 0.003, "face middles sampled"
    # other lots: same scoring of the seated screen
    for name in ("B", "C"):
        lots.use(name)
        assert task.tier(good) == 1.0
    lots.use("A")
    task.reset_record()
    del st
    # the real-trial reading: tab's outer face to the jamb's face. The bolt's tip is 25.5 mm beyond that face, so
    # 22.5 mm or less is the same as the scorer's 3 mm of bolt past the jamb's face, wherever the screen sits in its
    # side play and however far the tab has come back.
    assert abs(tab_to_jamb(screen(), "latch_l") - 0.0190) < 1e-9, "fully out and centred must read 19.0 mm"
    assert abs(tab_to_jamb(screen(ql=SHUT), "latch_l") - 0.0305) < 1e-9, "squeezed shut and centred must read 30.5 mm"
    n = 0
    for dy in np.arange(-0.0029, 0.00291, 0.0002):
        for q in np.arange(0.0, TRAVEL + 1e-9, 0.00025):
            for joint, kw in (("latch_l", {"ql": q}), ("latch_r", {"qr": q})):
                st = screen(dy=float(dy), **kw)
                reading = tab_to_jamb(st, joint)
                depth = task.bolt_depth(st, joint)
                if depth > 0:  # the tip is always 25.5 mm beyond the tab's outer face
                    assert abs(reading + depth - 0.0255) < 1e-9, (joint, dy, q, reading, depth)
                if abs(reading - 0.0225) > 1e-6:  # off the exact limit
                    assert task.latched(st, joint) == (reading < 0.0225), (joint, dy, q, reading)
                    n += 1
    assert n > 2000, n
    del st
    # the fabrication drawings carry the model's dimensions
    d = json.loads((pathlib.Path(__file__).resolve().parent / "fabrication" / "dimensions.json").read_text())
    mm = 1000.0
    same = [
        (d["screen"]["width"], task.W * mm), (d["screen"]["height"], task.H * mm),
        (d["screen"]["frame_thickness"], task.T * mm), (d["screen"]["frame_face"], task.FACE * mm),
        (d["housing_plate"]["thickness"], task.HOUSE_T * mm),
        (d["tab_and_lug"]["proud_of_plate"], task.TAB[0] * mm), (d["tab_and_lug"]["thickness"], task.TAB[1] * mm),
        (d["tab_and_lug"]["height"], task.TAB[2] * mm),
        (d["tab_and_lug"]["bottom_above_screen_bottom"], (task.B_Z - task.TAB[2] / 2) * mm),
        (d["tab_and_lug"]["tab_outer_face_from_side_at_rest"], (task.W / 2 - task.Y_TAB - task.TAB[1] / 2) * mm),
        (d["tab_and_lug"]["lug_face_from_side"], (task.W / 2 - task.Y_LUG - task.TAB[1] / 2) * mm),
        (d["tab_and_lug"]["travel"], task.TRAVEL * mm),
        (d["bolt"]["diameter"], task.BOLT_D * mm), (d["bolt"]["axis_above_screen_bottom"], task.B_Z * mm),
        (d["bolt"]["axis_behind_room_face"], task.T / 2 * mm), (d["bolt"]["proud_of_side_at_rest"], task.BOLT_OUT * mm),
        (d["bolt"]["inside_side_when_squeezed"], (task.TRAVEL - task.BOLT_OUT) * mm),
        (d["window"]["play_each_side"], task.PLAY * mm), (d["window"]["hole_front_to_back"], task.HOLE_W * mm),
        (d["window"]["hole_tall"], task.HOLE_H * mm), (d["window"]["hole_deep"], task.HOLE_DEPTH * mm),
        (d["window"]["jamb_face_to_seated_screen_face"], task.DEPTH * mm),
        (d["reading"]["bolt_past_jamb_face_min"], task.LATCH_IN * mm),
        (d["reading"]["tab_outer_face_to_bolt_tip"],
         (task.W / 2 - task.Y_TAB - task.TAB[1] / 2 + task.BOLT_OUT) * mm),
        (d["reading"]["latched_if_tab_outer_face_to_jamb_at_most"],
         (task.W / 2 - task.Y_TAB - task.TAB[1] / 2 + task.BOLT_OUT - task.LATCH_IN) * mm),
        (d["reading"]["seated_if_screen_face_behind_jamb_face_at_least"], (task.DEPTH - task.FLUSH_TOL) * mm),
        (d["reading"]["on_sill_if_feeler_does_not_enter"], task.SILL_TOL * mm),
        (d["spring"]["length_at_rest"] - d["spring"]["length_squeezed"], task.TRAVEL * mm),
    ]
    for drawn, model in same:
        assert abs(drawn - model) <= 0.06, (drawn, model)  # 3/4 in face 19.05 vs 19.1; 7/32 in bolt 5.56 vs 5.6
    assert abs(d["spring"]["force_at_rest_N"] - task.LATCH_SPRING * task.LATCH_PRELOAD) < 0.1
    assert abs(d["spring"]["force_squeezed_N"] - task.LATCH_SPRING * (task.LATCH_PRELOAD + task.TRAVEL)) < 0.2


_run()
