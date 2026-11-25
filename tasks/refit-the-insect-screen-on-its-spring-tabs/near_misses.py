"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so each case first
replays the step-side function observe() for 1.2 s of control steps with the given end state and the hands 50 mm away
(unless the case says otherwise), and then scores it. _run() afterwards drives observe() directly to check the 1 s
hold, the drop rule over the whole trial, unknown states, face sampling and the other lots, and raises if anything
scores wrongly. It also checks that the real-trial ruler readings (the bottom and top rails' visible heights beside
the two lips, and the screen face's depth behind the jambs) agree with the scorer, and that the drawings' dimensions
(fabrication/dimensions.json) are the model's.
"""

import json
import math
import pathlib

import numpy as np

import lots
import parts
import task
from task import SCREEN, SEAT, SPRING_CROWN, GAP_TOP

lots.use("A")
SEAT_Q = SPRING_CROWN - GAP_TOP  # each spring pressed 1 mm by the head channel's ceiling when the screen is seated


def screen(dx=0.0, dy=0.0, dz=0.0, ql=None, qr=None, rot=None, vz=0.0):
    """The screen at its seat moved by dx (toward the window), dy, dz, with the springs pressed in ql, qr."""
    r = np.eye(3) if rot is None else rot
    pos = SEAT + np.array([dx, dy, dz])
    # Unless explicitly testing inconsistent joints, place each crown against the ceiling if it reaches it.
    qs = []
    for side, q in ((1, ql), (-1, qr)):
        crown = pos + r @ np.array([0.0, side * task.SPRING_Y, task.H + SPRING_CROWN])
        inferred = max(0.0, min(SPRING_CROWN, (crown[2] - task.Z_HC) / r[2, 2])) if r[2, 2] > 0 else 0.0
        qs.append(inferred if q is None else q)
    return SCREEN.state(pos=tuple(pos), quat_xyzw=tuple(parts._matrix_to_quat(r)),
                        joints={"spring_l": qs[0], "spring_r": qs[1]}, lin_vel=(0.0, 0.0, vz))


def leaning(bottom_back_x, lean_deg, bottom_z=0.0):
    """The screen with its bottom-back corner line at ``bottom_back_x`` on the floor (+ bottom_z), leaning back."""
    a = math.radians(lean_deg)
    r = task._ry(a)
    o = np.array([bottom_back_x, 0.0, task.Z_SILL + bottom_z]) - r @ np.array([task.T / 2, 0.0, 0.0])
    qs = {}
    for side, j in ((1, "spring_l"), (-1, "spring_r")):
        top = max((o + r @ np.array([x, side * task.SPRING_Y + y, task.H + SPRING_CROWN]))[2]
                  for x in (-task.SPRING_WIDTH / 2, task.SPRING_WIDTH / 2)
                  for y in (-task.SPRING_PAD / 2, task.SPRING_PAD / 2))
        qs[j] = max(0.0, min(SPRING_CROWN, (top - task.Z_HC) / r[2, 2]))
    return SCREEN.state(pos=tuple(o), quat_xyzw=tuple(parts._matrix_to_quat(r)), joints=qs)


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


LIP_FRONT = task.X_LIPB - task.LIP_T  # the sill lip's room-side face

MAKERS = [
    lambda: case("crowns touch ceiling but no compression/load: top capture only", screen(dz=-0.001, ql=0.0, qr=0.0), 0.5),
    lambda: case("both spring crowns 3 mm clear of head: only top capture", screen(ql=0.004, qr=0.004), 0.5),
    lambda: case("one spring crown clear of head: only top capture", screen(qr=0.004), 0.5),
    lambda: case("both spring crowns 1 mm through ceiling", screen(ql=0.0, qr=0.0), 0.0),
    lambda: case("one spring crown through ceiling", screen(ql=0.0), 0.0),
    lambda: case("spring joint below its travel", screen(ql=-0.001), 0.0),
    lambda: case("spring joint beyond its travel", screen(qr=0.008), 0.0),
    lambda: case("top rail 2 mm above the channel ceiling: impossible penetration", screen(dz=GAP_TOP + 0.002), 0.0),
    lambda: case("screen through the stop by 2 mm", screen(dx=0.002), 0.0),
    lambda: case("bottom through the channel floor by 2 mm", screen(dz=-0.002), 0.0),
    lambda: case("seated in both channels against the stop, hands clear, 1 s still", screen(), 1.0),
    lambda: case("1.5 mm off the stop (exact limit: the channel's play)", screen(dx=-0.0015), 1.0),
    lambda: case("2.0 mm off the stop: top still captured, not seated", screen(dx=-0.0020), 0.5),
    lambda: case("bottom 1.5 mm above the channel floor (exact limit)", screen(dz=0.0015), 1.0),
    lambda: case("bottom 2.0 mm up (held up, not down in the sill channel): top captured", screen(dz=0.0020), 0.5),
    lambda: case("shifted 2.9 mm toward the left jamb", screen(dy=0.0029), 1.0),
    lambda: case("racked 1 degree in the window's plane (a corner 6 mm past a jamb)",
                 screen(rot=tipped(1.0, (1.0, 0.0, 0.0))), 0.0),
    lambda: case("tipped 0.2 degrees toward the room (top 1.2 mm off the stop)", screen(rot=tipped(-0.2)), 1.0),
    lambda: case("bottom against the stop, top fallen 4 mm toward the room in the 5/8 in head channel: not seated",
                 screen(rot=tipped(-0.65)), 0.5),
    lambda: case("top pressed to the ceiling, springs flat, bottom 6 mm up (as while swinging it in): top captured only",
                 screen(dz=GAP_TOP, ql=SPRING_CROWN, qr=SPRING_CROWN), 0.5),
    lambda: case("top up in the head channel, bottom on the sill in front of the sill lip (leaning back 3 degrees)",
                 leaning(LIP_FRONT - 0.0005, 3.0), 0.5),
    lambda: case("top captured, bottom resting on top of the sill lip", leaning(task.X_LIPB - 0.001, 2.2, 0.004), 0.5),
    lambda: case("leaning back 8 degrees with the bottom on the stool: top only 2.4 mm up behind the lip",
                 leaning(LIP_FRONT - 0.032, 8.0), 0.0),
    lambda: case("leaning back 16 degrees, held up 10 mm: top behind the lip but not up in the channel",
                 leaning(LIP_FRONT - 0.085, 16.0, 0.010), 0.0),
    lambda: case("still leaning on the stool where it started", SCREEN.state(), 0.0),
    lambda: case("seated, a fingertip 4 mm from a pull tab (still held)", screen(), 0.0, gap=0.004),
    lambda: case("seated, nearest hand point 5.5 mm away", screen(), 1.0, gap=0.0055),
    lambda: case("upside down (springs at the bottom) in the opening", screen(dz=task.H, rot=tipped(180.0, (1, 0, 0))),
                 0.0),
    lambda: case("turned round, mesh and pull tabs toward the window", screen(dx=0.0, rot=tipped(180.0, (0, 0, 1))),
                 0.0),
]
CASES = _Lazy(MAKERS)


def visible_bottom_rail(st):
    """Real-trial ruler reading: the bottom rail's room-side face showing above the sill lip, least of both ends (m)."""
    return min(float(task._local(st, (-task.T / 2, s * task.W / 2, task.FACE))[2]) for s in (-1, 1)) \
        - (task.Z_SILL + task.SILL_LIP_H)


def visible_top_rail(st):
    """Real-trial ruler reading: the top rail's room-side face showing below the head lip, most of both ends (m)."""
    return task.Z_HL - min(float(task._local(st, (-task.T / 2, s * task.W / 2, task.H - task.FACE))[2])
                           for s in (-1, 1))


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
    # movement during the hold restarts it, changing contact tier also restarts the hold
    task.reset_record()
    for k in range(25):
        task.observe({**good, "screen": screen(dx=-0.0012 if k == 15 else 0.0)}, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a 1.2 mm movement 0.45 s before the end must restart the hold"
    task.reset_record()
    for k in range(25):
        task.observe({**good, "screen": screen(ql=SEAT_Q + (0.002 if k == 15 else 0.0))}, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a spring's crown moving 2 mm 0.45 s before the end must restart the hold"
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
    # A missing physical spring state must invalidate the whole trial, even after later recovery.
    task.reset_record()
    missing = screen()
    del missing.joints["spring_l"]
    for k in range(30):
        task.observe({**good, "screen": missing} if k == 2 else good, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "missing spring state must fail closed"
    assert replay({**good, "screen": screen(ql=0.004, qr=0.004)}) == 0.5, "nonbearing springs prevent full credit"
    # The diagonal displacement is 1.273 mm despite each axis staying below 1 mm.
    task.reset_record()
    for k in range(25):
        st = screen(dx=-0.0009, dy=0.0009) if k >= 15 else screen()
        task.observe({**good, "screen": st}, 0.05, k / 20.0)
    assert task.score({**good, "screen": st}) == 0.0, "Euclidean diagonal motion must restart hold"
    for fault in ("missing", "nonfinite", "detached", "rotation"):
        bad = screen()
        if fault == "missing":
            del bad.links["spring_l"]
        elif fault == "nonfinite":
            bad.links["spring_l"][0][0] = float("nan")
        elif fault == "detached":
            bad.links["spring_l"][0][0] += 0.002
        else:
            bad.links["spring_l"] = (bad.links["spring_l"][0], parts._matrix_to_quat(tipped(2.0)))
        task.reset_record()
        for k in range(30):
            task.observe({**good, "screen": bad} if k == 2 else good, 0.05, k / 20.0)
        assert task.score(good) == 0.0, f"{fault} link must invalidate the whole trial"
    assert task.springs_bearing(screen()), "consistent seated springs must bear"
    for q in (0.0, 0.004, 0.007):
        assert replay({**good, "screen": screen(ql=q, qr=q)}) < 1.0, q
    # the hand gap: a point 0.5 mm off a pull tab's side face, and one 0.5 mm over a spring's crown, are caught
    tab = SEAT + np.array([task.TAB_XC, task.TAB_Y + task.TAB_THK / 2 + 0.0005, task.TAB_Z])
    g = task.hand_gap(good, np.array([tab]))
    assert 0.0004 < g < 0.0006, g
    crown = SEAT + np.array([0.0, task.SPRING_Y, task.H + SPRING_CROWN - SEAT_Q + 0.0005])
    g = task.hand_gap(good, np.array([crown]))
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
    # the real-trial readings agree with the scorer: with the screen against the stop and upright, its bottom is
    # within 1.5 mm of the floor exactly when the bottom rail shows 1.5 mm or less above its seated 15.05 mm
    # (19.05 mm rail, 4 mm lip), and the top is captured 3 mm or more exactly when the top rail shows 16.05 mm or less
    # below the head lip.
    seat_bottom = visible_bottom_rail(screen())
    assert abs(seat_bottom - (task.FACE - task.SILL_LIP_H)) < 1e-9, seat_bottom
    assert abs(visible_top_rail(screen()) - (task.FACE - task.HEAD_LAP)) < 1e-9
    # Ruler reconstruction also holds when leaning: use the vertical projection of the rail and frame thickness.
    for a in np.linspace(-15.0, 15.0, 61):
        st = screen(dz=0.015, rot=tipped(float(a)))
        r = task._rot(st.quat_xyzw)
        inferred_rise = task.FACE * r[2, 2] + min(0.0, task.T * r[2, 0]) - visible_top_rail(st)
        assert abs(inferred_rise - task.top_rise(st)) < 1e-9, (a, inferred_rise, task.top_rise(st))
    n = 0
    for dz in np.arange(0.0, 0.0060, 0.00025):
        st = screen(dz=float(dz))
        rb = visible_bottom_rail(st) - seat_bottom
        if abs(rb - task.SILL_TOL) > 1e-6:
            assert task.seated(st) == (rb < task.SILL_TOL), (dz, rb)
            n += 1
        rt = visible_top_rail(st)
        if abs(rt - (task.FACE - task.TOP_IN)) > 1e-6:
            assert (task.top_rise(st) >= task.TOP_IN) == (rt < task.FACE - task.TOP_IN), (dz, rt)
            n += 1
    for dx in np.arange(-0.0030, 0.00001, 0.00025):
        st = screen(dx=float(dx))
        depth = float(task._local(st, (-task.T / 2, task.W / 2, 0.0))[0]) - task.X_WALL  # jamb face to screen face
        if abs(depth - (task.DEPTH - task.FLUSH_TOL)) > 1e-6:
            assert task.seated(st) == (depth > task.DEPTH - task.FLUSH_TOL), (dx, depth)
            n += 1
    assert n > 40, n
    # Round-four physical landmark and access controls. Numeric tolerances stay unchanged.
    assert task._snap(good).shape == (4, 3), "hold uses the rigid frame, not fictional pad landmarks"
    for a in np.linspace(0, 22, 45):
        for q in np.linspace(0, SPRING_CROWN, 71):
            st = screen(ql=float(q), qr=float(q), rot=tipped(float(a)))
            calculated = task.spring_axial_resistance(st)
            assert np.all(calculated >= 226.0*q - 1e-9)
    assert task.SPRING_K*SPRING_CROWN >= task.SPRING_FLAT_N
    assert task.SPRING_K >= 226.0
    assert abs(task.SPRING_TARGET*task.SPRING_K + task.SPRING_DRIVE_OFFSET_N) < 1e-12
    # A 0.30 mm known-open apex gap must be reachable from the room on BOTH sides.
    # Sample the full straight blade volume, except its intentional contact with the port roof.
    # This proves model geometry, not a physical fit, blade accuracy or nondisturbance.
    window = [(np.array(lo), np.array(hi)) for lo, hi, _ in task.window_boxes()]
    for side in (-1, 1):
        xfar = task.SEAT[0] + task.SPRING_WIDTH/2
        for x in np.linspace(task.X_WALL-.005, xfar, 80):
            for y in np.linspace(side*task.SPRING_Y-.0065, side*task.SPRING_Y+.0065, 5):
                for z in (task.Z_HC-.0001, task.Z_HC-.000001):
                    point = np.array([x,y,z])
                    assert not any(np.all(point > lo+1e-8) and np.all(point < hi-1e-8)
                                   for lo,hi in window), ("blocked probe passage", point)
        assert abs((xfar-task.X_WALL)*1000 - 53.725) < 1e-8
    for dx in (-.0015, -.0005, 0.0):
        for dy in (-.003, 0.0, .003):
            for dz in (0.0, .0015):
                for a in (-.2, 0.0, .2):
                    st = screen(dx=dx,dy=dy,dz=dz,rot=tipped(a))
                    if task.tier({"screen":st}) == 1.0:
                        apex = task.spring_apices(st)
                        depth = apex[:,:,0] - task.X_WALL
                        assert np.all(depth > .045) and np.all(depth < .055), depth
                        for i, side in enumerate((1,-1)):
                            assert np.all(np.abs(apex[i,:,1] - side*task.SPRING_Y) < .0065)
    assert task.springs_bearing(screen()), "nominal apex contacts must bear"
    assert not task.springs_bearing(screen(ql=SEAT_Q+.0003, qr=SEAT_Q+.0003)), "0.30 mm gap must fail"
    assert replay({**good, "screen":screen(ql=SEAT_Q+.0003, qr=SEAT_Q+.0003)}) == .5
    assert task.springs_bearing(screen(ql=SEAT_Q+.00009, qr=SEAT_Q+.00009)), "0.09 mm gap is inside unchanged allowance"
    assert not task.springs_bearing(screen(ql=SEAT_Q+.00011, qr=SEAT_Q+.00011)), "0.11 mm gap is outside unchanged allowance"

    # the drawings carry the model's dimensions
    d = json.loads((pathlib.Path(__file__).resolve().parent / "fabrication" / "dimensions.json").read_text())
    mm = 1000.0
    same = [
        (d["screen"]["width"], task.W * mm), (d["screen"]["height"], task.H * mm),
        (d["screen"]["frame_thickness"], task.T * mm), (d["screen"]["frame_face"], task.FACE * mm),
        (d["spring"]["length"], task.SPRING_LEN * mm), (d["spring"]["width"], task.SPRING_WIDTH * mm),
        (d["spring"]["free_crown_above_top_edge"], SPRING_CROWN * mm),
        (d["spring"]["crown_centre_in_from_side"], (task.W / 2 - task.SPRING_Y) * mm),
        (d["spring"]["pressed_when_seated"], SEAT_Q * mm),
        (d["pull_tab"]["proud_of_face"], task.TAB_PROUD * mm), (d["pull_tab"]["tall"], task.TAB_TALL * mm),
        (d["pull_tab"]["centre_above_bottom_edge"], task.TAB_Z * mm),
        (d["pull_tab"]["right_centre_above_bottom_edge"], task.TAB_ZS[-1] * mm),
        (d["pull_tab"]["in_from_side"], (task.W / 2 - task.TAB_Y) * mm),
        (d["window"]["probe_port_width"], task.PROBE_PORT_W * mm),
        (d["window"]["probe_port_height"], task.PROBE_PORT_H * mm),
        (d["window"]["probe_port_length"], (task.X_LIPH-task.X_WALL)*mm),
        (d["window"]["probe_port_centre_from_opening_centre"], task.SPRING_Y*mm),
        (d["window"]["probe_port_bottom_above_floor"], (task.Z_HC-task.Z_SILL-task.PROBE_PORT_H)*mm),
        (d["window"]["probe_port_top_above_floor"], (task.Z_HC-task.Z_SILL)*mm),
        (d["window"]["play_each_side"], task.PLAY * mm),
        (d["window"]["reveal_from_screen_side"], task.REVEAL * mm),
        (d["window"]["jamb_face_to_seated_screen_face"], task.DEPTH * mm),
        (d["window"]["sill_channel_width"], (task.T + task.CLR) * mm),
        (d["window"]["head_channel_width"], task.HEAD_W * mm),
        (d["window"]["sill_lip_height"], task.SILL_LIP_H * mm), (d["window"]["sill_lip_thickness"], task.LIP_T * mm),
        (d["window"]["head_lip_covers_top"], task.HEAD_LAP * mm),
        (d["window"]["head_channel_ceiling_above_seated_top"], GAP_TOP * mm),
        (d["window"]["stop_lap"], task.STOP_LAP * mm), (d["window"]["stop_thickness"], task.STOP_T * mm),
        (d["window"]["stool_projection"], task.STOOL * mm),
        (d["window"]["sill_above_table"], (task.Z_SILL - task.TOP) * mm),
        (d["reading"]["seated_if_screen_face_behind_jamb_face_at_least"], (task.DEPTH - task.FLUSH_TOL) * mm),
        (d["reading"]["bottom_rail_showing_above_sill_lip_seated"], (task.FACE - task.SILL_LIP_H) * mm),
        (d["reading"]["bottom_down_if_bottom_rail_showing_at_most"],
         (task.FACE - task.SILL_LIP_H + task.SILL_TOL) * mm),
        (d["reading"]["top_captured_if_top_rail_showing_below_head_lip_at_most"], (task.FACE - task.TOP_IN) * mm),
    ]
    for drawn, model in same:
        assert abs(drawn - model) <= 0.011, (drawn, model)
    assert abs(d["spring"]["force_seated_N"] - task.SPRING_K * SEAT_Q) < 0.05
    assert abs(d["spring"]["force_flat_N"] - task.SPRING_K * SPRING_CROWN) < 0.05

    assert abs(d["spring"]["assumed_preload_N"]-task.SPRING_PRELOAD_N)<1e-10
    assert abs(d["spring"]["drive_target_m"]-task.SPRING_TARGET)<1e-10
    assert abs(d["spring"]["net_upright_seated_N"]-(task.SPRING_PRELOAD_N+task.SPRING_K*SEAT_Q))<1e-5
    assert abs(d["spring"]["net_upright_flat_N"]-(task.SPRING_PRELOAD_N+task.SPRING_K*SPRING_CROWN))<1e-5


_run()
