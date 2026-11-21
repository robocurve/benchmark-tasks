"""Near-successes and unintended strategies, scored offline by the factory checker.

End-state CASES preserve the placement limits. Step trajectories use measured-contact-shaped records for
base support, incidental contact, display grips and unsupported descents. Separate geometric checks exercise
contact_observation() with articulated slab-face witnesses. These are scorer checks, not physical execution.
"""

import math

import numpy as np

import lots
import task
from task import BOOKS_C, BOOKS_TOP, STAND_TOP, TARGET, TOP

lots.use("A")
CLEAR = dict(lid_gap=0.05, hand_gap=0.05, lid_contact=False, base_support=False, display_pinch=False)
HELD = dict(CLEAR, hand_gap=0.0005, base_support=True)
BRUSH = dict(HELD, lid_gap=0.0005, lid_contact=True)
LID_ONLY = dict(CLEAR, lid_gap=0.0005, hand_gap=0.0005, lid_contact=True)
PINCH = dict(LID_ONLY, display_pinch=True)


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def q_tilt(deg, yaw=0.0, axis="y"):
    """Yaw first, then a lean of ``deg`` about the world x or y axis."""
    a = math.radians(deg) / 2
    qa = np.array([math.sin(a), 0.0, 0.0, math.cos(a)]) if axis == "x" else np.array([0.0, math.sin(a), 0.0,
                                                                                         math.cos(a)])
    x1, y1, z1, w1 = qa
    x2, y2, z2, w2 = qz(yaw)
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def lot_of(name):
    return lots.all_lots()[lots.index(name)]


def laptop(pos, yaw=0.0, lid=0.0, quat=None):
    return task.LAPTOP.state(pos=pos, quat_xyzw=quat if quat is not None else qz(yaw), joints={"lid": lid})


def start(lot="A"):
    lt = lot_of(lot)
    return {"laptop": laptop(lt.pos("laptop"), lt.yaw_of("laptop")), "books": task.BOOKS.state(),
            "stand": task.STAND.state()}


def placed(lot="A", dx=0.0, dy=0.0, dz=0.0, lid=0.0, quat=None):
    s = start(lot)
    s["laptop"] = laptop((TARGET[0] + dx, TARGET[1] + dy, STAND_TOP + dz), 0.0, lid, quat)
    return s


def held_over(lot="A", z=0.04):
    s = start(lot)
    s["laptop"] = laptop((TARGET[0], TARGET[1], STAND_TOP + z), 0.0)
    return s


def lifted(lot="A", z=0.03):
    s = start(lot)
    lt = lot_of(lot)
    x, y, _ = lt.pos("laptop")
    s["laptop"] = laptop((x, y, BOOKS_TOP + z), lt.yaw_of("laptop"))
    return s


def preset():
    task.reset_record()
    task._rec.update(steps=400, gaps=(0.05, 0.05), since=18.0, held=True, t=20.0, lifted=True)


preset()
CASES = [
    ("laptop on the stand, front 55 mm over its front edge, hands clear, still 1 s", placed(), 1.0, True),
    ("35 mm further back on the stand (front overhang 20 mm)", placed(dx=0.035), 1.0, True),
    ("36 mm further back (front overhang 19 mm)", placed(dx=0.036), 0.5, False),
    ("centred front to back on the stand (front overhang 29 mm)", placed(dx=task.STAND_C[0] - TARGET[0]), 1.0, True),
    ("35 mm further forward (front overhang 90 mm)", placed(dx=-0.035), 1.0, True),
    ("36 mm further forward (front overhang 91 mm)", placed(dx=-0.036), 0.5, False),
    ("on the stand turned 15 degrees", placed(quat=qz(math.radians(15.0))), 1.0, True),
    ("on the stand turned 16 degrees", placed(quat=qz(math.radians(16.0))), 0.5, False),
    ("on the stand the wrong way round (screen toward the robot)", placed(quat=qz(math.pi)), 0.5, False),
    ("25 mm to the left of the stand's middle", placed(dy=0.025), 1.0, True),
    ("26 mm to the left", placed(dy=0.026), 0.5, False),
    ("held 4 mm above the stand", placed(dz=0.004), 0.5, False),
    ("resting on the stand tilted 5 degrees", placed(quat=q_tilt(5.0)), 1.0, True),
    ("on the stand but tilted 7 degrees (propped on something)", placed(quat=q_tilt(7.0)), 0.5, False),
    ("on the stand, lid 20 degrees further open", placed(lid=math.radians(20.0)), 1.0, True),
    ("on the stand, lid pushed 30 degrees toward closed", placed(lid=math.radians(-30.0)), 0.0, False),
    ("on the stand, lid shut (laptop asleep, upload stopped)", placed(lid=-task.OPEN), 0.0, False),
    ("held 40 mm over the stand", held_over(), 0.5, False),
    ("lifted 30 mm off the books", lifted(), 0.25, False),
    ("lifted only 5 mm off the books", lifted(z=0.005), 0.0, False),
    ("still on the books", start(), 0.0, False),
]


def _expect(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"{label}: scored {got}, expected {want}")
    print(f"ok    {label}: {got}")


def _run(lot, steps, label, want):
    """Drive observe() over ``steps``: (objects, gaps) per 0.05 s step, then score the last scene."""
    lots.use(lot)
    task.reset_record()
    t, last = 0.0, None
    for objects, g in steps:
        t += 0.05
        last = objects
        task.observe(objects, g, t)
    _expect(f"lot {lot}: {label}", float(task.score(last)), want)


def seq(*parts):
    out = []
    for n, objects, g in parts:
        out += [(objects, g)] * n
    return out


def _clouds():
    return {(arm, link): np.array([[2.0, 2.0, 2.0]])
            for arm in ("left_robot", "right_robot") for link in task.ARM_LINKS}


def _geometry_checks(lot):
    # Test the actual classifier at both articulated slab faces and its free edges, not just boolean flags.
    objects = start(lot)
    st = objects["laptop"]
    o, r = task.lid_pose(st)
    shape = task.LID_COLLIDE[0]
    centre = o + r @ (np.asarray(shape.at) - np.asarray(task.PIVOT))
    rr = r @ shape.R
    h = np.asarray(shape.size) / 2
    def point(x, y, z):
        return np.array([centre + rr @ np.array([x, y, z])])
    for y in (0.0, h[1], -h[1]):
        cloud = _clouds()
        cloud[("left_robot", "tip_left")] = point(-0.03, y, -h[2] - 0.0005)
        cloud[("left_robot", "tip_right")] = point(-0.03, y, h[2] + 0.0005)
        assert task.contact_observation(objects, cloud)["display_pinch"], "edge/face pinch missed"
        cloud[("left_robot", "tip_right")] = point(-0.03, y, -h[2] - 0.0005)
        assert not task.contact_observation(objects, cloud)["display_pinch"], "same-face brush called pinch"
    cloud = _clouds()
    cloud[("left_robot", "tip_left")] = point(-0.08, -0.12, -h[2] - 0.0005)
    cloud[("right_robot", "tip_right")] = point(0.08, 0.12, h[2] + 0.0005)
    assert not task.contact_observation(objects, cloud)["display_pinch"], "distant brushes called a pinch"
    for separation, want in ((0.00999, True), (0.01001, False)):
        cloud[("left_robot", "tip_left")] = point(0.0, 0.0, -h[2] - 0.0005)
        cloud[("right_robot", "tip_right")] = point(separation, 0.0, h[2] + 0.0005)
        assert task.contact_observation(objects, cloud)["display_pinch"] == want
    cloud = _clouds()
    cloud[("left_robot", "tip_left")] = point(-0.03, 0.0, -h[2] - 0.0005)
    assert not task.contact_observation(objects, cloud)["display_pinch"], "single hook called pinch"
    cloud[("right_robot", "tip_right")] = point(-0.03, 0.0, h[2] + 0.0005)
    assert task.contact_observation(objects, cloud)["display_pinch"], "two-arm display clamp missed"
    cloud = _clouds()
    rot, pos = task._rot(st.quat_xyzw), np.asarray(st.pos)
    cloud[("left_robot", "tip_left")] = np.array([pos + rot @ np.array([0.0, 0.0, -0.0005])])
    assert task.contact_observation(objects, cloud)["base_support"]
    for dz, expected in ((-0.002999, True), (-0.003001, False), (0.002999, True), (0.003001, False)):
        cloud[("left_robot", "tip_left")] = np.array([pos + rot @ np.array([0.0, 0.0, dz])])
        assert task.contact_observation(objects, cloud)["base_support"] == expected, "base-support tolerance wrong"
    cloud[("left_robot", "tip_left")] = np.array([pos + rot @ np.array([task.LAP_D / 2 + 0.0005, 0.0, task.BASE_T / 2])])
    assert not task.contact_observation(objects, cloud)["base_support"], "base side graze masks a lid lift"
    del cloud[("right_robot", "tip_right")]
    assert task.contact_observation(objects, cloud) is None, "missing tip did not fail closed"


def _checks():
    for lot in ("A", "B"):
        _geometry_checks(lot)
        s0, up, ov, pl = start(lot), lifted(lot), held_over(lot), placed(lot)
        dn = []
        for k in range(1, 9):
            d = dict(ov)
            d["laptop"] = laptop((TARGET[0], TARGET[1], STAND_TOP + 0.04 - 0.005 * k), 0.0)
            dn.append((1, d, HELD))
        good = [(10, s0, CLEAR), (20, up, HELD), (20, ov, HELD), *dn, (30, pl, CLEAR)]
        _run(lot, seq(*good), "base carry, placed and released", 1.0)
        _run(lot, seq((40, s0, CLEAR)), "nothing done", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, up, HELD)), "held clear of books", 0.25)
        _run(lot, seq((10, s0, CLEAR), (20, up, HELD), (20, ov, HELD)), "held above stand", 0.5)
        _run(lot, seq(*good[:-1], (30, pl, HELD)), "placed with hand still on base", 0.75)
        _run(lot, seq(*good[:-1], (20, pl, CLEAR)), "placed, released and still 0.95 s", 0.75)
        _run(lot, seq(*good[:-1], (23, pl, CLEAR)), "placed, released and still 1.1 s", 1.0)
        _run(lot, seq(*good[:-1], (30, pl, dict(CLEAR, hand_gap=0.025))), "hand 25 mm from base", 0.75)
        _run(lot, seq(*good[:-1], (30, pl, dict(CLEAR, hand_gap=0.030))), "hand 30 mm from base", 1.0)
        _run(lot, seq(*good[:-1], (30, pl, dict(CLEAR, hand_gap=0.0011, lid_gap=0.0011))),
             "hand 1.1 mm from lid without opposing grip", 0.75)
        tilted = dict(up)
        tilted["laptop"] = laptop(up["laptop"].pos, quat=q_tilt(21.0, lot_of(lot).yaw_of("laptop")))
        _run(lot, seq((10, s0, CLEAR), (20, tilted, HELD), (20, ov, HELD), *dn, (30, pl, CLEAR)),
             "held base tilted 21 degrees then placed", 1.0)
        _run(lot, seq(*good, (25, lifted(lot), HELD)), "placed then taken back above books", 0.25)
        _run(lot, seq(*good, (25, pl, HELD)), "placed then regripped on stand", 0.75)
        slid = dict(s0)
        lt = lot_of(lot)
        x, y, _ = lt.pos("laptop")
        slid["laptop"] = laptop((x - 0.060, y, BOOKS_TOP), lt.yaw_of("laptop"))
        _run(lot, seq((10, s0, CLEAR), (30, slid, CLEAR)), "slid forward, never lifted clear", 0.0)
        across = dict(s0)
        gap_y = (BOOKS_C[1] - task.BOOK_W / 2 + task.STAND_C[1] + task.STAND_W / 2) / 2
        across["laptop"] = laptop((TARGET[0], gap_y, BOOKS_TOP + 0.0075), quat=q_tilt(-4.2, axis="x"))
        _run(lot, seq((10, s0, CLEAR), (20, up, HELD), (30, across, CLEAR)), "resting across books and stand", 0.0)
        _run(lot, seq((10, s0, CLEAR), (1, s0, LID_ONLY), *good[1:]), "one lid brush before base grip", 1.0)
        _run(lot, seq((10, s0, CLEAR), (20, up, BRUSH), (20, ov, BRUSH), *dn, (30, pl, CLEAR)),
             "lid brush while base supported by hands", 1.0)
        _run(lot, seq((10, s0, CLEAR), (1, s0, PINCH), *good[1:]), "display pinch without lift, then recovery", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, up, LID_ONLY), *good[2:]), "unilateral lid hook lifts laptop", 0.0)
        # A 5.1 mm centre rise while the rear edge still touches the books is also a display lift.
        partial = dict(s0)
        partial["laptop"] = laptop((s0["laptop"].pos[0], s0["laptop"].pos[1], BOOKS_TOP + 0.0051),
                                  quat=q_tilt(2.8, lot_of(lot).yaw_of("laptop")))
        assert task.supported(partial["laptop"])
        _run(lot, seq((10, s0, LID_ONLY), (1, partial, LID_ONLY), *good[1:]),
             "unilateral display lift while back edge remains supported", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, up, PINCH), *good[2:]), "display pinch carries laptop", 0.0)
        # Desk contact is allowed; a laptop left there is still not progress toward the stand.
        desk = dict(pl)
        desk["laptop"] = laptop((0.70, -0.40, TOP), 0.0)
        approach = []
        for k in range(20, -1, -1):
            d = dict(desk)
            d["laptop"] = laptop((0.70, -0.40, TOP + k * 0.01), 0.0)
            approach.append((1, d, HELD))
        _run(lot, seq((10, s0, CLEAR), (20, up, HELD), *approach, (5, desk, CLEAR)),
             "controlled padded-desk set-down left there", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, up, HELD), *approach, (5, desk, CLEAR),
                      (10, ov, HELD), *dn, (30, pl, CLEAR)), "padded-desk rest, regrip and complete", 1.0)
        # Fast lowering remains controlled while base support is measured.
        fast = []
        for k in range(3, -1, -1):
            d = held_over(lot, z=k * 0.02)
            fast.append((1, d, HELD))
        _run(lot, seq((10, s0, CLEAR), (20, held_over(lot, z=0.06), HELD), *fast, (30, pl, CLEAR)),
             "held lowering at 0.4 m/s then release on support", 1.0)
        fall = []
        for k in range(3, -1, -1):
            fall.append((1, held_over(lot, z=k * 0.02), CLEAR))
        _run(lot, seq((10, s0, CLEAR), (20, held_over(lot, z=0.06), HELD), *fall, (30, pl, CLEAR)),
             "unheld fall at 0.4 m/s and correct final placement", 0.0)
        slow_fall = [(1, held_over(lot, z=0.06-k*0.005), CLEAR) for k in range(13)]
        _run(lot, seq((10, s0, CLEAR), (20, held_over(lot, z=0.06), HELD), *slow_fall, (30, pl, CLEAR)),
             "unheld descent at 0.1 m/s is not controlled lowering", 0.0)
        between = dict(pl)
        between["laptop"] = laptop((0.70, -0.40, TOP + 0.12), 0.0)
        falling = dict(between)
        falling["laptop"] = laptop((0.70, -0.40, TOP + 0.10), 0.0)
        _run(lot, seq((10, s0, CLEAR), (5, between, HELD), (1, between, CLEAR),
                      (1, falling, CLEAR), (10, ov, HELD), *dn, (30, pl, CLEAR)),
             "drop between supports, then recovered", 0.0)
        # First sampled contact and release may coincide; no invented proof of an inter-frame drop.
        first_contact = [(1, held_over(lot, z=0.005), HELD), (30, pl, CLEAR)]
        _run(lot, seq((10, s0, CLEAR), (20, ov, HELD), *first_contact),
             "opening at first sampled supported contact", 1.0)
        shut = placed(lot, lid=math.radians(-26.0))
        tipped = placed(lot, quat=q_tilt(26.0))
        _run(lot, seq((10, s0, CLEAR), (1, shut, HELD), *good[1:]), "lid beyond 25 degrees then recovered", 0.0)
        _run(lot, seq((10, s0, CLEAR), (1, tipped, HELD), *good[1:]), "base beyond 25 degrees then recovered", 0.0)
        _run(lot, seq((10, s0, CLEAR), (1, up, None), *good[1:]), "missing arm data then recovered", 0.0)
        bad = dict(up)
        from dataclasses import replace
        bad["laptop"] = replace(up["laptop"], joints={})
        _run(lot, seq((10, s0, CLEAR), (1, bad, HELD), *good[1:]), "missing lid joint then recovered", 0.0)
        bad["laptop"] = replace(up["laptop"], quat_xyzw=np.zeros(4))
        _run(lot, seq((10, s0, CLEAR), (1, bad, HELD), *good[1:]), "invalid quaternion then recovered", 0.0)
        for label, overrides in (
                ("missing lid link", dict(links={})),
                ("nonfinite lid joint", dict(joints={"lid": float("nan")})),
                ("nonfinite lid link", dict(links={"lid": (np.zeros(3), np.array([0., 0., float("nan"), 1.]))})),
                ("zero lid-link quaternion", dict(links={"lid": (np.zeros(3), np.zeros(4))})),
                ("nonfinite velocity", dict(lin_vel=np.array([0., 0., float("nan")])))):
            bad["laptop"] = replace(up["laptop"], **overrides)
            _run(lot, seq((10, s0, CLEAR), (1, bad, HELD), *good[1:]), label + " then recovered", 0.0)
        for invalid_t in (0.05, -1.0, float("nan")):
            task.reset_record()
            task.observe(s0, CLEAR, 0.05)
            task.observe(up, HELD, invalid_t)
            task.observe(pl, CLEAR, 0.10)
            assert not task._rec["known"] and task.score(pl) == 0.0, "bad time did not fail closed"
        _run(lot, seq((10, s0, LID_ONLY), (20, up, BRUSH), (20, ov, BRUSH),
                      *((n, obj, BRUSH) for n, obj, _ in dn), (30, pl, LID_ONLY), (30, pl, CLEAR)),
             "continuous harmless brush through base carry and supported release", 1.0)
        release = held_over(lot, z=0.06)
        caught = held_over(lot, z=0.0477)
        _run(lot, seq((10, s0, CLEAR), (1, release, CLEAR), (1, caught, HELD),
                      (20, ov, HELD), *dn, (30, pl, CLEAR)), "observed free fall caught then recovered", 0.0)
        yaw_motion = [(1, placed(lot, quat=qz(math.radians(10 if k % 2 else -10))), CLEAR) for k in range(32)]
        _run(lot, seq((10, s0, CLEAR), *yaw_motion), "yaw oscillation cannot complete stillness", 0.75)
        tilt_motion = [(1, placed(lot, quat=q_tilt(3.0, axis="x" if k % 2 else "y")), CLEAR) for k in range(32)]
        _run(lot, seq((10, s0, CLEAR), *tilt_motion), "changing tilt axis cannot complete stillness", 0.75)
        for name in ("books", "stand"):
            malformed = dict(pl); malformed[name] = None
            _run(lot, seq((10, s0, CLEAR), (1, malformed, CLEAR), (30, pl, CLEAR)),
                 "malformed " + name + " cannot recover credit", 0.0)
    lots.use("A")
    preset()


if __name__ == "__main__":
    _checks()
    st = start("A")
    for label, changes, want, want_success in CASES:
        s = dict(st)
        s.update(changes)
        got = 1.0 if task.success(s) else float(task.score(s))
        _expect(label, got, want)
else:
    _checks()
