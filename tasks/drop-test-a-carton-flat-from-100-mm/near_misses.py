"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Give a quaternion for every tilted or turned case.

The score depends on the trial's history (how the carton came down), so the single-state CASES below can only
show landings with no drop seen (0.2 at best). They are ordered so that no case disturbs an earlier landing
before the cases that expect 0.2: the checker does not return to the start pose between cases. The drops
themselves are simulated step by step in this file's own _run().
"""

import math

from task import C_SIZE, PAD, PAD_TOP, TOP


def rot(axis, deg):
    s = math.sin(math.radians(deg) / 2)
    return (s * axis[0], s * axis[1], s * axis[2], math.cos(math.radians(deg) / 2))


PX, PY = PAD.pos[0], PAD.pos[1]
FLAT_Z = PAD_TOP + C_SIZE[2] / 2


def tilted(deg, x=PX, y=PY):
    """Tilted deg about x with its lowest edge on the pad."""
    a = math.radians(deg)
    return ((x, y, PAD_TOP + 0.5 * (C_SIZE[1] * math.sin(a) + C_SIZE[2] * math.cos(a))), None, None, rot((1, 0, 0), deg))


def edge_case(out):
    """Tilted 5 deg about x, placed so the lowest face's outer corners are `out` metres past the pad's left edge."""
    a = math.radians(5.0)
    reach = C_SIZE[1] / 2 * math.cos(a) + C_SIZE[2] / 2 * math.sin(a)  # bottom-face corner, from the centre, in y
    (x, _, z), _s, _v, q = tilted(5.0)
    return ((x, PY + PAD.size[1] / 2 - reach + out, z), None, None, q)


CASES = [
    ("set down flat on the pad, no drop seen", {"carton": ((PX, PY, FLAT_Z), None, None)}, 0.2, False),
    ("set down, turned 4 deg about the vertical", {"carton": ((PX, PY, FLAT_Z), None, None, rot((0, 0, 1), 4))},
     0.2, False),
    ("set down exactly 5 deg off flat", {"carton": tilted(5.0)}, 0.2, False),
    ("6 deg off flat", {"carton": tilted(6.0)}, 0.0, False),
    ("on its side (80 x 60 face down)", {"carton": ((PX, PY, PAD_TOP + C_SIZE[0] / 2), None, None,
                                                    rot((0, 1, 0), 90))}, 0.0, False),
    ("on its long side (120 x 60 face down)", {"carton": ((PX, PY, PAD_TOP + C_SIZE[1] / 2), None, None,
                                                         rot((1, 0, 0), 90))}, 0.0, False),
    ("held 1.5 mm above the pad", {"carton": ((PX, PY, FLAT_Z + 0.0015), None, None)}, 0.0, False),
    ("overhanging the pad's edge by 5 mm", {"carton": ((PX + PAD.size[0] / 2 - C_SIZE[0] / 2 + 0.005, PY, FLAT_Z),
                                                       None, None)}, 0.0, False),
    ("still sliding on the pad", {"carton": ((PX, PY, FLAT_Z), None, (0.05, 0.0, 0.0))}, 0.0, False),
    ("set down tilted 5 deg near the pad edge, bottom corners 1 mm inside", {"carton": edge_case(-0.001)}, 0.2, False),
    ("the same, bottom corners 1 mm over the edge (mid-height corners still inside)", {"carton": edge_case(0.001)},
     0.0, False),
    ("on the table beside the pad", {"carton": ((PX, PY + 0.17, TOP + C_SIZE[2] / 2), None, None)}, 0.0, False),
]


def _run() -> int:
    """CASES in order, then multi-step drops: each a reset, a hold, a free fall sampled at 20 Hz, a landing."""
    import sys
    from pathlib import Path

    import numpy as np

    sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "factory"))
    import check

    import task

    size = np.asarray(C_SIZE)
    start_pos = np.asarray(task.CARTON.pos)
    bad = 0

    def report(got, want, label):
        nonlocal bad
        flag = "ok  " if abs(got - want) < 1e-9 else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")

    def call(pos, vel=(0, 0, 0), quat=None):
        return task.score({"drop_pad": check.state(PAD.pos, PAD.size),
                           "carton": check.state(pos, size, vel, quat)})

    def reset():
        call(start_pos + 0.3)  # anywhere far away, then back at the start: an episode reset
        call(start_pos)

    call(start_pos)
    for label, changes, want, want_success in CASES:
        pos, sz, vel, *q = changes["carton"]
        got = call(pos, vel or (0, 0, 0), q[0] if q else None)
        report(got, want, label)

    def drop(h, *, after=None, land=(PX, PY), land_quat=None, bounce=False, rng=np.random.default_rng(0)):
        """Hold the carton still with its underside h above the pad, let it fall, land, settle."""
        reset()
        x, y = land
        z0 = PAD_TOP + h + C_SIZE[2] / 2
        for k in range(10):  # carried in and held still
            call((x, y, z0 + rng.uniform(-1e-5, 1e-5)))
        t = 0.0
        while True:
            t += 0.05
            z = z0 - 0.5 * task.G * t * t
            if z <= FLAT_Z:
                break
            call((x, y, z), (0, 0, -task.G * t))
        if bounce:  # a 6 mm bounce on landing: two steps in the air, the apex nearly still
            call((x, y, FLAT_Z + 0.005), (0, 0, 0.15))
            call((x, y, FLAT_Z + 0.006), (0, 0, 0.02))
            call((x, y, FLAT_Z + 0.003), (0, 0, -0.25))
        last = 0.0
        for k in range(30):  # landed and at rest
            last = call((x, y, FLAT_Z + rng.uniform(-1e-6, 1e-6)), (0, 0, 0), land_quat)
        if after:
            last = after(call)
        return last

    for h, want in ((0.100, 1.0), (0.075, 1.0), (0.074, 0.5), (0.150, 1.0), (0.155, 0.5), (0.055, 0.5),
                    (0.040, 0.2), (0.030, 0.2), (0.240, 0.5), (0.260, 0.0)):
        report(drop(h), want, f"dropped from {h * 1000:.0f} mm, landed flat")
    report(drop(0.100, land_quat=rot((1, 0, 0), 8)), 0.0, "dropped from 100 mm, landed 8 deg off flat")
    report(drop(0.100, land=(PX + 0.07, PY)), 0.0, "dropped from 100 mm, landed overhanging the pad")

    def lowered(call_):
        """Lowered slowly at 5 cm/s from 100 mm instead of dropped."""
        reset()
        z = FLAT_Z + 0.100
        while z > FLAT_Z:
            call_((PX, PY, z), (0, 0, -0.05))
            z -= 0.0025
        out = 0.0
        for _ in range(20):
            out = call_((PX, PY, FLAT_Z))
        return out

    report(lowered(call), 0.2, "lowered at 5 cm/s and let go on the pad")

    def driven(accel, h, brake_steps):
        """Still in the gripper: driven down from h at `accel`, then braked to a stop on the pad over brake_steps."""
        reset()
        z0 = PAD_TOP + h + C_SIZE[2] / 2
        for _ in range(10):
            call((PX, PY, z0))
        t, z, v = 0.0, z0, 0.0
        while True:
            t += 0.05
            v = -accel * t
            z = z0 - 0.5 * accel * t * t
            if brake_steps and z - FLAT_Z <= 0.5 * v * v / (accel * 2) + 0.004:  # brake in time to stop at the pad
                break
            if z <= FLAT_Z:
                break
            call((PX, PY, z), (0, 0, v))
        for k in range(brake_steps):
            f = (k + 1) / brake_steps
            call((PX, PY, max(FLAT_Z, z - (z - FLAT_Z) * f)), (0, 0, v * (1 - f)))
        out = 0.0
        for _ in range(20):
            out = call((PX, PY, FLAT_Z))
        return out

    report(driven(0.7 * task.G, 0.100, 0), 0.2, "in the gripper, driven down at 0.7 g from 100 mm onto the pad")
    report(driven(1.0 * task.G, 0.060, 2), 0.2, "in the gripper, a 60 mm jab at 1 g, braked before the pad")

    report(drop(0.100, bounce=True), 1.0, "good drop with a 6 mm bounce before it settles")

    def tossed():
        """Tossed upward from 40 mm to a 100 mm apex, then falls (accepted: the drop height is the apex)."""
        reset()
        z0 = PAD_TOP + 0.040 + C_SIZE[2] / 2
        for _ in range(10):
            call((PX, PY, z0))
        apex = PAD_TOP + 0.100 + C_SIZE[2] / 2
        v0 = math.sqrt(2 * task.G * (apex - z0))
        t = 0.0
        while True:
            t += 0.05
            z = z0 + v0 * t - 0.5 * task.G * t * t
            if z <= FLAT_Z:
                break
            call((PX, PY, z), (0, 0, v0 - task.G * t))
        out = 0.0
        for _ in range(20):
            out = call((PX, PY, FLAT_Z))
        return out

    report(tossed(), 1.0, "tossed up to a 100 mm apex (accepted: scored by the apex)")

    def regrip(call_):
        """After a good drop, picked up again, held, and set down gently."""
        for k in range(10):
            call_((PX, PY, FLAT_Z + 0.002 * (k + 1)), (0, 0, 0.04))
        for k in range(10):
            call_((PX, PY, FLAT_Z + 0.020))
        for k in range(10):
            call_((PX, PY, FLAT_Z + 0.020 - 0.002 * (k + 1)), (0, 0, -0.04))
        out = 0.0
        for _ in range(20):
            out = call_((PX, PY, FLAT_Z))
        return out

    report(drop(0.100, after=regrip), 0.2, "good drop, then picked up and set down again")

    def nudge(call_):
        """After a good drop, pushed 8 mm along the pad."""
        for k in range(8):
            call_((PX + 0.001 * (k + 1), PY, FLAT_Z), (0.02, 0, 0))
        out = 0.0
        for _ in range(20):
            out = call_((PX + 0.008, PY, FLAT_Z))
        return out

    report(drop(0.100, after=nudge), 0.0, "good drop, then pushed 8 mm along the pad")

    def turn(call_):
        """After a good drop, turned 4 deg in place (within the limit)."""
        out = 0.0
        for _ in range(20):
            out = call_((PX, PY, FLAT_Z), (0, 0, 0), rot((0, 0, 1), 4))
        return out

    report(drop(0.100, after=turn), 1.0, "good drop, then turned 4 deg in place (within 5 deg)")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
