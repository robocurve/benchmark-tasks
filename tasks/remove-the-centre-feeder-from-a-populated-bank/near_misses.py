"""Near misses: every score level, its boundaries, and the shortcuts that must not earn credit.

check.py runs CASES (the untouched start). `python near_misses.py` runs the full list below against the trial record
the simulator would have written, and exits non-zero if any case scores other than declared."""
import copy
import math

import numpy as np

import task

CASES = [("start: all three seated, nothing observed", {}, 0.0, False)]

P = task.parts
R0 = P._quat_to_matrix(task.FEEDERS[task.MIDDLE].state().quat_xyzw)  # the seated orientation


def _q(r):
    return tuple(P._matrix_to_quat(r))


def seated(name):
    return task.FEEDERS[name].state()


def middle(rot=None, dpos=(0.0, 0.0, 0.0), at=None, low=None, lever=0.0):
    """The middle feeder turned by `rot` (world, about its own origin), moved by `dpos`, or put with its sampled
    centre over `at` (x, y) and its lowest point at height `low`."""
    spec = task.FEEDERS[task.MIDDLE]
    r = R0 if rot is None else rot @ R0
    st = spec.state(pos=tuple(np.asarray(spec.pos) + np.asarray(dpos)), quat_xyzw=_q(r), joints={"lever": lever})
    if at is not None:
        pts = task.feeder_points(st)
        c = (pts.min(0) + pts.max(0)) / 2
        shift = np.array([at[0] - c[0], at[1] - c[1], low - pts[:, 2].min()])
        st = spec.state(pos=tuple(np.asarray(st.pos) + shift), quat_xyzw=_q(r), joints={"lever": lever})
    return st


def about(point, axis, angle, st_spec_rot=R0):
    """Rotation and translation that turn the seated middle feeder by `angle` about a world axis through `point`."""
    rr = P._rot(axis, angle)
    spec = task.FEEDERS[task.MIDDLE]
    p0 = np.asarray(spec.pos, float)
    return rr, (rr @ (p0 - point) + point) - p0


def objects(mid=None, mat=None, **others):
    out = {n: seated(n) for n in task.NAMES}
    out[task.MIDDLE] = mid if mid is not None else out[task.MIDDLE]
    out.update(others)
    out[task.MAT.name] = mat if mat is not None else task.MAT.state()
    return out


def record(held=1.2, hand=0.05, zero=False, known=True):
    task._rec.update(task._fresh())
    start = {n: (np.asarray(seated(n).pos, float), np.asarray(seated(n).quat_xyzw, float)) for n in task.NAMES}
    task._rec.update(known=known, steps=40, t=10.0, zero=zero, why="near miss" if zero else "", hand=hand, held=held,
                     since=10.0 - held, gap=0.05, start=start)


def shifted(name, d=(0.0, 0.0, 0.0), turn=0.0):
    spec = task.FEEDERS[name]
    r = P._rot((1, 0, 0), turn) @ P._quat_to_matrix(spec.state().quat_xyzw)
    return spec.state(pos=tuple(np.asarray(spec.pos) + np.asarray(d)), quat_xyzw=_q(r))


FLAT_ON_SIDE = P._rot((0, 1, 0), -math.pi / 2)  # turns the feeder's across axis (world x when seated) to vertical
MAT_TOP = task.TOP + task.MAT_SIZE[2]


def run():
    old = copy.deepcopy(task._rec)
    results = []

    def case(label, objs, want, **rec):
        record(**rec)
        got = task.score(objs)
        ok = abs(got - want) < 1e-9 and task.success(objs) == (want == 1.0)
        results.append((label, ok, got, want))

    try:
        nose = np.array(task.world(task.NOSE[0][0], task.NOSE[0][1]))
        case("seated, observed", objects(), 0.0)
        case("lifted 3 mm, still in its slot", objects(middle(dpos=(0, 0, 0.003))), 0.0)
        rr, dp = about(nose, (1, 0, 0), math.radians(15))  # pick end down: the reel stands proud, hook still in
        case("tipped 15 degrees, hook still in the groove", objects(middle(rot=rr, dpos=dp, lever=0.3)), 0.0)
        out = middle(dpos=(0, 0.060, 0.030), lever=0.3)  # off the hook, up and out toward the reel side
        case("lifted out of its slot, still held", objects(out), 0.5, hand=0.0)
        case("lifted out and released (hands clear) but in the air", objects(out), 0.5)
        case("lifted 30 mm with the catch released, but the clip still over the slot blade",
             objects(middle(dpos=(0, 0, 0.030), lever=0.6)), 0.0)

        def at_gap(g):  # lifted 30 mm, then moved out until the nearest point is g from the rail parts
            lo, hi = 0.0, 0.08
            for _ in range(40):
                mid = (lo + hi) / 2
                if task.rail_gap(task.feeder_points(middle(dpos=(0, mid, 0.030), lever=0.3))) < g:
                    lo = mid
                else:
                    hi = mid
            return middle(dpos=(0, hi, 0.030), lever=0.3)

        case("4.9 mm clear of the rail parts", objects(at_gap(0.0049)), 0.0, hand=0.0)
        case("5.1 mm clear of the rail parts", objects(at_gap(0.0051)), 0.5, hand=0.0)
        mat = task.MAT.state()
        mc = np.asarray(mat.pos)
        lying = middle(rot=FLAT_ON_SIDE, at=(mc[0], mc[1]), low=MAT_TOP + 0.0005)
        case("lying flat on the mat, released, still 1 s", objects(lying), 1.0)
        case("lying flat on the mat, still only 0.9 s", objects(lying), 0.5, held=0.9)
        case("lying flat on the mat, a fingertip 4 mm away", objects(lying), 0.5, hand=0.004)
        case("lying flat on the mat, fingertip exactly 5 mm away", objects(lying), 1.0, hand=0.005)
        case("lying flat on the mat, but a neighbour knocked loose earlier", objects(lying), 0.0, zero=True)
        case("lying flat on the mat, observation unknown", objects(lying), 0.0, known=False)
        # the mat's long edge: the feeder's outline must lie entirely inside it
        pts = task.feeder_points(lying)
        edge_x = mc[0] + task.MAT_SIZE[0] / 2
        inside = middle(rot=FLAT_ON_SIDE, at=(mc[0] + (edge_x - pts[:, 0].max()) - 0.0005, mc[1]), low=MAT_TOP + 0.0005)
        case("lying flat, 0.5 mm inside the mat edge", objects(inside), 1.0)
        over = middle(rot=FLAT_ON_SIDE, at=(mc[0] + (edge_x - pts[:, 0].max()) + 0.010, mc[1]), low=MAT_TOP + 0.0005)
        case("lying flat, 10 mm over the mat edge", objects(over), 0.75)
        bench = middle(rot=FLAT_ON_SIDE, at=(mc[0] + 0.35, mc[1]), low=task.TOP + 0.0005)
        case("lying flat on the bench beside the mat", objects(bench), 0.75)
        tilted = middle(rot=P._rot((1, 0, 0), math.radians(25)) @ FLAT_ON_SIDE, at=(mc[0], mc[1]), low=MAT_TOP + 0.0005)
        case("on the mat but tilted 25 degrees off flat", objects(tilted), 0.5)
        case("on the mat, raised 8 mm above it", objects(middle(rot=FLAT_ON_SIDE, at=(mc[0], mc[1]), low=MAT_TOP + 0.008)),
             0.5)
        # a neighbour knocked out of its seat and put back is caught by the trial record, not the end state
        case("neighbour put back after being knocked loose", objects(lying), 0.0, zero=True)
        # the do-nothing and press-only habits
        case("pressed the pick end only (tipped, released, back in its seat)", objects(), 0.0)
        # hand-state failure closes the trial
        task._rec.update(task._fresh())
        task.observe(objects(lying), [], 10.0)
        results.append(("empty hand geometry fails closed", task.score(objects(lying)) == 0.0, task.score(objects(lying)), 0.0))
        task._rec.update(task._fresh())
        task.observe(objects(lying), [np.full((5, 3), np.nan)], 10.0)
        results.append(("NaN hand geometry fails closed", task.score(objects(lying)) == 0.0, task.score(objects(lying)), 0.0))
        # neighbours: knocked loose at any moment (hook out of the rail's slot, or catch let go), and seated at the end
        # (2 mm, 3 degrees)
        far = [np.array([[0.0, 0.0, 2.0]])]

        def history(*steps):
            task._rec.update(task._fresh())
            for k, objs in enumerate((objects(lying), *steps)):  # the first observation sets the starting seats
                task.observe(objs, far, 1.0 + 0.05 * k)
            for k in range(30):  # then the lying feeder rests, released, for 1.5 s
                task.observe(objects(lying), far, 2.0 + 0.05 * k)
            return task.score(objects(lying))

        def nb(name, uz, angle, lever=0.0):
            """A neighbour turned by `angle` (rad) about the rail's direction through its own feeder-frame point (u, z);
            a positive angle lifts the reel end. `lever` opens its catch lever (rad)."""
            spec = task.FEEDERS[name]
            k = -1 if name == "feeder_front" else 1
            pt = np.array(task.world(uz[0], uz[1], k))
            rr = P._rot((1, 0, 0), angle)
            p0 = np.asarray(spec.pos, float)
            return spec.state(pos=tuple(rr @ (p0 - pt) + pt), quat_xyzw=_q(rr @ P._quat_to_matrix(spec.state().quat_xyzw)),
                              joints={"lever": lever})

        def edge(test, lo, hi):  # the value where test() turns False (test(lo) True, test(hi) False)
            for _ in range(60):
                mid = (lo + hi) / 2
                lo, hi = (mid, hi) if test(mid) else (lo, mid)
            return (lo + hi) / 2

        hook, tooth = task.NOSE[0], task.TOOTH[0]
        # a wobble about the hook (reel end up): the catch lets go once a tooth comes out from under the blade
        lim = edge(lambda a: task.latched(nb("feeder_back", hook, a)), 0.0, math.radians(15))
        for a, want, what in ((math.radians(2.0), 1.0, "still hooked and latched: no zero"),
                              (lim - math.radians(0.2), 1.0, "0.2 degrees short of the catch letting go: no zero"),
                              (lim + math.radians(0.2), 0.0, "0.2 degrees past: the catch let go")):
            st = nb("feeder_back", hook, a)
            got = history(objects(lying, feeder_back=st))
            results.append((f"neighbour wobbled {math.degrees(a):.1f} degrees about its hook for one step, then back, "
                            f"{what} (hooked {task.hooked(st)}, latched {task.latched(st)})", got == want, got, want))
        # the hook lifted out of its slot while the catch still holds (turned inner end up about the lever's teeth)
        lim = edge(lambda a: task.hooked(nb("feeder_front", tooth, -a)), 0.0, math.radians(15))
        for a, want, what in ((lim - math.radians(0.2), 1.0, "nose axis just inside the slot's width: no zero"),
                              (lim + math.radians(0.2), 0.0, "nose axis just past the slot's edge")):
            st = nb("feeder_front", tooth, -a)
            got = history(objects(lying, feeder_front=st))
            results.append((f"neighbour's inner end lifted {math.degrees(a):.1f} degrees for one step, then back, {what} "
                            f"(hooked {task.hooked(st)}, latched {task.latched(st)})", got == want, got, want))
        # the catch opened by its lever (someone pressing a neighbour's lever), body still seated
        lim = edge(lambda q: task.latched(nb("feeder_back", hook, 0.0, lever=q)), 0.0, 0.6)
        for q, want, what in ((lim - 0.01, 1.0, "teeth still under the blade: no zero"),
                              (lim + 0.01, 0.0, "teeth clear of the blade: the catch let go")):
            got = history(objects(lying, feeder_back=nb("feeder_back", hook, 0.0, lever=q)))
            results.append((f"neighbour's lever opened {q:.3f} rad for one step, then back, {what}", got == want, got, want))
        # straight lifts, each branch alone: lever resting its teeth on the blade's corner, the nose rises out of its slot
        rest = edge(lambda q: task.latched(nb("feeder_back", hook, 0.0, lever=q)), 0.0, 0.6) - 0.002
        for dz, want, what in ((0.0036, 1.0, "nose still in its slot: no zero"), (0.0038, 0.0, "nose out of its slot")):
            st = nb("feeder_back", hook, 0.0, lever=rest)
            st = task.FEEDERS["feeder_back"].state(pos=tuple(np.asarray(st.pos) + (0, 0, dz)), quat_xyzw=st.quat_xyzw,
                                                   joints={"lever": rest})
            got = history(objects(lying, feeder_back=st))
            results.append((f"neighbour lifted {dz * 1000:.1f} mm for one step, lever teeth on the blade's corner, {what} "
                            f"(hooked {task.hooked(st)}, latched {task.latched(st)})", got == want, got, want))
        st = task.FEEDERS["feeder_front"].state(pos=tuple(np.asarray(task.FEEDERS["feeder_front"].pos) + (0, 0, 0.0026)))
        got = history(objects(lying, feeder_front=st))
        results.append((f"neighbour lifted 2.6 mm, lever closed: tooth centre above the blade's bottom face (hooked "
                        f"{task.hooked(st)}, latched {task.latched(st)})", got == 0.0, got, 0.0))
        # the backstop: sliding along the rail or turning about a vertical axis, which the simplified clip does not stop
        got = history(objects(lying, feeder_back=shifted("feeder_back", (0.0055, 0, 0))))
        results.append(("neighbour slid 5.5 mm along the rail for one step (hooked and latched): backstop, 0", got == 0.0,
                        got, 0.0))
        got = history(objects(lying, feeder_back=shifted("feeder_back", (0.0045, 0, 0))))
        results.append(("neighbour slid 4.5 mm along the rail for one step: no zero", got == 1.0, got, 1.0))
        spec = task.FEEDERS["feeder_front"]
        pt = np.array(task.world(tooth[0], tooth[1], -1))
        for deg, want in ((11.0, 0.0), (2.0, 1.0)):
            rr = P._rot((0, 0, 1), math.radians(deg))
            st = spec.state(pos=tuple(rr @ (np.asarray(spec.pos) - pt) + pt),
                            quat_xyzw=_q(rr @ P._quat_to_matrix(spec.state().quat_xyzw)))
            got = history(objects(lying, feeder_front=st))
            results.append((f"neighbour turned {deg:.0f} degrees about a vertical axis through its teeth for one step "
                            f"(hooked {task.hooked(st)}, latched {task.latched(st)})", got == want, got, want))
        got = history(objects(lying, feeder_back=shifted("feeder_back", (0, 0.006, 0))))
        results.append(("neighbour pulled 6 mm out of its slot at some moment, then back", got == 0.0, got, 0.0))
        got = history(objects(lying, feeder_front=shifted("feeder_front", turn=math.radians(11))))
        results.append(("neighbour tipped 11 degrees at some moment, then back", got == 0.0, got, 0.0))
        got = history(objects(lying, feeder_back=nb("feeder_back", hook, math.radians(1.5))),
                      objects(lying, feeder_back=nb("feeder_back", hook, math.radians(-0.5))))
        results.append(("neighbour rocked +1.5 / -0.5 degrees on its hook, hooked and latched: no zero", got == 1.0, got,
                        1.0))
        case("neighbour 2.5 mm off its seat at the end", objects(lying, feeder_back=shifted("feeder_back", (0, 0.0025, 0))),
             0.0)
        case("neighbour 1.9 mm off its seat at the end", objects(lying, feeder_back=shifted("feeder_back", (0, 0.0019, 0))),
             1.0)
        case("neighbour turned 4 degrees at the end", objects(lying, feeder_front=shifted("feeder_front",
                                                                                       turn=math.radians(4))), 0.0)
        case("neighbour turned 2.9 degrees at the end", objects(lying, feeder_front=shifted("feeder_front",
                                                                                         turn=math.radians(2.9))), 1.0)
        # the release check sees the reel's flat faces, not only its rims
        fr = task._shape_frames(lying)
        reel = [f for f in fr if f[0].kind == "cylinder" and f[0].size[0] > 0.05][0]
        _, c, r = reel
        pinch = [np.array([c + r @ np.array([0.050, s * (task.REEL_W / 2 + 0.001), 0.0]) for s in (1, -1)])]
        gap = task.hand_gap(pinch, lying)
        results.append(("fingertips 1 mm off the reel faces at mid radius are not released", gap < 0.0015, gap, 0.001))
        task._rec.update(task._fresh())
        for k in range(40):
            task.observe(objects(lying), pinch, 1.0 + 0.05 * k)
        got = task.score(objects(lying))
        results.append(("reel pinched at mid radius, lying on the mat: still held", got == 0.5, got, 0.5))
        task._rec.update(task._fresh())
        for k in range(40):
            task.observe(objects(lying), far, 1.0 + 0.05 * k)
        got = task.score(objects(lying))
        results.append(("same, hands far away for 2 s: full score", got == 1.0, got, 1.0))
        # An unobserved interval cannot count toward the continuous released/still hold.
        task._rec.update(task._fresh())
        task.observe(objects(lying), far, 1.0)
        for k in range(1, 21):
            task.observe(objects(lying), [np.array([[math.nan, 0.0, 0.0]])], 1.0 + 0.05 * k)
        got = task.score(objects(lying))
        results.append(("missing hand observations stay at zero", got == 0.0, got, 0.0))
        task.observe(objects(lying), far, 2.05)
        got = task.score(objects(lying))
        results.append(("first recovered sample cannot inherit an unobserved one-second hold", got == 0.5, got, 0.5))
        for k in range(1, 22):
            task.observe(objects(lying), far, 2.05 + 0.05 * k)
        got = task.score(objects(lying))
        results.append(("fresh continuous one-second hold after recovery earns full score", got == 1.0, got, 1.0))
        task._rec.update(task._fresh())
        task.observe(objects(lying), far, 1.0)
        task.observe(objects(lying), far, 2.05)
        got = task.score(objects(lying))
        results.append(("a missing 20 Hz callback interrupts the hold", got == 0.5, got, 0.5))
        for k in range(1, 22):
            task.observe(objects(lying), far, 2.05 + 0.05 * k)
        got = task.score(objects(lying))
        results.append(("continuous hold after missing callbacks recovers", got == 1.0, got, 1.0))
        from types import SimpleNamespace
        original_hand_state = task.observation.hand_state
        def unavailable_hand_state(sim):
            raise ValueError("missing native hand state")
        task._rec.update(task._fresh())
        task.observe(objects(lying), far, 1.0)
        try:
            task.observation.hand_state = unavailable_hand_state
            task._step(SimpleNamespace(states=lambda: objects(lying), t=2.0))
        finally:
            task.observation.hand_state = original_hand_state
        task.observe(objects(lying), far, 2.05)
        got = task.score(objects(lying))
        results.append(("native hand-read failure interrupts the hold", got == 0.5, got, 0.5))
    finally:
        task._rec.clear()
        task._rec.update(old)
    bad = 0
    for label, ok, got, want in results:
        print(("ok  " if ok else "FAIL"), f"{label}: {got} (declared {want})")
        bad += not ok
    return bad


if __name__ == "__main__":
    raise SystemExit(run())
