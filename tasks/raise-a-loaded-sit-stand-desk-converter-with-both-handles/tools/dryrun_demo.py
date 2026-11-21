"""Kinematic dry run of the scripted demonstration (and its two controls) through the real scorer.

Run from the task folder with a Python that has yam_arena: ``python tools/dryrun_demo.py [variant ...]``.
Variants: nominal, one_paddle, release_mismatch. Prints one summary line per run; ``--json`` writes the traces.
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "tools", ROOT / "tools/stubs"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


from kinematic_world import KinematicWorld  # noqa: E402
from rdb_desk_converter import fixed_scene as fixture  # noqa: E402
from rdb_desk_converter.grading import make_grade  # noqa: E402
from rdb_desk_converter.scripted import ConverterReference  # noqa: E402

EXPECTED = {"nominal": 1., "one_paddle": 0., "release_mismatch": .5}


def run(variant="nominal", max_steps=fixture.MAX_STEPS, history=None):
    world = KinematicWorld()
    reference = ConverterReference(None if variant == "nominal" else variant)
    grade = make_grade()
    actions = reference.sequence()
    last = None
    trace = []
    for _ in range(max_steps):
        reference.observe(world.q, world.state())
        chunk = next(actions)
        world.step(chunk.actions[0].data)
        top, rot, monitor, keyboard = world.poses()
        tcps = {a: world.tcp(a) for a in fixture.ARMS}
        last = grade.update(world.theta, world.latch, top, rot, monitor, keyboard, tcps, world.touching())
        if history is not None:
            history.append((world.tick, world.q.copy(), world.theta))
        trace.append((world.tick, last["setting"], last["latch_mm"]["left"], last["latch_mm"]["right"], last["score"]))
        if grade.finished:
            break
    return last, reference, trace


def main():
    names = [a for a in sys.argv[1:] if not a.startswith("--")] or list(EXPECTED)
    results = {}
    ok = True
    for name in names:
        last, reference, trace = run(name)
        passed = last["score"] == EXPECTED[name]
        ok &= passed
        print(f"{name:17s} score {last['score']:.2f} (expected {EXPECTED[name]:.2f}) {'PASS' if passed else 'FAIL'}  "
              f"steps {trace[-1][0]}  setting {last['setting']:.3f} peak {last['peak_setting']:.3f} locked "
              f"{last['locked_at']} clear {last['clear']} worst IK {1000 * reference.worst_ik:.2f} mm")
        print("   phases:", [(t, n) for t, n in reference.phases])
        print("   events:", [(e["tick"], e["event"]) for e in reference.log["events"]])
        results[name] = {"final": last, "phases": reference.phases, "events": reference.log["events"],
                         "trace": trace[::5]}
    if "--json" in sys.argv:
        (ROOT / "dryrun.json").write_text(json.dumps(results, indent=1, default=float))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
