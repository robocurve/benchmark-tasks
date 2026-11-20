"""Dry run of the scripted demonstration without the simulator, scored by the real scorer.

Run:  python tools/dryrun_demo.py --yam-arena <yam-isaac-lab-arena checkout> [--clearance <folder with clearance.py>]
See ``kinematic_world.py`` for what the stand-in world does and does not model.
"""

import argparse
import importlib
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def dry_run(*, max_steps=None, clearance=None, verbose=False, variant="", idle=False, delay=0, **world_kw):
    from yam_arena import ik
    from yam_arena.constants import READY

    os.environ["DEMO_VARIANT"] = variant
    import rdb_filter_clamp.scripted as scripted
    importlib.reload(scripted)
    from kinematic_world import World
    from rdb_filter_clamp import fixed_scene as fixture
    from rdb_filter_clamp.grading import FilterClampGrade

    world = World(ik, clearance=clearance, **world_kw)
    reference = scripted.FilterClampReference()
    actions = reference.sequence()
    grade = FilterClampGrade(holder_start=tuple(fixture.HOLDER_START), clamp_start=tuple(fixture.CLAMP_START))
    result = None
    seen = []                      # part poses handed to the demonstration ``delay`` steps late
    for tick in range(1, (max_steps or fixture.MAX_STEPS) + 1):
        seen = (seen + [world.state()])[-(delay + 1):]
        reference.observe(world.joint_pos(), seen[0])
        command = np.asarray(next(actions).actions[0].data, dtype=float)
        if idle:
            command = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        world.step(command)
        result = grade.update(world.grade_state())
        if verbose and tick % 40 == 0:
            print(tick, world.funnel_mode, world.clamp_mode, result["score"], result["funnel_offset_mm"],
                  result["clamp_off_axis_mm"], result["upper_support"])
        if grade.finished or (not idle and reference.phases and reference.phases[-1][1] == "hands off"):
            break
    report = dict(world.report)
    report.update(score=result["score"], steps=tick, worst_ik_mm=round(1000 * reference.worst_ik, 3),
                  final={k: result[k] for k in ("assembled", "foot_clearance_mm", "lift_verified", "failed")},
                  phases=reference.phases, grasps=reference.log["grasps"], seatings=reference.log["seatings"],
                  fittings=reference.log["fittings"], ref_events=reference.log["events"])
    return report


def setup_paths(yam_arena=None, clearance_dir=None):
    for path in (ROOT, ROOT / "tools", ROOT / "tools/stubs", *((yam_arena,) if yam_arena else ())):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    if clearance_dir:
        sys.path.insert(0, clearance_dir)
        import clearance
        return clearance
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yam-arena")
    parser.add_argument("--clearance", help="folder holding clearance.py (arm collision hulls), optional")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    clearance = setup_paths(args.yam_arena, args.clearance)
    cases = (("nominal", {}),
             ("more sag, slow arm, funnel 2 mm off in the hand", dict(sag=(0., .04, 0., .09, .02, 0.), lag_s=.2, slip_mm=(2., 0., -2.))),
             ("no sag, quick arm, funnel off the other way", dict(sag=(0.,) * 6, lag_s=.06, slip_mm=(-2., 0., 2.))),
             ("part poses 3 steps late, slow sagging arm", dict(delay=3, lag_s=.2, sag=(0., .04, 0., .09, .02, 0.))),
             ("control: funnel let go before the clamp (DEMO_VARIANT=no_hold)", dict(variant="no_hold")),
             ("idle robot", dict(idle=True, max_steps=200)))
    for name, kw in cases:
        report = dry_run(clearance=clearance if name == "nominal" else None, verbose=args.verbose, **kw)
        keep = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in report.items() if k not in ("phases",)}
        print(name, "->", keep)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
