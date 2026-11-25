"""Run threshold, duration and evidence-integrity regressions against the shipped judge.

These are generated instrument readings. For mechanical trajectories see the CPU
PhysX trial reports. The command prints machine-readable results, exiting nonzero
on any unexpected score, validity, fault reason or online/offline discrepancy.
"""
import json
import sys
from dataclasses import replace
from pathlib import Path

# Run from the revision root or directly from inside the submitted task.
for project in (Path.cwd(), *Path(__file__).resolve().parents):
    if (project / "checks/scorer.py").is_file() and (project / "evd/instrument.py").is_file():
        sys.path.insert(0, str(project))
        break

from checks.scorer import Frame, HZ, judge
from evd.instrument import Monitor


def samples(end=7201, *, water=(7.5, 7.5), paused=False, changes=None, modify=None):
    """Start at the wrong height with the tap closed. Tick one begins eligibility."""
    first = Frame(0, 210., 90., 190., 0., 0., 50., 50., 0., True)
    yield first
    basis = Frame(1, 210., 210., 310., 0. if paused else 90., 0., 50., 50., 0., True)
    basis = replace(basis, **(changes or {}))
    # The final window starts at tick 1. Exactly 3,600 intervals make each half.
    for i in range(1, end + 1):
        elapsed = i - 1
        v = water[0] * min(elapsed, 15 * HZ) / (15 * HZ)
        v += water[1] * max(0, elapsed - 15 * HZ) / (15 * HZ)
        frame = replace(basis, tick=i, volume_ml=0. if paused else v)
        yield modify(frame) if modify is not None else frame


def at(tick, **changes):
    return lambda f: replace(f, **changes) if f.tick == tick else f


def run():
    outcomes = []

    def case(name, expected, *, valid=True, reason=None, missing=None, terminal=None, **kw):
        end = kw.pop("end", 7201)
        frames = [f for f in samples(end, **kw) if f.tick != missing]
        result = judge(frames, terminal_tick=end if terminal is None else terminal,
                       ear_start=210., zero_start=90.)
        assert result.valid == valid and result.score == expected, (name, result, expected)
        if reason is not None:
            assert result.reason == reason, (name, result, reason)
        if valid:
            monitor = Monitor("raised")
            for frame in frames:
                monitor.observe(frame)
            assert monitor.score == result.score, (name, monitor.score, result)
        outcomes.append({"case": name, "score": result.score, "valid_recording": result.valid,
                         "reason": result.reason, "tick": result.tick,
                         "online_observer_agrees": True if valid else None})

    for which in (0, 1):
        for ml, score in ((5.999, 0), (6., 1), (6.001, 1),
                          (8.999, 1), (9., 1), (9.001, 0)):
            water = [7.5, 7.5]
            water[which] = ml
            case(f"water_half_{which + 1}_{ml:g}_ml", score, water=water)
    for field, values in (
        ("arm_clearance_mm", ((29.999, 0), (30., 1), (30.001, 1))),
        ("brake_gap_mm", ((0.499, 1), (0.5, 1), (0.501, 0))),
        ("tap_deg", ((87.999, 0), (88., 1), (88.001, 1))),
    ):
        for value, score in values:
            case(f"{field}_{value:g}", score, changes={field: value})
    for delta, score in ((-10.001, 0), (-10., 1), (-9.999, 1), (9.999, 1), (10., 1), (10.001, 0)):
        case(f"level_error_{delta:g}_mm", score,
             changes={"zero_mm": 210. + delta, "sill_mm": 310. + delta})

    for end, score in ((7200, 0), (7201, 1), (7202, 1)):
        case(f"open_continuity_{end - 1}_intervals", score, end=end)
    for end, score in ((480, 0), (481, 0.5), (482, 0.5)):
        case(f"closed_continuity_{end - 1}_intervals", score, paused=True, end=end)
    for value, score in ((1.999, 0.5), (2., 0.5), (2.001, 0)):
        case(f"closed_angle_{value:g}", score, paused=True, end=481, changes={"tap_deg": value})
    for leak, score in ((0.999, 0.5), (1., 0.5), (1.001, 0)):
        case(f"closed_loss_{leak:g}_ml_5s", score, end=1201, paused=True,
             modify=lambda f, leak=leak: replace(f, volume_ml=leak * min(1200, f.tick) / 1200))
    case("patency_at_10_degrees_before_correction", 1., end=7202,
         modify=at(1, tap_deg=10., zero_mm=90., sill_mm=190.))
    case("patency_just_over_10_degrees_before_correction", 0., end=7202,
         modify=at(1, tap_deg=10.001, zero_mm=90., sill_mm=190.),
         reason="patent tap at incorrect level")
    case("ear_witness_moved_and_restored", 0., modify=at(500, ear_mm=211.001),
         reason="reference moved or fixed setting disturbed")
    case("correct_pointer_wrong_sill", 0., modify=at(6000, sill_mm=311.001),
         reason="reference moved or fixed setting disturbed")
    case("single_tick_regrasp_late", 0., modify=at(6000, arm_clearance_mm=29.9))
    case("single_tick_regrasp_early_then_complete", 1., end=7203,
         modify=at(1, arm_clearance_mm=29.9))
    case("disturbed_hose_then_corrected", 0., modify=at(12, path_intact=False),
         reason="patient contact or circuit opened")
    case("receiver_history_decreases", 0., valid=False, modify=at(1000, volume_ml=-1.),
         reason="invalid cumulative volume")
    case("missing_single_interior_sample", 0., valid=False, missing=138)
    case("missing_final_sample", 0., valid=False, terminal=7202)
    case("extra_after_terminal", 0., valid=False, terminal=7200)
    case("missing_measurement", 0., valid=False, modify=at(17, zero_mm=float("nan")))
    case("nonnumeric_circuit_status", 0., valid=False, modify=at(27, path_intact=1))
    case("over_action_budget", 0., valid=False, terminal=14401)
    print(json.dumps({"method": "Generated readings fed to the submitted offline judge and, for valid recordings, the online observer",
                      "count": len(outcomes), "cases": outcomes}, indent=2))


if __name__ == "__main__":
    run()
