"""Offline judge for independently recorded water-trainer observations, in SI-derived units."""

from dataclasses import dataclass, fields
from math import isfinite

HZ = 240
ACTION_LIMIT = 1200
STEPS_PER_ACTION = 12
ORDER_MM = 100.0
LEVEL_TOL_MM = 10.0


@dataclass(frozen=True)
class Frame:
    tick: int
    ear_mm: float
    zero_mm: float
    sill_mm: float
    tap_deg: float
    brake_gap_mm: float
    arm_clearance_mm: float
    patient_clearance_mm: float
    volume_ml: float
    path_intact: bool


@dataclass(frozen=True)
class Result:
    score: float
    reason: str
    tick: int
    valid: bool = True


def judge(frames, *, terminal_tick, ear_start, zero_start):
    """The recorder supplies the terminal tick and surveyed reset independently of the trace."""
    if (
        type(terminal_tick) is not int
        or not 0 <= terminal_tick <= ACTION_LIMIT * STEPS_PER_ACTION
    ):
        return Result(0, "invalid terminal tick", 0, False)
    if any(
        type(v) not in (int, float) or not isfinite(v) for v in (ear_start, zero_start)
    ):
        return Result(0, "invalid reset survey", 0, False)
    history = []
    first_fault = None
    sealed = 0
    ready_open = 0
    ready_closed = 0
    for i, s in enumerate(frames):
        if (
            not isinstance(s, Frame)
            or type(s.tick) is not int
            or s.tick != i
            or i > terminal_tick
        ):
            return Result(0, "missing or out-of-order sample", i, False)
        if type(s.path_intact) is not bool or any(
            type(v) not in (float, int) or not isfinite(v)
            for f in fields(s)
            if f.name not in ("tick", "path_intact")
            for v in (getattr(s, f.name),)
        ):
            return Result(0, "invalid measurement", i, False)
        if not (-0.1 <= s.tap_deg <= 90.1) or not (-0.1 <= s.brake_gap_mm <= 8.1):
            return Result(0, "instrument out of range", i, False)
        if i == 0 and (
            abs(s.ear_mm - ear_start) > 1
            or abs(s.zero_mm - zero_start) > 1
            or s.tap_deg > 2
            or s.brake_gap_mm > 0.5
            or abs(s.volume_ml) > 0.01
        ):
            return Result(0, "wrong reset", i, False)
        if history and s.volume_ml < history[-1] - 1e-8:
            return Result(0, "invalid cumulative volume", i, False)
        history.append(s.volume_ml)
        if first_fault is not None:
            continue
        sealed = sealed + 1 if s.tap_deg <= 2 else 0
        aligned = abs(s.zero_mm - s.ear_mm) <= LEVEL_TOL_MM
        water_5s = history[-1] - history[-1 - 5 * HZ] if i >= 5 * HZ else 0.0
        if abs(s.ear_mm - ear_start) > 1 or abs((s.sill_mm - s.zero_mm) - ORDER_MM) > 1:
            first_fault = Result(0, "reference moved or fixed setting disturbed", i)
        elif s.patient_clearance_mm <= 0 or not s.path_intact:
            first_fault = Result(0, "patient contact or circuit opened", i)
        elif s.tap_deg > 10 and not aligned:
            first_fault = Result(0, "patent tap at incorrect level", i)
        elif sealed >= 5 * HZ + 1 and water_5s > 1.0 + 1e-8:
            first_fault = Result(0, "water loss with tap closed", i)
        elif water_5s > 4.0 + 1e-8:
            first_fault = Result(0, "excess water loss in five seconds", i)
        released = s.brake_gap_mm <= 0.5 and s.arm_clearance_mm >= 30
        ready_open = ready_open + 1 if aligned and released and s.tap_deg >= 88 else 0
        ready_closed = (
            ready_closed + 1 if aligned and released and s.tap_deg <= 2 else 0
        )

    end = len(history) - 1
    if end != terminal_tick:
        return Result(0, "incomplete recording", end, False)
    if first_fault is not None:
        return first_fault
    if ready_open >= 30 * HZ + 1:
        # Independent water loss in each half of the final 30-second observation.
        volumes = [
            history[end - i * 15 * HZ] - history[end - (i + 1) * 15 * HZ]
            for i in range(2)
        ]
        if all(6.0 - 1e-8 <= v <= 9.0 + 1e-8 for v in volumes):
            return Result(1, "released and verified water flow", end)
        return Result(0, "water-flow verification failed", end)
    if ready_closed >= 2 * HZ + 1 and abs(ear_start - zero_start) > LEVEL_TOL_MM:
        return Result(0.5, "corrected, secured and paused", end)
    return Result(0, "incomplete", end)


if __name__ == "__main__":
    import argparse
    import json
    from dataclasses import asdict
    from pathlib import Path

    parser = argparse.ArgumentParser(
        description="Judge newline-delimited water-trainer frames."
    )
    parser.add_argument("trace", type=Path)
    parser.add_argument("--terminal-tick", type=int, required=True)
    parser.add_argument(
        "--ear-start", type=float, required=True, help="surveyed height in mm"
    )
    parser.add_argument(
        "--zero-start", type=float, required=True, help="surveyed height in mm"
    )
    args = parser.parse_args()
    try:
        with args.trace.open() as stream:
            result = judge(
                (Frame(**json.loads(s)) for s in stream),
                terminal_tick=args.terminal_tick,
                ear_start=args.ear_start,
                zero_start=args.zero_start,
            )
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(2, f"Invalid input trace: {error}\n")
    print(json.dumps(asdict(result)))
    raise SystemExit(0 if result.valid else 2)
