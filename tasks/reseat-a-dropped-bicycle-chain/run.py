"""Run the physical fixture with a dual-YAM scripted policy or inactivity control."""

from conditions import CASES, build

import argparse
import json
from pathlib import Path

import numpy as np

from fixtures import Chain
from station import HZ, MAX_STEPS, Station

FIXTURES = {"chain": Chain}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture", choices=FIXTURES)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--case", choices=CASES, default="reference")
    parser.add_argument("--tool", choices=("flat", "pick"), default="flat")
    parser.add_argument("--variant", type=int, choices=[0], default=0)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--control", action="store_true")
    mode.add_argument("--replay", type=Path)
    mode.add_argument("--detail-control", action="store_true")
    mode.add_argument("--show-fault", action="store_true")
    mode.add_argument("--crank-only", action="store_true")
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    render = None
    if args.video:
        from video import Recorder

        if args.show_fault:
            from fault_video import FaultRecorder

            render = FaultRecorder(args.out)
        else:
            render = Recorder(args.out)
    fixture = build(args.case, args.tool, args.variant)
    trial = Station(fixture, render=render)
    status = None
    try:
        trial.trace.append(trial.reading())
        if args.replay:
            commands = json.loads(args.replay.read_text())
            array = np.asarray(commands, dtype=float)
            if (
                array.ndim != 2
                or array.shape[1] != 14
                or not 1 <= len(array) <= MAX_STEPS
                or not np.isfinite(array).all()
            ):
                raise ValueError(
                    "Expected 1 to 1200 finite 14-dimensional dual-YAM commands"
                )
            trial.phase = "Replaying recorded robot commands"
            for row in array:
                trial.action(row.reshape(2, 7))
        else:
            trial.wait(0.45)
            if args.control:
                fixture.control(trial)
            elif args.show_fault:
                from policy import reseat

                reseat(trial, test_before=True)
            elif args.crank_only:
                from policy import reseat

                reseat(trial, guide=False)
            elif args.detail_control:
                fixture.detail_control(trial)
            else:
                fixture.demo(trial)
    except Exception as error:
        status = str(error)
        raise
    finally:
        if render:
            render.close()
        result = {
            **trial.reading(),
            "title": fixture.title,
            "variant": args.variant,
            "case": args.case,
            "tool": args.tool,
            "mode": (
                "inactivity control"
                if args.control
                else (
                    "replay"
                    if args.replay
                    else (
                        "crank only"
                        if args.crank_only
                        else "short turn" if args.detail_control else "scripted"
                    )
                )
            ),
            "simulator": "MuJoCo dual YAM",
            "control_hz": HZ,
            "total_crank_turns": round(trial.joint("front_angle") / (2 * np.pi), 5),
            "total_rear_turns": round(trial.joint("rear_angle") / (2 * np.pi), 5),
            "solver_warnings": int(np.sum(trial.data.warning.number)),
            "error": status,
        }
        (args.out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        (args.out / "trace.jsonl").write_text(
            "".join(json.dumps(r) + "\n" for r in trial.trace)
        )
        (args.out / "commands.json").write_text(json.dumps(trial.commands) + "\n")
        print(json.dumps(result))


if __name__ == "__main__":
    main()
