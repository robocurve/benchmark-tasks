"""Run the same plant, save accepted 14-D actions, and compare matched rollouts."""

import argparse
import csv
import hashlib
import json
import os
import sys
from pathlib import Path
import numpy as np
from .model import Kitchen
from .reference import Server

HORIZON = 440
ROOT = Path(__file__).resolve().parents[1]


def revision():
    sha = hashlib.sha256()
    for p in sorted(
        [
            *Path(__file__).parent.glob("*.py"),
            *Path(__file__).with_name("assets").glob("*"),
        ]
    ):
        if p.is_file():
            sha.update(p.relative_to(Path(__file__).parent).as_posix().encode() + b"\0")
            sha.update(p.read_bytes())
    return sha.hexdigest()


def run(profile, mode, output, replay=None):
    output.mkdir(parents=True, exist_ok=False)
    e = Kitchen(profile)
    ctrl = Server(e, mode) if replay is None else None
    actions = []
    rows = []
    # A second recipient motion needs time for the same terminal dwell.
    horizon = 480 if profile == "return" else HORIZON
    for tick in range(horizon):
        a = ctrl.command() if ctrl else replay[tick]
        actions.append(np.array(a, copy=True))
        e.action(a)
        rows.append(
            dict(
                step=tick + 1,
                **e.report(),
                mug_x=round(e.d.body("receiving_hand").xpos[0], 5),
                pitcher_z=round(e.d.xpos[e.pot, 2], 5),
            )
        )
    result = e.report()
    arr = np.asarray(actions, dtype="<f8")
    np.save(output / "actions.npy", arr, allow_pickle=False)
    with (output / "measurements.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0])
        w.writeheader()
        w.writerows(rows)
    result.update(
        profile=profile,
        control=mode,
        actions_sha256=hashlib.sha256(arr.tobytes()).hexdigest(),
        actor_onset_s=e.onset,
        steps=len(actions),
        engine="MuJoCo",
        source_sha256=revision(),
    )
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result, arr


def suite(output):
    output.mkdir(parents=True, exist_ok=False)
    results = {}
    ref, steady = run("steady", "adaptive", output / "steady")
    results["steady"] = ref
    cases = [
        ("same_actions_shift", "reposition", "replay", steady, 0.0),
        ("contact_probe", "steady", "contact_probe", None, 0.0),
        ("reference", "reposition", "adaptive", None, 1.0),
        ("early", "early", "adaptive", None, 1.0),
        ("late", "late", "adaptive", None, 1.0),
        ("away", "away", "adaptive", None, 1.0),
        ("return", "return", "adaptive", None, 1.0),
        ("track", "reposition", "track", None, 1.0),
        ("pause_only", "reposition", "pause_only", None, 0.5),
        ("early_release", "reposition", "early_release", None, 0.0),
        ("hold", "reposition", "hold", None, 0.5),
        ("park_closed", "reposition", "park_closed", None, 1.0),
        ("idle", "reposition", "idle", None, 0.0),
    ]
    failures = []
    if ref["score"] != 1.0:
        failures.append("steady")
    for name, profile, mode, arr, want in cases:
        r, trajectory = run(profile, mode, output / name, arr)
        results[name] = r
        if r["score"] != want:
            failures.append(name)
        if arr is not None and (
            not np.array_equal(arr, trajectory)
            or r["on_person_ml"] < 2
            or r["first_fault"][0] != "liquid_on_resident"
        ):
            failures.append("matched safety consequence")
    delayed = np.vstack((np.repeat(steady[:1], 40, axis=0), steady[:-40]))
    for profile, want in [("steady", 1.0), ("reposition", 0.0)]:
        name = "delay_" + profile
        r, _ = run(profile, "delayed_replay", output / name, delayed)
        results[name] = r
        if r["score"] != want:
            failures.append(name)
    if (
        results["delay_reposition"]["on_person_ml"] < 2
        or results["contact_probe"]["first_fault"] is None
    ):
        failures.append("diagnostic intervention")
    (output / "summary.json").write_text(
        json.dumps(dict(failures=failures, trials=results), indent=2) + "\n"
    )
    for name, r in results.items():
        print(
            f"{name:19} {r['score']:.1f}   mug {r['received_ml']:7.2f} mL   resident {r['on_person_ml']:6.2f} mL"
        )
    if failures:
        raise SystemExit("Unexpected outcome: " + ", ".join(failures))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("command", choices=["suite", "trial"])
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--profile", default="reposition")
    ap.add_argument(
        "--control",
        default="adaptive",
        choices=["adaptive", "idle", "early_release", "pause_only", "hold", "track", "park_closed"],
    )
    args = ap.parse_args()
    if args.command == "suite":
        suite(args.out)
    else:
        print(json.dumps(run(args.profile, args.control, args.out)[0], indent=2))


if __name__ == "__main__":
    main()
