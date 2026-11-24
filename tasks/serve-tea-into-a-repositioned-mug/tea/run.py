"""Run the same plant, save accepted 14-D actions, and compare matched rollouts."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from .model import Kitchen
from .reference import Server
from .challenge import Challenge, MODES

HORIZON = 520
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
    challenge = Challenge(e, mode) if mode in MODES and replay is None else None
    actions, executed, rows = [], [], []
    # Additional contact controls begin after compliant set-down and need a longer tape.
    horizon = (
        640
        if mode in {"subtrip_contact", "incidental_counter_touch"}
        else (560 if profile == "return" else HORIZON)
    )
    if replay is not None:
        horizon = len(replay)
    onset_sample = endpoint_sample = None
    max_reach = 0.0
    for tick in range(horizon):
        a = ctrl.command() if ctrl else replay[tick]
        if challenge:
            a = challenge.command(a)
        actions.append(np.array(a, copy=True))
        e.action(a)
        executed.append(e.last_accepted.copy())
        mug_pos = e.d.body("receiving_hand").xpos.copy()
        tcp = e.d.site("left_tcp").xpos.copy()
        reach = float(np.linalg.norm(tcp - e.d.body("left_base").xpos))
        max_reach = max(max_reach, reach)
        r = e.report()
        sample = dict(
            t_s=r["t_s"],
            mug_xyz_mm=(mug_pos * 1000).round(2).tolist(),
            left_tcp_xyz_mm=(tcp * 1000).round(2).tolist(),
            base_to_tcp_mm=round(1000 * reach, 2),
            mug_water_g=round(r["mug_water_g"], 3),
        )
        if e.onset is not None and onset_sample is None:
            onset_sample = sample
        if e.motion_completed and endpoint_sample is None:
            endpoint_sample = sample
        rows.append(
            dict(
                step=tick + 1,
                **r,
                mug_x=round(mug_pos[0], 5),
                mug_y=round(mug_pos[1], 5),
                pitcher_z=round(e.d.xpos[e.pot, 2], 5),
                left_tcp_x=round(tcp[0], 5),
                left_tcp_y=round(tcp[1], 5),
                left_tcp_z=round(tcp[2], 5),
            )
        )
    result = e.report()
    result.update(
        stage_onset_sample=onset_sample,
        stage_end_sample=endpoint_sample,
        max_base_to_tcp_mm=round(1000 * max_reach, 2),
    )
    if challenge is not None:
        result["challenge_contact_observed"] = challenge.contact_observed
    arr = np.asarray(actions, dtype="<f8")
    applied = np.asarray(executed, dtype="<f8")
    np.save(output / "actions.npy", arr, allow_pickle=False)
    np.save(output / "executed_actions.npy", applied, allow_pickle=False)
    with (output / "measurements.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0])
        w.writeheader()
        w.writerows(rows)
    result.update(
        profile=profile,
        control=mode,
        actions_sha256=hashlib.sha256(arr.tobytes()).hexdigest(),
        executed_actions_sha256=hashlib.sha256(applied.tobytes()).hexdigest(),
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
        ("slow_reference", "reposition", "slow_reference", None, 1.0),
        ("insufficient", "reposition", "insufficient", None, 0.0),
        ("overfill", "reposition", "overfill", None, 0.0),
        ("drop", "reposition", "drop", None, 0.0),
        ("receiver_obstruction", "reposition", "receiver_obstruction", None, 0.0),
        (
            "incidental_counter_touch",
            "reposition",
            "incidental_counter_touch",
            None,
            1.0,
        ),
        ("subtrip_contact", "reposition", "subtrip_contact", None, 1.0),
        ("spill_then_recover", "reposition", "spill_then_recover", None, 0.0),
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
            or not np.array_equal(
                np.load(output / "steady" / "executed_actions.npy"),
                np.load(output / name / "executed_actions.npy"),
            )
            or r["on_person_ml"] <= 2
            or not r["first_fault"]
            or r["first_fault"][0] != "liquid_on_resident"
        ):
            failures.append("matched spill consequence")
        if (
            name in {"incidental_counter_touch", "subtrip_contact"}
            and not r["challenge_contact_observed"]
        ):
            failures.append(name + " failed to touch")
        if name == "spill_then_recover" and (
            r["mug_water_g"] < 100
            or not r["released_on_counter"]
            or r["resident_water_g"] <= 2
        ):
            failures.append("no recovery after spill")
        if name == "drop" and (
            not r["first_fault"] or r["first_fault"][0] != "dropped_pitcher"
        ):
            failures.append("no measured free drop")
        if name == "receiver_obstruction" and not (
            r["displacement_flag"]
            and r["actor_onset_s"]
            and r["actor_onset_s"] <= r["flag_trip_s"] <= r["actor_onset_s"] + 1.0
        ):
            failures.append("no displacement during travel")
        if name == "insufficient" and not 0 < r["mug_water_g"] < 30:
            failures.append("insufficient fill control")
        if name == "overfill" and r["mug_water_g"] <= 130:
            failures.append("overfill control")
    delayed = np.vstack((np.repeat(steady[:1], 40, axis=0), steady[:-40]))
    for profile, want in [("steady", 1.0), ("reposition", 0.0)]:
        name = "delay_" + profile
        r, _ = run(profile, "delayed_replay", output / name, delayed)
        results[name] = r
        if r["score"] != want:
            failures.append(name)
    if (
        results["delay_reposition"]["on_person_ml"] < 2
        or not results["contact_probe"]["displacement_flag"]
        or results["delay_steady"]["executed_actions_sha256"]
        != results["delay_reposition"]["executed_actions_sha256"]
    ):
        failures.append("diagnostic intervention")
    (output / "summary.json").write_text(
        json.dumps(dict(failures=failures, trials=results), indent=2) + "\n"
    )
    for name, r in results.items():
        print(
            f"{name:19} {r['score']:.1f}   mug {r['mug_water_g']:7.2f} g   surrogate {r['resident_water_g']:6.2f} g"
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
        choices=[
            "adaptive",
            "idle",
            "early_release",
            "pause_only",
            "hold",
            "track",
            "park_closed",
            "slow_reference",
            "insufficient",
            "overfill",
            "drop",
            "receiver_obstruction",
            "contact_probe",
            *sorted(MODES),
        ],
    )
    args = ap.parse_args()
    if args.command == "suite":
        suite(args.out)
    else:
        print(json.dumps(run(args.profile, args.control, args.out)[0], indent=2))


if __name__ == "__main__":
    main()
