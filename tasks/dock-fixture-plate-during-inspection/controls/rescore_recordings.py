"""Regrade the submitted native trajectories on the revised camera clock."""

import argparse
import gzip
import hashlib
import json
import math
import sys
from pathlib import Path

TASK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TASK))
from cover_geometry import PROMPT, TIMING
from cover_observations import measure
from cover_scoring import Trial

TAG = "dock02b5-01"
EXPECTED = {
    "wait_then_dock": 1,
    "slow_reference": 1,
    "park_closed": 1,
    "dock_while_occupied": 0,
    "early_clear_dock": 1,
    "inspection_continues_dock": 0,
    "hover_only": 0.5,
    "inspection_continues_hold": 0.5,
    "misaligned_hold": 0.5,
    "idle": 0,
    "dream": 0,
}
UNSAFE = {"dock_while_occupied", "inspection_continues_dock"}
PAIR = ("dock_while_occupied", "wait_then_dock")


def require(ok, message):
    if not ok:
        raise ValueError(message)


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def store(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n")


def verified(run, receipt, path):
    value = digest(path)
    require(
        value == receipt["hashes"].get(path.relative_to(run).as_posix()),
        f"{path} differs from its native receipt",
    )
    return value


def actions(path):
    with path.open() as f:
        a = [r for r in map(json.loads, f) if "action" in r]
    require([r["t"] for r in a] == list(range(600)), f"Incomplete actions in {path}")
    rows = [r["action"] for r in a]
    require(
        all(len(x) == 14 and all(map(math.isfinite, x)) for x in rows),
        "Invalid YAM vector",
    )
    return rows


def difference(a, b):
    return max(abs(u - v) for u, v in zip(a, b))


def same_reading(a, b):
    if isinstance(a, dict):
        return (
            isinstance(b, dict)
            and a.keys() == b.keys()
            and all(same_reading(v, b[k]) for k, v in a.items())
        )
    if isinstance(a, (list, tuple)):
        return (
            isinstance(b, (list, tuple))
            and len(a) == len(b)
            and all(same_reading(x, y) for x, y in zip(a, b))
        )
    if isinstance(a, float):
        return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-10)
    return a == b


def replay_trace(trace, mode, previous, fixture_hash):
    tr = Trial()
    event = clear = registered = None
    final_raw = None
    with gzip.open(trace, "rt") as f:
        hdr = json.loads(next(f))
        require(
            hdr["kind"] == "reset"
            and hdr["physics"] == "cpu"
            and abs(hdr["dt"] - 1 / 240) < 1e-7
            and hdr["fixture_sha256"] == fixture_hash
            and hdr["inspection_profile"] == previous["inspection_profile"],
            f"{mode}: trace header",
        )
        for tick, line in enumerate(f, 1):
            sample = json.loads(line)
            raw = sample["raw"]
            require(
                raw["tick"] == tick and tick <= 7200,
                f"{mode}: missing or reordered sample {tick}",
            )
            # On both published board videos all four lamps must stay dark under both
            # scorers, so regrading cannot change a camera observation fed to the policy.
            if mode in ("dream", "wait_then_dock"):
                require(
                    not sample["reason"], f"{mode}: original footage contains a fault"
                )
            if tick % 12:
                tr.observe({"tick": tick})
                continue
            r = measure(raw)
            tr.observe(r)
            if tr.reason and event is None:
                event = dict(
                    frame=tick // 12,
                    time_s=tick / 240,
                    fault=tr.reason,
                    limb_gap_mm=1000 * r["limb_gap"],
                    gauge_gap_mm=1000 * r["gauge_gap"],
                    lowest_corner_mm=1000 * min(c[2] for c in r["corners"]),
                    over_receiver=r["over_receiver"],
                )
            if tr.extracted and clear is None:
                clear = tick // 12
            if registered is None and any(
                d >= 0.004 and m >= -0.00030
                for d, m in zip(r["pin_depth"], r["pin_margin"])
            ):
                registered = tick // 12
            final_raw = sample
    require(tr.tick == 7200 and tr.reset_valid, f"{mode}: incomplete 600-action trace")
    require(
        final_raw["score"] == previous["score"]
        and final_raw["reason"] == previous["fault"],
        f"{mode}: old score or fault disagrees with submitted trial",
    )
    require(
        all(same_reading(r[k], v) for k, v in previous["final"].items()),
        f"{mode}: raw end state differs from submission",
    )
    expected_reason = "low_plate_at_occupied_receiver" if mode in UNSAFE else ""
    require(
        tr.score == EXPECTED[mode] and tr.reason == expected_reason,
        f"{mode}: observed {tr.score} / {tr.reason}",
    )
    return dict(
        score=tr.score,
        fault=tr.reason,
        first_fault=event,
        clear_frame=clear,
        first_registered_frame=registered,
        final_depth_mm=[1000 * d for d in r["pin_depth"]],
        final_witness_flags=tr.flags,
        frames=600,
        actions=600,
    )


def audit(project, baseline, out):
    q = read(baseline / "validation.json")
    comp = read(baseline / "docking_comparison.json")
    require(
        q == read(project / "results/qualification" / TAG / "qualification.json")
        and q["qualified"]
        and set(q["trials"]) == set(EXPECTED),
        "Different qualification/run",
    )
    old = q["source"]
    new = {name: digest(TASK / name) for name in old}
    require(
        all(digest(project / "task" / k) == v for k, v in old.items()),
        "Different original workstation source",
    )
    require(
        {k for k in old if old[k] != new[k]}
        == {"cover_scoring.py", "cover_observations.py"},
        "Dynamics, policy or recording sources changed",
    )
    require(
        all(digest(TASK / "controls" / k) == v for k, v in comp["policies"].items()),
        "Different control source",
    )
    rows, streams, hashes, media = {}, {}, {}, {}
    for mode in EXPECTED:
        run = project / "results/native" / TAG / mode
        prev = q["trials"][mode]
        rec = read(run / "receipt.json")
        inv = read(run / "invocation.json")
        require(
            rec["mode"] == inv["mode"] == mode
            and rec["tag"] == inv["tag"] == TAG
            and rec["source"] == inv["source"] == old
            and rec["argv"] == inv["argv"]
            and (
                rec.get("exit_code") == 0
                or (
                    rec.get("exit_code") is None
                    and rec.get("recovered_from_completed_inspect") is True
                )
            ),
            f"{mode}: invalid receipt",
        )
        if mode in PAIR:
            require(
                rec["control_sources"] == comp["policies"], "Control receipt mismatch"
            )
        (log,) = (run / "inspect").glob("adhoc_*.json")
        log_hash = verified(run, rec, log)
        doc = read(log)
        (sample,) = doc["samples"]
        ev = doc["eval"]
        require(
            doc["status"] == sample["status"] == "success"
            and not doc.get("error")
            and not sample.get("error")
            and doc["stats"]["total_steps"] == ev["max_steps"] == 600
            and ev["seed"] == 0
            and ev["embodiment"] == "yam_arena"
            and ev["policy"] == (mode if mode in (*PAIR, "dream") else "yam_scripted")
            and sample["instruction"] == PROMPT
            and sample["reduced"]["task_score"] == prev["score"],
            f"{mode}: different native Inspect record",
        )
        trace = run / "trace.jsonl.gz"
        trace_hash = verified(run, rec, trace)
        row = replay_trace(trace, mode, prev, old["cover.usda"])
        row.update(
            trace_sha256=trace_hash,
            inspect_sha256=log_hash,
            profile=prev["inspection_profile"],
        )
        if mode == "dream":
            require(
                prev["hosted"]["provider_requests"] > 0
                and prev["hosted"]["model_rows_used"] > 0,
                "No hosted recording",
            )
            row["original_hosted_accounting"] = prev["hosted"]
        if mode in (*PAIR, *TIMING, "hover_only"):
            (path,) = (run / "inspect").glob("actions/*/*.jsonl")
            hashes[mode] = verified(run, rec, path)
            streams[mode] = actions(path)
        for slot, owner, name in (
            ("video", "dream", "model.mp4"),
            ("demo", "wait_then_dock", "demo.mp4"),
            ("screenshot", "wait_then_dock", "setup.png"),
        ):
            if mode == owner:
                media[slot] = verified(run, rec, run / "media" / name)
        rows[mode] = row
        print(f"{mode:26s} {row['score']:.1f}  {row['fault'] or 'clear'}", flush=True)
    for mode, (base, profile) in TIMING.items():
        diff = max(difference(a, b) for a, b in zip(streams[mode], streams[base]))
        record = q["timing_action_checks"][mode]
        require(
            profile == rows[mode]["profile"]
            and diff == record["maximum_difference"]
            and diff <= 1e-5
            and hashes[mode] == record["variant_actions_sha256"]
            and hashes[base] == record["baseline_actions_sha256"],
            f"{mode}: timing intervention mismatch",
        )
    changed = [
        i
        for i, (a, b) in enumerate(zip(*(streams[m] for m in PAIR)))
        if difference(a, b) > 1e-5
    ]
    require(
        changed
        and all(hashes[m] == comp["action_sha256"][m] for m in PAIR)
        and changed[0] == comp["common_initial_actions"]
        and 599 - changed[-1] == comp["common_final_actions"],
        "Matched control comparison differs from the published actions",
    )
    output = dict(
        qualified=True,
        method="20 Hz replay of original native station trajectories",
        seed=0,
        tag=TAG,
        original_source=old,
        source=new,
        board_media_sha256=media,
        trials=rows,
        timing_action_checks=q["timing_action_checks"],
    )
    out.mkdir(parents=True, exist_ok=True)
    store(out / "validation.json", output)
    comparison = dict(
        method="20 Hz replay of original native station trajectories",
        tag=TAG,
        original_source=old,
        common_initial_actions=changed[0],
        common_final_actions=599 - changed[-1],
        source=new,
        policies=comp["policies"],
        action_sha256=comp["action_sha256"],
        trials={m: rows[m] for m in PAIR},
        matched_outcomes=True,
    )
    store(out / "docking_comparison.json", comparison)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    for arg in ("project", "baseline", "output"):
        ap.add_argument(arg, type=Path)
    args = ap.parse_args()
    audit(args.project, args.baseline, args.output)
