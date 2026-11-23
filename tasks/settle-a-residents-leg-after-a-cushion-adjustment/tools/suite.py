"""Run the physical controls, save the joint commands and replay the scores."""

import argparse
import gzip
import hashlib
import json
import platform
import re
from pathlib import Path

import mujoco as mj
import numpy as np
import scipy

from task.run import trial
from task.scoring import score
from task.diagnostics_v1 import score as diagnostic_grade

ROOT = Path(__file__).resolve().parents[1]
CASES = {
    "harmless_bed_touch": ("bed_touch", {}, 1.0),
    "reference": ("reference", {}, 1.0),
    "slow": ("slow", {}, 1.0),
    "park_closed": ("park_closed", {}, 1.0),
    "different_push_point": ("offset", {}, 1.0),
    "shorter_safe_push": ("edge", {}, 1.0),
    "short_push": ("short", {}, 0.0),
    "idle": ("idle", {}, 0.0),
    "still_at_cushion": ("linger", {}, 0.5),
    "fixed_nominal": ("clock", {}, 1.0),
    "fixed_partial": ("clock", {"resident_y": 0.080}, 0.5),
    "fixed_heel_on_foam": ("clock", {"resident_y": 0.200}, 0.0),
    "fixed_late_lift": ("clock", {"delay": 3.5}, 0.0),
    "fixed_later_lift": ("clock", {"delay": 4.5}, 0.0),
    "feedback_later_lift": ("reference", {"delay": 4.5}, 1.0),
    "feedback_late_lift": ("reference", {"delay": 3.5}, 1.0),
    "softer_foam": ("reference", {"stiffness": 620.0}, 1.0),
    "heavier_shin": ("reference", {"leg_mass": 3.45}, 1.0),
    "lighter_shin": ("reference", {"leg_mass": 2.55}, 1.0),
}


def write(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def signature():
    paths = sorted(
        list((ROOT / "task").glob("*.py"))
        + list((ROOT / "task/assets").rglob("*"))
        + [ROOT / "tools/suite.py", ROOT / "task/task.template.json"]
    )
    return {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in paths
        if p.is_file()
    }


def run(out):
    out.mkdir(parents=True, exist_ok=False)
    write(out / "sources.json", signature())
    results = {}
    commands = {}
    for name, (mode, options, target) in CASES.items():
        rows = trial(mode, **options)
        path = out / (name + ".jsonl.gz")
        with gzip.open(path, "wt") as f:
            for row in rows:
                f.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
        with gzip.open(path, "rt") as f:
            value, reason = score([json.loads(line) for line in f])
        commands[name] = np.array([r["action"] for r in rows], dtype="<f8")
        r = rows[-1]
        sel = (
            "heel_gap_mm",
            "free_gap_mm",
            "heel_n",
            "prox_n",
            "dist_n",
            "support_span_mm",
            "knee_pad_n",
            "achilles_pad_n",
            "arm_clear_mm",
        )
        results[name] = {
            "score": value,
            "reason": reason,
            "expected": target,
            "v1_force_diagnostic": diagnostic_grade(rows),
            "mode": mode,
            "settings": options,
            "initial_heel_n": rows[0]["heel_n"],
            "final": {k: r[k] for k in sel},
            "actions_sha256": hashlib.sha256(commands[name].tobytes()).hexdigest(),
        }
        print(
            f"{name:24s} score={value:3.1f}  heel={r['heel_n']:5.2f} N  free gap={r['free_gap_mm']:6.1f} mm",
            flush=True,
        )
    pairs = [
        "fixed_partial",
        "fixed_heel_on_foam",
        "fixed_late_lift",
        "fixed_later_lift",
    ]
    matched = {
        k: bool(np.array_equal(commands["fixed_nominal"], commands[k])) for k in pairs
    }
    passed = all(i["score"] == i["expected"] for i in results.values()) and all(
        matched.values()
    )
    report = {
        "engine": "MuJoCo",
        "mujoco": mj.__version__,
        "python": platform.python_version(),
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "hz": 20,
        "actions": 280,
        "results": results,
        "command_identity_vs_fixed_nominal": matched,
        "passed": passed,
    }
    write(out / "matrix.json", report)
    if not passed:
        raise SystemExit("Inspect matrix.json and individual traces")


def replay(out):
    if json.loads((out / "sources.json").read_text()) != signature():
        raise ValueError("Source hashes differ from this recorded experiment")
    report = json.loads((out / "matrix.json").read_text())
    if set(report["results"]) != set(CASES):
        raise ValueError("Unexpected trial set")
    action_hashes = {}
    for name, (_, _, expected) in CASES.items():
        with gzip.open(out / (name + ".jsonl.gz"), "rt") as f:
            rows = [json.loads(line) for line in f]
        outcome, reason = score(rows)
        item = report["results"][name]
        commands = np.array([r["action"] for r in rows], dtype="<f8")
        if commands.shape != (280, 8) or not np.isfinite(commands).all():
            raise ValueError(f"Invalid action matrix in {name}")
        h = hashlib.sha256(commands.tobytes()).hexdigest()
        action_hashes[name] = h
        if name == "park_closed" and not np.allclose(commands[-41:, 6:], 0):
            raise ValueError("Terminal jaws were not closed")
        if name == "harmless_bed_touch" and not any(
            r["witness"]["bed_touch"] for r in rows[210:240]
        ):
            raise ValueError("Bed-touch control did not touch the cleared bed")

        if (
            (outcome, reason) != (item["score"], item["reason"])
            or outcome != expected
            or list(diagnostic_grade(rows)) != item["v1_force_diagnostic"]
            or h != item["actions_sha256"]
        ):
            raise ValueError(f"Replay mismatch in {name}")
        if (
            any(rows[-1][k] != v for k, v in item["final"].items())
            or rows[0]["heel_n"] != item["initial_heel_n"]
        ):
            raise ValueError(f"Reported observations differ in {name}")
    for k in (
        "fixed_partial",
        "fixed_heel_on_foam",
        "fixed_late_lift",
        "fixed_later_lift",
    ):
        if action_hashes[k] != action_hashes["fixed_nominal"]:
            raise ValueError(f"Commands differ for {k}")
    print(
        f"Replayed {len(CASES)} traces and verified all four matched command comparisons."
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("phase", choices=["run", "replay"])
    p.add_argument("tag")
    a = p.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}", a.tag):
        p.error("Use a short alphanumeric run tag, hyphens or underscores")
    directory = ROOT / "results" / a.tag
    (run if a.phase == "run" else replay)(directory)
