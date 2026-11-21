"""Rerun controlled trials with identical budgets and compare to retained evidence."""

import argparse
import json
import subprocess
import sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

from conditions import CASES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=1)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    args.out.mkdir(exist_ok=True, parents=True)
    root = Path(__file__).resolve().parent
    path = root / "evidence" / "sensitivity.json"
    stored = json.loads(path.read_text()) if path.exists() else []
    prior = {(r["case"], r["tool"], r["mode"]): r for r in stored}
    matrix = [(name, tool, "repair") for name in CASES for tool in ("flat", "pick")]
    for case in ("deep_near_light", "sleeve_10"):
        matrix.extend([(case, "pick", "idle"), (case, "pick", "crank-only")])

    def run(entry):
        case, tool, mode = entry
        out = args.out / f"{case}-{tool}-{mode}.json"
        subprocess.run(
            [
                sys.executable,
                str(root / "trials.py"),
                case,
                "--tool",
                tool,
                "--mode",
                mode,
                "--out",
                str(out),
            ],
            check=True,
        )
        return json.loads(out.read_text())

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        recorded = list(pool.map(run, matrix))
    (args.out / "summary.json").write_text(json.dumps(recorded, indent=2) + "\n")
    unmatched = []
    for row in recorded:
        key = row["case"], row["tool"], row["mode"]
        if key in prior:
            old = prior[key]
            if (
                row["score"] != old["score"]
                or row["command_sha256"] != old["command_sha256"]
                or abs(row["crank_turns"] - old["crank_turns"]) > 1e-4
                or abs(row["driven_turns"] - old["driven_turns"]) > 1e-4
            ):
                unmatched.append(key)
        valid_start = row["starting_front_pockets"] == 0 and row["starting_rear_pockets"] >= 3
        unexpected_control_score = row["mode"] != "repair" and row["score"] != 0
        if not valid_start or unexpected_control_score or row["error"] or row["solver_warnings"]:
            unmatched.append(key)
    if stored and set(prior) != set(matrix):
        raise SystemExit("Recorded matrix and reference matrix differ.")
    if unmatched:
        raise SystemExit(
            f"Reproduction differs in {unmatched}. See {args.out} for all runs."
        )
    if stored:
        print(f"All {len(recorded)} runs matched the stored results.")


if __name__ == "__main__":
    main()
