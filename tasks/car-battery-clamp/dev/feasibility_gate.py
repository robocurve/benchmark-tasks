"""Feasibility gate: can the arm push the nut past 5 N m now that the nut's torque comes back through the wrench?

Run it from the task folder on a machine with the simulator (the same stack as README.md's "Run a trial"):

    python dev/feasibility_gate.py                 # seed 0
    python dev/feasibility_gate.py --seed 3 --log-dir ~/yam-eval/car-battery-clamp-gate-s3

It runs the scripted policy once with CLAMP_GATE=1: the clamp starts seated and pinched with the nut already at
CLAMP_GATE_T0 N m (4.5), the right hand stays clear, and the left hand takes the wrench, fits it, takes the handle at
the turning grip and makes CLAMP_GATE_STROKES strokes (2, about 1 N m each), then puts the wrench back. The task file
logs every control step; this script reads that log and prints, for each time the box end was on the nut, the nut's
turn against the wrench's swing and how far the box end's spring gave; the left arm's joint tracking and torques; and
how fast the nut turned (a spinning nut is a solver fault, not a real turn).

PASS needs all of: the peak torque passes 5.0 N m; the box end went on every time; on every stroke the nut followed
the wrench (the box end's spring gave under BOX_GIVE_MAX degrees from where it caught); the nut never turned faster
than SPIN_MAX rad/s. Exit codes: 0 PASS, 1 FAIL (the torque was not reached, or the nut did not follow the wrench or
spun: the physics is wrong), 2 no log, 3 not a valid torque test (the start torque was not set, or the box end did not
go on every time: an aiming problem).

Every CLAMP_* setting of the task file can be passed through the environment to try other strokes, e.g.
CLAMP_GRIP_PSI_DEG=-40 or CLAMP_GRIP_TURN=0.14. Nothing here changes the torque the nut needs.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent  # the task folder
INSTRUCTION = "Reconnect the positive battery cable to the battery."
LIMITS = (27.0, 27.0, 27.0, 7.0, 7.0, 7.0)  # N m: the YAM's joint effort limits (shoulder DM4340, wrist DM4310)
BOX_GIVE_MAX = 3.0  # degrees: the box end's spring gives 1.3 degrees at 7 N m; more means the nut slipped in it
SPIN_MAX = 5.0  # rad/s: the strokes turn the nut at under 1 rad/s
DT = 0.05  # s: one control step at 20 Hz
DEG = 3.141592653589793 / 180.0
NUT_I = 2e-4  # kg m^2: the nut's padded inertia about the bolt (the task file's NUT_I_AXIS)


def run(seed: int, log_dir: Path, max_steps: int) -> Path:
    log_dir.mkdir(parents=True, exist_ok=True)
    gate_log = log_dir / "gate.log"
    env = dict(os.environ, CLAMP_GATE="1", CLAMP_GATE_LOG=str(gate_log))
    cmd = ["inspect-robots", "run", "--embodiment", "yam_arena", "-E", "task=car_battery_clamp.py",
           "-E", "object_jitter=0", "-E", "control_hz=20", "-E", "cam_width=320", "-E", "cam_height=176",
           "--instruction", INSTRUCTION, "--scorer", "task_score", "--max-steps", str(max_steps), "--store-frames",
           "--policy", "yam_scripted", "--seed", str(seed), "--log-dir", str(log_dir)]
    print("running:", " ".join(cmd), flush=True)
    # answer the end-of-run "did the robot succeed?" prompt, so it never waits for a keyboard
    subprocess.run(cmd, cwd=HERE, env=env, check=False, input="n\n\n", text=True)
    return gate_log


def parse(gate_log: Path) -> list[dict]:
    rows = []
    for line in gate_log.read_text().splitlines():
        parts = line.split()
        row = {"step": int(parts[0])}
        for kv in parts[1:]:
            k, v = kv.split("=", 1)
            row[k] = [float(x) for x in v.split(",")] if "," in v else float(v)
        rows.append(row)
    return rows


def _strokes(rows: list[dict]) -> list[list[dict]]:
    """The stretches of rows with the box end on the nut."""
    out, cur = [], None
    for r in rows:
        if r["caught"]:
            cur = [] if cur is None else cur
            cur.append(r)
        elif cur is not None:
            out.append(cur)
            cur = None
    if cur is not None:
        out.append(cur)
    return out


def report(rows: list[dict]) -> int:
    """Print the report; return the exit code (see the module's docstring)."""
    t0 = float(os.environ.get("CLAMP_GATE_T0", "4.5"))
    n_strokes = int(os.environ.get("CLAMP_GATE_STROKES", "2"))
    peak = max(r["peak"] for r in rows)
    print(f"\nsteps logged: {len(rows)}")
    print(f"torque at the start {rows[0]['torque']:.2f} N m, at the end {rows[-1]['torque']:.2f} N m, "
          f"PEAK {peak:.2f} N m")
    strokes = _strokes(rows)
    print(f"box end on the nut {len(strokes)} time(s); ring offset at each catch: "
          + ", ".join(f"{s[0]['ring_r']:.2f} mm" for s in strokes))
    worst_give = 0.0
    for i, rs in enumerate(strokes, 1):
        nut = rs[-1]["turn"] - rs[0]["turn"]
        swing = rs[0]["wrench"] - min(r["wrench"] for r in rs)
        give = max(abs(r["box"] - rs[0]["box"]) for r in rs)
        worst_give = max(worst_give, give)
        tmax = max(r["torque"] for r in rs)
        tilt = max(r.get("mis", 0.0) for r in rs)
        print(f"  on {i}: nut turned {nut:6.1f} deg, wrench swung {swing:6.1f} deg down, box end gave up to "
              f"{give:4.1f} deg and tilted up to {tilt:4.1f} deg, torque up to {tmax:.2f} N m")
    caught = [r for s in strokes for r in s] or rows
    q_err = [max(abs(r["q_err"][j]) for r in caught) for j in range(6)]
    effort = [max(abs(r["effort"][j]) for r in caught) for j in range(6)]
    print("left arm, box end on:  max |joint target - joint| (rad): " + " ".join(f"{v:.3f}" for v in q_err))
    print("                       max |joint torque| (N m):         " + " ".join(f"{v:.2f}" for v in effort)
          + "   limits " + " ".join(f"{v:.0f}" for v in LIMITS))
    # the nut's turning speed from its turn between control steps (the task file caps its speed at 20 rad/s, under
    # the 63 rad/s at which a step's turn would alias past half a turn), and the solver's reported speed for reference
    spin = max((abs(b["turn"] - a["turn"]) * DEG / DT for a, b in zip(rows, rows[1:])), default=0.0)
    w_rep = max(abs(r.get("nut_w", 0.0)) for r in rows)
    w_off = max(r.get("nut_w_off", 0.0) for r in rows)
    acc = max((abs(b["nut_w"] - a["nut_w"]) / DT for s in strokes for a, b in zip(s, s[1:]) if "nut_w" in a),
              default=0.0)
    print(f"nut's turning speed on the bolt, from its turn: up to {spin:.2f} rad/s (solver's reported speed: up to "
          f"{w_rep:.2f} on the bolt, {w_off:.2f} across it); with the box end on, its reported angular acceleration "
          f"up to {acc:.1f} rad/s^2, so its padded inertia ({NUT_I:.0e} kg m^2) takes at most {NUT_I * acc:.3f} N m")
    turns = [s[-1]["turn"] - s[0]["turn"] for s in strokes]
    if turns:
        per = sum(turns) / len(turns)
        projected = 1.0 + (6 * per - 30.0) / 30.0  # the demo: six strokes, the bore closing after the first 30 degrees
        print(f"nut turned {per:.1f} deg per stroke on average (30 planned): six such strokes end the demo at about "
              f"{projected:.2f} N m" + ("  <- under 5.5 N m: tune the strokes before recording" if projected < 5.5
                                        else ""))
    if any(s[0]["ring_r"] > 1.5 for s in strokes):
        print("note: some catches were near the 2 mm limit (the aim)")
    if abs(rows[0]["torque"] - t0) > 0.05:
        print(f"\nGATE INVALID: the nut started at {rows[0]['torque']:.2f} N m, not {t0:.2f}: the gate start was not set")
        return 3
    if len(strokes) < n_strokes:
        print(f"\nGATE INCONCLUSIVE (aiming): the box end went on {len(strokes)} of {n_strokes} times. Fix the aim "
              "(camera estimate, sag) or the slide-off before reading the torque.")
        return 3
    faults = []
    if spin > SPIN_MAX:
        faults.append(f"the nut spun at {spin:.0f} rad/s (over {SPIN_MAX:.0f})")
    if worst_give > BOX_GIVE_MAX:
        faults.append(f"the nut slipped {worst_give:.1f} deg in the box end (over {BOX_GIVE_MAX:.0f})")
    if faults:
        print("\nGATE FAIL (physics): " + "; ".join(faults) + ". The torque reading is not trustworthy.")
        return 1
    ok = peak > 5.0
    print(f"\nGATE {'PASS' if ok else 'FAIL'}: peak {peak:.2f} N m {'>' if ok else '<='} 5.0 N m"
          + ("; reached 6 N m too" if peak >= 6.0 else ""))
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--log-dir", default="~/yam-eval/car-battery-clamp-gate")
    ap.add_argument("--max-steps", type=int, default=1100)
    ap.add_argument("--parse-only", action="store_true", help="read an existing gate.log in --log-dir; do not run")
    a = ap.parse_args()
    log_dir = Path(os.path.expanduser(a.log_dir))
    gate_log = log_dir / "gate.log" if a.parse_only else run(a.seed, log_dir, a.max_steps)
    if not gate_log.exists() or not gate_log.read_text().strip():
        print(f"no gate log at {gate_log}: the run failed before the first step (see the output above)")
        return 2
    return report(parse(gate_log))


if __name__ == "__main__":
    sys.exit(main())
