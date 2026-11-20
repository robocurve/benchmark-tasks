#!/usr/bin/env python3
"""Check (b) of the inhaler task, no arm: the dime in the mouthpiece's tube of true bore (tube_task.py).

    scripts/rta simpy <id> mech/tasks/inhaler_coin/tube_trial.py /workspace/robocurve/outputs/<id>

Episode 1: 3 s with the mouthpiece level (does the coin stay put, how much does it jitter); then the mouthpiece tipped
down at 2 deg/s to 85 deg (the angle at which the coin starts to slide and the one at which it leaves; does its middle
stay inside the bore on the way out); 3 s more with the coin on the bench.
Episode 2: tipped to 12 deg under the angle at which it slid in episode 1, held 1 s, then shaken there: +-10 deg at
3 Hz for 4 s (does it leave sooner when shaken).
Episode 3: tipped to 85 deg in 0.5 s, as a hand turns it, then 3 s: how it leaves and how it lies on the bench.
Writes <out dir>/tube.json and the first pictures.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

MOVED = 0.001  # m along the bore from where it rested: the coin slides
SPEED_LIMIT = 3.0  # m/s: faster is a blow-up (a dime falling 10 cm reaches 1.4 m/s)


def _hold():
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    return Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))


def _save(out: Path, name: str, frame: np.ndarray) -> None:
    import cv2
    import demo_kit as kit

    cv2.imwrite(str(out / f"{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out / f"{name}_policy.png"), cv2.cvtColor(kit.policy_picture(frame), cv2.COLOR_RGB2BGR))


class Run:
    """One episode: the joint's rest moved per control step, every step logged."""

    def __init__(self, embodiment, label: str) -> None:
        self.e, self.label = embodiment, label
        self.m = embodiment.task.module
        self.mk = self.m.mk
        self.p = self.m.P
        self.hz = float(embodiment.info.control_hz)
        self.rows: list[dict] = []
        self.flags: list[str] = []

    def step(self, rest_deg: float) -> dict:
        mk, P = self.mk, self.p
        mk.set_spring("tilt", rest=math.radians(rest_deg))
        self.e.step(_hold())
        c = self.m.coin_in_inhaler()
        v = float(np.linalg.norm(mk.velocity("coin")[0]))
        axis = mk.pose("coin")[1][:, 2]
        row = {"t": round(mk.now(), 3), "rest": round(rest_deg, 2), "tilt": round(math.degrees(mk.joint("tilt")), 2), "coin_mm": (1000 * c).round(2).tolist(),
               "speed": round(v, 4), "coin_tilt": round(math.degrees(math.acos(min(1.0, abs(float(axis[2]))))), 1),
               "force": round(mk.contact_force("coin", "inhaler"), 4), "z_mm": round(1000 * (mk.pose("coin")[0][2] - self.m.TOP), 2)}  # fmt: skip
        if BORE_BACK_X < c[0] < P.MOUTH_X - 0.001 and (c[1] / P.BORE_A) ** 2 + (c[2] / P.BORE_B) ** 2 > 1.0:
            self.flags.append(f"{row['t']} s: the coin's middle outside the bore at {row['coin_mm']} mm")
        if v > SPEED_LIMIT:
            self.flags.append(f"{row['t']} s: the coin at {v:.2f} m/s")
        self.rows.append(row)
        return row

    def ramp(self, start: float, end: float, seconds: float) -> None:
        n = max(1, int(round(seconds * self.hz)))
        for i in range(1, n + 1):
            self.step(start + (end - start) * i / n)

    def hold(self, rest_deg: float, seconds: float) -> None:
        for _ in range(int(round(seconds * self.hz))):
            self.step(rest_deg)


BORE_BACK_X = 0.016  # m: past the barrel's side (the bore's back), where the in-bore check starts


def _rest_stats(rows: list[dict], t0: float, t1: float) -> dict:
    sel = [r for r in rows if t0 <= r["t"] <= t1]
    if not sel:
        return {}
    pts = np.array([r["coin_mm"] for r in sel])
    speeds = np.array([r["speed"] for r in sel]) * 1000.0
    return {"from_s": t0, "to_s": t1, "coin_mm_first": pts[0].round(2).tolist(), "drift_mm": round(float(np.linalg.norm(pts - pts[0], axis=1).max()), 3),
            "speed_max_mm_s": round(float(speeds.max()), 2), "speed_rms_mm_s": round(float(np.sqrt((speeds**2).mean())), 2),
            "coin_tilt_deg": sel[-1]["coin_tilt"], "force_N_mean": round(float(np.mean([r["force"] for r in sel])), 4)}  # fmt: skip


def _slide_and_exit(rows: list[dict], P) -> dict:
    x0 = rows[0]["coin_mm"][0]
    slide = next((r for r in rows if abs(r["coin_mm"][0] - x0) > 1000 * MOVED), None)
    out = next((r for r in rows if r["coin_mm"][0] > 1000 * (P.MOUTH_X + P.LODGED_R) + 1.0), None)
    return {"slid_at": None if slide is None else {"t": slide["t"], "tilt_deg": slide["tilt"]},
            "left_at": None if out is None else {"t": out["t"], "tilt_deg": out["tilt"]}}


def episode_1(embodiment, out: Path) -> dict:
    from inspect_robots import Scene

    obs = embodiment.reset(Scene(id="tube1", instruction="kit check"), seed=0)
    for camera, name in (("top_cam", "tube_start_top"), ("left_cam", "tube_start_left_wrist"), ("right_cam", "tube_start_right_wrist")):
        _save(out, name, np.asarray(obs.images[camera]))
    run = Run(embodiment, "1")
    run.hold(0.0, 3.0)
    rest = _rest_stats(run.rows, 0.25, 3.0)
    run.ramp(0.0, -85.0, 42.5)
    run.hold(-85.0, 3.0)
    found = {"rest_level": rest, **_slide_and_exit(run.rows, run.p), "on_bench": _rest_stats(run.rows, run.rows[-1]["t"] - 1.0, run.rows[-1]["t"]),
             "flags": run.flags, "rows": run.rows}  # fmt: skip
    return found


def episode_2(embodiment, slid_deg: float | None) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="tube2", instruction="kit check"), seed=0)
    run = Run(embodiment, "2")
    centre = -(abs(slid_deg) - 12.0) if slid_deg is not None else -15.0
    run.hold(0.0, 0.5)
    run.ramp(0.0, centre, 1.0)
    run.hold(centre, 1.0)
    held = _slide_and_exit(run.rows, run.p)
    t0 = run.rows[-1]["t"]
    for i in range(int(round(4.0 * run.hz))):
        run.step(centre + 10.0 * math.sin(2.0 * math.pi * 3.0 * (i + 1) / run.hz))
    run.hold(centre, 1.0)
    return {"centre_deg": centre, "before_shaking": held, "shaking_from_s": t0, **_slide_and_exit(run.rows, run.p), "flags": run.flags, "rows": run.rows}


def episode_3(embodiment) -> dict:
    from inspect_robots import Scene

    embodiment.reset(Scene(id="tube3", instruction="kit check"), seed=0)
    run = Run(embodiment, "3")
    run.hold(0.0, 0.5)
    run.ramp(0.0, -85.0, 0.5)
    run.hold(-85.0, 3.0)
    return {**_slide_and_exit(run.rows, run.p), "speed_max_m_s": round(max(r["speed"] for r in run.rows), 3),
            "on_bench": _rest_stats(run.rows, run.rows[-1]["t"] - 1.0, run.rows[-1]["t"]), "flags": run.flags, "rows": run.rows}  # fmt: skip


def main() -> int:
    import demo_kit as kit

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "tube_task.py")
    results = {}
    try:
        results["episode_1"] = episode_1(embodiment, out)
        slid = results["episode_1"]["slid_at"]
        results["episode_2"] = episode_2(embodiment, None if slid is None else slid["tilt_deg"])
        results["episode_3"] = episode_3(embodiment)
    finally:
        (out / "tube.json").write_text(json.dumps(results, indent=1, default=str))
    summary = {k: {kk: vv for kk, vv in v.items() if kk != "rows"} for k, v in results.items()}
    print(f"TUBE {json.dumps(summary, default=str)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
