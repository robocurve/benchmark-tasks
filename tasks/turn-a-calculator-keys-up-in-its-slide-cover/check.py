#!/usr/bin/env python3
"""The calculator's slide cover in the simulator under known loads (mechkit.push); the arms stay at READY:

    scripts/rta simpy <id> mech/tasks/calc_cover/check.py /workspace/robocurve/outputs/<id>

The cover is held where it lies by a weld made for this check ("cover_held"); the loads work on the calculator along
the rails, its weight carried for it. F_r = RAIL_FORCE (1.5 N), F_d = DETENT_FORCE (6 N).

Trial A, from a reset (keys down, the cover home on the front):
 1. held by the click: pulled out with 0.8 F_d for 1 s: no further than 1.5 x the spring's stretch, still home;
 2. leaves home: 1.2 F_d: past the click within 1 s;
 3. held on the rails: 1.4 F_r to 40 mm, eased to 0.6 F_r: from 0.3 s on it moves no more than SETTLE in 1 s;
 4. slides: 1.4 F_r for 0.5 s (stopped 20 mm before the rails' end): at least 10 mm;
 5. comes free: 1.4 F_r on: let go past the rails' end, the cover off the front;
 6. taken on the back within the play: the calculator turned keys up and put IN_RAILS into the cover's back rails,
    0.2 mm across, 0.3 mm up and turned 0.15 degree about its ON-key end's middle (no shape in another; the rule
    reads the offsets at that leading end: 0.2 mm and 0.3 mm; run w12c4 was made when the rule read them at the cover's
    origin, 69.5 mm past that end, and recorded 0.38 mm across),
    still: taken onto the rails within 0.5 s, the correction the take makes recorded and within the play;
 7. the click: pushed in as a hand pushes, at FAST and from SLOW_FROM at SLOW (the push held at speed, up to
    1.4 F_r; SLOW moves 1 mm a control step, less than the click's 1.5 mm, which the rails' rule reads once a step):
    stopped by the click, between CLICK_IN and DETENT_FROM; then 1.4 F_r for 0.5 s: still there; then 1.2 F_d: home;
    no load and the cover let go for 1 s: still home within HOME_WITHIN, and the score is 1.0.
Trial B, from a reset, taken off as in 2-5 (no readings): offers NEAR out of the open end, 2.5 mm across and 2 mm up,
each pushed in along the rails with 1.4 F_r for 1 s (pulled out again if taken): every correction a take makes stays
within the play (reported: taken or not, where the calculator stands). The physical entry by a hand is the demo's.

Writes <out dir>/check.json and prints a line per check. Exit code 1 if any check fails.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

NEAR = 0.003  # m out of the cover's open end where an offer starts
IN_RAILS = 0.008  # m into the rails where the calculator is put for trial A's take
SETTLE = 0.0002  # m the calculator held under a load below the rails' force may still move
GRAVITY = 9.81
FAST, SLOW = 0.08, 0.02  # m/s: a hand's push along the rails, and near home
SLOW_FROM = 0.020  # m off home: slow from here
GAIN = 1.0  # N per m/s: the push held at speed (GAIN dt / m = 0.5 at 20 Hz on the 100 g calculator: it settles)
STILL = 0.002  # m/s: stopped (for 3 control steps)


def mm(value: float) -> float:
    return round(1000.0 * float(value), 3)


def main() -> int:
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    hold = Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))
    weight = np.array([0.0, 0.0, m.CALC_MASS * GRAVITY])
    play = {"across (mm)": 1000 * m.LATERAL, "up (mm)": 1000 * m.ACROSS, "tilt (deg)": math.degrees(m.LINE_TILT)}
    result: dict = {"F_r (N)": m.RAIL_FORCE, "F_d (N)": m.DETENT_FORCE, "play": play, "checks": {}}
    checks = result["checks"]
    loaded = {"calc": False}

    def reset() -> None:
        embodiment.reset(Scene(id="cover", instruction=m.INSTRUCTION), seed=0)
        loaded["calc"] = False

    def run(seconds: float, load=None, until=None, watch=None) -> list:
        """Step up to ``seconds`` with ``load(k)`` -> the force on the calculator, or None (nothing on it); stop after
        the step at which ``until()`` holds; ``watch()`` read after each step."""
        seen = []
        for k in range(max(1, int(round(seconds * hz)))):
            force = load(k) if load else None
            if force is None:
                if loaded["calc"]:
                    mk.push("calc")
                loaded["calc"] = False
            else:
                mk.push("calc", force=tuple(float(v) for v in force))
                loaded["calc"] = True
            embodiment.step(hold)
            seen.append(watch() if watch else None)
            if until is not None and until():
                break
        return seen

    def along() -> np.ndarray:
        """The way the calculator leaves the cover (world): its own +y, the display end first."""
        return mk.pose("calc")[1] @ np.array([0.0, 1.0, 0.0])

    def pull(force: float):
        """``force`` N on the calculator along the rails (out of the cover; negative: in), its weight carried."""
        return lambda k: force * along() + weight

    def travel() -> float:
        return float(mk.joint("rails"))

    def speed_in() -> float:
        """How fast the calculator moves into the cover (m/s; the cover is held)."""
        return -float(mk.velocity("calc")[0] @ along())

    def push_in(speed: float):
        """A hand's push into the cover along the rails, held at ``speed`` m/s (0 to 1.4 F_r), the weight carried."""

        def load(k):
            force = min(max(m.RAIL_FORCE + GAIN * (speed - speed_in()), 0.0), 1.4 * m.RAIL_FORCE)
            return -force * along() + weight

        return load

    def within_play(corrections: list) -> bool:
        return all(abs(c["across (mm)"]) <= play["across (mm)"] + 1e-6 and abs(c["up (mm)"]) <= play["up (mm)"] + 1e-6 and c["tilt (deg)"] <= play["tilt (deg)"] + 1e-6
                   for c in corrections)  # fmt: skip

    def put(depth_out: float, across: float = 0.0, up: float = 0.0, turn: float = 0.0) -> None:
        """The free calculator turned keys up in line with the cover's back rails, its ON-key end ``depth_out`` m out of
        the open end (negative: in), ``across``/``up`` off the line (its own x, z) and turned ``turn`` rad about its
        own z through its ON-key end's middle; still, its carrier home in it (a slide's child does not follow)."""
        vp, vr = mk.pose("cover")
        r_line = vr @ m.FLIP  # the calculator's turn with the cover home on its back
        p_line = vp - r_line @ np.array([0.0, -(m.RAILS + depth_out), 0.0])
        on_end = np.array([0.0, -m.CALC[1] / 2, 0.0])
        c, s = math.cos(turn), math.sin(turn)
        twist = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        r = r_line @ twist
        p = p_line + r_line @ on_end - r @ on_end + r_line @ np.array([across, 0.0, up])
        mk.write_pose("calc", p, mk.matrix_to_quat(r))
        mk.set_joint("rails", 0.0)

    # --- Trial A ---
    reset()
    run(0.1)
    mk.make("cover_held")
    start = travel()
    seen = run(1.0, pull(0.8 * m.DETENT_FORCE), watch=travel)
    stretch = 0.8 * m.DETENT_FORCE / m.STIFFNESS
    checks["A1 held by the click under 0.8 F_d"] = {"most out (mm)": mm(max(seen) - start), "on": m.STATE["on"],
                                                   "pass": bool(max(seen) - start <= 1.5 * stretch and m.STATE["on"] == "front")}  # fmt: skip
    seen = run(1.0, pull(1.2 * m.DETENT_FORCE), until=lambda: travel() > m.DETENT_FROM + 0.001, watch=travel)
    checks["A2 leaves home under 1.2 F_d"] = {"steps": len(seen), "at (mm)": mm(seen[-1]), "pass": bool(seen[-1] > m.DETENT_FROM + 0.001)}
    run(3.0, pull(1.4 * m.RAIL_FORCE), until=lambda: travel() >= 0.040)
    run(0.3, pull(0.6 * m.RAIL_FORCE))
    seen = run(1.0, pull(0.6 * m.RAIL_FORCE), watch=travel)
    checks["A3 held on the rails under 0.6 F_r"] = {"at (mm)": mm(seen[-1]), "moved (mm)": mm(max(seen) - min(seen)), "on": m.STATE["on"],
                                                   "pass": bool(max(seen) - min(seen) <= SETTLE and m.STATE["on"] == "front")}  # fmt: skip
    first = travel()
    seen = run(0.5, pull(1.4 * m.RAIL_FORCE), until=lambda: travel() >= m.RAILS - 0.020, watch=travel)
    checks["A4 slides under 1.4 F_r"] = {"moved (mm)": mm(seen[-1] - first), "pass": bool(seen[-1] - first >= 0.010)}
    run(8.0, pull(1.4 * m.RAIL_FORCE), until=lambda: m.STATE["on"] is None)
    checks["A5 comes free past the rails' end"] = {"left the front": m.STATE["left_front"], "events": m.STATE["events"][-1:],
                                                  "pass": bool(m.STATE["on"] is None and m.STATE["left_front"])}  # fmt: skip
    put(-IN_RAILS, across=0.0002, up=0.0003, turn=math.radians(0.15))
    seen = run(0.5, lambda k: weight, until=lambda: m.STATE["on"] is not None)
    corrections = list(m.STATE["corrections"])
    checks["A6 taken on the back within the play"] = {"steps": len(seen), "on": m.STATE["on"], "travel (mm)": mm(travel()), "corrections": corrections,
                                                     "pass": bool(m.STATE["on"] == "back" and len(corrections) == 1 and within_play(corrections))}  # fmt: skip
    run(4.0, push_in(FAST), until=lambda: travel() <= SLOW_FROM)
    still = {"steps": 0}

    def stopped() -> bool:
        still["steps"] = still["steps"] + 1 if abs(speed_in()) < STILL else 0
        return still["steps"] >= 3

    seen = run(4.0, push_in(SLOW), until=stopped, watch=lambda: (travel(), speed_in()))
    stopped_at, most_speed = seen[-1][0], max(v for _, v in seen)
    seen = run(0.5, pull(-1.4 * m.RAIL_FORCE), watch=travel)
    at_click, moved = seen[-1], max(seen) - min(seen)
    run(1.0, pull(-1.2 * m.DETENT_FORCE), until=lambda: travel() <= 0.0003)
    mk.cut("cover_held")
    seen = run(1.0, watch=travel)
    score = float(m.score(None))
    clicked = m.CLICK_IN < stopped_at <= m.DETENT_FROM + 0.001 and m.CLICK_IN < at_click <= m.DETENT_FROM + 0.001 and moved <= SETTLE
    checks["A7 the click, then home"] = {"stopped at (mm)": mm(stopped_at), "fastest on the way (m/s)": round(most_speed, 4),
                                        "under 1.4 F_r for 0.5 s at (mm)": mm(at_click), "moved (mm)": mm(moved), "home, 1 s after (mm)": mm(seen[-1]),
                                        "on": m.STATE["on"], "score": score, "report": m.report(),
                                        "pass": bool(clicked and max(seen) <= m.HOME_WITHIN and m.STATE["on"] == "back" and score == 1.0)}  # fmt: skip

    # --- Trial B ---
    reset()
    run(0.1)
    mk.make("cover_held")
    run(1.0, pull(1.2 * m.DETENT_FORCE), until=lambda: travel() > m.DETENT_FROM + 0.001)
    run(8.0, pull(1.4 * m.RAIL_FORCE), until=lambda: m.STATE["on"] is None)
    for label, across, up in (("2.5 mm across", 0.0025, 0.0), ("2 mm up", 0.0, 0.002)):
        if m.STATE["on"] is not None:
            run(8.0, pull(1.4 * m.RAIL_FORCE), until=lambda: m.STATE["on"] is None)
        before = len(m.STATE["corrections"])
        put(NEAR, across=across, up=up)
        seen = run(1.0, pull(-1.4 * m.RAIL_FORCE), watch=lambda: m.offer())
        made = m.STATE["corrections"][before:]
        last = seen[-1]
        checks[f"B offered {label}, pushed along the rails"] = {"taken": bool(made), "corrections": made, "tip in (mm)": mm(m.RAILS - last["travel"]),
                                                                "across, up (mm)": [mm(last["across"]), mm(last["up"])], "pass": within_play(made)}  # fmt: skip

    failed = [name for name, c in checks.items() if not c["pass"]]
    for name, c in checks.items():
        print("CHECK " + json.dumps({"check": name, **c}, default=str), flush=True)
    result["failed"] = failed
    (out / "check.json").write_text(json.dumps(result, indent=1, default=str))
    print(f"CHECKS {len(checks) - len(failed)} of {len(checks)} pass", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
