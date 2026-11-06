"""Offline (CPU only): replay a run's telemetry through the scorer, then score one probe state.

    python tools/replay_scorer.py runs/<run>/telemetry.jsonl [...]

Each episode in the trace (it starts at a reset line) is fed step by step to a fresh scorer, from the logged
measured state, and the replayed score is printed beside the one the run logged. The hand flags are the logged ones,
so a trace recorded before a change to the hand rule replays with the rule it was recorded under.

The probe: the jack holds the beam 3 mm above both stands' tops (stands set, nothing resting on them) and a gripper
is in the crush zone. That load is held by the jack alone, so it must score 0.
"""

import json
import sys

from jack_stands.scoring import JackStandsScore

STANDS = ("stand_left", "stand_right")


def inputs(r: dict) -> dict:
    return dict(raised_mm=r["raised_mm"], carry_gap_mm=r["carry_gap_mm"],
                under=[r[f"{n}_under"] for n in STANDS], upright=[r[f"{n}_upright"] for n in STANDS],
                stand_gap_mm=[r[f"{n}_gap_mm"] for n in STANDS], stand_still=[r[f"{n}_still"] for n in STANDS],
                tilt_deg=r["tilt_deg"], load_still=r["load_still"], load_vz=r["load_vz"],
                load_shift_m=r["load_shift_m"], hand_in_zone=any(r["hand_in_zone"]),
                part_xy=r["wrench_pose"][:2], part_clear_m=r["part_clear_m"], part_still=r["part_still"])


def replay(path: str) -> list[tuple]:
    episodes, sc, logged, score, n = [], None, None, 0.0, 0
    for line in open(path):
        r = json.loads(line)
        if r.get("reset"):
            if sc is not None:
                episodes.append((logged, score, sc.reason, n))
            sc, logged, score, n = JackStandsScore(20.0), None, 0.0, 0
            continue
        if sc is None:  # a trace without a reset line: start scoring at its first step
            sc = JackStandsScore(20.0)
        score, logged, n = sc.update(**inputs(r)), r.get("score"), n + 1
    if sc is not None:
        episodes.append((logged, score, sc.reason, n))
    return episodes


if __name__ == "__main__":
    for path in sys.argv[1:]:
        for i, (logged, score, why, n) in enumerate(replay(path)):
            print(f"{path} episode {i}: {n} steps, logged {logged} -> replayed {score} ({why})")
    sc = JackStandsScore(20.0)
    probe = dict(raised_mm=3.0, carry_gap_mm=0.0, under=[True, True], upright=[True, True], stand_gap_mm=[3.0, 3.0],
                 stand_still=[True, True], tilt_deg=0.0, load_still=True, load_vz=0.0, load_shift_m=0.0,
                 hand_in_zone=True, part_xy=(0.265, 0.065), part_clear_m=-0.04, part_still=True)
    print("probe (jack holds the beam 3 mm above both stands, a hand in the zone):", sc.update(**probe), sc.reason)
    # A wrench taken out correctly is not counted later: the beam settles on both stands (0.75), the wrench is pulled
    # out (but not held still long enough for 1.0), then the beam is lifted 2 mm off the stands with the wrench lying
    # where it was put. Expected: no violation, 0.75 kept.
    sc = JackStandsScore(20.0)
    rest = dict(probe, raised_mm=0.0, carry_gap_mm=33.0, stand_gap_mm=[0.0, 0.0], hand_in_zone=False)
    for _ in range(12):
        sc.update(**rest)
    sc.update(**dict(rest, part_xy=(0.165, 0.065), part_clear_m=0.036, part_still=False))
    off = dict(rest, stand_gap_mm=[2.0, 2.0], carry_gap_mm=0.0, part_xy=(0.165, 0.065), part_clear_m=0.036)
    score = sc.update(**off)
    print("probe (wrench out on the stands, then the beam lifted 2 mm off them):", score, "violation:", sc.failed,
          sc.reason)
