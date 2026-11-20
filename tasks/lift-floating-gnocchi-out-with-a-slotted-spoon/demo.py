#!/usr/bin/env python3
"""Reference runs of the gnocchi task: the full score, and the runs that show what each level and each zero rule
costs.

    scripts/rta demo <id> mech/tasks/gnocchi_lift [run,run,...] [--dry] [window=<from>,<to>[,<late>]]

The right arm takes the slotted spoon off its rest and waits over the pan. Whenever pieces float it brings the
bowl down behind them, nose down, slides it under them, lifts, carries them level to the plate, tips them out nose
down and comes back. When no piece is left in the pan it lays the spoon back on its rest while the left arm puts a
fingertip on the hob's key.

What the script reads. Where the floating pieces are and since when each has floated it reads from the simulator:
a person at the pan sees both. It never reads where a piece on the floor lies or when it will be done, and it
decides nothing from a camera picture. Every lift is chosen for the most floating pieces over the bowl, then for
the piece that has floated longest, then for the fewest pieces the bowl comes down on. A piece that no scoop
fits (pushed against the pan's wall, or drifted into the part of the pan nearest the arm) is left: when nothing
more is to float and no way has been found for 12 s, the arm tidies up. (Tried and dropped: once
a piece had floated for 15 s, only lifts that took it. That gave one piece a trip and a longest wait of 48.4 s
against 1.5 pieces and 32.1 s: runs l7t and l7q.) The arm's way there is checked against its joint limits and
the scene before it is taken (skim.py).

Runs: ``full`` everything, score 1.0. ``nine``, ``six``, ``three``: the arm stops taking pieces once that many
lie on the plate and tidies up (0.75, 0.5, 0.25). ``heat_on``: everything, but the key is not touched (0.5).
``spoon_kept``: everything, but the spoon stays in the gripper (0.5). ``late``: once all but three pieces have
left the water the arm waits until every piece still in it has floated for longer than a floating piece may
(the task's LATE_S), then takes them: they do not count (0.75). Until then it takes no spoonful that would leave
fewer than three in the water. ``handled``: before the spoon starts, the open left hand goes down over two
floating pieces, one after the other, as if to pick them out; they are served with the rest and do not count
(0.75). ``shove``: everything, and before the key the left hand pushes the pan back on the hob, more than 3 cm
(0). ``raw``: before it takes the spoon the right hand, empty, reaches into the water, closes on what lies on
the floor there and puts it on the plate; then everything (0). The hand goes down at a place fixed in this
file, the part of the pieces' side of the pan that it can reach; it is not read from the simulator.
``grip_only``: take the slotted spoon, one scoop in the empty water, one serving, back over the pan: where the
bowl really is against where the hand's pose puts it. ``shove_only``, ``raw_only``: those two motions alone, for tuning them. ``tip_only``: the left hand pinches the
pan's wall and goes 7 cm up, to see whether a hand can tip the pan (it cannot: the task file says why tipping
is not scored). ``--dry``: no simulator; the fixed parts
of the plan (taking the spoon, a scoop at each start place of seed 0, serving at every landing place, laying the
spoon back, the key, the hand into the pan) through the IK and the collision check, and the full run's times by
arithmetic (``forecast``). ``window=<from>,<to>[,<late>]``: a what-if with other timing numbers than the task
file's (when the pieces reach the surface, how long a floating piece may stay); the run limit follows from them
(task.set_timing), and every run's ``trips.json`` names the numbers it ran with.
"""

from __future__ import annotations

import json
import math
import sys
import traceback
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # demo_kit, skim

RUNS = {"full": None, "nine": 9, "six": 6, "three": 3, "heat_on": None, "spoon_kept": None, "late": None, "handled": None, "shove": None, "raw": None,
        "shove_only": None, "raw_only": None, "tip_only": None, "grip_only": None}  # run -> pieces on the plate at which it stops taking more  # fmt: skip
# Seconds of a scoop: over the water, into it, under the pieces, still, out, level. Into the water in 0.6 s the bowl
# swung 5 mm high at the end of the plunge and 8 mm low 0.65 s later (run l7g1), about what the bowl has to spare
# under a gnocco: on one layout the scoops came out full, on the next empty (runs l7q and l7z).
SCOOP_S = (1.2, 1.0, 1.0, 0.2, 0.8, 0.5)
SERVE_S = (1.3, 0.7, 0.4, 0.5)  # to the plate, tip, let slide, level
TRIP_S = sum(SCOOP_S) + sum(SERVE_S)  # 7.6 s from one scoop's start to the next
OUT_S = 4.85  # s into a trip at which its pieces have left the water (4.8 to 4.85 s: run l7ref full)
SCORED_S = 4.5  # s from the start of the tidying until the spoon lies and the heat is off (run l7ref full: 85.85 to 90.35 s)
HEADINGS = (225, 210, 240, 195, 255, 180, 270)  # handle headings tried, the easiest for the arm first
PITCHES = (10, 15)  # nose down, degrees
AHEAD = 0.03  # m: the piece the scoop is aimed at lies this far nose-side of the bowl's middle
CARRY, TIP, TIP_DOWN = math.radians(225), math.radians(285), math.radians(60)
# Where the nose points over the plate, from the plate's middle (m). A gnocco slides out late and lands about 7 cm
# on along the nose's last direction (105 degrees), or it leaves early in the swing and lands about 11.5 cm on along
# 66 to 78 degrees (the second of two on the bowl: runs l7g3 late, l7r nine and six, where it came down on the
# plate's rim and off it). At these six places both landings are 3 cm or more inside the rim and the arm tips with
# 9.4 degrees or more to spare at its joints (the map: ~/.robocurve/builder/l7/land1.py; at the earlier place
# (0.035, -0.03) the early one came down 0.6 cm outside the rim). Six places so that twelve do not land on one heap.
LANDINGS = ((-0.02, -0.05), (0.01, -0.06), (-0.03, -0.04), (0.00, -0.05), (-0.02, -0.06), (-0.01, -0.06))
MARGIN = 8.0  # degrees every joint keeps on a scoop's way
PAIR = 0.045  # m: two floating pieces nearer than this are also tried together, the bowl aimed between them
TIDY_S = 14.0  # s kept at the end for laying the spoon down and the key
WAIT = 0.25
LATE_LEFT = 3  # pieces the ``late`` run leaves in the water until they have floated too long
HANDLED = 2  # floating pieces the ``handled`` run's left hand goes down over
GIVE_UP = 12.0  # s without a way to any piece left in the pan, none of them still to float: the arm tidies up
# What the simulated arm falls short by where the spoon lies, 56 cm from its base (world x, y, z in m): at the end of
# the pinch its grasp point was 1.4 mm nearer the arm, 0.4 mm to the left and 2.2 mm lower than its target (run
# l7g1; 0.3 mm off at the earlier place 22 cm from the base, run l7q). The fingers then held the handle 2.2 mm from
# where the plan has them, the bowl rode that much higher, and the scoops that had come out full came out empty
# (run l7z late: 1 of 7). The hand is sent this much beyond the planned grasp point.
TAKE_SHORT = (0.0014, -0.0004, 0.0022)
SHOVE = 0.20  # m the ``shove`` run's hand travels, the last 15 cm of it against the pan (16 cm moved the pan 36.7 mm: run l7u)
# Where the ``raw`` run's hand goes into the water, from the pan's middle (m), and its turn about the vertical: of
# the side of the pan the pieces lie on, the hand pointing straight down reaches the floor only along the edge
# nearest its arm (joints 5.7 degrees inside their limits here, 6.0 at the pan's middle, nothing beyond).
GRAB_AT, GRAB_YAW = (0.02, -0.02), math.radians(30)


def forecast(surface, ready_s: float, per_trip: int = 1, hold_s: float = 1.0) -> dict:
    """The full run by arithmetic, with no simulator: the times at which the pieces reach the surface, in; the
    trips, the longest a piece floats before it leaves the water, and the run's end, out. The arm is over the pan
    at ``ready_s``; whenever it is free and pieces float it makes a trip of TRIP_S with up to ``per_trip`` of them,
    those that have floated longest first; they leave the water OUT_S into the trip. When the pan is empty the
    tidying takes SCORED_S and the score has to hold for ``hold_s``. One piece a trip is the slowest the script can
    be: in the recordings a trip brought out one to three (1.33 on average in run l7ref full)."""
    waiting = sorted(float(v) for v in surface)
    if not waiting:
        raise ValueError("no piece to come up")
    t, trips, waits = float(ready_s), [], []
    while waiting:
        t = max(t, waiting[0])
        taken = [up for up in waiting if up <= t][:per_trip]
        waits.extend(t + OUT_S - up for up in taken)
        trips.append(t)
        waiting = waiting[len(taken):]
        t += TRIP_S
    return {"trips": len(trips), "first trip at (s)": round(trips[0], 2), "pan empty at (s)": round(t, 2), "longest a piece floats (s)": round(max(waits), 2),
            "ends at (s)": round(t + SCORED_S + hold_s, 2)}  # fmt: skip


def floating(m, mk) -> list[tuple[int, np.ndarray, float]]:
    """(piece, where its middle is (world x, y), since when it has floated) for every piece afloat in the pan."""
    found = []
    for k, name in enumerate(m.NAMES):
        if np.isnan(m.STATE["floated"][k]) or not m.STATE["in_water"][k]:
            continue
        at = mk.pose(name)[0]
        if math.hypot(at[0] - m.PAN_XY[0], at[1] - m.PAN_XY[1]) < m.PAN_R:
            found.append((k, np.array(at[:2], dtype=float), float(m.STATE["floated"][k])))
    return found


def choices(layout, afloat: list, most: int | None = None) -> list[dict]:
    """Every scoop worth trying, best first: aimed at each floating piece and between each two that float within
    PAIR of each other, for every heading and tilt at which the spoon fits the pan. ``most``: no scoop that has
    more pieces than this over the bowl."""
    import skim

    places = [at for _, at, _ in afloat]
    aims = list(places) + [(places[a] + places[b]) / 2 for a in range(len(places)) for b in range(a) if np.linalg.norm(places[a] - places[b]) <= PAIR]
    found = []
    for aim in aims:
        for heading in HEADINGS:
            for pitch in PITCHES:
                poses = layout.scoop_poses(aim, math.radians(heading), math.radians(pitch), AHEAD)
                if min(layout.spoon_gap(poses[k]) for k in ("in", "under")) < 0.001 or min(layout.spoon_gap(poses[k]) for k in ("over", "out", "level")) < skim.CLEAR:
                    continue
                taken = layout.over_bowl(poses["under"], places)
                if not taken or (most is not None and len(taken) > most):
                    continue
                pushed = [k for k in layout.under_bowl(poses["in"], places) if k not in taken]
                found.append({"aim": aim, "heading": heading, "pitch": pitch, "takes": [afloat[k][0] for k in taken], "comes down on": [afloat[k][0] for k in pushed],
                              "floating since": min(afloat[k][2] for k in taken)})  # fmt: skip
    return sorted(found, key=lambda c: (-len(c["takes"]), c["floating since"], len(c["comes down on"]), HEADINGS.index(c["heading"]), c["pitch"]))


def plan_trip(layout, afloat: list, arm, t: float, tries: int = 10, most: int | None = None):
    """(moves of the scoop, what was chosen) for the best choice the arm can do from where it is, or (None, why).
    ``most``: the most pieces the scoop may have over the bowl."""
    ranked = choices(layout, afloat, most)
    refused = []
    for chosen in ranked[:tries]:
        moves = layout.scoop(chosen["aim"], math.radians(chosen["heading"]), math.radians(chosen["pitch"]), AHEAD, seconds=SCOOP_S)
        got = layout.check(moves, start_q=arm.q, start=arm.state(t)[:2])
        if layout.passes(got, margin=MARGIN):
            return moves, {**{k: v for k, v in chosen.items() if k != "aim"}, "aim (mm from the pan's middle)": [round(1000 * float(chosen["aim"][i] - layout.pan[i]), 1) for i in range(2)],
                           "joint margin (deg)": got["margin_deg"][0], "choices": len(ranked), "refused by the arm's check before it": refused}  # fmt: skip
        refused.append({"takes": chosen["takes"], "heading": chosen["heading"], "pitch": chosen["pitch"], "joint margin (deg)": got["margin_deg"][0], "arm's gap (mm)": got["arm_gap_mm"][0],
                        "spoon's gap (mm)": got["spoon_gap_mm"][0], "ik (mm)": got["ik_mm"]})  # fmt: skip
    return None, f"none of {min(tries, len(ranked))} choices (of {len(ranked)}) passes the arm's check: {refused}"


def main() -> int:
    import demo_kit as kit
    import skim
    from demo_kit import Move, down
    from yam_arena import station
    from yam_arena.constants import FINGER_OPEN

    args = [a for a in sys.argv[1:] if a != "--dry" and not a.startswith("window=")]
    windows = [a for a in sys.argv[1:] if a.startswith("window=")]
    dry = "--dry" in sys.argv
    out = Path(args[0])
    runs = args[1].split(",") if len(args) > 1 else ["full"]
    if any(run not in RUNS for run in runs):
        raise SystemExit(f"a run is one of {sorted(RUNS)}; got {runs}")
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    if windows:  # a what-if: other timing numbers than the task file's
        m.set_timing(*(float(v) for v in windows[-1].split("=", 1)[1].split(",")))
    timing = {"the pieces reach the surface (s after the start): from, to": [m.FLOAT_FROM, m.FLOAT_TO], "a floating piece may stay (s)": m.LATE_S,
              "run limit (steps)": m.RUN_LIMIT_STEPS, "the task file's own numbers": not windows}  # fmt: skip
    print(f"TIMING {json.dumps(timing)}", flush=True)
    layout = skim.Layout(m)
    up = np.array([0.0, 0.0, 1.0])

    flat = skim.spoon_rotation(CARRY)
    home = skim.Pose(np.array([layout.pan[0], layout.pan[1], layout.carry_height(flat)]), flat)
    over_rest = skim.Pose(np.array([m.REST_XY[0], m.REST_XY[1], layout.carry_height(flat) + 0.03]), skim.spoon_rotation(m.SPOON_YAW))
    take_aside = [*layout.take_spoon(TAKE_SHORT), layout.move(1.5, over_rest, label="level over the rest")]
    take = [*take_aside, layout.move(2.0, home, label="over the pan")]

    def serve(n: int) -> list:
        return layout.serve(CARRY, TIP, TIP_DOWN, nose_at=LANDINGS[n % len(LANDINGS)], seconds=SERVE_S)

    def lay_spoon_down() -> list:
        """Back onto its rest as it lay, 3 mm above; open; away along the hand's own axis; back to the start."""
        grasp, rot = layout.hand(layout.rest_pose)
        return [
            layout.move(1.8, over_rest, label="over the rest"),
            Move(1.4, pos=grasp + 0.003 * up, rot=rot, label="spoon onto the rest"),
            Move(0.6, grip=1.0, label="let go"),
            Move(1.0, pos=grasp - 0.07 * rot[:, 2], label="away from the spoon"),
        ]

    key = np.array([m.HOB_XY[0] + m.KEY_XY[0], m.HOB_XY[1] + m.KEY_XY[1], m.TOP + m.HOB_H])
    tip_reach = 0.0128  # the closed fingertips end this far past the grasp point

    def touch_key() -> list:
        """The left hand, closed, straight down: its fingertips 1 mm over the key for half a second."""
        return [
            Move(2.0, pos=key + 0.08 * up, rot=down(0.0), grip=0.0, label="over the key"),
            Move(0.8, pos=key + (tip_reach + 0.001) * up, label="onto the key"),
            Move(0.5, label="on the key"),
            Move(0.8, pos=key + 0.08 * up, label="off the key"),
        ]

    wall = m.PAN_XY[0] - m.PAN_R - m.PAN_WALL  # the pan's wall nearest the arms, on the pan's centre line
    shove_z = m.TOP + m.HOB_H + 0.015 + tip_reach  # the fingertips 15 mm over the glass

    def shove_pan() -> list:
        """The left hand, closed, straight down in front of the pan, then SHOVE away from the arms: the pan (4 kg
        with its water; about 28 N to start it sliding on the glass) goes part of the way with it. The arm is
        driven by position only: it pushes as hard as it lags behind its target."""
        start = np.array([wall - 0.050, m.PAN_XY[1], shove_z])
        return [
            Move(2.0, pos=start + 0.08 * up, rot=down(0.0), grip=0.0, label="in front of the pan"),
            Move(0.8, pos=start, label="down beside the pan"),
            Move(3.0, pos=start + np.array([SHOVE, 0.0, 0.0]), label="shove the pan"),
            Move(0.8, pos=start + np.array([SHOVE - 0.05, 0.0, 0.08]), label="up from the pan"),
        ]

    def tip_pan() -> list:
        """The left hand, open, straight down over the pan's wall where it is nearest the arms, a finger on either
        side of it, the grasp point 10 mm under the rim; it closes on the wall, goes 7 cm up, holds, comes back."""
        at = np.array([m.PAN_XY[0] - m.PAN_R - m.PAN_WALL / 2, m.PAN_XY[1], m.PAN_AT[2] + m.PAN_BASE + m.PAN_DEPTH - 0.010])
        return [
            Move(2.0, pos=at + 0.08 * up, rot=down(-math.pi / 2), grip=1.0, label="over the pan's wall"),
            Move(1.0, pos=at, label="down over the wall"),
            Move(0.8, grip=0.0, label="pinch the wall"),
            Move(2.0, pos=at + 0.07 * up, label="lift the wall"),
            Move(1.0, label="hold it up"),
            Move(1.5, pos=at, label="set it down"),
            Move(0.6, grip=1.0, label="let go"),
            Move(0.8, pos=at + 0.08 * up, label="up"),
        ]

    def grab_raw() -> list:
        """The right hand, empty and open, straight down into the water at GRAB_AT until its fingertips are 4 mm
        over the pan's floor; it closes on what lies between them, lifts it out, carries it to the plate (the side
        of it towards the arms, where the spoon's pieces do not land) and lets it go 3 cm over it."""
        floor = m.PAN_AT[2] + m.PAN_BASE
        low = np.array([m.PAN_XY[0] + GRAB_AT[0], m.PAN_XY[1] + GRAB_AT[1], floor + tip_reach + 0.004])
        high = np.array([low[0], low[1], floor + m.PAN_DEPTH + 0.06])
        over_plate = np.array([m.PLATE_XY[0] - 0.05, m.PLATE_XY[1], m.TOP + m.PLATE_T + tip_reach + 0.03])
        return [
            Move(2.0, pos=high, rot=down(GRAB_YAW), grip=1.0, label="over the pan"),
            Move(1.2, pos=low, label="hand into the water"),
            Move(0.8, grip=0.0, label="close the hand"),
            Move(1.2, pos=high, label="lift out"),
            Move(2.0, pos=over_plate + 0.08 * up, label="to the plate"),
            Move(0.8, pos=over_plate, label="down to the plate"),
            Move(0.6, grip=1.0, label="let go"),
            Move(0.8, pos=over_plate + 0.08 * up, label="up from the plate"),
        ]

    if dry:
        show = lambda got: {k: v for k, v in got.items() if k not in ("end_q", "end")}  # noqa: E731
        got = layout.check(take, holding_from=6.2)
        print(f"DRY take the spoon and go over the pan: {show(got)}", flush=True)
        q, end = got["end_q"], got["end"]
        m.begin(0)
        for k, (x, y) in enumerate(m.STATE["place"]):
            afloat = [(k, np.array([m.PAN_XY[0] + x, m.PAN_XY[1] + y]), 0.0)]
            moves, chosen = plan_trip(layout, afloat, type("A", (), {"q": q, "state": lambda self, t: end})(), 0.0)
            print(f"DRY scoop at start place {k} ({1000 * x:.0f}, {1000 * y:.0f}) mm: {chosen}", flush=True)
        level = [layout.move(1.0, home, label="over the pan")]
        for n in range(len(LANDINGS)):
            got = layout.check([*level, *serve(n), layout.move(1.2, home, label="back")], start_q=q, start=end)
            print(f"DRY serve at landing {n} {LANDINGS[n]}: passes {layout.passes(got, margin=MARGIN)} {show(got)}", flush=True)
        got = layout.check(lay_spoon_down(), start_q=q, start=end)
        print(f"DRY lay the spoon down: passes {layout.passes(got, margin=MARGIN, spoon_gap=-0.001)} {show(got)}", flush=True)
        got = layout.check(touch_key(), holding_from=1e9, side="left")
        print(f"DRY the key (left arm): {show(got)}", flush=True)
        got = kit.dry_run({"left": [*shove_pan(), *touch_key()]}, station.BASES)
        print(f"DRY shove the pan, then the key (left arm; it is meant to touch the pan): {got['left']}", flush=True)
        got = kit.dry_run({"left": tip_pan()}, station.BASES)
        print(f"DRY pinch the pan's wall and lift (left arm): {got['left']}", flush=True)
        got = layout.check(grab_raw(), holding_from=1e9, hand_in_pan=True, finger=FINGER_OPEN)
        print(f"DRY the empty right hand into the pan and to the plate (hand open, its corners against the open pan): {show(got)}", flush=True)
        got = layout.check([*grab_raw(), *take], holding_from=sum(mv.seconds for mv in grab_raw()) + 6.2, hand_in_pan=True)
        print(f"DRY the same, then take the spoon and go over the pan: {show(got)}", flush=True)
        surface = m.STATE["done_at"] + m.RISE_TRAVEL_S  # seed 0, as laid out above
        ready = sum(mv.seconds for mv in take)
        limit = m.RUN_LIMIT_STEPS / m.CONTROL_HZ
        print(f"DRY seed 0: the pieces reach the surface at {sorted(round(float(v), 1) for v in surface)} s; the arm is over the pan at {ready:.1f} s; run limit {limit:.0f} s", flush=True)
        for per_trip in (1, 2):
            print(f"DRY the full run by arithmetic, {per_trip} to a trip: {forecast(surface, ready, per_trip)} (a floating piece may stay {m.LATE_S:.0f} s; the script stops taking pieces at {limit - TIDY_S:.0f} s)", flush=True)
        return 0

    def hand_over(at: np.ndarray, turned: bool = False) -> list:
        """The left hand, open and straight down, its fingers either side of a floating piece at ``at`` (world x,
        y) along the pan's wall, the grasp point at the piece's middle; half a second there; up again.
        ``turned``: the hand half a turn further round (the same two finger places; the wrist may only reach one
        of the two)."""
        yaw = math.atan2(at[1] - m.PAN_XY[1], at[0] - m.PAN_XY[0]) + (math.pi if turned else 0.0)
        middle = np.array([at[0], at[1], m.PAN_AT[2] + m.SURFACE + 0.001 - m.PIECE[2] / 2])
        high = np.array([at[0], at[1], m.PAN_AT[2] + m.PAN_BASE + m.PAN_DEPTH + 0.06])
        return [Move(2.0, pos=high, rot=down(yaw), grip=1.0, label="over a piece"), Move(0.9, pos=middle, label="hand down on it"), Move(0.5, label="hand on it"), Move(0.9, pos=high, label="hand up")]

    failed = []
    for run in runs:
        try:
            quota = RUNS[run]
            state = {"phase": "handle" if run == "handled" else "skim", "trips": [], "skipped": {}, "left_started": False}

            def out_of_water() -> int:
                return int((~m.STATE["in_water"]).sum())

            def landed() -> int:
                """Pieces over the plate and down on it or on one another, at rest or not: what a look at the plate counts."""
                places = [mk.pose(name)[0] for name in m.NAMES]
                return sum(1 for at in places if math.hypot(at[0] - m.PLATE_XY[0], at[1] - m.PLATE_XY[1]) <= m.PLATE_R and at[2] < m.TOP + m.PLATE_T + 0.04)

            def on_plate() -> int:
                return sum(1 for k in range(m.PIECES) if m.on_plate(k))

            def director(t: float, arms: dict, pictures, run=run, quota=quota, state=state) -> None:
                right, left = arms["right"], arms["left"]
                state["hand"] = right.state(t)[:2]  # where the right hand is told to be: extra() reads the spoon against it
                if t < right.seconds - 1e-6 or run == "grip_only":
                    return
                if state["phase"] == "handle":  # the right arm waits beside the pan while the left hand is in it
                    afloat = floating(m, mk)
                    if "handled" not in state and len(afloat) >= HANDLED:
                        plan, picked, q, start = [], [], left.q, left.state(t)[:2]
                        for k, at, _ in sorted(afloat, key=lambda f: -f[1][1]):  # those nearest the left arm first
                            ways = [(turned, layout.check(hand_over(at, turned), start_q=q, start=start, holding_from=1e9, side="left", hand_in_pan=True, finger=FINGER_OPEN)) for turned in (False, True)]
                            turned, got = max(ways, key=lambda way: (layout.passes(way[1]), way[1]["margin_deg"][0]))
                            if layout.passes(got):
                                plan.extend(hand_over(at, turned))
                                picked.append({"piece": k, "hand turned": turned, "joint margin (deg)": got["margin_deg"][0], "hand's gap (mm)": got["arm_gap_mm"][0]})
                                q, start = got["end_q"], got["end"]
                            if len(picked) == HANDLED:
                                break
                        if len(picked) == HANDLED:
                            state["handled"] = picked
                            left.moves.extend([Move(max(0.0, t - left.seconds), label="wait"), *plan, Move(1.5, pos=left.start_pos, rot=left.start_rot, grip=1.0, label="park")])
                            print(f"t {t:.2f}: the left hand goes down over {picked}", flush=True)
                    if "handled" in state and t >= left.seconds - 1e-6:
                        state["phase"] = "skim"
                        right.moves.append(layout.move(2.0, home, label="over the pan"))
                    else:
                        right.moves.append(Move(WAIT, label="wait"))
                    return
                if state["phase"] == "skim" and run == "late" and "went on at (s)" not in state and out_of_water() >= m.PIECES - LATE_LEFT:
                    came_up = m.STATE["floated"][m.STATE["in_water"]]  # of the pieces still in the water
                    if len(came_up) and (np.isnan(came_up).any() or t < float(came_up.max()) + m.LATE_S + 1.0):
                        if "waited from (s)" not in state:
                            state["waited from (s)"] = round(t, 2)
                            print(f"t {t:.2f}: {out_of_water()} pieces are out of the water; waiting until the rest have floated for more than {m.LATE_S:.0f} s", flush=True)
                        right.moves.append(Move(WAIT, label="wait"))
                        return
                    state["went on at (s)"] = round(t, 2)
                if state["phase"] == "skim":
                    afloat = [f for f in floating(m, mk) if state["skipped"].get(f[0], -1.0) <= t]
                    in_pan = sum(1 for k in range(m.PIECES) if m.STATE["in_water"][k])
                    enough = quota is not None and landed() >= quota
                    time_up = t >= m.RUN_LIMIT_STEPS / embodiment.info.control_hz - TIDY_S
                    stuck = "no way since" in state and t - state["no way since"] >= GIVE_UP
                    if enough or in_pan == 0 or time_up or stuck:
                        state["phase"] = "tidy"
                        state["why"] = "enough lie on the plate" if enough else "the pan is empty" if in_pan == 0 else "time is up" if time_up else f"no way to what is left in the pan for {GIVE_UP:.0f} s"
                    elif afloat:
                        # the late run keeps LATE_LEFT pieces in the water until their time is up: no spoonful that would take one of them
                        most = m.PIECES - LATE_LEFT - out_of_water() if run == "late" and "went on at (s)" not in state else None
                        moves, chosen = plan_trip(layout, afloat, right, t, most=most)
                        if moves is None:
                            for k, _, _ in afloat:
                                state["skipped"][k] = t + 2.0  # look at these again in 2 s: they wander
                            if not np.isnan(m.STATE["floated"][m.STATE["in_water"]]).any():  # nothing more is to float
                                state.setdefault("no way since", t)
                            right.moves.append(Move(WAIT, label="wait"))
                            print(f"t {t:.2f}: no trip for {[k for k, _, _ in afloat]}: {chosen}", flush=True)
                        else:
                            state.pop("no way since", None)
                            n = len(state["trips"])
                            state["trips"].append({"at (s)": round(t, 2), **chosen, "landing": n % len(LANDINGS), "afloat": [k for k, _, _ in afloat], "out of the water before": int((~m.STATE["in_water"]).sum())})
                            right.moves.extend([*moves, *serve(n)])
                            print(f"t {t:.2f}: trip {n + 1}: {chosen}", flush=True)
                    else:
                        right.moves.append(Move(WAIT, label="wait"))
                if state["phase"] == "tidy":
                    state["phase"] = "done"
                    state["tidy at (s)"] = round(t, 2)
                    if run != "spoon_kept":
                        right.moves.extend(lay_spoon_down())
                    right.moves.append(Move(2.0, pos=right.start_pos, rot=right.start_rot, label="park"))
                    if run != "heat_on":
                        left.moves.extend([Move(max(0.0, t - left.seconds), label="wait"), *(shove_pan() if run == "shove" else []), *touch_key(),
                                           Move(1.5, pos=left.start_pos, rot=left.start_rot, grip=1.0, label="park")])  # fmt: skip
                    print(f"t {t:.2f}: {state['why']}: {on_plate()} on the plate; tidying up", flush=True)

            def extra() -> dict:
                spoon_p, spoon_r = mk.pose("spoon")
                pan_p, pan_r = mk.pose("pan")
                meant = layout.spoon_of(*state["hand"]) if "hand" in state else None  # the spoon as the hand's pose puts it, if it sat in the hand as planned
                off = None if meant is None else meant.rot.T @ (spoon_p - meant.origin)  # in the meant spoon's frame: along the handle, across, up
                turned = None if meant is None else math.degrees(math.acos(float(np.clip((np.trace(meant.rot.T @ spoon_r) - 1) / 2, -1.0, 1.0))))
                nose = None if meant is None else math.degrees(math.asin(float(np.clip((meant.rot.T @ spoon_r)[2, 0], -1.0, 1.0))))  # how far the nose hangs below the meant spoon's (deg)
                return {
                    "bowl off (mm): along the handle, across, up": None if off is None else [round(1000 * float(v), 1) for v in off], "spoon turned (deg)": None if turned is None else round(turned, 1),
                    "nose low (deg)": None if nose is None else round(nose, 1), "bowl high (mm)": None if meant is None else round(1000 * float(spoon_p[2] - meant.origin[2]), 1),
                    "pieces": [[round(float(v), 3) for v in mk.pose(name)[0]] for name in m.NAMES],
                    "spoon": [round(float(v), 4) for v in spoon_p], "spoon x": [round(float(v), 4) for v in spoon_r[:, 0]], "spoon z": [round(float(v), 4) for v in spoon_r[:, 2]],
                    "pan moved (mm)": round(1000 * float(np.linalg.norm((pan_p - np.array(m.PAN_AT))[:2])), 2), "pan tilt (deg)": round(math.degrees(m.tilt(pan_r)), 2),
                    "on the plate": on_plate(), "count": int(m.good().sum()), "afloat": int((~np.isnan(m.STATE["floated"]) & m.STATE["in_water"]).sum()),
                }  # fmt: skip

            limit = m.RUN_LIMIT_STEPS / float(embodiment.info.control_hz)
            empty_trip = [*layout.scoop(layout.pan + np.array([0.0, 0.035]), CARRY, math.radians(15), AHEAD, seconds=SCOOP_S), *serve(0), layout.move(1.2, home, label="back over the pan")]
            first = {"handled": {"right": take_aside}, "raw": {"right": [*grab_raw(), *take]}, "raw_only": {"right": grab_raw()}, "shove_only": {"left": shove_pan()}, "tip_only": {"left": tip_pan()},
                     "grip_only": {"right": [*take, *empty_trip, *empty_trip]}}.get(run, {"right": take})  # fmt: skip
            result = kit.run(embodiment, first, out / run, seed=0, director=None if run in ("shove_only", "raw_only", "tip_only") else director, extra=extra, limit_s=limit, tail_s=1.5, sheet_frames=24, crop={"top_cam": (200, 60, 520, 330)})
            rows = result["rows"]
            scores = [row["info"].get("score", 0.0) for row in rows]
            firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 0.75, 1.0)}
            report = result["report"]
            floated, left = report["floated at (s)"], report["left the water at (s)"]
            waits = [None if f is None or lw is None else round(lw - f, 1) for f, lw in zip(floated, left)]
            taken = [w for w in waits if w is not None]
            for n, trip in enumerate(state["trips"]):
                later = state["trips"][n + 1]["out of the water before"] if n + 1 < len(state["trips"]) else int(sum(lw is not None for lw in left))
                trip["came out"] = later - trip["out of the water before"]
            gaps = [round(b["at (s)"] - a["at (s)"], 2) for a, b in zip(state["trips"], state["trips"][1:])]
            readings = {
                "trips": len(state["trips"]), "pieces per trip": round(sum(t["came out"] for t in state["trips"]) / max(1, len(state["trips"])), 2),
                "seconds from one trip's start to the next, least / median / most": [min(gaps), float(np.median(gaps)), max(gaps)] if gaps else None,
                "seconds a piece floated before it left the water, by piece": waits, "longest": max(taken) if taken else None, "limit": m.LATE_S,
                "pan moved (mm), most": max(row["extra"]["pan moved (mm)"] for row in rows), "pan tilt (deg), most": max(row["extra"]["pan tilt (deg)"] for row in rows),
                "on the plate at the end": rows[-1]["extra"]["on the plate"], "count at the end": rows[-1]["extra"]["count"], "tidying began (s)": state.get("tidy at (s)"), "why": state.get("why"),
                "pieces the left hand went down over": state.get("handled"), "in the pan at the end": int(m.STATE["in_water"].sum()), "late run: waited from (s)": state.get("waited from (s)"), "went on at (s)": state.get("went on at (s)"),
                "nearest a gripper's grasp point came to a piece's middle (mm)": round(1000 * min(float(np.linalg.norm(np.array(row["grip"][side][:3]) - np.array(piece))) for row in rows for side in row["grip"] for piece in row["extra"]["pieces"]), 1),
            }  # fmt: skip
            (out / run / "trips.json").write_text(json.dumps({"score": scores[-1], "first reached": firsts, "timing": timing, **readings, "each trip": state["trips"]}, indent=1, default=str))
            print(f"DEMO {run}: score {scores[-1]:.2f} success {result['terminated'] and result['termination_reason'] == 'success'} after {result['seconds']:.1f} s ({result['steps']} steps of {m.RUN_LIMIT_STEPS}); "
                  f"first reached {firsts}; report { {k: v for k, v in report.items() if k not in ('done at (s)',)} }; worst IK {result['worst_ik']}", flush=True)  # fmt: skip
            print(f"READINGS {run}: {json.dumps(readings, default=str)}", flush=True)
            for n, trip in enumerate(state["trips"]):
                print(f"TRIP {run} {n + 1}: {json.dumps(trip, default=str)}", flush=True)
        except Exception:  # this run is lost; the others are still recorded and the script ends with an error
            traceback.print_exc()
            failed.append(run)
            print(f"DEMO {run}: FAILED (the traceback is above)", flush=True)
    if failed:
        print(f"DEMO runs that failed: {failed}", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
