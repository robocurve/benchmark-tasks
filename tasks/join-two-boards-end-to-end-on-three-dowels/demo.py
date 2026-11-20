#!/usr/bin/env python3
"""Reference runs of the dowel task: the full score, the two lower levels as the same join cut short, the join made
with one board turned 2 degrees, and the hands that stay.

    scripts/rta demo <id> mech/tasks/dowel_join [run,run,...] [--quick]

Each hand comes down over its own board, 25 mm from the board's middle away from the seam end, pointing down and open,
its fingers either side of the board's 80 mm width, the grasp point 15 mm over the bench (held at their middles, the
boards stopped with the dowels 14 mm in, where the two arms' wrists met, the grasp points 154 mm apart: recordings c5f,
c5g, c5h; 25 mm further out each, they are 200 mm apart when the seam is closed); it closes on the board's long sides and slides the board
along the bench towards the middle, 6 cm a second, until the boards' ends are 30 mm apart (the dowels' tips 10 mm
short of the sockets). A board turns a little in the fingers as it slides (recording c5e: both boards half a degree
the same way, which put the ends 1.5 mm out of line, and the dowels stood on the walnut board's end); so the right
hand moves its board across until the two ends are in line within 0.3 mm. Then the right hand holds the cherry board
where it is, and the left hand lets go of the walnut board, comes down closed behind its outer end, turned a quarter
turn so that its broad face meets that end, and pushes the board along the bench onto the dowels, 2.5 mm a second,
until the seam is closed: free on the bench, the walnut board finds the dowels' line by itself, as the free boards in
the checks do (trials.py). With both boards held, the dowels stood at the sockets' mouths (recordings c5i, c5j: the
arms keep the ends in line only to 0.2 to 0.3 mm and level only to 0.1 to 0.16 mm; the bore leaves a dowel 0.1 mm a
side). Then the right hand opens, and both hands go up 8 cm and stay there.

What the script reads from the simulator: the seam's two gaps and how far the ends are out of line (a person sees the
ends close and come into line, and feels them meet), to line the ends up, to stop ``tips`` and ``half`` where they
are meant to stop and to know when the seam is closed. Nothing else: not the score.

Runs. ``full``: 1.0. ``tips``: the left hand stops when both gaps are 15 mm or less, the tips in, and both let go
(0.25). ``half``: when both are 6 mm or less (0.5). ``skew``: both hands keep their boards and bring the ends together,
2.5 mm a second each, the right hand's board turned 2 degrees about the middle of its end (whatever it scores).
``held``: as ``full``, but the hands stay on the boards (0.75). ``both_gripped``: both hands keep their boards, in line
and square, and bring the ends together as ``skew`` does (whatever it scores: recordings c5i and c5j, the dowels
stood at the sockets' mouths). There is no run with one arm alone: the walnut board where it starts is 0.60 m from
the right arm's base, out of its reach.

``--quick``: every run's plan against the arms' reach and joint limits, without the simulator.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

RUNS = ("full", "tips", "half", "skew", "held", "both_gripped")
GRASP_Z = 0.015  # m the grasp point over the bench: the fingertips' ends 2 mm over it
GRASP_OUT = 0.025  # m from the board's middle, away from its seam end, that each hand grasps its board
OPEN, HOLD = 1.0, 0.78  # the hand's opening (0 closed .. 1 open, 94 mm): open over the board; closed on its 80 mm width, pressing
OVER = 0.08  # m over the grasp height before and after
YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # the fingers close along x, across the board
SLIDE, NEAR = 0.06, 0.030  # m/s along the bench; m between the ends when the slide ends
LINED_UP, LINE_UP_TRIES, SETTLE = 0.0003, 5, 0.8  # m the ends may be out of line; corrections; s a correction is given to settle before the next look (an arm follows a new place with a lag of about a second: recording c5k)
PAUSE_AT, PAUSE_S = 0.026, 1.0  # m: both gaps at most this, the left hand stops pushing, and the ends are lined up again before the tips arrive; s it waits first, for the board to stop
TIPS_AT = 0.021  # m: a seam narrower than this has the dowels' tips at the sockets' mouths (skew: kept in line until then)
STEP_S, JOIN = 0.1, 0.00025  # s between two looks at the seam; m a hand moves its board on in a step: 2.5 mm a second
PUSH_YAW = 0.0  # the left hand turned a quarter turn from its grasp to push: its fingers close along y, and the closed hand's broad face meets the walnut board's outer end
FINGER = 0.0039  # m a fingertip is thick: the closed hand's broad faces stand that far either side of the grasp point (as in gantry_square)
BEHIND = 0.006  # m beyond the walnut board's outer end (where the slide put it) that the left hand comes down
BACK = 0.015  # m the left hand moves back off the walnut board's end as it goes up: more than it is ever ahead of the board
STOP_AT = {"tips": 0.015, "half": 0.006}  # m: both gaps at most this, the hands let go
CLOSED = 0.0005  # m: both gaps at most this, the seam is closed
PAST = 0.010  # m a hand is asked at most beyond where its board would close the seam: the boards lag the hands a little (recording c5f: 8 mm in the join), and a hand that is ahead presses its board on gently
SKEW = math.radians(2.0)
DRY_STEPS = 100  # join steps of a plan without the simulator: as far as a hand is ever asked (25 mm)


class Plan:
    def __init__(self, m, see=None) -> None:
        import demo_kit as kit

        self.m, self.see, self.kit, self.Move = m, see, kit, kit.Move
        self.asked = {"left": 0.0, "right": 0.0}  # m each hand has been asked towards the other board in the join
        self.shift = 0.0  # m the right hand has moved its board across (x) to line the ends up

    def hands(self, name: str) -> tuple:
        return ("left", "right")

    def grasp(self, side: str) -> np.ndarray:
        m = self.m
        x, y = m.CHERRY_XY if side == "right" else m.WALNUT_XY
        outward = -1.0 if side == "right" else 1.0  # the cherry board's seam end faces +y, the walnut board's -y
        return np.array([x, y + outward * GRASP_OUT, m.TOP + GRASP_Z])

    def slid(self, side: str, name: str) -> np.ndarray:
        """Where a hand stands when its slide ends: its board's end NEAR/2 from the midline."""
        m = self.m
        reach = m.BOARD_L / 2 + GRASP_OUT  # from the board's seam end to the grasp point
        y = -NEAR / 2 - reach if side == "right" else NEAR / 2 + reach
        at = np.array([m.CHERRY_XY[0], y, m.TOP + GRASP_Z])
        if name == "skew" and side == "right":  # turned about the middle of its end: the hand, behind that end on the board's axis, goes the other way across
            at[0] += reach * math.sin(SKEW)
            at[1] += reach * (1.0 - math.cos(SKEW))
        return at

    def turn(self, side: str, name: str) -> float:
        return YAW[side] + (SKEW if name == "skew" and side == "right" else 0.0)

    def steps(self, name: str):
        """Run ``name`` as steps, each a dict of moves for the hands that work, planned when it is asked for."""
        m, Move, down = self.m, self.Move, self.kit.down
        hands = self.hands(name)
        up = np.array([0.0, 0.0, OVER])
        yield {s: [Move(2.5, pos=self.grasp(s) + up, rot=down(YAW[s]), grip=OPEN, label="over its board"), Move(1.5, pos=self.grasp(s), label="down"), Move(0.6, grip=HOLD, label="close")] for s in hands}
        slide_s = {s: float(np.linalg.norm(self.slid(s, name) - self.grasp(s))) / SLIDE for s in hands}
        longest = max(slide_s.values())
        yield {s: [Move(longest, pos=self.slid(s, name), rot=down(self.turn(s, name)), label="slide"), Move(0.4, label="settle")] for s in hands}
        yield from self.line_up(name, "line up")
        if name in ("skew", "both_gripped"):
            yield from self.both_hold(name)
        else:
            yield from self.push_home(name)

    def line_up(self, name: str, label: str):
        """The right hand moves its board across until the two ends are in line within LINED_UP, looking again only
        when a move has settled."""
        Move = self.Move
        for _ in range(LINE_UP_TRIES):
            if self.see is None:
                return
            off = self.see()["across"]
            if abs(off) <= LINED_UP:
                return
            self.shift -= off
            yield {"right": [Move(0.4, pos=self.held_at("right", name), label=label), Move(SETTLE, label=label)], "left": [Move(0.4 + SETTLE, label=label)]}

    def held_at(self, side: str, name: str) -> np.ndarray:
        """Where a hand that holds its board is asked to be: where the slide put it, the right hand moved across to
        line the ends up, both moved on towards the other board as far as they have been asked."""
        toward = -1.0 if side == "left" else 1.0
        return self.slid(side, name) + np.array([self.shift if side == "right" else 0.0, toward * self.asked[side], 0.0])

    def push_home(self, name: str):
        """The right hand holds the cherry board where it is; the left hand lets go of the walnut board, comes down
        closed behind its outer end, turned a quarter turn so that its broad face meets that end, and pushes the
        board along the bench onto the dowels, 2.5 mm a second, until the seam is closed (``tips``, ``half``: until
        both gaps are 15 or 6 mm or less). Free on the bench, the walnut board finds the dowels' line by itself, as
        the free boards in the checks do. Then the right hand opens, and both go up."""
        m, Move, down = self.m, self.Move, self.kit.down
        up = np.array([0.0, 0.0, OVER])
        behind = np.array([m.WALNUT_XY[0], NEAR / 2 + m.BOARD_L + FINGER + BEHIND, m.TOP + GRASP_Z])
        let_go = [Move(0.3, label="hold"), Move(0.6, grip=OPEN, label="lets go"), Move(1.0, pos=self.held_at("left", name) + up, label="up"),
                  Move(1.5, pos=behind + up, rot=down(PUSH_YAW), grip=0.0, label="over the outer end"), Move(1.5, pos=behind, label="down behind it"), Move(0.3, label="there")]  # fmt: skip
        yield {"left": let_go, "right": [Move(sum(move.seconds for move in let_go), label="holds its board")]}
        most = BEHIND + NEAR + PAST  # as far as the left hand is ever asked: the walnut board's end where the seam would close, and PAST beyond
        paused = False
        while self.asked["left"] < most - 1e-9:
            if self.see is not None:
                gaps = self.see()["gaps"]
                if name in STOP_AT and max(gaps.values()) <= STOP_AT[name]:
                    break
                if max(gaps.values()) <= CLOSED:
                    break
                if not paused and max(gaps.values()) <= PAUSE_AT:  # a board pushed along the bench at one end does not go quite straight (recording c5k, pushed without this stop: the ends came to the dowels 2.3 mm out of line, and the tips stood on the walnut board's end): line the ends up again before the tips arrive
                    paused = True
                    yield {"left": [Move(PAUSE_S, label="waits")], "right": [Move(PAUSE_S, label="holds its board")]}
                    yield from self.line_up(name, "lines up again")
                    continue
            self.asked["left"] = min(most, self.asked["left"] + JOIN)
            yield {"left": [Move(STEP_S, pos=behind - np.array([0.0, self.asked["left"], 0.0]), label="push")], "right": [Move(STEP_S, label="holds its board")]}
        if name == "held":
            yield {s: [Move(3.0, label="hands stay")] for s in ("left", "right")}
            return
        back = behind - np.array([0.0, self.asked["left"] - BACK, 0.0])
        yield {"left": [Move(0.6, pos=back, label="backs off")], "right": [Move(0.6, label="holds its board")]}  # off the board's end first: pressing on while the right hand lets go would shove the pair along (recording c5k)
        yield {"right": [Move(0.3, label="hold"), Move(0.6, grip=OPEN, label="open")], "left": [Move(0.9, label="stays")]}
        yield {"right": [Move(1.2, pos=self.held_at("right", name) + up, label="up"), Move(1.5, label="away")], "left": [Move(1.2, pos=back + up, label="up"), Move(1.5, label="away")]}  # fmt: skip

    def both_hold(self, name: str):
        """Both hands keep their boards and bring the ends together, 2.5 mm a second each, the right hand keeping them
        in line until the dowels' tips are at the sockets, and on until the seam is closed or as far as they are ever
        asked; then they open and go up."""
        Move = self.Move
        hands = ("left", "right")
        up = np.array([0.0, 0.0, OVER])
        most = NEAR / 2 + PAST
        for k in range(10_000):
            if self.see is not None:
                seam = self.see()
                gaps = seam["gaps"]
                if max(gaps.values()) <= CLOSED:
                    break
                if min(gaps.values()) > TIPS_AT and abs(seam["across"]) > LINED_UP:  # kept in line while the ends come together, until the tips are at the sockets
                    self.shift -= 0.3 * seam["across"]
            elif k >= DRY_STEPS:
                break
            if all(self.asked[s] >= most - 1e-9 for s in hands):  # as far as they are ever asked: the run ends as it stands
                break
            for s in hands:
                self.asked[s] = min(most, self.asked[s] + JOIN)
            yield {s: [Move(STEP_S, pos=self.held_at(s, name), label="join")] for s in hands}
        yield {s: [Move(0.3, label="hold"), Move(0.6, grip=OPEN, label="open")] for s in hands}
        yield {s: [Move(1.2, pos=self.held_at(s, name) + up, label="up"), Move(1.5, label="away")] for s in hands}

    def whole(self, name: str) -> dict:
        out: dict = {side: [] for side in self.hands(name)}
        for step in self.steps(name):
            for side, moves in step.items():
                out[side].extend(moves)
        return out


def main() -> int:
    import demo_kit as kit

    quick = "--quick" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--quick"]
    out = Path(args[0])
    runs = args[1].split(",") if len(args) > 1 else ["full"]
    if any(run not in RUNS for run in runs):
        raise SystemExit(f"a run is one of {sorted(RUNS)}; got {runs}")
    if quick:
        from yam_arena import station, task_file

        m = task_file.load(str(HERE / "task.py")).module
        for run in runs:
            print(f"DRY {run}: {json.dumps(kit.dry_run(Plan(m).whole(run), station.BASES, limit_s=60.0), default=str)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    failed = []
    for run in runs:
        try:
            plan = Plan(m, m.seam)
            script = plan.steps(run)
            hands = plan.hands(run)
            state = {"over": False}

            def director(t: float, arms: dict, pictures, script=script, state=state, hands=hands) -> None:
                if pictures is None or mk.MECH.environment is None or not mk.MECH.poses:
                    return
                while not state["over"] and all(t >= arms[side].seconds - 1e-6 for side in hands):  # the next step is planned when the last is through
                    step = next(script, None)
                    if step is None:
                        state["over"] = True
                        break
                    for side, moves in step.items():
                        arms[side].moves.extend(moves)

            def extra(plan=plan) -> dict:
                s = m.seam()
                return {"gaps (mm)": {k: round(1000 * v, 3) for k, v in s["gaps"].items()}, "across (mm)": round(1000 * s["across"], 3), "turned (deg)": round(math.degrees(s["turned"]), 3),
                        "height (mm)": round(1000 * s["height"], 3), "tilts (deg)": {k: round(math.degrees(v), 2) for k, v in s["tilts"].items()},
                        "boards (x, y mm)": {n: [round(1000 * float(v), 1) for v in mk.pose(n)[0][:2]] for n in m.NAMES},
                        "hands to boards (mm)": {side: round(1000 * min(m.to_board(n, mk.gripper(side)[0]) for n in m.NAMES), 1) for side in m.SIDES},
                        "at rest": m.at_rest(), "level": m.level(), "asked (mm)": {k: round(1000 * v, 2) for k, v in plan.asked.items()}, "shift (mm)": round(1000 * plan.shift, 2)}  # fmt: skip

            folder = out / run
            result = kit.run(embodiment, {side: [] for side in hands}, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=1.0, sheet_frames=12,
                             crop={"top_cam": (200, 110, 460, 256)})  # fmt: skip
            rows = result["rows"]
            scores = [row["info"].get("score", 0.0) for row in rows]
            firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 0.75, 1.0)}
            last = rows[-1]["extra"]
            readings = {
                "score": scores[-1], "first reached": firsts, "seconds": rows[-1]["t"], "ended": result.get("termination_reason"),
                "seam gaps at the end (mm)": last["gaps (mm)"], "across at the end (mm)": last["across (mm)"], "turned at the end (deg)": last["turned (deg)"],
                "height and tilts at the end": [last["height (mm)"], last["tilts (deg)"]], "narrowest seam (mm)": min(max(row["extra"]["gaps (mm)"].values()) for row in rows),
                "most across (mm), most turned (deg)": [max(abs(row["extra"]["across (mm)"]) for row in rows if max(row["extra"]["gaps (mm)"].values()) < 40.0) if any(max(row["extra"]["gaps (mm)"].values()) < 40.0 for row in rows) else None,
                                                        max(row["extra"]["turned (deg)"] for row in rows)],
                "boards at the start and the end (x, y mm)": [rows[0]["extra"]["boards (x, y mm)"], last["boards (x, y mm)"]],
                "hands from the boards at the end (mm), at rest": [last["hands to boards (mm)"], last["at rest"]],
                "asked at the end (mm)": last["asked (mm)"], "report": result["report"], "worst IK": result["worst_ik"],
            }  # fmt: skip
            (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
            print(f"DEMO {run}: score {scores[-1]:.2f} success {result['terminated'] and result['termination_reason'] == 'success'} after {result['seconds']:.1f} s ({result['steps']} steps of {m.RUN_LIMIT_STEPS}); "
                  f"first reached {firsts}; report {result['report']}", flush=True)  # fmt: skip
            print(f"READINGS {run}: {json.dumps(readings, default=str)}", flush=True)
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"RUN {run} FAILED: {error!r}", flush=True)
            failed.append(run)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
