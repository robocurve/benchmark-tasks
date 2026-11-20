#!/usr/bin/env python3
"""Scripted runs of the cupcake task: the full score and the lower levels the spec names.

    scripts/rta demo <id> mech/tasks/cupcake_candles [run,run,...] [--quick]

For each cupcake, nearest the right arm first: the right hand, pointing down and open 11 mm, pinches a candle 1 cm
under its wick end and pulls it out of the Play-Doh; the left hand comes from the near side, 15 degrees under level,
and closes on the cupcake's rim 1.5 mm past touch; the right hand brings the candle over the middle of the top and
pushes it down along its own axis (a force asked of the arm, 4 N, and the hand's target lowered 2 mm at a time)
until it is 16 mm in; then it lets go and rises, and the left hand opens and backs away.

What the script reads: nothing of the simulator's state. The places come from the layout (seed 0); whether a candle
is in the hand from the gripper's own reading of its fingers (closed on a candle they stop about 5 mm apart, on nothing
they meet); the push is a commanded descent from a height that follows from the layout and the pinch point.

Runs. ``full``: all three cupcakes (1.0), the one full-length recording. The lower levels are short trials of the
middle cupcake alone (the owner's ruling of 2026-11-20: one full-length recording per task): ``t_shallow``: its
candle pushed 8 mm in only; it falls over by itself when the hands are gone. ``t_squashed``: the left hand closes 16 mm
past touch: squashed. ``t_one_hand``: no left hand, the candle pushed into the unheld cupcake at its middle (the
spec's allowed second way). ``t_one_hand_off``: as ``t_one_hand``, but 26 mm off the middle, past the edge of its
base: it tips. ``t_crooked``: the hand drifts sideways while it pushes: the candle snaps; the spare is pushed in in
its place.

``--quick``: every run's plan against the arms' reach and joint limits, without the simulator.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

RUNS = ("full", "t_one", "t_shallow", "t_squashed", "t_one_hand", "t_one_hand_off", "t_crooked")  # t_: a short trial of one cupcake
PADS_OPEN = 0.015  # m between the pads over a candle: 5 mm either side of it, 4 to 5 mm from its neighbours (hands.py)
PINCH_GAP = 0.0025  # m the pads are told to close to on the 5 mm candle: 1.25 mm per finger, about 6 N a pad (run w7c-t2: told
# to meet, 12 N a pad, they went through a light candle); 2 x 0.45 x 6 N holds it against the icing's 3 N
PINCH_BELOW_TOP = 0.010  # m under the wick end
HOLD_PAST_TOUCH, SQUASH_PAST_TOUCH = 0.0015, 0.016  # m the left hand's commanded gap closes past the rim
GOAL, SHALLOW = 0.016, 0.008  # m deep
PUSH_N, STEP_DOWN, STEP_S = 4.0, 0.002, 0.25
CROOKED_SIDE = 0.0025  # m sideways per step while pushing crooked
OFF_MIDDLE = 0.026  # m: past the base's edge (25 mm)


def down_rot() -> np.ndarray:
    """Pointing down, closing along x (the candles' square has a neighbour 22 mm off along x and y)."""
    return np.column_stack([(1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)])


def hold_rot() -> np.ndarray:
    import hands

    return hands.side_grip(0.0, 15.0) @ np.diag([-1.0, -1.0, 1.0])  # the half turn the left arm reaches (hands.py)


class Plan:
    def __init__(self, m, kit, live: bool = False) -> None:
        self.m, self.kit, self.live = m, kit, live
        self.dry_depth: dict[str, float] = {}
        self.turn: dict[str, np.ndarray] = {}  # candle -> the hand's turn that takes it (default: closing along x)
        self.mk = m.mk
        self.cakes = dict(m.placement(0))

    # --- what is known: the layout (seed 0), and what the arm itself senses (builder brief, rule 3) ---
    def cake_xy(self, cake: str) -> np.ndarray:
        return np.array(self.cakes[cake])

    def candle_top(self, candle: str) -> np.ndarray:
        return np.array(self.mk.MECH.bodies[candle].pos) + np.array([0.0, 0.0, self.m.cd.CANDLE_L])

    def in_hand(self) -> bool:
        """Whether the right hand holds something: closed on a 5 mm candle its pads stop near 5 mm apart, on nothing
        they meet (the gripper's own reading of its fingers)."""
        if not self.live:
            return True
        return self.mk.gripper("right")[1] * self.m.PAD_GAP_OPEN > 0.00375  # between the told 2.5 mm and the candle's 5

    # --- moves ---
    def grip(self, gap: float) -> float:
        return gap / self.m.PAD_GAP_OPEN

    def pick(self, candle: str) -> list:
        Move = self.kit.Move
        top = self.candle_top(candle)
        at = top - np.array([0.0, 0.0, PINCH_BELOW_TOP])
        # the hand stays 2.5 cm over the candle tops (its fingertips 12 mm over them) until it is straight over the candle,
        # comes down onto it slowly, and pulls it straight up out of the Play-Doh (run w7c-f1 came in lower and swept two out)
        high = top + np.array([0.0, 0.0, 0.025])
        return [Move(1.5, pos=high + np.array([-0.06, 0.0, 0.0]), rot=self.turn.get(candle, down_rot()), grip=self.grip(PADS_OPEN), label=f"towards {candle}"),
                Move(1.2, pos=high, label=f"over {candle}"), Move(0.5, label=f"settle over {candle}"),
                Move(1.5, pos=at, label=f"down to {candle}"), Move(0.5, label=f"settle at {candle}"), Move(1.0, grip=self.grip(PINCH_GAP), label=f"pinch {candle}"),
                Move(2.0, pos=high, label=f"pull out {candle}"), Move(1.5, pos=high + np.array([-0.08, 0.0, 0.01]), label=f"lift {candle}")]  # fmt: skip

    def tcp_over(self, cake: str, above_icing: float, off: float = 0.0) -> np.ndarray:
        """The grasp point with the held candle's tip ``above_icing`` m over the cupcake's top, ``off`` m left of its middle."""
        x, y = self.cake_xy(cake)
        return np.array([x, y + off, self.m.BOARD_TOP + self.m.CAKE_H + above_icing + self.m.cd.CANDLE_L - PINCH_BELOW_TOP])

    def carry(self, cake: str, off: float, candle: str = "") -> list:
        Move = self.kit.Move
        return [Move(2.5, pos=self.tcp_over(cake, 0.015, off), rot=self.turn.get(candle, down_rot()), label=f"over {cake}"),
                Move(1.5, pos=self.tcp_over(cake, 0.004, off), label=f"onto {cake}")]

    def hold(self, cake: str, past: float) -> list:
        Move = self.kit.Move
        x, y = self.cake_xy(cake)
        grasp = np.array([x, y, self.m.BOARD_TOP + 0.0269])
        a = hold_rot()[:, 2]
        back = grasp - 0.08 * a + np.array([0.0, 0.0, 0.04])
        return [Move(3.0, pos=back, rot=hold_rot(), grip=self.grip(0.075), label=f"towards {cake}"), Move(2.0, pos=grasp - 0.03 * a, label=f"in front of {cake}"),
                Move(2.0, pos=grasp, label=f"round {cake}'s rim"), Move(1.0, grip=self.grip(2 * self.m.R_TOP - past), label=f"hold {cake}")]  # fmt: skip

    def let_go(self, cake: str) -> list:
        Move = self.kit.Move
        x, y = self.cake_xy(cake)
        grasp = np.array([x, y, self.m.BOARD_TOP + 0.0269])
        a = hold_rot()[:, 2]
        return [Move(1.0, grip=self.grip(0.075), label=f"let go of {cake}"), Move(2.0, pos=grasp - 0.08 * a + np.array([0.0, 0.0, 0.04]), label=f"back from {cake}")]

    def release(self, cake: str) -> list:
        Move = self.kit.Move
        return [Move(0.8, grip=self.grip(PADS_OPEN), push=(0.0, 0.0, 0.0), label=f"let go of the candle in {cake}"),
                Move(2.0, pos=self.tcp_over(cake, 0.015) + np.array([0.0, -0.06, 0.0]), label=f"up from {cake}")]

    def steps(self, run: str):
        """Yields {side: [moves]}; the director adds each when both arms are through with the last."""
        Move = self.kit.Move
        order = [("cake2", "candle2"), ("cake1", "candle1"), ("cake0", "candle0")] if run == "full" else [("cake1", "candle1")]
        special_runs = run not in ("full", "t_one")
        held = run not in ("t_one_hand", "t_one_hand_off")
        for cake, candle in order:
            special = special_runs
            if run == "t_squashed":  # the act is the squeeze: hold too hard, let go
                yield {"left": self.hold(cake, SQUASH_PAST_TOUCH)}
                yield {"left": [Move(1.0, label="squeezed")]}
                yield {"left": self.let_go(cake)}
                continue
            goal = SHALLOW if (run == "t_shallow" and special) else GOAL
            off = OFF_MIDDLE if (run == "t_one_hand_off" and special) else 0.0
            tries = [candle, "candle3"] if (run == "t_crooked" and special) else [candle]
            for attempt, which in enumerate(tries):
                crooked = run == "t_crooked" and special and attempt == 0
                yield {"right": self.pick(which)}
                if not self.in_hand():  # the pads met: nothing in the hand; open and take it once more
                    yield {"right": [Move(0.8, grip=self.grip(PADS_OPEN), label=f"open over {which}")]}
                    yield {"right": self.pick(which)}
                if held and attempt == 0:
                    yield {"left": self.hold(cake, HOLD_PAST_TOUCH)}
                yield {"right": self.carry(cake, off, which)}
                # a commanded descent: the tip from 4 mm over the icing (a height that follows from the layout and the
                # pinch point) to the depth wanted, 2 mm a step, pressing 4 N down along the candle; nothing is read
                target = self.tcp_over(cake, 0.004, off)
                for _ in range(round((0.004 + goal) / STEP_DOWN)):
                    target = target + np.array([CROOKED_SIDE if crooked else 0.0, 0.0, -STEP_DOWN])
                    yield {"right": [Move(STEP_S, pos=target, push=(0.0, 0.0, -PUSH_N), label=f"push {which} into {cake}")]}
                yield {"right": [Move(0.5, label=f"rest on {which}")]}
                yield {"right": self.release(cake)}
                if not crooked:  # the crooked push snaps the candle (that is what it shows): the spare goes in after it
                    break
            if held:
                yield {"left": self.let_go(cake)}
        yield {"right": [Move(2.0, pos=np.array([0.35, -0.25, 0.90]), rot=down_rot(), label="away")],
               "left": [Move(2.0, pos=np.array([0.30, 0.25, 0.90]), rot=down_rot() @ np.diag([-1.0, -1.0, 1.0]), label="away")]}


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
        plan = Plan(m, kit)
        for run in runs:
            moves = {"left": [], "right": []}
            for step in plan.steps(run):
                for side, ms in step.items():
                    other = "left" if side == "right" else "right"
                    wait = sum(x.seconds for x in moves[side]) - sum(x.seconds for x in moves[other])
                    if wait > 0:  # the other arm waits for this one: both start each step together
                        moves[other].append(kit.Move(wait, label="wait"))
                    elif wait < 0:
                        moves[side].append(kit.Move(-wait, label="wait"))
                    moves[side].extend(ms)
            print(f"DRY {run}: {json.dumps(kit.dry_run(moves, station.BASES, limit_s=400.0), default=str)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    failed = []
    for run in runs:
        try:
            plan = Plan(m, kit, live=True)
            script = plan.steps(run)
            state = {"over": False}

            folder = out / run
            folder.mkdir(parents=True, exist_ok=True)

            def director(t: float, arms: dict, pictures, script=script, state=state, folder=folder) -> None:
                if mk.MECH.environment is None or not mk.MECH.poses:
                    return
                label = arms["right"].target(t)[3]
                if pictures is not None and label.startswith("up from ") and label not in state:
                    state[label] = True  # the first picture after the hand let go of the candle
                    import cv2

                    for cam in ("left_cam", "top_cam", "right_cam"):
                        cv2.imwrite(str(folder / f"{label.replace(' ', '_')}_{cam}.png"), cv2.cvtColor(np.asarray(pictures[cam]), cv2.COLOR_RGB2BGR))
                if state["over"] or t < max(arm.seconds for arm in arms.values()) - 1e-6:
                    return
                step = next(script, None)
                if step is None:
                    state["over"] = True
                    return
                for side, ms in step.items():
                    arm = arms[side]
                    if arm.seconds < t:  # an arm that has been idle holds where it is until now
                        arm.moves.append(kit.Move(t - arm.seconds, label="wait"))
                    arm.moves.extend(ms)

            def extra() -> dict:
                r = m.readings()
                r["score"] = m.score(None)
                return r

            result = kit.run(embodiment, {"left": [], "right": []}, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_S,
                             tail_s=3.0, record=True)
            rows = [row for row in result["rows"] if "extra" in row]
            last = rows[-1]["extra"]
            readings = {"score": last["score"], "seconds": result["seconds"], "report": result["report"], "worst IK": result["worst_ik"],
                        "most squeeze (mm)": result["report"].get("most squeeze (mm)"), "end": {k: last[k] for k in last if k.startswith(("candle", "cake"))}}  # fmt: skip
            (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
            print(f"DEMO {run}: score {last['score']:.2f} after {result['seconds']:.1f} s; report {json.dumps(result['report'], default=str)}", flush=True)
        except Exception as error:  # noqa: BLE001 - one run's fault must not hide the others'
            import traceback

            traceback.print_exc()
            failed.append(run)
            print(f"DEMO {run} FAILED: {error!r}", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
