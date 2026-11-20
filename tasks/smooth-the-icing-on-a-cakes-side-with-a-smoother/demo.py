#!/usr/bin/env python3
"""The reference run of the cake task (the full score), and a check of the take alone.

    scripts/rta demo <id> mech/tasks/cake_smooth [run,run,...] [--quick]

The right hand, pointing level along the smoother's plate towards its straight edge and open, comes up behind the
plate standing in its block, slides its fingers over the plate and closes on it at mid-height 60 mm from the straight
edge. It draws the plate up out of the groove and off the back it leans on, carries it over to the cake's right side
and sets the straight edge upright
against the icing, its lower corner on the cake board, the plate swung 20 degrees towards the arms. There it holds
still. The left hand, pointing down with its fingers closed, comes down beside the stand's rim on the far left,
presses the rim's edge inward from outside and pushes it round to the near side, so that the cake turns anticlockwise
seen from above; it lets go, goes back round outside the rim and pushes again, until the cake has turned SMOOTH_TURNS
times under the edge. The icing comes at the edge from the side of the plate's free end, the way a scraper is held:
dragged along by the cut, the edge then swings off the icing, not into it. On the last stroke the right hand draws the
edge away from the icing along the turning; then it carries the smoother back, lowers it into the groove 1 mm off the
back and lets go, and the plate settles onto the back; both hands go up.

What the script reads from the simulator: where the smoother, the block and the cake are (to put the hand on them),
the icing's mean thickness near the edge (a person sees where the icing's surface is) and the stand's top's turn (a
person sees the cake go round); nothing else: not the icing's ridges, not the score.

Runs. ``full``: 1.0, the one recorded run. ``take``: the smoother taken out of its block and moved off it, then held:
a check of that act alone, not a level. The lower levels are shown by rule tests that set the icing directly
(tests/test_cake_smooth.py).

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

RUNS = ("full", "take")  # take: a check of the take alone (out of the block and off it), not a level
SWING = math.radians(-20.0)  # the plate swung from the cake's radius towards the arms (the reach check: margins 15-25 deg)
SIDE = math.radians(-90.0)  # where round the cake the edge stands: its right side
LEVER = 0.060  # m from the straight edge to where the hand pinches the plate
BEHIND = 0.060  # m behind the pinch, along the plate: the open fingertips 5 mm short of its far end, before they slide over it
OFF_BLOCK = 0.07  # m the plate is moved off the block, level, before the hand turns it over
OFF_BACK = math.radians(5.0)  # the plate is drawn out of the groove and put back along its own plane tilted this far off the back
DRAW = 0.020  # m it is drawn so: its foot then clears the lip by 7.7 mm as it is moved off the block, away from the back
BACK_GAP = 0.001  # m off the back the plate is lowered into the groove before it is let go
TURN_S, TURN_STEPS = 3.0, 12  # s the hand takes to turn the plate over between the block and the cake; poses on the way
OPEN, CLOSED = 0.6, 0.0  # the right hand's opening (0 closed .. 1 open, 94 mm)
OUT = 0.015  # m the edge stands off the icing before it goes in
DRAW_AWAY, DRAW_STROKES = 0.003, 2  # m the edge is drawn away while the cake turns on, over this many strokes, before it is taken clear. Drawn
# away over one stroke's push (1.3 s), the roll it carries filled 3 columns and stood 1.33 mm high (b2k46); over two, it
# fills about 8 columns and stands about 0.5 mm high (the fill rule's arithmetic: a cell is filled up to the edge)
ASIDE = 0.075  # m off the icing: the edge just outside the stand's rim (the plate wholly beside the top)
RIM_YAW = math.radians(-60.0)  # the left hand's turn about the vertical while it pushes the rim
DEPTH = 0.0005  # m under the mean surface the edge is set at; the held edge sits about 0.9 mm deeper than set (b2k46: 1.2 set, the side left 3.005 mm thick)
CLEAR_BOARD = 0.0005  # m the plate's lower corner stands over the board
SMOOTH_TURNS = 3.0
MOST_STROKES = 16
# The rim is pushed round from the far left to the near left: the cake turns anticlockwise seen from above and the
# icing comes at the edge from the side of the plate's free end. Turned the other way (b2k28), the cut's drag swung the
# held edge about the hand into the icing (4 mm in 0.5 s, down to the foam) and the plate shoved the cake 75 mm.
STROKE_FROM, STROKE_TO = math.radians(70.0), math.radians(165.0)
RIM_TOUCH, RIM_CLEAR = 0.0072, 0.020  # m from the rim's edge to the left hand's grasp point: the closed fingertips' side (7.7 mm) pressing 0.5 mm in; clear of it
PRESS = 4.0  # N the left fingertips press the rim's edge inward with while they push it round
TIP = 0.0129  # m: the closed fingertips' ends past the grasp point
STROKE_S, RETURN_S = 1.5, 1.0
UP = 0.03
DRY_TURN_PER_STROKE = 0.8 * (STROKE_TO - STROKE_FROM)  # rad the cake turns a stroke in a dry run


def rotation(closing, approach) -> np.ndarray:
    c = np.asarray(closing, dtype=float) / np.linalg.norm(closing)
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    return np.column_stack([c, np.cross(a, c), a])


class Plan:
    def __init__(self, m, see=None) -> None:
        import demo_kit as kit

        self.m, self.see, self.kit, self.Move = m, see, kit, kit.Move
        self.strokes = 0
        self.turned = 0.0  # rad the cake has turned since the edge was set (dry run: counted)

    # --- what is seen ---
    def pose(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        if self.see is not None:
            return self.see()["poses"][name]
        return self.m.mk.built_pose(name)

    def slot(self) -> tuple[np.ndarray, np.ndarray]:
        """The block's groove as the block stands: on its floor where the plate's middle plane stands against the back;
        its frame (x along, y across towards the back, z up the back)."""
        block_p, block_r = self.pose("block")
        return block_p + block_r @ self.m.SLOT_AT, block_r @ self.m.SLOT_TILT

    def spin(self) -> float:
        return float(self.see()["spin"]) if self.see is not None else self.strokes * DRY_TURN_PER_STROKE

    def surface(self) -> float:
        """The icing's mean thickness on the side (m): where a person sees the surface."""
        return float(self.see()["mean"]) if self.see is not None else self.m.THICK

    # --- the right hand on the smoother ---
    def hold_pose(self, plate_p, plate_r) -> tuple[np.ndarray, np.ndarray]:
        """The pinch on a plate standing at (plate_p, plate_r): its grasp point and the hand's rotation."""
        x, y = plate_r[:, 0], plate_r[:, 1]
        grasp = plate_p + (self.m.PLATE_W / 2 - LEVER) * x
        return grasp, rotation(y, x)  # closing along the plate's +y: the only side the wrist reaches at the block (b2k10)

    def plate_at_cake(self, distance: float, lift: float = CLEAR_BOARD) -> tuple[np.ndarray, np.ndarray]:
        """The plate's frame with its straight edge upright ``distance`` m from the foam on the cake's right side,
        swung SWING towards the arms, its lower corner ``lift`` m over the board. The plate is held turned over
        (its z down) there: the hand closes along its +y, and at the cake the wrist reaches only the closing side
        that this gives (b2k9); the plate is the same either way up."""
        m = self.m
        cake_p, _ = self.pose("cake")
        radial = np.array([math.cos(SIDE), math.sin(SIDE), 0.0])
        d = np.array([math.cos(SIDE + SWING), math.sin(SIDE + SWING), 0.0])  # from the edge into the plate
        edge = cake_p + (m.DUMMY_R + distance) * radial
        x, z = -d, np.array([0.0, 0.0, -1.0])
        r = np.column_stack([x, np.cross(z, x), z])
        centre = edge + (m.PLATE_W / 2) * d + np.array([0.0, 0.0, m.EDGE_L / 2 + lift])
        return centre, r

    def right_at(self, distance: float, lift: float = CLEAR_BOARD) -> tuple[np.ndarray, np.ndarray]:
        return self.hold_pose(*self.plate_at_cake(distance, lift))

    # --- the left hand on the rim ---
    def rim_point(self, angle: float, up: float = 0.0, off: float = RIM_TOUCH) -> np.ndarray:
        """The left hand's grasp point beside the stand's rim at ``angle``: ``off`` m out from the rim's edge (at
        RIM_TOUCH the closed fingertips' side touches it), the fingertips' ends 4 mm under the top's lower face, so
        their sides stand against its 3 mm edge; ``up`` m higher."""
        m = self.m
        top_p, _ = self.pose("turntable")
        r = m.TOP_R + off
        return np.array([top_p[0] + r * math.cos(angle), top_p[1] + r * math.sin(angle), top_p[2] - m.TOP_T / 2 - 0.004 + TIP + up])

    @staticmethod
    def inward(angle: float, newtons: float) -> tuple[float, float, float]:
        return (-newtons * math.cos(angle), -newtons * math.sin(angle), 0.0)

    def stroke(self) -> list:
        """One push of the rim from outside, from STROKE_FROM to STROKE_TO: the hand pointing down, its fingers closed,
        comes down beside the rim, touches its edge, presses PRESS inward and goes round with it; then out, up and
        back. Its body stays 14 mm or more from the iced side (b2k25); pressing on the top instead, it stood over the
        cake and shoved it (b2k20)."""
        Move, rot = self.Move, self.kit.down(RIM_YAW)
        a0 = STROKE_FROM
        moves = [Move(0.6, pos=self.rim_point(a0, up=UP, off=RIM_CLEAR), rot=rot, grip=CLOSED, label="over the rim"), Move(0.5, pos=self.rim_point(a0, off=RIM_CLEAR), label="down beside the rim"),
                 Move(0.4, pos=self.rim_point(a0), label="to the rim"), Move(0.3, push=self.inward(a0, PRESS), push_at=(0.0, 0.0, TIP - 0.0055), label="press")]  # fmt: skip
        n = 7
        for i in range(1, n + 1):
            a = STROKE_FROM + (STROKE_TO - STROKE_FROM) * i / n
            moves.append(Move(STROKE_S / n, pos=self.rim_point(a), push=self.inward(a, PRESS), label="push round"))
        a1 = STROKE_TO
        moves += [Move(0.2, push=(0.0, 0.0, 0.0), label="ease"), Move(0.3, pos=self.rim_point(a1, off=RIM_CLEAR), label="off the rim"), Move(0.4, pos=self.rim_point(a1, up=UP, off=RIM_CLEAR), label="lift")]
        back = 5  # back round the rim outside it: straight across, the hand's body would pass over the cake's side
        for i in range(1, back + 1):
            a = STROKE_TO + (STROKE_FROM - STROKE_TO) * i / back
            moves.append(Move(RETURN_S / back, pos=self.rim_point(a, up=UP, off=RIM_CLEAR), label="back"))
        return moves

    @staticmethod
    def length(moves: list) -> float:
        return sum(mv.seconds for mv in moves)

    # --- the run, phase by phase: each phase is {side: moves}, both arms start it together ---
    def phases(self, name: str):
        m, Move = self.m, self.Move
        # take the smoother out of its block: the hand turned as the plate leans, slid over it from behind, closed,
        # and the plate drawn up its own plane, OFF_BACK off the back, until its foot is out of the groove
        plate_p, plate_r = self.pose("smoother")
        grasp, rot = self.hold_pose(plate_p, plate_r)
        behind = grasp - BEHIND * plate_r[:, 0]  # the open fingers' tips short of the plate's far end
        out_of_groove = grasp + self.draw(plate_r)
        yield {"right": [Move(3.0, pos=behind, rot=rot, grip=OPEN, label="behind the smoother"),
                         Move(1.5, pos=grasp, label="slide over the plate"), Move(0.6, grip=CLOSED, label="close"), Move(1.2, pos=out_of_groove, label="out of the groove")]}  # fmt: skip
        if name == "take":
            yield {"right": [Move(1.0, pos=out_of_groove + OFF_BLOCK * self.off_block_way(), label="off the block"), Move(1.5, label="holds it")]}
            return
        # over to the cake: first beside the stand's rim at the height it will work at, then in over the top, then down
        # onto the board beside the icing (carried straight, the plate's lower edge would cross the rim below its top)
        mean = self.surface()
        distance = mean - DEPTH
        aside_grasp, aside_rot = self.right_at(mean + ASIDE, lift=0.03)
        off_block = out_of_groove + OFF_BLOCK * self.off_block_way()
        over = [Move(TURN_S / TURN_STEPS, pos=pos, rot=r, label="turns it over") for pos, r in self.turn_path(off_block, rot, aside_grasp, aside_rot)]
        yield {"right": [Move(1.0, pos=off_block, label="off the block"), *over, Move(0.3, label="beside the stand"), Move(1.2, pos=self.right_at(mean + OUT, lift=0.03)[0], label="in over the top"),
                         Move(1.0, pos=self.right_at(mean + OUT)[0], label="down to the board"), Move(1.5, pos=self.right_at(distance)[0], label="edge in")]}  # fmt: skip
        start_spin = self.spin()
        while self.strokes < MOST_STROKES and abs(self.spin() - start_spin) < SMOOTH_TURNS * 2.0 * math.pi:
            self.strokes += 1
            left = self.stroke()
            yield {"left": left, "right": [Move(self.length(left), label="holds the edge")]}
        # the take-off: drawn away along the turning, a share in each push of DRAW_STROKES more strokes (the edge moves
        # only while the cake turns: drawn out with the cake standing, the roll would fill one column)
        left_ready, right_ready = self.ready("left"), self.ready("right")
        for i in range(1, DRAW_STROKES + 1):
            left = self.stroke()
            push_s = self.length(left[:3])
            right = [Move(push_s + 0.2, label="holds the edge"), Move(STROKE_S - 0.2, pos=self.right_at(distance + DRAW_AWAY * i / DRAW_STROKES)[0], label="draws away along the turning")]
            if i == DRAW_STROKES:
                right.append(Move(0.5, pos=self.right_at(distance + 0.03)[0], label="clear"))
            else:
                right.append(Move(self.length(left) - push_s - STROKE_S, label="holds the edge"))
            yield {"left": left, "right": right}
        yield {"left": [Move(1.5, pos=left_ready[0], rot=left_ready[1], label="away")]}
        # back into the block: up, out beside the rim, turned over again on the way, over the groove leaning as the
        # back leans, down the line it was drawn out along to BACK_GAP off the back; let go, it settles onto the back;
        # the open fingers drawn back off the plate's far end, the way they came
        slot_p, slot_r = self.slot()
        home = slot_p + slot_r @ np.array([0.0, -BACK_GAP, m.EDGE_L / 2 + 0.0005])
        home_grasp, home_rot = self.hold_pose(home, slot_r)
        above = home_grasp + self.draw(slot_r)
        aside_back = self.right_at(mean + ASIDE, lift=0.03)
        beside_block = above + OFF_BLOCK * self.off_block_way()
        back = [Move(TURN_S / TURN_STEPS, pos=pos, rot=r, label="turns it back") for pos, r in self.turn_path(*aside_back, beside_block, home_rot)]
        yield {"right": [Move(1.0, pos=self.right_at(mean + OUT, lift=0.03)[0], label="up"), Move(1.2, pos=aside_back[0], label="out beside the stand"),
                         *back, Move(1.0, pos=above, label="over the groove"), Move(1.5, pos=home_grasp, label="into the groove"), Move(0.5, grip=OPEN, label="let go"),
                         Move(1.5, pos=home_grasp - BEHIND * slot_r[:, 0], label="back off the plate"), Move(2.0, pos=right_ready[0], rot=right_ready[1], label="away")]}  # fmt: skip
        yield {"left": [Move(1.0, label="away")], "right": [Move(1.0, label="away")]}

    def draw(self, plate_r) -> np.ndarray:
        """The way the plate is drawn out of the groove (and put back, reversed): DRAW up its own plane, OFF_BACK off
        the back. Drawn out of a slot 3 mm wide between two jaws instead, it jammed and the block came away on it
        (b2k20, b2k28, b2k30)."""
        return DRAW * (math.cos(OFF_BACK) * plate_r[:, 2] - math.sin(OFF_BACK) * plate_r[:, 1])

    def off_block_way(self) -> np.ndarray:
        """Level, square to the plate standing in the block, away from the arm's base: the way the plate is taken
        off the block before it is turned over (turned over just out of the slot, it swept through the block: b2k18)."""
        _, block_r = self.pose("block")
        way = -block_r[:, 1]
        way[2] = 0.0
        return way / np.linalg.norm(way)

    @staticmethod
    def solved(pos, rot, start=None) -> np.ndarray:
        """The right arm's joints for the grasp point ``pos`` turned ``rot``: of several starts, the solution nearest
        ``start`` among those that reach it (or the best reach)."""
        from yam_arena import ik, station

        base = station.BASES["right"]
        starts = [np.array(v, dtype=float) for v in ([0.0, 1.0, 1.0, -0.6, 0.0, 0.0], [0.0, 1.6, 1.4, -0.4, 0.0, 0.0], [0.0, 0.6, 0.6, -1.2, 0.0, 0.0], [0.0, 1.2, 1.8, 0.4, 0.0, 0.0])]
        if start is not None:
            starts.insert(0, np.asarray(start, dtype=float))
        found = []
        for s0 in starts:
            q, err, rerr = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), s0, base_pos=base, iters=200)
            found.append((err > 0.0005 or rerr > 0.005, 0.0 if start is None else float(np.abs(q - start).max()), err + rerr, q))
        return min(found, key=lambda f: (f[0], f[1], f[2]))[3]

    def turn_path(self, p0, r0, p1, r1) -> list:
        """The hand's poses along a straight line in the arm's joint space from one grip to the other: there the hand
        turns the plate over (the block's grip and the cake's are turned about 150 degrees from each other about the
        pointing axis, and the wrist reaches each only one way round: b2k10, b2k9); a straight blend of the poses
        instead took a joint onto its stop (b2k13, b2k14)."""
        from yam_arena import ik, station

        base = station.BASES["right"]
        q0 = self.solved(p0, r0)
        q1 = self.solved(p1, r1, start=q0)
        return [ik.tcp_pose(q0 + (q1 - q0) * i / TURN_STEPS, base) for i in range(1, TURN_STEPS + 1)]

    def ready(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        """The hand's grasp point and turn in the arm's ready pose."""
        from yam_arena import ik, station
        from yam_arena.constants import READY

        return ik.tcp_pose(np.array(READY, dtype=float), station.BASES[side])

    def whole(self, name: str) -> dict:
        """Both arms' moves of run ``name`` (dry run: no simulator), phases kept in step."""
        moves = {"left": [], "right": []}
        for phase in self.phases(name):
            common = max(self.length(moves["left"]), self.length(moves["right"]))
            for side in moves:
                pad = common - self.length(moves[side])
                if pad > 1e-9:
                    moves[side].append(self.Move(pad, label="waits"))
                moves[side].extend(phase.get(side, []))
        return moves


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
        m.begin(0)
        for run in runs:
            moves = Plan(m).whole(run)
            print(f"DRY {run}: {json.dumps(kit.dry_run(moves, station.BASES, limit_s=m.RUN_LIMIT_STEPS / 20.0), default=str)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    failed = []
    for run in runs:
        try:

            def seen() -> dict:
                return {"poses": {name: mk.pose(name) for name in ("smoother", "block", "cake", "turntable")}, "spin": mk.joint("spin"), "mean": float(m.band().mean())}

            plan = Plan(m, seen)
            script = plan.phases(run)
            state = {"over": False}

            def director(t: float, arms: dict, pictures, script=script, state=state, plan=plan) -> None:
                if pictures is None or mk.MECH.environment is None or not mk.MECH.poses:
                    return
                while not state["over"] and t >= max(arm.seconds for arm in arms.values()) - 1e-6:
                    phase = next(script, None)
                    if phase is None:
                        state["over"] = True
                        break
                    common = max(arm.seconds for arm in arms.values())
                    for side, arm in arms.items():
                        pad = common - arm.seconds
                        if pad > 1e-9:
                            arm.moves.append(plan.Move(pad, label="waits"))
                        arm.moves.extend(phase.get(side, []))

            def extra() -> dict:
                r = m.report()
                return {"gap under the rule (mm)": r["largest gap, rule upright (mm)"], "gap under the gauge (mm)": r["largest gap, arc gauge (mm)"], "pins (mm)": r["pin readings' mean (mm)"],
                        "mean (mm)": r["mean thickness of the band (mm)"], "thinnest, thickest (mm)": r["thinnest, thickest (mm)"],
                        "marks with ridges": m.MARKS - r["marks with the rule's gap under 1.5 mm"], "worked": r["columns worked"], "bare": r["bare cells"], "flaw": r["one flaw"],
                        "roll (mm3)": r["roll on the edge (mm3)"], "on the blade (mm3)": r["on the blade's face (mm3)"], "spin (deg)": round(math.degrees(mk.joint("spin")), 1),
                        "cut force (N)": [round(float(v), 3) for v in m.STATE["force"]], "contact rows": len(m.STATE["contact"] or {}),
                        "edge from the foam, row 17 (mm)": None if 17 not in (m.STATE["contact"] or {}) else round(1000 * m.STATE["contact"][17][1], 3),
                        "in the block": r["in the block"], "hands clear (mm)": r["hands clear (mm)"], "cake off (mm)": r["cake off the top's axis (mm)"], "level": m.level(),
                        "block from its place (mm)": [round(1000 * float(v), 2) for v in mk.pose("block")[0] - np.asarray(mk.built_pose("block")[0], dtype=float)],
                        "smoother from its place (mm)": [round(1000 * float(v), 2) for v in mk.pose("smoother")[0] - np.asarray(mk.built_pose("smoother")[0], dtype=float)]}  # fmt: skip

            folder = out / run
            result = kit.run(embodiment, {"left": [], "right": []}, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=1.0, sheet_frames=16)
            rows = result["rows"]
            scores = [row["info"].get("score", 0.0) for row in rows]
            firsts = {level: next((row["t"] for row, s in zip(rows, scores) if s >= level), None) for level in (0.25, 0.5, 0.75, 1.0)}
            parts = m.regions()
            readings = {"score": scores[-1], "first reached": firsts, "seconds": rows[-1]["t"], "ended": result.get("termination_reason"), "strokes": plan.strokes,
                        "largest gap under the rule or the gauge: plain side, over the scar (mm)": [round(1000 * parts["plain side"], 3), round(1000 * parts["over the scar"], 3)],
                        "report": result["report"], "worst IK": result["worst_ik"], "end": rows[-1]["extra"]}  # fmt: skip
            (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
            np.save(folder / "field.npy", m.STATE["field"])
            print(f"DEMO {run}: score {scores[-1]:.2f} success {result['terminated'] and result['termination_reason'] == 'success'} after {result['seconds']:.1f} s ({result['steps']} steps of {m.RUN_LIMIT_STEPS}); "
                  f"first reached {firsts}; report {result['report']}", flush=True)  # fmt: skip
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"RUN {run} FAILED: {error!r}", flush=True)
            failed.append(run)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
