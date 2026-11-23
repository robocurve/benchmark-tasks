"""Scripted demo for secure-the-cutting-board-on-a-mat, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the board's state every 10 steps

full        Both arms pinch the board's end handles, lift it 5 cm, carry it toward the robot over the non-slip mat,
            set it down on the mat and let go.
edge_on     Both arms pinch the handles, lift the board 15 cm, turn it about the line through the handles toward
            standing on edge, carry it toward the mat that way, turn it back level above the mat, set it down and
            let go (how the board is held on the way is not scored). The turn and the share of the carry done
            tilted are the first in EDGE_ORDER (90 deg and the whole carry first) at which both arms' IK stays on
            target along the whole plan, turning either way (recorded: 70 deg, far edge up, for the whole carry;
            at most 69.28 deg measured; 1.0).
Failure and cheating variants:
drag        the right arm alone slides the board by its handle across the counter onto the mat (expected: the mat is
            pushed away)
one_hand    the right arm alone lifts the board by its handle and carries it to the mat (recorded: carried 8 deg
            tilted and set down short, 40% over the mat, 0.5; a one-arm carry is possible)
half_on     both arms set the board down 7 cm short, partly over the mat's edge (0.5)
drop        both arms let go of the board 5 cm above the mat (0)
"""

from __future__ import annotations

import math
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "edge_on", "drag", "one_hand", "half_on", "drop")
_CACHE: dict = {}
LIFT = 0.050
EDGE_LIFT = 0.150  # m: edge_on lifts the handles this high, so the board turned on edge clears the counter and mat
# edge_on: (turn about the handle line in deg, share of the carry done on edge), in the order tried
EDGE_ORDER = [(90, 1.0), (80, 1.0), (70, 1.0), (90, 0.5), (80, 0.5), (70, 0.5),
              (60, 1.0), (60, 0.5), (50, 1.0), (50, 0.5), (40, 1.0), (40, 0.5)]
EDGE_TOL = 0.002  # worst IK error accepted at edge_on's waypoints (m + 0.05 rad)
EDGE_JUMP = 0.30  # rad: largest joint step between control steps accepted along edge_on's plan (no branch flips)


def _closing(axis: int):
    return [R for tl in (0, 10, 20, 30) for az in range(0, 360, 45) for roll in (0, 90)
            for R in [dc.R_approach(tl, az, roll)] if abs(R[axis, 0]) >= 0.95]


def handle_offset(t, side: str) -> np.ndarray:
    sy = 1.0 if side == "left" else -1.0
    return np.array([0.0, sy * (t.EAR_Y + 0.004), t.EAR_Z])


def _ry(deg: float) -> np.ndarray:
    """Rotation by ``deg`` about the world y axis: the line through the two handles (and the two TCPs)."""
    return dc.rot((0.0, 1.0, 0.0), math.radians(deg))


def add_edge_on(A, t, side: str, b0, b1, R, tilt: float, f_end: float, home) -> None:
    """edge_on's segments for one arm. Each TCP sits on the handle line, so turning the board about that line only
    turns the grippers (R -> Ry(tilt) @ R) and leaves the TCPs in place."""
    g = handle_offset(t, side)
    up0, up1 = b0 + g + [0, 0, EDGE_LIFT], b1 + g + [0, 0, EDGE_LIFT]
    upm = up0 + f_end * (up1 - up0)
    Rt = _ry(tilt) @ R
    A.add(2.5, b0 + g + [0, 0, 0.06], R, 1.0, "board: above the handle")
    A.add(1.5, b0 + g, R, 1.0, "board: around the handle")
    A.add(1.0, b0 + g, R, 0.0, "board: grip")
    A.add(3.5, up0, R, 0.0, "board: lift high")
    A.add(0.5, up0, R, 0.0, "board: steady")
    A.add(3.0, up0, Rt, 0.0, "board: turn on edge")
    A.add(3.0 * f_end, upm, Rt, 0.0, "board: carry on edge")
    A.add(3.0, upm, R, 0.0, "board: turn level")
    if f_end < 1.0:
        A.add(3.0 * (1.0 - f_end), up1, R, 0.0, "board: over the mat")
    A.add(3.0, b1 + g + [0, 0, 0.002], R, 0.0, "board: down")
    A.add(1.2, b1 + g - [0, 0, 0.008], R, 0.0, "board: onto the mat")
    A.add(0.5, b1 + g - [0, 0, 0.008], R, 0.0, "board: settle onto the mat")
    last = b1 + g - [0, 0, 0.001]
    A.add(0.8, grip=1.0, label="board: let go")
    A.add(1.5, last + [0, 0, 0.05], R, 1.0, "board: up")
    A.add(2.0, home[0], home[1], 1.0, f"{side}: home")


def _edge_points(t, side: str, b0, b1, f_end: float):
    """edge_on's TCP waypoints for one arm: (position, fraction of the turn applied)."""
    g = handle_offset(t, side)
    up0, up1 = b0 + g + [0, 0, EDGE_LIFT], b1 + g + [0, 0, EDGE_LIFT]
    upm = up0 + f_end * (up1 - up0)
    level = [b0 + g + [0, 0, 0.06], b0 + g, up0, upm, up1, b1 + g + [0, 0, 0.002], b1 + g - [0, 0, 0.008],
             b1 + g + [0, 0, 0.049]]
    turned = ([(up0, u) for u in (1 / 3, 2 / 3, 1.0)] + [(up0 + f * (upm - up0), 1.0) for f in (0.5, 1.0)]
              + [(upm, u) for u in (2 / 3, 1 / 3)])
    return [(p, 0.0) for p in level] + turned


def _points_ok(side: str, ri: int, R, tilt: float, pts) -> bool:
    memo = _CACHE.setdefault("edge_memo", {})
    for p, u in pts:
        key = (side, ri, tuple(np.round(p, 5)), round(u * tilt, 3))
        if key not in memo:
            memo[key] = dc.best_rotation(side, [p], [_ry(u * tilt) @ R])[1]
        if memo[key] > EDGE_TOL:
            return False
    return True


def _path_ok(t, side: str, b0, b1, R, tilt: float, f_end: float):
    """Solve IK step by step along the whole edge_on plan, as the run does: on target, and no joint jumps."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    start = ik.tcp_pose(np.array(READY), station.BASES[side])
    A = dc.Arm(side, station.BASES[side], READY, *start)
    add_edge_on(A, t, side, b0, b1, R, tilt, f_end, start)
    q_prev, jump = None, 0.0
    for k in range(int(A.duration * dc.CONTROL_HZ) + 1):
        a, _ = A.action7(k / dc.CONTROL_HZ)
        if q_prev is not None:
            jump = max(jump, float(np.abs(a[:6] - q_prev).max()))
        q_prev = a[:6]
    mm, deg = max(v[0] for v in A.worst.values()), max(v[1] for v in A.worst.values())
    return mm <= 3.0 and deg <= 2.0 and jump <= EDGE_JUMP, (mm, deg, jump)


def edge_on_plan(t, b0, b1, G):
    """One turn for both arms (they hold one rigid board), the share of the carry done on edge, and each arm's grip
    rotation, tried in EDGE_ORDER, either way about the handle line; the full demo's grip is tried before the other
    closing rotations."""
    if "edge" in _CACHE:
        return _CACHE["edge"]
    t0 = time.time()
    for tilt, f_end in EDGE_ORDER:
        for sgn in (-1, 1):
            pick = {}
            for side in ("left", "right"):
                pts = _edge_points(t, side, b0, b1, f_end)
                cands = [G[side]] + [R for R in _closing(0) if not np.allclose(R, G[side])]
                for ri, R in enumerate(cands):
                    if not _points_ok(side, ri, R, sgn * tilt, pts):
                        continue
                    ok, info = _path_ok(t, side, b0, b1, R, sgn * tilt, f_end)
                    print(f"  edge_on {side}: turn {sgn * tilt:+d} deg, {f_end:.0%} of the carry on edge, grip "
                          f"approach {np.round(R[:, 2], 2).tolist()}: path IK worst {info[0]:.1f} mm {info[1]:.1f} "
                          f"deg, largest joint step {info[2]:.2f} rad -> {'ok' if ok else 'rejected'}", flush=True)
                    if ok:
                        pick[side] = R
                        break
                if side not in pick:
                    break
            if len(pick) == 2:
                _CACHE["edge"] = (sgn * tilt, f_end, pick["left"], pick["right"])
                print(f"edge_on: turn {sgn * tilt:+d} deg about the handle line, {f_end:.0%} of the carry on edge "
                      f"(search {time.time() - t0:.0f} s)", flush=True)
                return _CACHE["edge"]
    raise RuntimeError("edge_on: no turn in EDGE_ORDER keeps both arms' IK on target")


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    b0 = np.array(t.BOARD_POS)
    b1 = np.array([*t.MAT_XY, t.TOP + t.MAT[2] + 0.0005]) + ([0.07, 0, 0] if variant == "half_on" else [0, 0, 0])
    if "grips" not in _CACHE:
        _CACHE["grips"] = {}
        for side in ("left", "right"):
            g = handle_offset(t, side)
            pts = [b0 + g, b0 + g + [0, 0, LIFT], b1 + g + [0, 0, LIFT], b1 + g, b1 + g + [0.12, 0, 0],
                   np.array([*t.MAT_XY, t.TOP]) + g]
            _CACHE["grips"][side], e = dc.best_rotation(side, pts, _closing(0))
            print(f"{side} handle grip chosen: worst IK error {1000 * e:.1f} (mm + 0.05 rad)", flush=True)
    G = _CACHE["grips"]
    if variant == "edge_on":
        tilt, f_end, RL, RR = edge_on_plan(t, b0, b1, G)
        for side, R in (("left", RL), ("right", RR)):
            add_edge_on(arms[side], t, side, b0, b1, R, tilt, f_end, starts[side])
        return arms["left"], arms["right"], {}
    users = ("right",) if variant in ("drag", "one_hand") else ("left", "right")
    for side in users:
        A, g, R = arms[side], handle_offset(t, side), G[side]
        A.add(2.5, b0 + g + [0, 0, 0.06], R, 1.0, "board: above the handle")
        A.add(1.5, b0 + g, R, 1.0, "board: around the handle")
        A.add(1.0, b0 + g, R, 0.0, "board: grip")
        if variant == "drag":
            A.add(4.0, np.array([*t.MAT_XY, b0[2]]) + g, R, 0.0, "board: slide")
            last = np.array([*t.MAT_XY, b0[2]]) + g
        else:
            A.add(1.5, b0 + g + [0, 0, LIFT], R, 0.0, "board: lift")
            A.add(3.0, b1 + g + [0, 0, LIFT], R, 0.0, "board: over the mat")
            if variant == "drop":
                A.add(1.0, b1 + g + [0, 0, 0.05], R, 0.0, "board: hold")
                last = b1 + g + [0, 0, 0.05]
            else:
                A.add(1.8, b1 + g + [0, 0, 0.002], R, 0.0, "board: down")
                A.add(1.2, b1 + g - [0, 0, 0.008], R, 0.0, "board: onto the mat")
                A.add(0.5, b1 + g - [0, 0, 0.008], R, 0.0, "board: settle onto the mat")
                last = b1 + g - [0, 0, 0.001]
        A.add(0.8, grip=1.0, label="board: let go")
        A.add(1.5, last + [0, 0, 0.05], R, 1.0, "board: up")
        A.add(2.0, starts[side][0], starts[side][1], 1.0, f"{side}: home")
    return arms["left"], arms["right"], {}


def servos_for(name, arm):
    return []


def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "onto the mat", "slide"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    print(f"  dbg {k}: share {r.get('mat_share')} tilt {r.get('tilt')} h {r.get('height_over_mat')} max "
          f"{r.get('max_lift_tilt')} mat_moved {r.get('mat_moved')} rel {r.get('released')} drop {r.get('drop_flag')} "
          f"| {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
