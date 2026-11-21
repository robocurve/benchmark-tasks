"""Contact gate: two-pad pickup of the guide-supported free end, then extraction.

Right arm only (left holds READY). Approach +x along the strip, fingers closing
along world z: the upper pad lands on the printed face, the lower pad rises
through the slot between the exit-guide prongs. The jaw centre is 4 mm above
the resting sheet so the lower pad lifts it off the prongs and the sheet
centres between the pads. Robot joint targets only.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from task import TAG_Y  # noqa: E402

GRASP = (.4185, TAG_Y, .984)
PRE = (.340, TAG_Y, .984)
TUG = (.3485, TAG_Y, .984)
CLEAR = (.180, TAG_Y, 1.340)
MOUTH_X = .500
# Close begins after pre-grasp, settle and slide; hold ends the pinch phase.
CLOSE_START_S = 1.5 + .5 + 1.0
HOLD_END_S = CLOSE_START_S + 1.2 + .6
TUG_END_S = HOLD_END_S + 1.5 + .5


def pickup_waypoints(hold=1.0):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from bagtag_probe import W, frame
    R = frame((1, 0, 0), (0, 0, 1))
    return [
        W(1.5, PRE, R, FINGER_OPEN, "pre-grasp short of free end"),
        W(.5, PRE, R, FINGER_OPEN, "settle before slide"),
        W(1.0, GRASP, R, FINGER_OPEN, "straddle sheet and prongs"),
        W(1.2, GRASP, R, FINGER_CLOSED, "close pads through guide slot"),
        W(.6, GRASP, R, FINGER_CLOSED, "hold pinch"),
        W(1.5, TUG, R, FINGER_CLOSED, "tug to release presenter"),
        W(.5, TUG, R, FINGER_CLOSED, "hold after tug"),
        W(3.0, CLEAR, R, FINGER_CLOSED, "draw stock out of channel"),
        W(hold, CLEAR, R, FINGER_CLOSED, "hold extracted tag"),
    ]


def panel_frame(xyz, quat):
    from extract_pose_probe import panel_axes
    ax = panel_axes(quat)
    return np.asarray(xyz), np.column_stack((ax["length"], ax["width"], ax["normal"]))


def pinch_metrics(state):
    """Panel09 relative to the actual (FK) TCP: closing-axis offset, in-plane offsets, normal alignment."""
    from yam_arena import ik, station
    q = np.asarray(state["robots"]["right"]["joints"])
    tcp, R = ik.tcp_pose(q[:6], station.BASES["right"])
    c, P = panel_frame(state["panel_xyz"][9], state["panel_quat_xyzw"][9])
    local = R.T @ (c - tcp)
    free_end = c - .0215 * P[:, 0]
    return {
        "tcp_actual": tcp.round(4).tolist(),
        "fingers": np.round(q[6:8], 5).tolist(),
        "panel09_centre": c.round(4).tolist(),
        "panel09_free_end": free_end.round(4).tolist(),
        "panel09_in_tcp_frame_mm": (local * 1000).round(2).tolist(),
        "panel09_normal_dot_closing": round(float(abs(P[:, 2] @ R[:, 0])), 4),
        "panel09_to_tcp_mm": round(float(np.linalg.norm(c - tcp) * 1000), 2),
    }


def main(out: Path):
    from yam_arena import station
    from bagtag_probe import Plan, Run, hold_ready

    right = Plan(pickup_waypoints(), station.BASES["right"])
    run = Run(out, "exit_guide_pickup_probe")
    try:
        run.trace.append(run.state(("reset", "reset")))
        run.snap("reset")
        steps = round(right.duration * 20)
        hold_step = round(HOLD_END_S * 20)
        tug_step = round(TUG_END_S * 20)
        snaps = (round(CLOSE_START_S * 20), hold_step, tug_step, tug_step + 30, steps)
        run.go(steps, hold_ready(), right.action7, every=2, snaps=snaps)
        reset, final = run.trace[0], run.trace[-1]
        hold = next(s for s in run.trace if s["step"] == hold_step)
        tug = next(s for s in run.trace if s["step"] == tug_step)
        p0 = np.asarray(reset["panel_xyz"][0])
        h, f = pinch_metrics(hold), pinch_metrics(final)

        def pinched(m):
            return (all(j <= -.0003 for j in m["fingers"])
                    and abs(m["panel09_in_tcp_frame_mm"][0]) < 2.0
                    and m["panel09_normal_dot_closing"] > .97)

        summary = {
            "stage": "exit_guide_pickup_contact_gate_not_scored",
            "steps": steps,
            "right_worst_ik_mm": right.worst_ik_mm,
            "right_worst_ik_deg": right.worst_ik_deg,
            "reset_panel_xyz": [np.round(p, 4).tolist() for p in reset["panel_xyz"]],
            "reset_panel09": pinch_metrics(reset)["panel09_free_end"],
            "hold_step": hold_step,
            "hold": h,
            "tug_step": tug_step,
            "panel00_moved_after_tug_mm": round(float(np.linalg.norm(
                np.asarray(tug["panel_xyz"][0]) - p0) * 1000), 2),
            "final": f,
            "panel00_moved_final_mm": round(float(np.linalg.norm(
                np.asarray(final["panel_xyz"][0]) - p0) * 1000), 2),
            "two_pad_pinch_at_hold": pinched(h),
            "two_pad_pinch_final": pinched(f),
            "full_extraction_conservative": all(p[0] + .0215 < MOUTH_X for p in final["panel_xyz"]),
        }
        run.save(summary)
    finally:
        run.close()


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    main(Path(sys.argv[1]))
