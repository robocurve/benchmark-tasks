"""The scripted bike-light job (the reference run): the left hand switches the light off where it sits caught on the
bracket (closed fingertips on the button from above, held 1.6 s), then pinches it across its 31 mm; the right hand
presses the bracket's tab down from behind; the left draws the light 45 mm forward off the rail (it is free at
30 mm), the right lets go; the left lifts it over to the bowl, lowers it in and lets go; both hands go back.

    scripts/rta simpy <id> mech/tasks/bike_light/demo.py outputs/<id> dry|trial|record [seed]

``dry``: the plan through the IK only (worst IK error, least joint margin, per arm). ``trial``: in the simulator,
no videos, with per-step readings (light, joints, both grasp poses) in result.json. ``record``: the same with videos.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

PRESS_TAB = 5.0  # N down on the tab (3 N at 2 mm, its stop at 3 mm takes the rest)
PRESS_BUTTON = 4.0  # N down on the button (3 N at its 1 mm stop)
SLIDE = 0.045  # m the light is drawn forward (free at 30 mm)
LIFT = np.array([0.0, 0.0, 0.040])
OVER_BOWL = 0.050  # m the light's underside clears the bowl's rim on the way over
IN_BOWL = 0.018  # m the light's underside over the bench where the hand lets it go (it rests at about 15 mm)
OPEN_IN_BOWL = 0.6  # the hand's opening to let go in the bowl (56 mm between the pads: clear of the bowl's wall)


def ready(side: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.asarray(READY[:6], dtype=float), station.BASES[side])


def moves(m) -> dict:
    import demo_kit as kit
    import plan

    rl, rr = plan.light_rot(), plan.lever_rot()
    at_l, at_r = kit.lowest_point(rl), kit.lowest_point(rr)
    button = plan.button_touch(m)
    pinch = plan.light_grasp(m)
    tab = plan.lever_touch(m)
    tab_down = tab + np.array([0.0, 0.0, -(plan.ABOVE + m.LEVER_TRAVEL)])
    bowl = np.array([*m.BOWL_XY, m.TOP])
    carry_z = m.TOP + m.BOWL_H + OVER_BOWL + m.LIGHT[2] / 2
    home_l, home_r = ready("left"), ready("right")
    left = [
        kit.Move(4.0, pos=button + np.array([0.0, 0.0, 0.025]), rot=rl, grip=0.0, label="over the button"),
        kit.Move(1.5, pos=button, label="onto the button"),
        kit.Move(0.5, pos=button + np.array([0.0, 0.0, -(plan.ABOVE + m.BUTTON_TRAVEL)]), push=(0.0, 0.0, -PRESS_BUTTON), push_at=at_l, label="press the button"),
        kit.Move(1.6, label="hold the button"),
        kit.Move(0.5, pos=button, push=(0.0, 0.0, 0.0), label="let the button go"),
        kit.Move(1.0, pos=button + np.array([0.0, 0.0, 0.02]), label="up off the button"),
        kit.Move(2.0, pos=pinch + np.array([0.0, 0.0, 0.04]), grip=0.6, label="over the light"),
        kit.Move(1.5, pos=pinch, label="down to the light"),
        kit.Move(0.8, grip=0.0, label="pinch the light"),
        kit.Move(3.0, label="wait for the tab"),
        kit.Move(2.5, pos=pinch + np.array([SLIDE, 0.0, 0.0]), label="draw the light off"),
        kit.Move(1.5, pos=pinch + np.array([SLIDE, 0.0, 0.0]) + LIFT, label="lift the light"),
        kit.Move(3.0, pos=np.array([bowl[0], bowl[1], carry_z]), label="over the bowl"),
        kit.Move(2.0, pos=np.array([bowl[0], bowl[1], m.TOP + IN_BOWL + m.LIGHT[2] / 2]), label="into the bowl"),
        kit.Move(0.8, grip=OPEN_IN_BOWL, label="let the light go"),
        kit.Move(1.5, pos=np.array([bowl[0], bowl[1], carry_z + 0.03]), label="up from the bowl"),
        kit.Move(3.5, pos=home_l[0], rot=home_l[1], grip=1.0, label="back"),
    ]
    right = [
        kit.Move(9.0, label="wait"),
        kit.Move(4.0, pos=tab - 0.06 * rr[:, 2], rot=rr, grip=0.0, label="behind the tab"),
        kit.Move(1.5, pos=tab, label="onto the tab"),
        kit.Move(0.6, pos=tab_down, push=(0.0, 0.0, -PRESS_TAB), push_at=at_r, label="press the tab"),
        kit.Move(5.4, label="hold the tab"),
        kit.Move(0.5, pos=tab, push=(0.0, 0.0, 0.0), label="let the tab go"),
        kit.Move(1.0, pos=tab - 0.06 * rr[:, 2], label="back off the tab"),
        kit.Move(3.5, pos=home_r[0], rot=home_r[1], grip=1.0, label="back"),
    ]
    return {"left": left, "right": right}


CUE_TIMES = (2.0, 3.0, 4.0, 8.6, 10.0, 11.1, 12.6)  # s: the cameras' pictures saved at the policy's size (on before 6.9 s, off after)


def cue_saver(out: Path):
    """A director that only looks: it saves the left wrist and top cameras' pictures at CUE_TIMES (policy size)."""
    import cv2
    import demo_kit as kit

    done: set = set()

    def director(t, arms, pictures) -> None:
        if pictures is None:
            return
        for c in CUE_TIMES:
            if c not in done and t >= c - 1e-9:
                done.add(c)
                for cam in ("left_cam", "top_cam"):
                    cv2.imwrite(str(out / f"cue_{cam}_{c:.1f}s.png"), cv2.cvtColor(kit.policy_picture(pictures[cam]), cv2.COLOR_RGB2BGR))

    return director


def readings(m) -> dict:
    mk = m.mk
    p, r = mk.pose("light")
    return {
        "light": [round(float(v), 5) for v in p], "light_quat": [round(float(v), 5) for v in mk.matrix_to_quat(r)],
        "grasp": {s: [round(float(v), 5) for v in mk.gripper(s)[0]] + [round(float(mk.gripper(s)[1]), 3)] for s in ("left", "right")},
        "turn": {s: [round(float(v), 5) for v in mk.gripper_turn(s).reshape(-1)] for s in ("left", "right")},
        "on": m.light_on(), "off_rail": m.off_rail(), "in_bowl": m.in_bowl(), "score": m.score(None),
    }


def main() -> int:
    out = Path(sys.argv[1])
    mode = sys.argv[2] if len(sys.argv) > 2 else "dry"
    seed = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    import demo_kit as kit

    if mode == "dry":
        from yam_arena import station, task_file

        m = task_file.load(HERE / "task.py").module
        print("DRY " + json.dumps(kit.dry_run(moves(m), station.BASES)), flush=True)
        return 0
    if mode not in ("trial", "record"):
        raise SystemExit(f"mode: dry, trial or record; got {mode}")
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    result = kit.run(embodiment, moves(m), out, seed=seed, director=cue_saver(Path(out)), extra=lambda: readings(m), tail_s=2.0, record=mode == "record")
    print("RUN " + json.dumps({k: v for k, v in result.items() if k != "rows"}, default=str)[:3000], flush=True)
    print("REPORT " + json.dumps(m.report(), default=str), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
