# Punch a card's two filing holes on their marks

| Item | Rule |
|---|---|
| Card | Yellow A6 card stock with two black crosshair marks 12 mm from its long edge, 80 mm apart. It starts on a foam stand 4 cm short of the punch's back stop and 1.5 cm off the centre line. |
| Punch | Desk two-hole punch fixed to the table: 6 mm punches 80 mm apart, 12 mm in front of a back stop, driven by a lever with a red handle knob. |
| A hole | The punch has gone through the card with the hole's centre within 1.5 mm of its mark. |
| Score | 1.0 for both holes (held 1 s, then the trial ends); 0.5 for one; 0.0 for none. |
| Time limit | 1,200 control steps at 20 Hz: 60 s. |

## Run a trial

Use the official Robocurve launchable. From the repository root:

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e tasks/hole-punch
python -m pytest -q tasks/hole-punch/test_scoring.py        # scoring rules, no simulator
INSTR="$(python -c 'import json; print(json.load(open("tasks/hole-punch/task.json"))["instruction"])')"
inspect-robots run --embodiment yam_hole_punch --policy hole_punch_scripted \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 -E telemetry=runs/hole-punch/telemetry.jsonl \
  --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/hole-punch/demo
```

- Model trial: replace the policy with `--policy dream`.
- Failure checks: `-P mode=nocard` (the lever pressed with the card left on the stand) and `-P mode=offset` (the card
  aimed 4 mm off its marks). Both should score 0.0.
- Margin runs: `DEMO_AIM_SHIFT_MM="dx,dy"` shifts where the demo aims the card.

The package registers the embodiment `yam_hole_punch`, which is the stock robot, cameras and actions plus the punch,
lever, sprung head, foam stand, card and discs. It also registers the policy `hole_punch_scripted`. The setup is fixed,
so the seed changes nothing. Telemetry logs every step: card and disc poses, head travel, lever angle, release times,
the score, and each arm's measured tool point.

## How punching is modelled

Each hole is a 6 mm disc pre-scored in the card at its mark and held by a fixed joint. The task file switches that
joint off only when three things hold at the same moment:

- the punch's tip has come down onto the card;
- the disc's centre is within 1.5 mm of that punch's axis;
- the card lies flat on the bed.

A hole counts once its disc has left the card and fallen into the die. So pressing the lever with no card, with the
card short of the punches, or with it off its marks frees nothing. The hedge-shears and laminator tasks already on
the board use the same approach: the task file frees a part only when the material is in the tool's path. A 500 N/m
return spring stands in for cutting resistance.

## Check a real trial

Punch a fresh pre-marked card. With a steel rule (0.5 mm marks), measure each hole's centre from the centre of its
crosshair. Within 1.5 mm counts. A photo of the card on a scale also works.

## Results (November 15, 2026, official simulator on an L40S)

| Run | Card's centre off the centre line at the press | Score | Steps |
|---|---|---|---|
| Scripted demo | +0.2 mm | 1.0 (both discs released at 24.15 s) | 504 |
| Margin: aim +1 mm in y | +0.7 mm | 1.0 | 505 |
| Margin: aim -1 mm in y | -1.1 mm | 1.0 | 504 |
| Margin: aim +1 mm in x | +0.2 mm | 1.0 | 505 |
| Margin: aim -1 mm in x | +0.1 mm | 1.0 | 504 |
| `mode=offset` (aimed 4 mm off) | +3.8 mm | 0.0 | 591 |
| `mode=nocard` (lever pressed, card left on the stand) | not moved | 0.0 | 315 |
| Dream-1 | not moved | 0.0 | 1,200 (the limit) |

- **The demo reads no simulator state.** It uses only the robot's joint angles and the fixed positions in the setup.
  The right arm opens its gripper, pushes the card's side edge until the card's middle is on the centre line, then
  pushes its back edge until the card meets the stop, and goes home. The left arm waits until the right is clear
  (the grippers collide near the centre line), then presses the lever knob.
- Every hold and press point passes the stock IK gate (`dev/punch_reach.py`).
- Runtime per trial: 25.2 s of simulated time for the demo, 60 s for a full 1,200-step run. Wall time was about 4
  minutes for the demo and 5.5 minutes for Dream-1.
- `dev/probe_punch.py` and `dev/out_v10/` are the earlier mechanism probe, run with no robot.
