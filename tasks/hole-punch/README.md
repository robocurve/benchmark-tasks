# Punch a card's two filing holes on their marks

| Item | Rule |
|---|---|
| Card | Yellow A6 card stock with two black crosshair marks 12 mm from its long edge, 80 mm apart. Each mark is two 30 mm lines, so it stays visible around a punched hole. The card starts on a foam stand in front of the punch, at a position and turn drawn from the trial seed. |
| Punch | Desk two-hole punch fixed to the table: 6 mm punches 80 mm apart, 12 mm in front of a back stop, driven by a lever with a red handle knob. A stock equivalent is the Rapesco 835-P (6 mm holes, 80 mm apart). |
| A hole | The punch has gone through the card with the hole's centre within 1.5 mm of its mark. |
| Score | 1.0 for both holes, held 1 s (then the trial ends); 0.5 for one hole, or for two before the hold is complete; 0.0 for none. |
| Time limit | 1,200 control steps at 20 Hz: 60 s. |

## Run a trial

Use the official Robocurve launchable. From the repository root:

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e tasks/hole-punch
python -m pytest -q tasks/hole-punch/test_scoring.py        # scoring rules, no simulator
INSTR="$(python -c 'import json; print(json.load(open("tasks/hole-punch/task.json"))["instruction"])')"
inspect-robots run --embodiment yam_hole_punch --policy hole_punch_scripted --seed 0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 -E telemetry=runs/hole-punch/telemetry.jsonl \
  --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/hole-punch/demo
```

- Model trial: replace the policy with `--policy dream`.
- Seeds: `--seed N` sets the card's start. The embodiment draws the card's offset (x within ±0.8 cm, y within
  ±1.2 cm of the nominal start) and turn (within ±10 degrees) from the trial seed that Inspect Robots derives from
  `--seed`. With no seed the card lies at the nominal start. The first line of the telemetry file records the start
  poses.
- Failure checks, each expected to score 0.0:
  - `-P mode=nocard`: the lever is pressed with the card left on the stand.
  - `-P mode=offset`: the card is aimed 4 mm off its marks.
  - `-P mode=short`: the card is put on its marks, but the press is soft (the knob is commanded 4 mm below where
    the lever bottoms out; the demo uses 90 mm): the punches come down onto the card without the force to cut it.
- Margin runs: `DEMO_AIM_SHIFT_MM="dx,dy"` shifts where the demo aims the card.

The package registers the embodiment `yam_hole_punch`, which is the stock robot, cameras and actions plus the punch,
lever, sprung head, foam stand, card and discs. It also registers the policy `hole_punch_scripted`. Telemetry logs
every step: card and disc poses, how far each disc has been pushed out of the card, head travel, lever angle, release
times, the score, and each arm's measured tool point.

## How punching is modelled

Each hole is a 6 mm disc pre-scored in the card at its mark. It sits on a slide along the card's normal, and a force
drive on that slide resists the punch with up to 30 N per hole, so the punch must push it down through the card.
The task file cuts a disc free only when all of these hold at the same moment:

- the punch has pushed the disc the card's full thickness, and its tip is 0.5 mm below the card's underside (below
  the bed's top face);
- the disc's centre is within 1.5 mm of that punch's axis;
- the card lies on the bed: no more than 0.5 mm above its seated height there. (Up to 2 mm below is allowed,
  because under the full press the simulated card sinks up to about 1.5 mm into the bed, which a real card cannot
  do.)

A hole counts once its disc has left the card and fallen into the die. So pressing the lever with no card, with the
card short of the punches or off its marks, or stopping with the punches on or in the card, frees nothing.

The 30 N is an estimate. Paperboard's shear strength is about the geometric mean of its two tensile strengths, about
16 N/mm. A flat 6 mm punch shearing its whole 18.9 mm rim at once would need about 300 N. Desk punch tips are
shear-ground and enter point first. The die-design rule for a shear-ground punch scales the flat force by p t / s:
real thickness t = 0.35 mm, penetration at fracture p = 0.4-0.5, shear depth s = 1.5-2 mm. That gives 0.07-0.12 of
300 N, which is 20-35 N per hole. Two holes need about 60 N at the punches, which is about 17 N at the knob through
the lever's 3.5:1 ratio. A 500 N/m return spring lifts the punches.

Measured in Isaac Sim with a straight-down force on the punch head and the card on its marks: 65 N does not cut
the card, 70 N cuts both discs (60 N for the discs plus the spring). One disc pushed on its own holds at 28 N and
gives way at 33 N. Pressed straight down on the knob, 20 N does not cut and 30 N does. The tips and discs collide as square prisms of about the same area as the round shapes they
show (5.4 mm tips, 6 mm discs), and each disc is given 10 g in the model, because with round 0.5 g discs the tips
sank through them at under 20 N.

## Check a real trial

Punch a fresh pre-marked card. With a steel rule (0.5 mm marks), measure each hole's centre from the centre of its
crosshair, where its lines would cross. Within 1.5 mm counts. A photo of the card on a scale also works.

## The demo

- **The demo reads no simulator state.** Before it moves, it finds the card in the top camera's first image. It
  takes the yellow pixels, projects them onto the card's plane with the station's calibrated top camera, and fits
  the card's centre and turn. After that it uses only the robot's joint angles and the punch's fixed position.
- The right arm, gripper part-open with its two fingertips along x, pushes the card's side edge toward the centre
  line, which turns it toward square; it stops 16 mm short. Then it pushes the card's back edge until the card meets
  the stop, which squares it. Last, it pushes the side edge again until the card's middle is on the centre line, and
  lifts clear. (Squaring a card turned near -10 degrees in the back push slides it several mm toward the robot's
  left, so the first push stops short and the last push only ever pushes that way.) The left arm waits until the right is clear (the grippers collide near the centre line), then
  presses the lever knob down; its target is 90 mm below where the lever bottoms out, which drives the punches
  the full stroke.
- `dev/probe_punch.py`, `dev/out_v10/` and `dev/punch_reach.py` are from the earlier version, with no random start
  and a 3 mm tip.

## Results

Measured 2026-11-18 on the current version, one trial per row, Isaac Sim on an L40S GPU (with other jobs sharing it).
"Discs freed at" is the sim time each disc was cut free; the trial ends 1 s after both are free.

| Run | Seed | Card start (dx, dy mm; yaw deg) | Score | Expected | Steps | Discs freed at (s) | Lowest head travel (mm) | Wall time (s) |
|---|---|---|---|---|---|---|---|---|
| Demo | 0 | +3.3, -11.1; +6.3 | 1.0 | 1.0 | 627 | 30.30, 30.30 | -9.0 | 352 |
| Margin: aim +1 mm y | 0 | +3.3, -11.1; +6.3 | 1.0 | 1.0 | 627 | 30.30, 30.30 | -9.0 | 376 |
| Margin: aim -1 mm y | 0 | +3.3, -11.1; +6.3 | 1.0 | 1.0 | 627 | 30.30, 30.30 | -9.0 | 346 |
| Margin: aim +1 mm x | 0 | +3.3, -11.1; +6.3 | 1.0 | 1.0 | 627 | 30.30, 30.30 | -9.0 | 408 |
| Margin: aim -1 mm x | 0 | +3.3, -11.1; +6.3 | 1.0 | 1.0 | 627 | 30.30, 30.30 | -9.0 | 458 |
| Start: most turned + | 141 | -7.1, +3.4; +9.9 | 1.0 | 1.0 | 627 | 30.30, 30.30 | -9.0 | 456 |
| Start: most turned - | 32 | -3.2, +0.0; -9.9 | 1.0 | 1.0 | 627 | 30.30, 30.30 | -9.0 | 453 |
| Start: far forward and left | 271 | +7.9, +11.8; -2.6 | 1.0 | 1.0 | 627 | 30.30, 30.30 | -9.0 | 412 |
| Start: far right, turned - | 75 | -1.8, -11.6; -9.4 | 1.0 | 1.0 | 627 | 30.30, 30.30 | -9.0 | 408 |
| Failure: no card | 0 | +3.3, -11.1; +6.3 | 0.0 | 0.0 | 315 | -, - | -9.0 | 223 |
| Failure: off marks | 0 | +3.3, -11.1; +6.3 | 0.0 | 0.0 | 725 | -, - | -5.3 | 505 |
| Failure: short stroke | 0 | +3.3, -11.1; +6.3 | 0.0 | 0.0 | 725 | -, - | -4.8 | 554 |
| Dream-1 | 0 | +3.3, -11.1; +6.3 | 0.0 | (model) | 1200 | -, - | 0.0 | 1299 |

- The demo scores 1.0 from the camera at seed 0, with its aim shifted 1 mm either way in x and y, and at all four
  extreme starts, including the two turned almost -10 degrees.
- No card, and the card aimed 4 mm off its marks, score 0.0.
- Short stroke scores 0.0 with the tips on the card: the head stays between 4.5 and 4.85 mm down (the tips meet the
  card at 4.5 mm) for 4.5 s, the discs are pushed at most 0.18 mm, and nothing is freed. The old rule, which freed
  a disc when the tip reached the card's top face, would have scored this run.
- The short mode's press depth was set after the other scripted runs were recorded (its first setting, 40 mm, cut
  the card). That constant is read only in `mode=short`, so it changes the short-stroke row and no other.
- Dream-1 scores 0.0: in 1,200 steps it neither moves the card nor presses the lever down. It ran on the final code.

Runtime per trial: the demo ends at about 31 s of sim time (627 steps); a full 1,200-step run is 60 s of sim time.
Wall time here was 220-560 s per scripted trial, including about 2 minutes of start-up.
