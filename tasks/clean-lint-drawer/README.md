# Clean a dryer's lint screen before sliding it back into the dryer

A bimanual YAM task in Isaac Lab Arena. A tumble dryer's lint screen has been pulled out of its drawer slot and
lies on the table in front of the dryer with a pad of lint on it. The robot has to take the lint off and drop it
in the waste bin, then slide the screen straight back into the slot until it meets the back stop, and let go.
Order and destination both count: putting the screen back with the lint still on it scores 0, and so does
getting the screen in while the lint ends up anywhere but the bin (scraped off at the opening, or brushed onto
the table). The task measures a short ordered procedure with a precise, constrained insertion at the end.

The whole task is one task file, [`clean_lint_drawer.py`](clean_lint_drawer.py), for the `yam_arena` embodiment
from [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (Isaac Sim 6, Isaac Lab 3, Isaac
Lab Arena 0.3). The scene is built from boxes only, so nothing is downloaded.

## Run a trial

From this folder, in the Arena environment:

```bash
INSTRUCTION="Clean the dryer's lint screen and put it back: lift the grey lint pad off the white screen and drop it in the black bin, then slide the screen straight back into the slot in the dryer until it stops, and let go of it."

# Dream-1 (needs DREAM_API_KEY in ~/.dreamlabs/credentials)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=clean_lint_drawer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clean_lint_drawer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/demo

# One MP4 per camera, in runs/<policy>/frames/<run>/
inspect-robots video runs/dream/*.json
```

The run prints `task_score` at the end and keeps it in the log.

- **Run limit:** 1,200 control steps (60 s at 20 Hz).
- **Seeds:** none. `-E object_jitter=0` starts every object exactly where the task file puts it, the layout in
  the screenshot. The scripted demo is open-loop and expects that layout.
- **Runtime per trial:** the scripted demo's trial ends at full score after about 500 steps (25 s simulated) and took about 1.6 minutes of wall time on an L40S, Isaac Sim start included. The Dream-1 trial ran the full 1,200 steps in about 3.5 minutes. Add a few minutes for Isaac Sim's first start on a fresh machine.

## Scene

Positions are in cm, from the midpoint between the two arm bases: x forward, y left, heights above the table
top. The arm bases are at y = +30.5 (left) and y = -30.5 (right).

| Object | Type and material | Size | Position | Mass | |
|---|---|---|---|---|---|
| Lint-drawer housing | light grey rigid slot: two 1 cm side walls, a 1 cm roof and a 1 cm back stop | slot 8 cm wide (y), 2.2 cm high, 11 cm deep | centred on y -12; opening at x 34, back stop's face at x 45 | | fixed |
| Dryer cabinet | dark grey block on the housing's roof | 10 (x) x 14 (y) x 6 cm | x 37 to 47, y -12 | | fixed |
| Waste bin | open box, black plastic: 1 cm floor, four 1 cm walls 6 cm tall | 10 x 10 cm inside, 12 x 12 cm outside, rim 5 cm above the floor | x 20, y 0 | | fixed |
| Lint screen | box, white plastic frame with mesh (rigid) | 16 (x) x 7 (y) x 1.5 cm | x 20, y -12, lying flat, long side along x (spans x 12 to 28) | 120 g | → back into the slot |
| Lint pad | box, grey felted lint (rigid) | 9 (x) x 5.5 (y) x 1.6 cm | lying flat, centred on top of the screen | 15 g | → bin |

Everything is square to the table edges. The screen lies on the slot's axis with its far end 6 cm short of the
opening, so it goes straight in once the lint is off.

## Scoring

Scored by `score()` and `success()` in the task file, every control step, from the object poses. Extents are
computed from each box's pose and true size.

- **Hard 0:**
  - the screen or the lint ends with its lowest point more than 5 cm below the table top (fell off the table);
  - any part of the screen is inside the bin's outer footprint below its rim (the screen was thrown away);
  - the screen's far end is 2 cm or more inside the opening while the lint is still on it (the lint's centre over
    the screen's top face, checked in the screen's own frame, and its underside within 1 cm of that face).
- **Otherwise:** `0.9 x (0.35 L + 0.65 L d)`
  - `L`: 1 if the lint is wholly in the bin (within the inner walls, 2 mm tolerance, no part above the rim, its
    underside within 5 mm of the bin floor), else 0.
  - `d`: how far the screen's far end is inside the opening, 0 to 11 cm, divided by 11. It counts only while the
    far end is within the slot's 8 cm width and the screen's underside is below the slot's 2.2 cm roof (a screen
    lying on top of the housing has `d` = 0).
- **1.0:** the lint in the bin; the screen's far end within 5 mm of the back stop, its long axis within 5 degrees
  of the slot's axis and lying flat (tilt under 5 degrees); both still (under 1 cm/s and 0.5 rad/s); and every
  fingertip of both grippers at least 3 cm from the screen and the lint; held for 1 s in a row. This ends the
  trial.

Examples: doing nothing scores 0. Binning the lint only scores 0.315. Binning the lint and pushing the screen
halfway in scores 0.6. Sliding the screen in with the lint on it scores 0; so does sliding it in after brushing
the lint onto the table. Everything done but still holding the screen scores 0.9.

To score a real trial: look in the bin, and measure with a tape how far the screen sticks out of the opening
(5 cm when fully seated: 16 cm screen, 11 cm slot). "Let go" means both grippers are open and at least 3 cm from
the screen and the lint for a second.

`test_scorer.py` checks the scorer offline (no simulator) against these and other made-up outcomes:
`python test_scorer.py`.

## Scripted demo

`demo()` plays an open-loop plan (`yam_scripted` policy) with the right arm only; the left arm stays parked.
The right arm comes down over the lint pad with its fingertips 2 mm above the screen's top face, closes slowly
across the pad's 5.5 cm width, lifts it over the bin, lowers it to just above the bin floor and opens. It then
grips the screen across its 7 cm width 2 cm in from its near end, lifts it 3 mm, slides it straight along the
slot's axis until its far end meets the back stop, sets it down, opens and backs away. The pick targets include a
small reach correction measured in simulation. The plan reaches full score after about 500 control steps.

## Fidelity notes

- The housing, the cabinet and the bin are fixed rigid boxes, like a dryer and a bin standing on the floor. The
  slot is scaled to the screen.
- The screen is one rigid box; the lint is a rigid felt block, since a task file has no cloth or fibres. Real
  lint clings to the mesh and comes off in a sheet; here it lies loose on the screen and can't tear or shed.
- The slot gives 7 mm of vertical clearance over the screen, so lint left on it is scraped off at the opening,
  as on a real drawer, rather than carried inside.
- Friction is the simulator default for every object; it can't be set per material in a task file.
- Grasp, contact and release are approximate. The release check uses the fingertip positions from the arms'
  joint states by forward kinematics (below).

## Dependencies

- yam-isaac-lab-arena at commit `0059d02a1a91bc6834c1d0846022891df0c78730` (2026-11-01), with Isaac Sim 6.1, Isaac Lab 3.0 and Isaac Lab Arena 0.3.
- `success()` checks the release with the arms' fingertip positions. The task-file hooks receive only object
  states, so it reads the joint positions of the `yam_arena` embodiment that calls it (found up the call stack)
  and runs `yam_arena.urdf.forward_kinematics`. This depends on the embodiment's private state at the commit
  above; if the state can't be found, scoring raises an error instead of guessing.
