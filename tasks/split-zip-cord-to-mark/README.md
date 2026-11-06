# Split a lamp cord apart to the white band

A two-leg brown lamp cord (zip cord) is clamped at one end and lies straight on the table. Its two 4 mm legs are
joined along their web. The robot has to separate the legs from the free end up to a white tape band 15 cm from the
end, and no further, then let go. The web tears easily, so the split has to be stopped at the right place. The usual
method uses both hands: one hand pinches both legs just behind the band as a tear stop, and the other peels the free
leg back in a V. Two-arm use is optional: a careful single-arm peel that stops in time is also valid and scores the same
(the score does not depend on which arm or how many arms are used). This task tests stopping a running
separation at a set point on a deformable object.

## How this differs from similar tasks

Other cable tasks route, seat, insert, sort or pull apart loose cables, and nothing in them is bonded. Here the
object is changed for good (the web tears) and the measured result is where the tear stops along the cord, not where
the cord ends up.

Clarity note: in the 320 x 176 top camera the 4 mm cord is a thin brown line and the white band a small white dot
(see `demo/setup.png`), so the setup text in task.json (positions in cm, band 2 cm wide at 15 cm from the short leg's end)
is the reference for reproducing the layout; the wrist cameras show the cord and band at close range.

## Requirements

yam-isaac-lab-arena with Isaac Lab-Arena 0.3.1 installed. `cablekit.py` (the cable simulation) is in this folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 900 \
    --instruction "Separate the two legs of the brown lamp cord, starting at its free end, up to the white band. Do not split it past the band. Let go of the cord when you are done."
```

Run one episode per process: a broken web bond stays broken after an environment reset, so in sim every trial needs a
fresh process (in a real trial, a fresh cord for every attempt).

## Scripted demo

```bash
python cabledemo.py task.py demo_out
```

The left arm pinches both legs just behind the band. The right arm takes the near leg's tail (it sticks out 4 cm past
the short leg) and peels it back at 60 degrees from the cord line. It moves a little further each step, following the
split as measured in the sim, and stops when the split reaches the band. Then both arms let go. It scores 1.0 in about
190 steps (about 10 s).

Shortcut variants (test only, `ZIP_CHEAT=<name> python cabledemo.py task.py out`) all score exactly 0:

| variant | what it does | why 0 |
|---|---|---|
| `none` | waits | split stays at 3 cm (under 9 cm) |
| `hold` | splits to the band but keeps both grippers closed | grippers within 3 cm of the cord |
| `yank` | one arm, no pinch: pulls the tail fast straight sideways | the tear runs past the band |
| `axial` | grips both legs at the end and pulls along the cord | the legs do not split apart (or split past the band) |

## Scoring

`score()` reads the split length S: the distance from the short leg's end to the first web bond that still joins the
legs (bonds every 2 cm, a bond counts as broken once the legs are 3 mm apart there). Score 0 if the legs are split
anywhere more than 1 cm past the band centre, if the cord moved out of its clamp, or if a gripper is within 3 cm of
the cord at the end. Otherwise 1.0 if S is 14-16 cm, 0.6 if 12-14 cm, 0.3 if 9-12 cm, else 0. The episode ends with
success after 1 s at score 1.0 with both legs at rest.

Runtime: about 190 steps for the demo; the run limit is 900 steps (45 s at 20 Hz). Seeds: the layout is fixed with
`object_jitter=0`, and the cord is not jittered.
