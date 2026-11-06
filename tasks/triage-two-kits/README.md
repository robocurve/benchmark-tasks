# Give two trauma kits by triage colour

Three casualty stand-ins lie across the far half of the table, each with a triage tag on its chest: green (robot's
left), red (centre) and yellow (robot's right). There are only two trauma kits. Under START triage, red (immediate)
comes first, then yellow (delayed), then green (minor), so one kit goes on the red casualty's pad and one on the
yellow casualty's pad. The kits start next to the green casualty's pad, so the nearest casualty is the one who
should wait, and the casualty farthest from the kits (yellow) is one who needs a kit.

Files: `task.py` (scene, scoring, scripted demo), `task.json`, `test_task.py` (offline checks).

## Run a trial

Built on `yam-isaac-lab-arena` at `bd62771` (the `yam_arena` embodiment for Inspect Robots). From the folder that
holds this task:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=triage-two-kits/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Three casualties, two trauma kits. Treat by triage tag: red first, then yellow, then green. Put one kit on the white pad in front of each casualty you treat." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/triage-dream
```

Scripted demo (both arms, 24.2 s = 484 planned steps). The left arm takes the outer kit to the red pad and parks;
the right arm waits outboard of the yellow casualty until then, takes the inner kit to the yellow pad and parks.
The last release ends at 19.7 s (394 planned steps), so success can fire from about 1 s later; on an RTX 5090 it
fired at step 412. The park and 2 s hold are margin:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=triage-two-kits/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Three casualties, two trauma kits. Treat by triage tag: red first, then yellow, then green. Put one kit on the white pad in front of each casualty you treat." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/triage-demo
```

`--scorer task_score` prints the trial's score at the end: 1.0 on success, otherwise `score()` at the last step
(`SCORE_OVER = "end"`).

- **Run limit:** 2,400 steps (2 minutes at 20 Hz), about five times the scripted demo.
- **Seeds / jitter:** always pass `-E object_jitter=0`. The layout and the tag mix are fixed; there are no
  variants and no environment variables. With no jitter the seed does not change anything.
- **Runtime per trial:** on an RTX 5090, 51 s wall clock for the demo and 219 s for Dream-1, including Isaac
  start-up. Simulated time: up to 120 s for the Dream-1 trial, about 21 s for the demo.
- **Assets:** none. Every object is a box or cylinder primitive defined in `task.py`; nothing to attribute.

## How it is scored

A kit is **on a pad** when its centre (seen from above) is over the pad (x 245-375 mm, the casualty's y +-55 mm)
and its lowest point is at most 15 mm above the pad top. Orientation does not matter (a kit on its side counts).
A kit stacked on the other kit, held more than 15 mm above a pad, left on a casualty's body, on the table or on the
floor is on no pad.

Score = 0.6 if any kit is on the red pad + 0.4 if any kit is on the yellow pad - 0.4 per kit on the green pad,
never below 0. A second kit on the red or yellow pad adds nothing.

| kits on | red + yellow | red (+ nothing or red) | yellow (+ nothing or yellow) | red + green | yellow + green, green, none |
|---|---|---|---|---|---|
| score | 1.0 | 0.6 | 0.4 | 0.2 | 0 |

Red is worth more than yellow so that "red first" matters when only one kit is delivered. The green penalty equals
the yellow credit: a kit given to the minor casualty is the kit the delayed one did not get. Success: one kit on the
red pad and the other on the yellow pad, both still (`resting()`: under 1 cm/s and 0.5 rad/s) for 1 s. Which kit
goes where, and in what order, is not scored; the hooks see only the end state.

`distance()`: the horizontal distance from the two kits to the red and yellow pads' footprints, summed, under the
better pairing (0 at the goal).

## Why this tag mix (green, red, yellow from the robot's left)

Only the default scene is run, so the mix is fixed and chosen to defeat blind strategies. The kits start left of
the midline, beside the green pad. The tested outcomes of the obvious shortcuts:

| blind strategy | kits end on | score |
|---|---|---|
| the two pads nearest the kits | green + red | 0.2 |
| each kit to its own nearest pad | green + red | 0.2 |
| each arm straight ahead to the pad in front of it | green + yellow | 0 |
| the first two pads from the robot's left (reading order) | green + red | 0.2 |
| both kits to the nearest casualty | green + green | 0 |

Only reading the tags gets 1.0. With three pads and two kits, some fixed ordering always lands on the right pair
by chance (here, "the two pads from the robot's right"); no single layout can rule that out. The task is still a
two-kit pick-and-place: the measurement is whether the allocation follows the tags or the geometry.

## Geometry choices (and why)

- **Gripper:** pads are 93.9 mm apart fully open (`FINGER_OPEN = -0.04695` m per finger in
  `yam_arena/constants.py`); usable grip taken as 84 mm. The kits are gripped across their 50 mm side: 44 mm of
  margin on the open width, 34 mm on the usable grip.
- **Grip height:** TCP 28 mm above a kit's bottom. The pads (27 mm, from the tip 13 mm below the TCP) hold it from
  15 to 42 mm of its 45 mm height; the fingertips stay 15 mm above the table and the finger carriage 37 mm above
  the kit's top.
- **Free space for the fingers (at least 20 mm a side):** fingers close along world y (yaw 0). The kits are 110 mm
  apart centre to centre; an open finger's outer face reaches 57 mm from the gripped kit's centre, leaving 28 mm to
  the other kit. Nothing fixed lies within 20 mm of the fingers' path at either kit.
- **Grip offset:** in simulation the pads hold objects 12-20 mm toward -y (18-19 mm on 42-45 mm blocks). The demo aims
  each place 6 mm toward +y, and the offline sweep and goal check run with the carried kit at 0, -15 and -20 mm: no
  contact, and each placed kit's centre stays at least 20 mm inside its pad (the pads are 110 mm across for a
  50 mm kit).
- **Two arms, never together:** the yellow pad (y -220 mm) is only reachable by the right arm, and the inner kit
  (y +70 mm) is reachable by both. The left arm takes the outer kit (y +180 mm) to the red pad (y 0), retreats
  7.6 cm straight up and parks at (130, +420) mm; only then does the right arm, waiting at (140, -380) mm, reach
  across the midline for the inner kit. The arms' link hulls stay over 10 cm apart throughout (tested).
- **Carry height:** TCP 110 mm above the table, under the ~140 mm joint4 limit. A carried kit's bottom passes 43 mm
  over the tallest fixed part (the tags), though no carry passes over a casualty.
- **Shallow pads:** 3 mm foam, so the default lowering speed (1.2 s) and a 3 mm release height are used.
- **Pads and bodies:** each pad is 15 mm short of its casualty and 110 mm of bare table separates neighbouring pads,
  so a kit's centre is over at most one pad and which casualty a kit was given to is never in doubt.
- **Camera:** projecting the arm link hulls into the calibrated top camera, no point of a tag, a pad or a kit is
  behind an arm in the first frame (both arms at the ready pose) or the last (both parked outboard at y +-420 mm),
  and every tag spans at least 6 px each way at 320 x 176 (tested). The tags differ by colour, not size.

## What makes this task distinctive

There are fewer kits than casualties, so the right answer deliberately leaves one casualty without, and that
casualty is the one nearest the kits. Which casualties get a kit is a priority read from the tags' colours (START:
red, then yellow, then green); no kit is matched to its own colour. The scoring separates missing the critical
casualty (yellow only: 0.4) from wasting a kit on the minor one (red + green: 0.2) from following the geometry
(0-0.2).

## Offline tests

```bash
pip install numpy pytest
YAM_ARENA_DIR=~/yam-isaac-lab-arena python -m pytest -q test_task.py   # a clone at bd62771
```

The checks: the file loads with two identical loose kits and everything else fixed; the tag mix is green, red,
yellow; both demo plans are reachable at 20 Hz within the joint limits and end together with a still hold of at
least 1.5 s; travel height under the joint4 limit; a straight 3 cm+ rise after every release; a kinematic sweep of
both arms (URDF link hulls) keeps every link 5 mm off the casualties, pads and the other kit, the carried kit (at
0, -15 and -20 mm grip offset) off everything, and the two arms over 10 cm apart; the demo ends on red and yellow
with score 1.0; every allocation of two kits to {red, yellow, green, nowhere} scores as stated; kits on their
side, off centre, held above, on a body, beside a pad, stacked or on the floor; a moving kit does not succeed;
the five blind strategies above; no overlaps at the start; supports; finger free space; gripper fit; the top
camera's first and last frames; tag size in pixels; task.json fields.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success at step 412; 51 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 2,400-step run limit; 219 s wall clock.
- Offline tests: 47 passed.
- Top camera: in `setup.png` all three tags, pads and both kits are visible. The trial ends on success before the right arm
  parks, so in the demo's last frame the right arm covers part of the yellow tag and its pad; both kits stay
  visible.

## Real-world notes

Real counterparts: three casualty stand-ins built from foam or wooden blocks with a disc head (a tabletop triage
drill), START triage tags as coloured tiles, white foam pads taped to the table, and two navy belt-pouch trauma kits
on a rigid insert, 100 x 50 x 45 mm, 150 g. A person does this in about 10 s (estimate, not timed). Scoring needs
only a photo: which pad each kit's centre lies over.
