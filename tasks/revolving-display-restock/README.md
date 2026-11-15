# Turn a revolving countertop display round and restock its two empty back pockets

A four-sided revolving countertop display stands on the counter in front of the arms, like the ones at a store's
checkout that hold boxed phone chargers and other small boxed goods. It is a 30 cm square turntable on a weighted
base, with a row of two pockets on each of its four sides round a central column, and a short steel upright at each
corner. Every pocket holds one boxed USB charger except the two on the side facing away from the arms, which are
empty. Two boxed chargers stand on the counter, one to each side of the display. The robot must put them in the two
empty pockets.

The turntable turns freely on its bearing. The empty pockets start 45.75 cm in front of the arms, on the far side of
the column and the stocked boxes, out of reach, so the display has to be turned half a turn to bring them to the
front. A display that turns this easily also turns away whenever a box is pushed against one of its pockets, and the
pockets leave only 5 mm round a box, so a box goes in cleanly only while the display is held still.

What the task tests:

- **Turning a free-turning fixture by hand:** half a turn, in strokes on its corner uprights, ending with the empty
  side square to the arms (the pockets have to line up within about 2 degrees for a box to drop in).
- **Holding it still with one hand while the other works on it,** and handing that hold from one arm to the other.
- **Lowering a box straight down into a close pocket,** held only by its top, past the pocket walls.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `spinkit.py` | Rigid parts built from boxes and cylinders, with spring-loaded sliders, free-turning hinges, or fixed in place, for yam_arena task files |
| `demo.py` | The scripted full-score demo |
| `test_scoring.py` | Offline checks: every scoring outcome, and the layout's geometry (no GPU) |

`spinkit.py` hooks `yam_arena.scene` and the embodiment's reset without modifying yam_arena, so the task runs with
the standard `yam_arena` embodiment and `task_score` scorer. It follows the part kit used by other tasks in this
repository, and adds a revolute joint (`kind="hinge"`): here the turntable's bearing.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/display-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 2400              # writes demo_*_cam.mp4, setup.png, result.json
DEMO_DEBUG=1 python demo.py task.py debug_out --no-record      # also prints the display's pose and turn, and each box's place in it
python -m pytest test_scoring.py                               # offline checks
```

1. The left arm pinches the front-left corner upright and carries it round the axis to the front (45 degrees,
   counterclockwise seen from above). The right arm takes the same upright on round to the right side (90 degrees),
   then pinches the next upright, now at the front, and carries it to the front-right (45 degrees). The empty side
   now faces the arms, and the right arm keeps hold of that upright.
2. While the right arm holds the display still, the left arm picks up the left box by its top, carries it out beside
   the display, lifts it over the pocket walls, lowers it straight into the empty pocket on its side and lets go.
3. The left arm takes hold of the front-left upright; the right arm lets go, and puts the right box in the other
   empty pocket the same way. The left arm lets go.

The demo runs open loop: it reads no simulator state. How far each stroke actually turned the display, and how far
the arm sags and lags under a box held over its pocket (about 1 cm), were measured once with
`DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record` (which runs the same moves, reads the display's pose,
turntable angle and the held box's position from the simulator and prints the numbers) and are fixed in `CAL` in
`demo.py`. The layout is fixed and the simulation deterministic. The demo reaches 1.0 (success) at step 1,354 (68 s).

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). Positions are taken
relative to the turntable (as a ruler on the display would measure them), so turning or sliding the whole display is
allowed.

- 0 if the display was at any point tipped more than 20 degrees (knocked over).
- 1.0 (success, which must hold for 1 s): each of the two formerly empty pockets holds a box seated in it, that is,
  the box's centre within 6 mm of the pocket's centre both across and along the pocket row, its bottom within 8 mm of
  the deck, and the box upright within 10 degrees; and both boxes still (moving less than 1 mm over the last second).
- 0.5: one of the two pockets holds a seated box.
- 0.25: neither does, but the display has been turned so the empty side faces the arms (its outward direction within
  20 degrees of pointing straight at them).
- Otherwise 0.

## Runtime and seeds

About 2 minutes per trial for the full 2,400 steps on an L40S, plus about 1 minute for Isaac Sim to start. The
layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

Rigid bodies built from boxes and cylinders, at full size. The display's frame is solid plates where a real one is
usually bent wire or perforated steel; its pockets are 80 x 45 mm inside 3 mm walls 60 mm tall. The turntable is one
rigid body on a revolute joint with no stops and a viscous drag of 0.05 N m s/rad (a ball-bearing lazy-susan plate:
pushed, it turns easily and coasts a few degrees), on a 3 kg base resting on rubber feet (friction 0.8). The six
stocked boxes are modelled as part of the turntable, so they cannot be taken out. The loose boxes are 70 x 35 x 110 mm,
150 g card cartons (friction 0.5). Light angular damping is added to the parts. No contact force cap is enforced in
simulation.

## Real-world build

Use a four-sided revolving countertop display with a 30 cm turntable (or build one: a 30 cm square plywood or steel
deck on a 30 cm lazy-susan bearing screwed to a weighted 32 cm base, 3 mm steel or acrylic pocket walls 60 mm tall
glued to the deck to the dimensions in `task.py`, a 17 cm square column, and 16 mm steel tube uprights 60 mm tall at the
corners). Fill six pockets with boxed chargers (or 70 x 35 x 110 mm card boxes weighted to 150 g); stand two more on
their marked outlines.

## Assets

All geometry is defined in `task.py` from boxes and cylinders; no external assets.
