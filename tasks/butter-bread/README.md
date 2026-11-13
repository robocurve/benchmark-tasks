# Butter a fragile slice of bread, holding it still

Instruction given to the policy: `Butter the whole slice of bread on the plate evenly with the knife, without tearing it: the bread is soft. Then put the knife down off the bread, and leave the plate where it is.`

A slice of soft bread lies on a plate on the table, with a butter dish and a butter knife beside it. Buttering it
takes two hands: on the smooth plate the knife's drag pulls an unheld slice along, so nothing spreads. The robot has to
steady the slice with one arm, gently, while the other loads the knife from the block and spreads the butter evenly
over the whole slice, going back for more as it runs out, then put the knife down off the bread. The instruction states
the goal and the constraints (no tearing, the knife put down off the bread) but not how to keep the bread still. The
bread tears if it is squeezed, poked, pressed hard or dragged by a fast stroke, which scores 0.

## Files

| File | What |
|---|---|
| `butter_bread.py` | The yam_arena task file: objects, the bread and butter models, `success()`, `score()` and the offline checks |
| `partkit.py` | Parts built from boxes and cylinders with jointed links, for yam_arena task files (extended from `tasks/dual-latch-lid` with decals that are drawn but do not collide and can be shown or hidden during a run, contact reporting with forces and points from chosen bodies such as the robot's links, and forces applied to a link) |
| `demo.py` | The scripted full-score demo |

`partkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Assets

The task uses no third-party 3D models, meshes or textures. Every object (plate, bread, butter dish and
block, knife) is built in `butter_bread.py` from boxes and cylinders, with solid colours, by `partkit.py`, which is
extended from `tasks/dual-latch-lid` in this repository. The robot arms, table and cameras are the standard `yam_arena`
station of the program's embodiment.

## How the bread, the butter and the knife work

After every control step the task reads the bread, the knife, the butter dish and the contact sensors, and updates:

**The bread is fragile.** The crumb (8 x 8 cm, inside a 1 cm crust frame) is one plate on three springs in series:
down (500 N/m) and sideways along x and y (1000 N/m each), so the forces on it show as how far it moves. In a real
trial the slice scores 0 for visible damage: a tear or hole, or the crumb squashed to under half its thickness. The
simulation does not model the damage itself; it detects it: the slice counts as torn when the crumb is pressed down
5 mm (2.5 N) or sheared 1.5 mm (1.5 N) against the crust, or when contact sensors on the slice read more than 3 N from
the robot's own links on the crust (a squeeze, a pinch, a crushing press) or 0.5 N on the crumb (a finger poking it).
The arms are compliant (about 1 N when commanded a few millimetres into a surface), and the thresholds are scaled to
that.

**Butter is an amount.** The block sits on a spring (800 N/m) in the dish. Each of 15 points across the blade that is
flat (within 30 degrees), on the block, pressing it at least 0.4 mm and sliding over it at 5 mm/s or more picks up
butter in proportion to the distance it slides and how hard the block is pressed (full rate at 1 mm), up to 1.5 cm3 on
the blade, shown as a yellow pat. The crumb is scored as 5 x 5 squares of 1.6 cm. Each blade point that is flat, on the
crumb, pressing it at least 0.4 mm (0.2 N) and sliding over it at 5 mm/s or more **relative to the bread** moves butter
between the blade's layer and the square under it, a share of the difference for each millimetre slid, thicker giving
to thinner both ways. So a stroke lays butter thinner as it goes, a lump can be spread out, and a scrape-and-stroke
covers about half the slice: three cover it evenly. A square is buttered at 0.2 mm (it turns yellow) and lumpy above
1.2 mm (a darker pat appears on it).

**Butter drags.** Between the blade and the crumb, butter's viscous drag (16 N per m/s for the whole blade, about
0.3 N at a 2 cm/s stroke) pulls the crumb along with the knife, and back on the knife. Together with the blade's
friction on the buttered crumb (0.2, the crumb's own material) it is a little more than the glazed plate's grip on the
pressed slice (0.4), so an unheld slice creeps along under each stroke, about 1 to 2 cm, turns a little, and ends up
patchy; a slice steadied at its edge stays put and is covered evenly. The drag is applied once per 50 ms control step,
so while the slice slides it is capped at the plate's friction plus what brings the slice up to the knife's speed:
the slice can creep but is never thrown. A slice that is not sliding feels the full drag, which shears the crumb hard
enough to tear it at about 10 cm/s.

## Offline check, no GPU

From this folder, with the yam-arena package importable (on the program's Brev launchable, after
`source ~/IsaacLab-Arena/yam.env`):

```bash
python butter_bread.py
```

It runs the models on hand-built knife, bread and dish states and checks the scoring on hand-built end states:

- scraping loads in proportion to travel and pressing, and nothing when not pressed, not moving or on the blade's
  edge; the blade carries at most 1.5 cm3;
- butter is conserved; one stroke covers part of the slice, thinner further on; two loads leave the far edge thin and
  three butter all 25 squares with no lumps, without tearing; spreading over a lump flattens it;
- no butter goes on when the slice slides along with the knife, the blade hovers, the knife is unloaded or the blade is
  on its edge; a 15 cm/s stroke drags the crumb with over 1 N;
- pressing 6 mm, shearing 2 mm, a 4 N squeeze on the crust and a 1 N poke on the crumb tear the bread, gentle contact
  does not;
- scores: start 0; loaded 0.25; 9 squares 0.25; 10 squares 0.5; 20 0.75; 23 with the knife still on the bread or held
  up 0.75; 23 or 25 with the knife down 1.0; 25 with the knife dropped off the table 0.75; 25 with a lump 0.75; the
  the plate pushed 4 cm 0.75, nudged 2 cm 1.0; the bread at the plate's edge, off it or tipped 0; the bread still moving 0.75; torn 0.

Then it loads the file as yam_arena does.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/butter-bread/butter_bread.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Butter the whole slice of bread on the plate evenly with the knife, without tearing it: the bread is soft. Then put the knife down off the bread, and leave the plate where it is." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/butter-bread-dream
```

`task_score` reports 1.0 when `success()` holds for 1 s and the episode ends with `termination_reason="success"`;
otherwise `score()` at the end of the episode. The CLI prints `task_score: <value>`.

Result (seed 0): **task_score 0.0**. Dream-1 hit the 2,400-step limit. The right arm reached toward the knife and
the butter dish but never scraped the block, so the knife was never loaded; the bread was not touched.

## Scripted demo

```bash
cd tasks/butter-bread
python demo.py butter_bread.py demo_out --max-steps 2400   # full score: demo_*_cam.mp4, setup.png, result.json
DEMO_NO_HOLD=1 python demo.py butter_bread.py onehanded_control_not_demo --max-steps 2400   # control: the left arm never holds the bread
```

1. The left arm opens its fingers 7 cm and stands them on the plate just past the bread's left edge, along it: a
   two-point stop the slice is pushed against, the way a hand steadies a slice against its fingertips, so it can
   neither slide nor turn. Nothing presses on the bread.
2. The right arm picks up the knife by its handle, 4 cm behind the handle's middle (well clear of the neck), and turns
   it to point away from the robot.
3. It lays the blade flat on the butter, pressed 3 mm, and scrapes 4 cm across the block at 2 cm/s.
4. It lays the loaded blade flat on the crumb's right side and draws it 6.2 cm to the left, toward the stop, at 2 cm/s.
5. It repeats 3 and 4, four loads in all.
6. The left hand lifts off the plate and goes back to its ready pose, leaving the finished slice in full view of the
   cameras; the right arm lays the knife back exactly as it lay, lets go and goes back to its ready pose.

The demo is open loop. It plans every move from the starting layout, read once after the objects settle, which is the
layout the setup describes. The knife's offset in the hand comes from the commanded grasp. While it runs it reads no
simulator state (object poses, slider positions or butter amounts) to choose its actions; it only prints them to the
log.

Result: **task_score 1.0**. Success at step 1,695 of 2,400: all 25 squares buttered (0.29 to 0.96 mm), no lumps, the
bread never torn (largest crumb shear 0.61 mm, of the 1.5 mm that tears it) and still in the middle of the plate.

Control, `DEMO_NO_HOLD=1` (the same strokes with the left arm left at its ready pose): the unheld slice creeps along and
turns a little under each stroke, about 5 cm in all, staying on the plate, and only 15 of 25 squares are buttered:
**0.5**. Steadying the slice is what lets the butter go on evenly.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time).
- Runtime per trial on an L40S: the demo about 8 min wall clock (it renders three cameras every step); the first run
  after boot adds a few minutes of Isaac Sim warm-up.
- Seeds: `--seed 0`; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed.

## Reset between trials

The butter and the bread are used up: lay a fresh slice in the middle of the plate, smooth the top of the butter block
(or use a fresh one), wipe the knife and put it back on its mark, and put the plate back on its mark.
