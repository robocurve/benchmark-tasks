# Fit a spring tension rod between two cupboard walls

Two cupboard walls stand 300 mm apart. A spring tension rod, 320 mm long at rest, lies in front of them. The robot
squeezes the rod shorter, sets it between the walls at the marked height and level, and lets go; only the rod's
spring pushing its rubber caps against the walls holds it up. Full credit (1.0) also needs it in the band, level (cap
centres within 10 mm in height) and square across the gap (within 10 mm front to back); a rod let go and holding
itself up between the walls but outside those limits scores 0.5.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment
for Isaac Lab-Arena), on the `station` scene. It uses `parts.py` (in this folder) for the walls with their marked
band and for the rod, a loose part whose inner bar slides on a spring. `near_misses.py` lists near-successes and
wrong outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit the spring tension rod between the two white cupboard walls, level, at the yellow band."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/tension-rod-dream

# Scripted demo (reaches 1.0 in about 18 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/tension-rod-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 40 s of simulated time (800 steps at 20 Hz); about 1 minute of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Runs use `-E object_jitter=0`, so every object starts exactly where `task.py` puts it.
  With jitter on, seeded resets shift only the rod.

## The physics

Fitted, the rod is 20 mm shorter than its free length, so its 70 N/m spring presses each cap on its wall with about
1.4 N. With the rod's friction (1.0) and the walls' (0.6), averaged by the simulator to 0.8, the two caps can hold
about 2.2 N against the rod's 0.98 N weight. Set down without squeezing, the rod does not fit between the walls;
released before both caps bear, it drops; set crooked or at the wrong height, it may hold but out of true, which
scores 0.5.

## How the simulator scores it

- The cap centres come from the rod's two links (tube and bar) and its slide position; "cap face against the wall"
  is the rod's world-aligned bounding box reaching within 0.3 mm of each inner face (y = +-0.150 m) and not past it
  by more than 0.3 mm, so a credited rod is in spring-loaded contact with both walls, not hovering near one.
- "Holds itself up" (0.5 and 1.0) also needs each cap centre on its wall's face: between the front and back edges
  (x 0.250 to 0.420 m) and not above the wall top, so at least half of each cap bears on its wall; the rod's lowest
  point at least 3 mm above the table, so the table is not carrying it; the rod still; and the rod let go (below).
- Full credit (1.0) adds: each cap centre at least 15 mm (half a cap) inside both edges; each cap's top not above
  the wall tops; both cap centres 120 to 150 mm above the table; the two within 10 mm in height; and within 10 mm of
  each other in x ("square across the gap"). A rod that holds itself up but misses any of these scores 0.5.
- "Still" is moving under 1 cm/s and turning under 0.5 rad/s. Every limit is compared with a 1e-6 margin.
- **Let go:** every control step, an `on_step` hook measures the closest distance from either arm's gripper and
  finger links to the rod's boxes. The rod earns nothing while that distance is under 5 mm, so a rod held still
  between the walls by a gripper scores 0, in or out of the band. Before the first measurement after a reset the
  check fails closed (score 0). `near_misses.py` sets a flag so its offline cases stand for states with the grippers
  away, and its `_run()` (run `python near_misses.py`) also checks the rule with a gripper at 0, 4.9 and 5.0 mm.

## Scripted demo

`demo()` drives both arms with the fingers closing front to back across the rod (the left arm turned -90 degrees,
the right +90). Each grips near an end (13 cm either side of the midline), they lift the rod to the band's height,
bring it to 25 mm in front of the walls, squeeze it by moving 30 mm toward each other each (the trace shows the
slide at about 46 mm), carry it straight back between the walls to their middle and open together. Within about 2.5 s they rise, back
out of the gap and move aside, then wait. The 4 s success hold starts only once no gripper is within 5 mm of the
rod; in the final demo run it ended at about 18.1 s, with the last part showing the rod alone. The rod
springs out to the walls (slide about 21 mm) with its caps about 131 mm up, and holds.

## GPU traces while building

- The bar's riser first stood partly under the tube end, 3.5 mm too high there; the rod started tilted. The riser
  was moved clear of the tube.
- The grippers, turned 90 degrees, caught the front corner of a wall on the way down to the rod; the rod's start was
  moved 30 mm nearer the robot and an approach waypoint added. Offline check of the path: at least 16 mm from the
  walls while between them, and 12 mm between the two arms at the closest.
- With the hold extended (`SUCCESS_HOLD_S` raised in a copy of the task), the released rod stayed fitted, without
  measurable creep (under 0.1 mm), for the 5.8 s that remained after the demo ended.
