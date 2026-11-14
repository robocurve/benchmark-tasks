# Fit an inside-bolt-circle gasket into a spread pipe-flange joint and put all four bolts in

A short flanged pipe joint is held open for a new gasket. The lower pipe stands on the table with its flange 20 mm
up. The upper flange, on its own length of pipe, rests on two loose spacer blocks (the spreader) that hold it 16 mm
above the lower one. A ring gasket that just fits inside the bolt circle waits on a block in front of the gap, and
four M10 x 45 size bolts stand on the table. The gasket has to end centred on the pipe with all four bolts in. With
the two near bolts in, it can't pass between them. Bolts put through both flanges end 4 mm below the lower flange;
with the spacers out and the joint closed onto the gasket, 16 mm stands below it for a nut.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. `parts.py` next to it builds the lower pipe and flange with the gasket's support
block (one fixed part), and the upper flange with its pipe, the two spacer blocks, the gasket and the bolts (loose
parts). It patches `yam_arena`'s scene spawning and step and reset at import time, in the task's own process only,
and changes nothing on disk. `near_misses.py` holds near-successes, fixed habits and wrong outcomes with the score
each must get. It runs as plain Python without a simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit the gasket between the flanges, centred on the pipe, put all four bolts in and let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/gasket-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/gasket-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, and `task.json` states it in words.

- **Runtime per trial:** at most 70 s of simulated time (1,400 steps at 20 Hz). The trial ends at 1.0 once
  success has held for 3 s (`SUCCESS_HOLD_S`), so the video ends with the arms parked clear of the joint. The demo
  lets go of the last bolt at about 54 s and reaches 1.0 at about 57 s.
- **Seeds:** none. Run with `-E object_jitter=0`, so the layout is the same every trial.

## Scoring in the simulator

- **Score:** with the gasket in, 0.2 plus 0.2 per hole with a bolt in, in any order (1.0 with all four); with the
  gasket not in, 0. Gasket in with both near bolts and one far bolt scores 0.8, the same as both far bolts and one
  near bolt.
- **Still:** a part is still when it moves under 1 cm/s and turns under 0.5 rad/s (`resting()`), judged at each
  step.
- **Positions:** each part's `pos` is its origin: the centre of the gasket's underside, the end of a bolt's shank, the
  centre of the upper flange's underside. A seated bolt's origin is 16 mm above the table, with its head resting on
  the upper flange.
- **Bolts are judged in the upper flange's frame.** The upper flange is loose on its spacers, so a bolt's head
  height and its offset from a hole are measured from that flange's actual pose. If the spacers are taken out and
  the joint closes onto the gasket, bolts in it still count.
- **Release check:** every step an `on_step` hook places the hulls of both arms' gripper and finger links at their
  simulated poses (fingers as they are). It measures their distance to every shape of the gasket and of each bolt.
  A part counts only while no gripper is within 5 mm of it. Before the first measurement after a reset nothing is
  credited (the check fails closed). Offline there is no robot: `near_misses.py` sets the task's offline flag, and
  also checks the rule with gripper distances filled in by hand.
- **Holes and bolts:** the bolts are identical, so any bolt may fill any hole.

## Geometry

- **Gasket fit:** the gasket's outer radius (59.5 mm) is 0.5 mm inside the bolts' inner edges (65 mm bolt circle
  less the 5 mm shank radius).
- **Near bolts block the gasket:** they are 92 mm apart centre to centre, 82 mm clear, so a 119 mm gasket cannot
  pass between them.
- **Far bolts centre the gasket:** pushed against the two far bolts, it stops 0.7 mm past the axis when they sit in
  the middle of their 13 mm holes. It stops at most about 3.7 mm past when the push shoves them to the far side of
  their holes. Both are inside the 4 mm tolerance, and the near bolts then clear its edge.
- **Spacers clear the gasket:** their inner faces are 66 mm from the axis, 6.5 mm outside the gasket's edge.
- **Lower flange holes:** these are 16 mm clearance holes, so a bolt is located by the upper flange alone.
- **Closing the joint:** with the spacers out, the upper flange would rest on the 4 mm gasket, 12 mm lower than now;
  the bolts' ends would then stand 16 mm below the lower flange and 4 mm above the table.

## Habits that must not pass

The layout is fixed, so each ordinary habit was checked against it:
- **Nearest holes first** (the near holes are nearer the arms): the gasket then stops against the near bolts
  40 mm short, and the trial scores 0.
- **All four bolts first:** the gasket can't get in, and the trial scores 0.
- **Gasket first with no bolts,** then all four bolts: fair, if it is centred by eye to about 2 mm in each diagonal
  direction.
- **Gasket pushed in hard with no stops** ends 10 mm past the axis: 0.

## Scripted demo

`demo()` uses fixed waypoints and reads no simulator state. The arms take turns, so their grippers are never near
each other:
1. The left arm puts a bolt in the far-left hole.
2. The right arm puts a bolt in the far-right hole.
3. The right arm grips the gasket's tab and slides the gasket in to 3 mm past touching the far bolts, then lets go.
4. The left arm puts a bolt in the near-left hole.
5. The right arm puts a bolt in the near-right hole.

For each bolt the arm grips the head (fingers 5 mm past its faces) and carries it over the hole. It lowers the bolt
until the shank end is 2 mm above the upper flange and settles there. Then it lowers the bolt in until the head is
3 mm above its seat, and lets go. The demo never touches the spacers or the upper flange.

In demo-20261112T214236 (GPU run, 1,139 steps), every bolt went in without aim offsets and the trial ended at 1.0.
The gasket was in against the far bolts at about 31 s (0.6). In a GPU trace of the same scene it ended 2.3 mm from
the axis, and the upper flange and spacers did not move. The last frame shows the gasket's tab and all four bolt
heads with both arms parked.

## Dream-1

In dream-20261112T214256 (the revised scene, 1,400 steps, 320 x 176 cameras) Dream-1 scored 0.0. It raised both
arms and held them up for the whole trial, never touching the gasket, the bolts or the flanges.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases cover:
- the gasket tolerance boundaries: centre at 4.0 and 4.1 mm, height at 1.5 and 2 mm, tilt at 2.9 and 3.1 degrees;
- partial credit in any order: the gasket alone (0.2), one far bolt (0.4), both far or both near bolts (0.6), both far
  plus one near and both near plus one far (0.8);
- bolts with no gasket in (0), and the habits above;
- the loose upper flange: slid 2 mm with its bolts (1.0), slid 4 mm away from its bolts (0.2), and the spacers out
  with the joint closed onto the gasket (1.0);
- the bolt limits: a 3 degree lean, head 2.6 mm off centre (in), and 9 degrees (out), head height at 2 and 2.5 mm, head centre at 3 and
  3.5 mm from the hole centre;
- the release rule: a gripper 4.9 and 5.0 mm from a bolt, a gripper on the gasket, and no measurement yet;
- a moving gasket, and doing nothing.
