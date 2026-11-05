# Fit toe-boards along a platform's two open edges

A raised work platform has walls on two sides and two open edges. The robot stands three loose pine toe-boards on
the platform along the open edges, each against the edge, so that the barrier has no gap wider than 6 mm at the
joint between boards, at the corner or next to a wall, without knocking any of the three balls on the platform
off it.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena), run through Inspect Robots on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Stand the three wooden toe-boards on the platform along its two open edges, the edge nearest you and the left edge, with each board's outer face against the edge and no gap wider than 6 mm between boards, at the corner or next to a wall. Do not let any ball fall off the platform."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/toe-boards-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/toe-boards-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in
`task.json`.

- **Runtime per trial:** at most 90 s of simulated time (1,800 steps at 20 Hz); one to two minutes of wall time on
  an L40S once Isaac Sim has started (its first start takes several minutes).
- **Seeds:** none needed. Submissions run with `-E object_jitter=0`, so every object starts exactly where
  `task.py` puts it and the layout is the same every trial. With jitter on, seeded resets shift only the loose
  objects (the boards and balls); large jitter can push a near board against the platform, so use
  `object_jitter=0`.
- **Trial history:** `score()` remembers whether a ball has left the platform during the trial, and clears that
  record when every loose object is back at its starting place. Run one trial per process, or reset the layout
  between trials.

## Scripted demo

`demo()` drives both arms. The left arm grips the left near board across its thickness (fingers turned to close
front to back), sets it on the platform against the near edge, then sets the side board against the left edge
with its near end 1.5 mm behind the first board. The right arm waits, then sets the right near board against the
near edge, 2.5 mm from the right wall. The planned gaps are 0.5 to 3 mm. Each board is let go with its bottom
commanded 6 mm above the platform top, so it is standing free when the score counts it.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: every board set (1.0); a 6 mm middle joint or corner gap (1.0) and a 7 mm one (0.87); a board
4 mm inside the edge (1.0) and 5 mm inside (0.64); a board overhanging the edge by 4 mm (1.0); only one or two
boards set (0.27 to 0.50); the side board alone along the near edge (0.37); a board turned over or with its other
face out (1.0); leaning 5 degrees (1.0) and 6 degrees (it covers nothing); turned 2 degrees about the vertical
(1.0) and 4 degrees (0.64); a board lying flat, on its end, held 1.5, 2.5 or 10 mm up, or still moving (it covers
nothing); boards stood on the table against the platform, or set 20 mm in from the edges (0); a ball held above
the platform (0); a ball resting against a board (1.0); a ball fallen to the table (0), and the same trial after
it is put back (0). Rotated cases use real quaternions, with the rotated bounding box the simulator reports.
