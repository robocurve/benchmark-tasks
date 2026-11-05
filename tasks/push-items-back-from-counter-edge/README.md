# Move every item at least 15 cm back from a counter's far edge

A low counter block stands on the table. Its far edge is the side a toddler could reach from, and the instruction
says a toddler reaches 15 cm in from that edge. Five kitchen items (a mug, a knife in its sheath, a medicine
bottle, a spray-cleaner box and a frying pan) all reach into that 15 cm band. The robot moves every item so that
no part of it is within 15 cm of the far edge, without pushing anything off the counter.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator. Scoring is judged over the whole trial: an item that leaves the counter at any
moment zeroes the trial even if it is put back.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="A toddler on the far side of the counter can reach anything within 15 cm of the counter's far edge. Move every item so that no part of it is within 15 cm of that edge. Keep everything on the counter."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/counter-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/counter-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in
`task.json`.

- **Runtime per trial:** at most 90 s of simulated time (1,800 steps at 20 Hz); about 2 minutes of wall time on
  an L40S once Isaac Sim has started (its first start takes several minutes).
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`, so every object starts exactly where `task.py`
  puts it and the layout is the same every trial. With jitter on, seeded resets shift only the five loose items;
  the knife starts 20 mm from the far edge, so large jitter could push it off before the trial starts.

## Scripted demo

`demo()` uses both arms (about 29 s). The left arm picks the knife, turns it a quarter turn and lays it left to
right near the counter's near edge, then moves the mug and the medicine bottle back. The right arm moves the
spray-cleaner box back, then lowers its closed fingers behind the pan's far side and drags the pan toward the
robot. Each picked item is released about 7 mm above where it was gripped, so it drops a few millimetres
onto the counter after the fingers open rather than standing on it while still held.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: all five back (1.0); the mug's far side exactly on the line (1.0) or 1 mm past it (0.8); the
pan's rim 5 mm past the line with its centre back (0.8); the pan, mug and spray box turned about the vertical with
their true far rim or corner inside the line (1.0) or 2 mm past it (0.8); the knife left front to back across the
line (0.8) or leaning 25 degrees propped on something (0.8); an item 1 mm proud (1.0), held still 2 to 2.5 mm above
the counter (0.8: that item earns nothing), still sliding, stacked, overhanging an edge or tipped 10 degrees
(0.8); the bottle or mug lying on its side, wholly back (1.0); and then, in order, an item pushed off the near
edge (0), the same item put back with everything else safe (still 0: judged over the whole trial), knocked off
the far edge or a side (0), and a reset to the start followed by all five back (1.0 in the new trial).
