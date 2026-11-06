# Put back the bottle you had to move to reach a spice jar at the back of a pantry shelf

A low pantry shelf holds three tall bottles along its front and three short spice jars behind them. The blue jar
is in the middle of the back row, boxed in: 10 mm from the white jars on each side and 10 mm behind the middle
bottle, gaps narrower than a gripper finger, and the bottles are too tall to lift the jar over. To take it out
the robot has to take the middle bottle off its spot, and the instruction asks it to put back anything it moved.
The fetch earns credit only when the bottle is back on its spot.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the blue spice jar from the back of the shelf on the yellow mat. Put anything you move back where it was."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/dream

# Scripted demo (reaches 1.0 after about 33 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 750 --store-frames --log-dir logs/demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started (its first start takes several minutes).
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`, so every item starts exactly where `task.py`
  puts it. The task needs `object_jitter=0`: the jars stand 10 mm apart, and jitter could push them into each
  other, the bottles or the wall.
- **Trial history:** the scorer remembers whether anything has been knocked over (tilted past 45 degrees, lying
  down, or off the table) since the trial started, and scores 0 from then on. The record clears when every item
  is back at its starting pose, which is how a new trial starts.

## Why the bottle has to come off its spot

- The gaps beside and in front of the blue jar are 10 mm; a closed gripper finger is about 17 mm thick, so the
  jar can only be gripped with one finger in front of it, where the middle bottle stands.
- With the bottle slid forward far enough to free that finger space, the jar can be gripped, but it still cannot
  come out: its way forward is through the bottle, and lifting it over the 180 mm bottle needs the gripper about
  290 mm above the table there. A straight-down grip over the shelf reaches about 190 to 210 mm (computed from the
  arm's kinematics), and the 10 mm gaps leave no room to tilt the grip.
- Pushing the bottle sideways is limited by the 40 mm gaps to its neighbours, less than the 60 mm the jar needs.

So any position of the bottle that still scores (within 20 mm of its spot) leaves the jar blocked: credit needs
the bottle taken clear and then put back.

This was tried in the simulator with a scripted attempt (not part of the submission): slide the middle bottle
25 mm forward, grip the blue jar through the opened gap, lift it as high as the arm goes and carry it out over the
bottle. The jar caught the bottle and was knocked over along with a white jar, and the trial scored 0 throughout.

## Scripted demo

`demo()` drives the left arm alone, with the fingers part open (pads 76 mm apart) so they fit the 40 mm gaps
between the bottles. The bottles' tops are above the height a straight-down grip reaches over the shelf, so the
demo takes the bottle from the front:

1. From its start pose (above and just behind the left bottle) the arm pulls back in front of the shelf, then
   slides its open fingers in level with the middle bottle's top, one either side, grips 35 mm below the top,
   lifts it 15 mm, draws it forward off the shelf and stands it on the table 18 cm in front of the bases and
   27 cm left of the midline.
2. It turns the grip a quarter turn so the fingers close front to back, one finger in the space the bottle left
   and one in the 35 mm gap in front of the wall, lifts the blue jar 15 mm, draws it out through that space and
   stands it on the yellow mat.
3. It grips the parked bottle from the front again and stands it back on its spot.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: the bottle put back 9 mm off (1.0), 12 mm off (0.5) and 19 mm off (0.5); the bottle only slid
25 mm forward or pushed 35 mm sideways and never put back (0); the bottle stood in the blue jar's old spot (0);
two bottles or the two white jars swapped (1.0, since they are identical); the bottle left on the table or
still held (0); the shelf cleared onto the table (0); the blue jar overhanging the mat, held above it, still
moving, or left on the shelf (0); and anything knocked over, including an item stood back up afterwards (0).
