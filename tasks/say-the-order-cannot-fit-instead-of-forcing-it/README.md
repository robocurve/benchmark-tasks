# Pack the order that fits and flag the one that cannot, instead of forcing it

Two packing orders share the table, each with its own box and flag pad, told apart by colour. The blue order fits:
its three blocks go into the blue box flat and below the rim, though only one way round (the 140 mm block has to lie
along the box's 170 mm length, and the 90 mm and 60 mm blocks side by side along the other wall). The orange order
cannot fit in any orientation, so the right answer is to pack none of it and put the red card on the orange pad.
Half the score is for each order, so a policy that packs everything or flags everything earns at most 0.5.

Why the orange order cannot fit (a person can check this with a ruler): lying flat, its footprints add up to 16,700
mm² against the orange box's 15,000 mm²; on any other face each block stands at least 50 mm tall, above the 34 mm
rim; two stacked are 60 mm. The blue order fills 13,900 mm² of the blue box's 19,550 mm².

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena), on the `station` scene. `near_misses.py` lists near-successes and wrong outcomes
with the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable (replace `<task.py>` and `<log dir>`):

```bash
I="Pack the blue blocks into the blue box and the orange blocks into the orange box, each block lying flat inside its box and below the rim. If an order cannot all fit in its box, pack none of that order and put the red card on the flag pad of the same colour as that order instead."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=<task.py> -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1800 --store-frames --log-dir <log dir>

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=<task.py> -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 700 --store-frames --log-dir <log dir>
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 90 s of simulated time (1,800 steps at 20 Hz); one to two minutes of wall time on
  an L40S once Isaac Sim has started (its first start takes several minutes).
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`, so every object starts exactly where `task.py`
  puts it. With jitter on, seeded resets shift only the loose objects (the blocks and the card).

## Scripted demo

`demo()` drives both arms. The left arm puts blue block C along the right wall of the blue box, then A and B side
by side along the left wall. It lets go of each block with the finger tips 3 mm above the rim, so the block drops
about 31 mm onto the box floor and the opening fingers clear the walls. The right arm waits 4 s, then moves the red
card onto the orange pad and lets go 6 mm above it. The orange blocks are not touched.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: the full answer (1.0); always flag and always pack (0.5 each); the card on the blue pad (0); two
or one blue blocks packed; an orange block packed in its box (0.75), on edge or stacked in it (0.5); a blue block
above the rim (on edge, or C propped 2 degrees along its length), held 1.5 mm up or moving (only that block loses
credit; C propped 1.5 degrees still counts); an orange block flat on a flag pad (counts as table); both sides of the 5 degree flatness, 1 mm resting and pad limits; blocks
in the wrong box; an orange block leaning 20 degrees against the outside of its box (not flat: orange half 0); an
orange block moved elsewhere flat (allowed); and the card on its side, still moving, off the pad or in a box.
