# Stow bars on a narrow shelf without anything over the walkway

Three wooden bars lie on the table, pointing away from the robot. They go on a narrow shelf, 6 cm deep, which
they only fit turned crosswise. Right behind the shelf is a striped walkway, and no part of any bar may be over
it at any moment of the run, at any height: not while a bar is being turned during the carry, not when one is
set down too far back, not for one control step. A full score needs all three bars lying wholly on the shelf
with the walkway never entered. A single bar over the walkway, even one that ends up correctly stowed, scores 0.

## Files

- `no_overhang_on_the_shelf.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo()`).
- `belimits.py`: shared geometry helpers the task file imports (outlines from each bar's pose).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Put the three wooden bars on the narrow grey shelf. The striped strip behind the shelf is a walkway: no part of any bar may be over it at any time, at any height, even for a moment. Each bar must end up lying wholly on the shelf."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=no_overhang_on_the_shelf.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/noos-dream

# Scripted demo (full score): the left arm stows the two left bars, the right arm the right one; each bar is
# picked across its width, turned a quarter turn during the carry, and set down 1 cm in from the shelf's front
# edge; both arms then park in front of the shelf
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=no_overhang_on_the_shelf.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/noos-demo
```

- **Seed and layout:** `-E object_jitter=0` is required. The walkway rule is held over the whole run, and the
  scorer recognises the start of a new trial by every bar being back on its marked spot (within 2 mm and 3
  degrees).
- **Runtime per trial:** the demo finishes in about 21 s of simulated time (about 420 steps); a Dream-1 trial
  runs up to 2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 80 s for the demo and about 350 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 1 s (all three bars on the shelf, the walkway
  never entered, every bar still), otherwise `score()` at the end of the run.

## Scoring

1. **Score 0 if any part of any bar was ever over the walkway** (x 28 to 42 cm, y -22 to +22 cm), at any
   height, at any moment of the run. In simulation this is checked at every control step from each bar's real
   outline seen from above (its eight corners through its pose), and once set it holds to the end of the trial.
   In a real trial it is read from video by a camera at the table side with its lens in the vertical plane of
   the walkway's front edge (x 28 cm), so that plane shows as one line; any part of a bar beyond it, between
   y -22 and +22 cm, is over the walkway. The arms' path is not scored; only where the bars go.
2. **At the end** (grippers opened), score 0 if any bar is over the shelf and higher than its top but not wholly
   on it (ramped, leaning, or sticking out past an edge), or any bar has fallen off the table.
3. **Otherwise 1/3 per bar lying wholly on the shelf:** on one long face, tilted 10 degrees or less, its
   underside within 1 cm of the shelf top, and its whole outline within the shelf's (0.3 cm allowed). A bar
   standing on end, on another bar, still held up, or left on the table earns 0 for that bar.

Doing nothing scores 0. Set down on the walkway, pushed off the back of the shelf, or swung over the walkway
during the turn and then stowed correctly: 0. A bar laid on the shelf without being turned sticks out past both
of its edges, and in this layout over the walkway too, so it scores 0 through rule 2 even without rule 1; it is
not counted as evidence for the walkway rule.

The simulator scores the final step without opening the grippers, so a bar whose underside is more than 1 cm
above every support counts as not placed, and a bar over the walkway counts at any height, held or not.

**Known residual (simulation only):** the scorer only sees the bars, so it starts a new trial when all three are
back within 2 mm and 3 degrees of their marked spots and still. A run that crosses the walkway and then puts
every bar back exactly on its mark clears the walkway record; it would then have to stow all three again
without a crossing. This path is covered by an offline unit test. A real trial, read from video, has no
such reset.

## Real-world reset

Mark the bars' spots on the table before the first trial. Between trials, put the bars back on their marks,
square to the table edges. The shelf and the tape stay down.
