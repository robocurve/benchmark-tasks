# Cook a pot on a hot plate for 15 seconds without burning it

`cook_but_dont_burn.py` is a `yam_arena` task file: the scene, the score hooks and a scripted demo.

## Run a trial

The instruction is the one in `task.json`:

    INSTR="Cook the grey pot on the red hot plate for 15 seconds, then move it off onto the table. Don't leave it on the hot plate for more than 18 seconds, or the food will burn."

Dream-1:

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=cook_but_dont_burn.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "$INSTR" --scorer task_score --grader none --max-steps 1200 --store-frames \
      --log-dir ~/yam-eval/cook-but-dont-burn/dream

Scripted demo (reaches 1.0):

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=cook_but_dont_burn.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "$INSTR" --scorer task_score --grader none --max-steps 900 --store-frames \
      --log-dir ~/yam-eval/cook-but-dont-burn/demo

## Score

`task_score` gives 1.0 on success, otherwise the score of the final state.

- Cooking time is the total time the pot rests on the hot plate: its centre over the plate and its bottom at
  most 1 cm above the plate top.
- 0 if the cooking time is more than 18 s (burnt).
- 0 if the pot tips over (more than 20 degrees from upright) or falls off the table at any time. The API has no
  failure hook, so the trial runs on to its limit, but the score stays at 0 and success can't happen.
- 0 if, at the end, the pot isn't standing still on the table (not held up), clear of the hot plate.
- Otherwise the cooking time divided by 14 s, up to 1.0.
- Success (score 1.0, the trial ends): 14 to 18 s of cooking, and the pot has stood clear of the plate for 1 s.

Cooking time and spills are a history, and the hooks get no clock. So the task file keeps its own state, and
reads the embodiment's simulation time (`YamArenaEmbodiment._t`, which goes back to 0 at each reset) from the
call stack. It changes nothing in `yam_arena`. The time is correct at any `-E control_hz` and for each of
several `--epochs`.

## Runtime and seeds

- A model trial is at most 1,200 steps (1 minute of simulated time). The run limit is much longer than the
  18 s burn limit, so a pot left on the plate always burns.
- The demo plan is 33 s and keeps the pot on the plate for about 16 s; the trial ends on success at about 33 s.
- Run every trial with `-E object_jitter=0`: the pot starts at the position in `task.json`.
