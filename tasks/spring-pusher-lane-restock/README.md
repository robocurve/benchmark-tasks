# Restock a spring-pusher shelf lane behind the older stock, holding the pusher back

Two red cartons stand at the front of a shelf lane, with a grey spring pusher resting on its stop just behind them. Two blue cartons stand on
the table beside it. The robot has to pull the pusher back and hold it open with one arm, set both blue cartons in the
gap behind the red ones with the other, without passing either over the red ones, then let the pusher forward so it closes up the row. The pusher sits below
the carton tops (its handle is on a rigid side arm at y -15 cm that crosses above the right rail, so the right arm works away from the left arm; this side-arm handle is a benchmark simplification, because a real pusher has a tab on its plate and neighbouring lanes would block a side arm) and the spring pulls it forward at all times, so the space for the new stock exists only until the first carton is in the gap. The old (red) stock must stay at the front; new stock comes in from behind so it is never carried over product already on sale.

## Files

- `spring_pusher_lane_restock.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo()`).
- `mechkit.py`: the shared helper that builds the lane and its sprung pusher from boxes (a slide joint with a spring).
- `belimits.py`: the shared geometry helpers the scorer uses (footprints, tilt, height above a support).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Pull the grey pusher back and hold it. Set both blue cartons in the lane behind the red ones, without passing either over the red ones, then let the pusher forward so it closes up the row."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=spring_pusher_lane_restock.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/splr-dream

# Scripted demo (full score): the right arm pulls the pusher back and holds it while the left arm sets both blue
# cartons in the gap, then the right arm brings the pusher forward to 11.4 cm (just outside the score band), opens, rises and draws back, and the spring closes the row
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=spring_pusher_lane_restock.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/splr-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the cartons and the pusher's rest mark are measured from
  their set positions.
- **Runtime per trial:** the scripted demo plays for about 45 s of simulated time (about 900 steps) before the policy holds the last pose; a Dream-1 trial
  runs up to 2,400 steps (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 190 s for the demo and about 460 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 4 s (full score, with every carton still), otherwise `score()` at the end of the run.

## Scoring

Words used below: **J** is how far the pusher sits from its rest mark (the mark is where its plate rests, about 1 mm behind the second
red carton); **in the lane** means a carton's outline is inside the rails and the front wall (touching, or within 2 mm past a face, counts as inside), its bottom is within
1 cm of the lane floor (tape equivalent: its top at most 9.8 cm above the table), it is tilted at most 10 degrees (top edge at most 1.4 cm sideways of the bottom edge), and it lies wholly in front of the pusher plate (its back face at most 4 mm past the plate face).

1. **Score 0** if a blue carton is ever over the red cartons' place (from the front wall to their back face, rails
   included) at any height, or a blue carton in the lane ends nearer the front wall than a red carton, or a red carton is not in the lane.
2. **1.0**: all four in the lane, J 9.2 to 11.0 cm, and every carton still (under 1 cm/s), held 4 s or, if the run ends first, on its last step.
3. **0.75**: all four in the lane otherwise (J outside the range, or a carton still moving).
4. **0.5**: both reds and one blue in the lane.
5. **0.25**: the pusher was 6 cm or more back for 1 s (20 steps) while both reds were in the lane and within 1 cm of their start positions.
6. Otherwise 0.

Doing nothing scores 0. Two blue cartons and two reds take up 20 cm, so a row with no gaps leaves J about 9.8 cm (row 6.5 to 26.5 cm, plate rest at 16.7 cm) and
the 9.2 to 11.0 cm band allows for small gaps; a ruler on the slide reads it. The pusher is measured at the end, but "ever over the red cartons' place" is
latched over the whole run (the simulator clears the latch when the embodiment resets between trials, through mechkit's `on_reset`; carrying a carton back by hand does not). A carton on top of another, held up, leaning on a rail, lying on its side or set behind
the pusher is not in the lane. A carton pushed in from the side cannot pass the rails. A carton in the lane can push the plate back.

## Real-world reset

Stand the red cartons against the front wall (the second about 1 mm from the pusher plate; the pusher rests on its stop) and
the blue cartons on their marks on the table. Measure J on a ruler along the slide before the first trial: it should
read 0, and put a mark on the rail at 6 cm back from the rest mark. In a real trial the grippers are opened before J and the cartons are measured; the simulator scores the final
step without opening them, so a carton still held in the air never scores better than it would if released.

**Measuring:** read the plan tests from a top-down video taken by a camera directly above x 16.6 cm, y 0, at least
1 m above the table, so the back-face line reads without parallax, and the bottom and tilt tests from a side photo from a camera at table height, 1 m to the left of the lane, looking along y. An overlap under 3 mm between a blue carton and the
red cartons' place is contact, not "over". The slide travels 0 to 20 cm and the spring (15 N/m) pulls with about 2.9 N at the 19 cm full pull. The pusher's damping is 10 N s/m (a soft-close damper on the drawer slide gives this in a real lane). The lane is 8 cm wide between the rails, so a 5 cm carton has 1.5 cm to spare each side. "Let the pusher forward" is all that is
asked: releasing it outright scores the same as easing it.
