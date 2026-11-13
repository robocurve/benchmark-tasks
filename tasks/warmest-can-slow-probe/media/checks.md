# Checks for the cold-chain thermometer task (revision of TK-GA3ZMU, ported to Isaac Lab-Arena)

All runs on one TITAN RTX, Isaac Lab-Arena 0.3.1 with yam_arena.

## 1. Reviewer feedback on TK-GA3ZMU

> "Use a demo that only uses observable inputs, and align the Inspect score with the rubric."

- **Observable-only demo.** `script.txt` decides only from the number on the display, decoded from which LED segments
  are lit (`Seg7.read`, the same prims the cameras render), plus the cans' visible poses, its own joints and its own
  clock. Grep of `script.txt` for hidden-state names:
  `grep -nE "TEMPS|WARMEST|_STATE|TAU|SIGMA|sensor|readings|noise|draw\(" script.txt` matches only comment lines
  (6-8, 226). The script calls `task.settled_from` and `task.winner_z` on its own list of readings: these are pure
  functions of (time, value) lists, the same rules stated in task.json's scoring text.
- **Inspect score = rubric.** `score()` in `task.py` is the rubric itself: the gates and the 0.5 / 0.75 / 1.0
  rungs in task.json's scoring text, computed from the log of displayed readings and the end state. `task_score` in
  Inspect is 1.0 on success, else `score()` at the end; the demo's `result.json` records both `task_score` and the
  rubric score from `report()`, and they agree on every run below (table in section 3).

## 2. The significance test and its error rates

Rule (task.json scoring, `settled_from` and `winner_z` in task.py): settled readings per can, then the warmest mean
must beat each other can's by more than 3 standard errors of the difference, SD pooled from the readings' own
scatter. It uses nothing but displayed readings and their times.

Offline model (`model_check.py`): the same sensor rules and the same scoring functions as task.py, without the
simulator, run over hidden draws (seeds 0-299), with 8 s of carrying between measurements.

| Procedure (500 hidden draws, `mc/final_500.json`) | Mean score | Full score | Picks the warmed can | Proves a wrong can | Time (median / max) |
|---|---|---|---|---|---|
| Demo procedure (watch until settled, test, re-measure) | 0.99 | 496 / 500 | 500 / 500 | 0 | 164 s / 214 s |
| Quick: one reading 2 s after setting each can down | 0 | 0 | 345 / 500 | 0 | 96 s |
| Quick: one reading 10 s after setting each can down | 0 | 0 | 471 / 500 | 0 | 120 s |

- **False proofs.** Over 1,500 procedure runs above, the rule never proved a can that was not the warmest.
- **Null test.** With all three cans at the same temperature (6.0 C), the demo procedure, re-measuring up to 4
  rounds, ended with a "proof" in 7 of 500 draws (1.4%). One comparison at 3 standard errors has a one-sided false
  rate of 0.13%; the excess comes from up to five looks and from the minimum over two comparisons.
- **Why 3 standard errors, and the 12-reading settle rule.** With 2 standard errors and an 8-reading settle rule
  (the MuJoCo version) the null-test rate was 12-15% and the quick 10 s reading reached full score in 11% of draws:
  unsettled readings slipped into the means. The stricter rules remove both (rows above).
- In 4 of 500 draws the demo procedure still had no proof after 4 re-measuring rounds and scored 0 (the 0.25 rung for settled readings without a proof was removed on Nov 13, so only a proven measurement earns credit). The quick
  procedures fail by never having settled readings for every can (they score 0 even when their pick is right).

## 3. Baselines and demo in the simulator (Inspect score = rubric score)

| Run | Steps | Inspect task_score | Rubric score | Proven | On mat | Gate | Settled readings (b/g/y) | z |
|---|---|---|---|---|---|---|---|---|
| Demo, hidden draw 0 (warmest blue) (`demo_s0`) | 3645 | 1.0 | 1.0 | can_blue | can_blue | - | 42/35/37 | 9.15 |
| Demo, hidden draw 0, final script (recorded video) (`demo_s0_final`) | 4027 | 1.0 | 1.0 | can_blue | can_blue | - | 40/40/39 | 9.62 |
| Demo, hidden draw 1 (warmest green) (`demo_s1`) | 4011 | 1.0 | 1.0 | can_green | can_green | - | 39/36/42 | 8.79 |
| Demo, hidden draw 2 (warmest green) (`demo_s2`) | 3859 | 1.0 | 1.0 | can_green | can_green | - | 31/28/34 | 6.31 |
| Demo, hidden draw 6 (warmest yellow) (`demo_s6`) | 3913 | 1.0 | 1.0 | can_yellow | can_yellow | - | 35/33/38 | 8.87 |
| Quick: one reading 2 s after setting each can down, max to the mat (`bl_quick`) | 3140 | 0.0 | 0.0 | None | - | - | 0/0/0 | 0.0 |
| Guess: a random can to the mat, no measuring (picks a cold can) (`bl_guess_wrong`) | 962 | 0.0 | 0.0 | None | can_yellow | - | 0/0/0 | 0.0 |
| Guess: a random can to the mat, no measuring (picks the warmed can) (`bl_guess_right`) | 976 | 0.0 | 0.0 | None | can_blue | - | 0/0/0 | 0.0 |
| Random reaches and grasps, seed 0 (`bl_reach_s0`) | 6000 | 0.0 | 0.0 | None | - | can_green fell over or off the table | 0/0/0 | 0.0 |
| Random reaches and grasps, seed 1 (`bl_reach_s1`) | 2400 | 0.0 | 0.0 | None | - | - | 0/0/0 | 0.0 |
| Random joint targets, seed 0 (`bl_random_s0`) | 2400 | 0.0 | 0.0 | None | - | can_green fell over or off the table | 0/0/0 | 0.0 |
| Inspect run, yam_scripted with an empty plan (does nothing) (`insp_noop`) | - | 0.0 | see log | - | - | - | - | - |
| Inspect run, yam_scripted: blue can onto the plate and left there (`insp_one_can`) | - | 0.0 | see log | - | - | - | - | - |

Rows `demo_s0_final`, `demo_s1`, `demo_s2` and `demo_s6` use the final script; `demo_s0` is the first version (same result at draw 0).
`insp_*` rows are real Inspect runs (`inspect-robots run --policy yam_scripted --scorer task_score`, 2,400 steps),
with the eval's `results.metrics.task_score` in `logs_insp_*/*.json`. In the other rows, `task_score` is computed
the way Inspect's task_score scorer does (1.0 on success, else `score()` at the end), and the rubric score is the
ladder recomputed by `report()`; the two agree on every run. The random runs at 2,400 steps never had a can
standing alone and untouched on the plate long enough to settle; the gate rows show the random arm knocking a can
over, which itself gives 0.

## 4. Every demo action feeds the answer

Seed-0 demo (`demo_s0.log`): 3 measurements and 7 carries.
- Carry each can to the plate, park the arm clear, and watch until the can's readings settle (24-28 s), then 10 s more.
  Each measurement adds that can's settled readings (42, 35, 37) to the test.
- Carry each can back to its spot. This frees the plate for the next can, and the next measurement needs the plate.
- One significance test on the demo's own readings (z = 8.1, so no re-measuring round was needed). Its answer
  picks the can that goes to the mat.
- The warmest can goes to the mat by way of the plate: the left arm sets it on the plate and the right arm takes it
  to the mat. This is transport, needed because neither arm reaches the far side of the table with a full can; it
  adds a few more readings of the same can and changes no decision.
- No test, carry or wait is made that the answer does not use. The re-measure branch runs only when the test is
  not yet significant, and then only for the leader and the closest rival, or for a can with no settled stretch.

## 5. Reproducibility

- The final script reached 1.0 on every hidden draw it was run on (draws 0, 1, 2 and 6). Those draws cover all
  three cans as the warmest one. The table above has the rows.
- With the same seed, the physics and the noise draws are deterministic: two runs of the same script and seed
  logged the same carries.
- Earlier script versions failed at the last step, and those runs are kept for the record:
  - The first version proved the right can on draws 2 and 6 but left it leaning on the mat, scoring 0.5. The can
    hung a few degrees off vertical in the fingers and was let go above the surface. Those logs were overwritten.
  - A second version re-centred the open fingers before lifting. It reached 1.0 on draw 6 (`demo_s6_centring.log`).
    On draws 1 and 2 it re-gripped the leaning can and knocked it over, which trips the gate (0;
    `demo_s2_oldscript.log`).
  - The final version presses the can down until it is level and seated before letting go; it checks the can's
    pose to do this. It has reached 1.0 on every draw since.
- The offline model gives the statistical spread: over 500 hidden draws, 496 runs reach full score (section 2).
- Kit start-up on the shared machine sometimes failed with CUDA out-of-memory on GPU 0, which another user fills.
  Runs now set CUDA_VISIBLE_DEVICES=5 and were retried. The failed starts never reached step 1 and are not counted.

## 6. Accepted-task checklist (ACCEPTED_CHECKLIST.md)

| # | Item | Status |
|---|---|---|
| 1 | Concrete job, O*NET code, the failure that matters | Yes. Shipping/receiving clerks (43-5071), food service managers (11-9051); FDA Food Code 5 C receiving limit; a warmed can passed into stock. |
| 2 | Reasoning inside real work | Yes. Checking chilled deliveries with a slow probe is daily receiving work; the reasoning (wait for the reading to settle, repeat until the difference is real) is what a careful clerk does. |
| 3 | Category | Impact on jobs. |
| 4 | Real sizes, materials, masses; no miniaturization or stand-in shapes | Yes. Cans 73 x 110 mm, 450 g tinplate (a standard 400 g food can); real cylinders, gripped as cans. The thermometer plate is a simplification of a probe, stated in fidelity. |
| 5 | Setup with frame, coordinates, sRGB, sizes, masses, fixed vs loose | Yes (255 words; median 224). |
| 6 | Honest fidelity | Yes (136 words): code-driven sensor, rigid cans, fixed temperatures, plate stands in for a probe. |
| 7 | Short plain instruction, the clever part not spelled out | 57 words. It says what the plate and display are, not that the reading lags or jitters. |
| 8 | Ladder with zero gates, thresholds, released, doing nothing 0, random can't reach a rung | Yes. Gates: can tipped over 30 deg or off the table, display box moved over 1 cm, a cold can on the mat. Rungs 0.5 / 0.75 / 1.0; noop and random 0 (section 3). |
| 9 | Inspect score = rubric | Yes (section 1, section 3). |
| 10 | Dream-1 0.0, demo 1.0, run limit | Demo 1.0 (section 3). Dream-1: see task.json scores. Run limit 6,000 steps (5 min). |
| 11 | Demo uses no privileged information | Yes (section 1 grep). |
| 12 | Significance test with FPR, baselines 0, observable demo, every action feeds the answer, seeds | Sections 2-5. |
| 13 | No accepted task tests the same thing | Closest: **analytical-balance** (TK reads a live display while dosing granules to 1.00 g: closed-loop control to a target on an instant, noise-free reading) and **infantometer-length-read** (reading an instrument after a hard manipulation). Ours is a comparison through a slow, noisy instrument: the skill is waiting for settling and replicating readings until the difference is statistically real, which neither tests. No accepted task uses a lagging or noisy sensor for a decision. **Risk:** two other teams' tasks of the same shape were closed on Nov 12 as "difficulty isn't robotic": TK-5RE4DK (weigh three look-alike blocks on a spring scale, reject the heavy one: "identifying the heavy block from a scale reading is a non-robotic decision ... ordinary manipulation") and TK-HSSGEB (soak samples to a core temperature, learning the probe lag: "primary difficulty is non-robotic"). Our reviewer allowed a revision rather than closing, but the same flag could be raised. |
