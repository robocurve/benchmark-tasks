# Checks for the roll-race port (TK-PCCB8A)

All runs on one TITAN RTX, Isaac Lab-Arena 0.3.1 with yam_arena.

## 1. Physics: the three kinds roll as theory says

An ideal rolling cylinder with I = k m r^2 accelerates down the 15 degree ramp at g sin(15) / (1 + k). From rest
against the closed gate the run is 17.8 cm down the slope and 18 cm on the flat to the finish line, which gives
690 ms (solid, k 0.50), 724 ms (thick-walled, 0.65) and 773 ms (thin-walled, 0.88). Measured in the simulation
(noise study below, final geometry, 40 runs each): 701.3 +- 1.5 ms, 736.1 +- 1.5 ms and 783.6 +- 2.2 ms
(mean +- sd). The simulated times are 1.4 to 1.7 % slower than ideal (energy lost at the ramp-foot kink), and the order and gaps are
as theory says. Log: `noise_probe.log`; probe: `probe_race.txt`; summary: `analyze_noise.py`.

## 2. Significance test: run-to-run noise, threshold, false-positive rate

Probe `probe_race.txt` (not a policy): it puts two cylinders behind the closed gate with a start scatter as large as
a valid start allows (+-4 mm across the lane, 0 to 4 mm along it, +-3 degrees of yaw, uniform), opens the gate and
reads the scorer's finish times. 60 races: 30 between identical cylinders (10 each kind) and 30 between different
kinds (each pair in both lane orders). Output of `python analyze_noise.py noise_probe.log`:

```
races: 60, with both finish times: 60, missing a time: 0
  thin : n 40  mean   783.6 ms  sd   2.2 ms  range 780.3-788.5 ms
  thick: n 40  mean   736.1 ms  sd   1.5 ms  range 733.9-742.3 ms
  solid: n 40  mean   701.3 ms  sd   1.5 ms  range 697.4-705.9 ms
identical pairs (lane 0 minus lane 1): n 30  mean -0.3 ms (lane bias)  sd 3.6 ms  max |gap| 8.4 ms
threshold 20 ms: false positives 0/30 identical races (a gap above the threshold between identical cylinders)
threshold / largest identical gap = 2.4; threshold / sd = 5.5
  solid vs thick: n 10  slower-kind lag mean 34.7 ms  min 32.9 ms  wrong order 0  below threshold 0
  solid vs thin: n 10  slower-kind lag mean 82.8 ms  min 79.8 ms  wrong order 0  below threshold 0
  thick vs thin: n 10  slower-kind lag mean 48.2 ms  min 45.7 ms  wrong order 0  below threshold 0
```

- **Noise.** The gap between two identical cylinders has sd 3.6 ms and never exceeded 8.4 ms in 30 races. Lane bias
  (lane 0 minus lane 1, identical cylinders) is -0.3 ms.
- **Threshold.** A race counts only when the gap is above 20 ms: 2.4 times the largest identical gap seen and 5.5 sd.
- **False-positive rate.** 0 of 30 identical races cleared 20 ms. Under a normal model with sd 3.6 ms, a gap above
  20 ms has probability 3e-8 per race.
- **Power.** The closest two kinds (thick vs thin) differ by 45.7 ms or more (mean 48.2), solid vs thick by 32.9 ms or more (mean 34.7): every
  one of the 30 mixed races cleared the threshold in the right order (0 wrong, 0 below threshold).
- **Old design, for the record.** With 6.8 cm walled lanes and +-8 degrees of start yaw, a skewed cylinder rubbed a
  wall and one identical race gave a 140 ms gap. That is why the lanes are now painted and a start must be squared
  within 3 degrees.

## 3. Baselines (all must score 0)

Final task (v6: fixed-inertia bodies dealt to colours at reset). Runs one
process at a time; `python3 tabulate.py noop.log rj.log rr.log onerace.log guess.log`.

| Policy | Seed | Score | What happened |
|---|---|---|---|
| do nothing | 0 | 0 | no run, bin empty, nothing fell (600 steps) |
| random joint targets | 0 | 0 | no run, bin empty, nothing fell (4,800 steps, the full limit) |
| random joint targets | 1 | pending | |
| random reach | 0, 1 | pending | |
| one race, bin the loser | 0, 1 | pending | |
| bin without racing | 0, 1 | pending | |

On the v5 task (same rules, runtime-set inertia) the same baselines all scored 0: guess on seeds 0-3 (twice the
thin tube binned by luck, 0 because no run shows it), one race on seeds 0, 1 and 3 (green, the thin tube, binned
on seed 1 and still 0).

## 4. The demo uses only observable inputs

`grep -n -E "_STATE|kinds|thin_name|inertia|K_OF|hidden|proven|slower_pairs|report|debug_line|set_mass|get_mass"
script.txt guess.txt one_race.txt random_joints.txt random_reach.txt noop.txt` finds nothing (exit 1). The only names
the demo reads from the task file are layout constants and geometry helpers: `BIN_XY BUTTON_XY CRADLE_Y FINISH_Y
LANE_X PIT_Y R RACK_X THETA TOP Y_GATE axis_yaw surface`. From the simulator it reads the cylinders' poses (what the
cameras show: where each band colour is, and how it is turned) and its own joint angles. It decides which cylinder
lost each race from which one crosses the finish line second, as seen in the cylinders' positions. It never reads
which colour is which kind, the inertias, or the scorer's run records. (`DEMO_DEBUG=1` makes the runner, not the
policy, print the scorer's state; the published runs were made without it.)

## 5. Inspect score equals the README rubric

`score()` and `success()` both call `_update()` in task.py; it returns exactly the README table:

| README row | `_update()` |
|---|---|
| a cylinder off the table at any point: 0 | `_STATE["failed"]` (set in `_monitor` when any cylinder centre is 5 cm below the table top) returns 0 |
| any other cylinder in the bin: 0 | `if binned - {thin}: return 0` |
| runs do not show the thin tube slower than both: 0 | `if proven_slowest() != thin: return 0` |
| shown slowest, thin tube alone in the bin, released, all at rest: 1.0 (held 1 s) | `binned == {thin} and still and not held` returns (1.0, True); the 1 s hold is yam_arena's `SUCCESS_HOLD_S` |
| shown slowest, bin otherwise empty: 0.5 | the remaining case returns 0.5 |

The README's timed-run and "what a run shows" rules are `start_lane()` (2 cm, 8 cm, 3 degrees, 5 cm/s), the 6 cm held
test in `_monitor`, `RACE_WINDOW = 4.0`, `GAP_MIN = 0.020` in `slower_pairs()`, and the transitive closure in
`proven_slowest()`. There is no other partial credit: the old 0.25 (touched) and 0.5 (rolled past the line) rungs that
the reviewers flagged are gone, so a run that only touches things, presses the button or rolls cylinders scores 0.

## 6. Every demo action feeds the answer

- Race 1 (red vs green): needed, it compares two of the three.
- Bringing race 1's loser back from the catch pit through the middle cradle: needed, it is one of the two cylinders
  of race 2, and the right arm cannot reach the ramp top nor the left arm the pit.
- Race 2 (race 1's loser vs blue): needed; with race 1 it shows one cylinder slower than both others, directly or
  through the chain.
- Each button press: one opens the gate for a race; the press after each race closes it so the next start is behind
  a closed gate.
- Binning the slowest one: the goal.

Nothing else is done: no cylinder is moved that is not raced next or binned, and no third race is run. The race-1
winner and the race-2 winner stay where they stopped, in the catch pit.

## 7. Materials and look-alike claims

All three: OD 50.0 mm, 60 mm long overall (56 mm body + two 2 mm ABS caps), one grey epoxy paint over body and caps,
10 mm coloured tape bands at both ends. Masses (density x volume, body 56 mm long):

| Part | Body | Body volume | Body mass | + caps (2 x 4.1 g) | k of body | k with caps |
|---|---|---|---|---|---|---|
| thin-walled tube | C360 brass 8.50 g/cm3, ID 44.24 mm | 23.9 cm3 | 203 g | 211.2 g | (1 + 0.783) / 2 = 0.891 | 0.876 |
| thick-walled tube | 6061 aluminium 2.70 g/cm3, bore 28.12 mm | 75.2 cm3 | 203 g | 211.2 g | (1 + 0.316) / 2 = 0.658 | 0.652 |
| solid rod | G10 glass-epoxy 1.85 g/cm3 | 110.0 cm3 | 203 g | 211.6 g | 0.500 | 0.500 |

The simulation uses k = 0.88, 0.65 and 0.50 and one mass, 211 g. Same size: yes by construction. Same mass: within
0.4 g. Same finish: one paint over everything, the same caps and tape. Not separable by a magnet (none is
ferromagnetic). They would sound different if tapped; the robot cannot hear, and the README tells the trial operator
not to tap them. In the simulation the three bodies are the same collider and the same colours apart from the bands.

## 8. Real-world relevance and today's rejection flags

Relevance: receiving and quality-control inspectors (O*NET 51-9061) sort capped, painted round parts of the same size
and weight; the failure that matters is rejecting the wrong part and passing the thin-walled tube on, where it buckles
under a clamp or bearing load, and the score is 0 whenever any other cylinder is in the bin.

| Flag | Self-check |
|---|---|
| Egregious simplification | Real part sizes and materials; the cylinders are real cylinders with real inertias; the only code-driven piece is the gate toggle (a solenoid in the real build). |
| Difficulty isn't robotic | Each timed start needs a cylinder released behind the gate, squared to the lane within 3 degrees and 2 cm of the lane centre; the demo needs closed-loop placement for it. Recovering cylinders from the catch pit, the hand-over and binning are two-armed work. The reasoning (which pair to race next) is one step. |
| Miniaturization | 50 mm x 60 mm, 211 g parts are ordinary bar-stock offcut and roller sizes; the ramp is an inspection-bench fixture, not a scaled model of a larger one. |
| Invented event | None. |
| Variant / duplicate | This is the revision of our own TK-PCCB8A. No other board task races cylinders or tells hollow from solid parts (board search for hollow, inertia, ramp, race, tube, solid rod). |
| Reviewer's question on TK-PCCB8A ("actual materials", "look-alike") | Answered in section 7 and the README materials table. |

## 9. Reproducibility over seeds

Seeds 0, 1, 3, 5, 7 and 11 cover all six kind-to-colour orders.

| Seed | Kinds (red, green, blue) | Race 1 (ms) | Race 2 (ms) | Binned | Score | Task |
|---|---|---|---|---|---|---|
| 7 | thin, solid, thick | red 784.5, green 700.4 | red 780.8, blue 734.7 | red | 1.0 (2,558 steps) | v6 |
| 0 | solid, thin, thick | red 702.6, green 785.6 | green 787.0, blue 736.2 | green | 1.0 (2,565 steps) | v6 |
| 1 | thin, thick, solid | red 784.1, green 737.3 | red 784.8, blue 702.0 | red | 1.0 (2,385 steps) | v6 |
| 3 | solid, thick, thin | red 700.9, green 736.3 | green 737.7, blue 788.1 | blue | 1.0 (2,433 steps) | v5 |
| 5 | thick, solid, thin | red 735.3, green 700.7 | red 736.8, blue 785.6 | blue | 1.0 (2,493 steps) | v5 |
| 11 | thick, thin, solid | red 731.4, green 784.9 | green 781.9, blue 735.8 | green | 1.0 (2,333 steps) | v5 |

No race needed a re-run and no start needed re-placing in the v6 runs. On v5, seed 7 scored 0: blue (thick) ran
785.9 ms because the inertia set at reset had not taken effect, the race tied, and that demo binned the loser
without checking the gap. v6 fixes both: inertias are fixed at spawn, and the demo re-runs any race whose gap is
not over 25 ms and bins nothing without a clear result. In seed 11 (v5) blue ran 735.8 ms although it was the
solid rod, the same stale-inertia fault; that run still scored 1.0, since the thin tube lost both races.

## 10. Accepted-task checklist (ACCEPTED_CHECKLIST.md) and the closest accepted tasks

| # | Item | Status |
|---|---|---|
| 1 | Concrete job, O*NET code, the failure that matters | Yes: receiving / QC inspectors, O*NET 51-9061; rejecting the wrong part passes a thin-walled tube on to buckle under load (context field, README "Why this task"). |
| 2 | Reasoning inside real work | Yes: sorting sealed look-alike round stock; the comparison is a physical roll test with a real two-lane fixture. |
| 3 | Category | Impact on jobs. |
| 4 | Real sizes, materials, masses; no shrinking or stand-in shapes | Yes: 50 x 60 mm, 211 g brass / aluminium / G10 parts (section 7); a real cylinder collider, no stand-in. |
| 5 | Setup with frame, coordinates, colours, sizes, masses, fixed vs loose | Yes: README design table and task.json setup. |
| 6 | Honest fidelity | Yes: README / task.json fidelity (code-toggled gate, set inertias, no rolling resistance, slick facing). |
| 7 | Short plain instruction, clever part not spelled out | 64 words; it does not say how many races, which pair, or that inertia is the cue. |
| 8 | Scoring ladder with zero gates, exact thresholds, held 1 s, released, doing nothing = 0, random never reaches a rung | Yes: 0 / 0.5 / 1.0 with zero gates for a wrong part in the bin and a cylinder off the table; thresholds in cm, degrees, ms; random baselines in section 3. |
| 9 | Inspect score equals the README rubric | Yes, section 5. |
| 10 | Dream-1 0.0 and demo 1.0, run limit | Demo: section 9. Dream-1: 0.0 over 4,800 steps at seed 0. Run limit 4,800 steps (4 min): two races plus hand-overs take 2,500 steps in the seed-0 demo. |
| 11 | Demo uses no privileged information | Yes, section 4. |
| 12 | Significance test with false-positive rate; random, guess, naive score 0; observable demo; every action feeds the answer; several seeds | Sections 2, 3, 4, 6, 9. |
| 13 | Differs from every accepted task | See below. |

Closest accepted tasks and how this one differs:
- **pin-fit** ("Pick the pin that fits the hole and seat it"): also picks the one right part out of look-alikes. There
  the right pin is found by trying the fit (geometry, one at a time); here no single test on one part answers it: the
  property (mass distribution) shows only in a comparison, and the fixture compares two at a time, so the robot must
  plan which comparisons to run and keep them fair.
- **hidden-ball-peg** (a hidden cause found by probing): there the hidden state is revealed by one probe (the early
  stop); here it is a ranking of three hidden quantities that needs two designed experiments and a chain of inference.
- **trace-cable-to-matching-end** (hidden structure found by acting): there each tug answers one question about one
  cable directly; here each race only orders two parts, and the answer needs the right second race.
- **analytical-balance**, **infantometer-length-read** (measurement): those read an instrument's display; here the
  "instrument" is a comparison the robot has to set up with fair starts.
