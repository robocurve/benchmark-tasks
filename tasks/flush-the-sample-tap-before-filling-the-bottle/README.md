# Flush the sample tap before filling the bottle

A sample tap on a standpipe, a waste cup under its spout, a square sample bottle in a holder and a white mark on the
bench. The robot takes a water sample: open the tap, let it run to waste long enough to clear the stagnant water from
the line, fill the bottle to between its two lines, set it on the mark, and shut the tap. Water operators do this at
every sample point; Standard Methods 1060 B tells them to flush the line first.

The task needs both arms: the left opens the lever, handles the bottle and sets it down, and the right shuts the lever
once the bottle is on the mark.

## Files

| File | What |
|---|---|
| `flush_the_sample_tap_before_filling_the_bottle.py` | The yam_arena task file: objects, the water ledger, `success()`, `score()`, `distance()` and the scripted `demo()` |
| `benchkit.py` | Tools built from several boxes with sliders (the lever), and per-step hooks |
| `plantkit.py` | The flow lamp, the clock and the slider checks |
| `test_flush_the_sample_tap_before_filling_the_bottle.py` | Offline tests: no GPU needed |

`benchkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=flush_the_sample_tap_before_filling_the_bottle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take a water sample: open the tap and let it run to the waste cup for at least ten seconds, then hold the bottle under the tap and fill it to between the two lines, set the bottle on the mark, and close the tap." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tap-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=flush_the_sample_tap_before_filling_the_bottle.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Take a water sample: open the tap and let it run to the waste cup for at least ten seconds, then hold the bottle under the tap and fill it to between the two lines, set the bottle on the mark, and close the tap." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tap-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/tap-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial always runs to its step limit and scores the last step, so a robot that
sets the bottle down still has time to shut the tap, and one that opens it again loses the credit. The scripted demo
stops by itself shortly after its plan ends.

Offline tests (no GPU), with `yam_arena` importable:

```bash
python -m pytest -q test_flush_the_sample_tap_before_filling_the_bottle.py
```

## Scoring

Scored from the end state and the water ledger. The tap runs at 15 mL/s while the lever is pushed at least 70% of its
3 cm travel and is shut when it is back within 30%. Water lands in the bottle when the mouth is within 2 cm of the
point under the spout's tip, below it and above the cup's rim, with the bottle leaning under 30 degrees; otherwise it
runs into the waste cup, or onto the bench if the tap has been moved off the cup or knocked over. A bottle holding
water that leans past 30 degrees empties onto the bench, and past 220 mL it overflows. **Flushed** means at least
150 mL (10 s) ran to waste before the first drop entered the bottle. **Filled** means 150 to 200 mL at the end,
between the lines. **On the mark** means the bottle's centre within 3 cm of the mark's, its bottom under 2 cm up,
leaning under 15 degrees, and no gripper on its post.

| Outcome | Score |
|---|---|
| Flushed, filled, bottle upright on the mark and let go, tap shut | 1.0 |
| Flushed and filled, but the tap left open or the bottle not set on the mark (still held, or anywhere else) | 0.7 |
| Flushed and tap shut, bottle not filled (under 150 mL, or untouched) | 0.3 |
| Bottle over 200 mL (overfilled): no fill credit, so at most | 0.3 |
| Filled, on the mark and tap shut, without the flush | 0.2 |
| Filled without the flush, and the tap open or the bottle not on the mark | 0 |
| Tap left running with nothing in the bottle | 0 |
| More than 30 mL on the bench, whatever else happened | at most 0.5 |
| Nothing done | 0 |

## Design notes

- **The flush is the rule, so skipping it scores below flushing alone.** A bottle filled without the flush, however
  neatly set down, earns 0.2, less than the 0.3 for a robot that flushed, shut the tap and never filled the bottle.
  The ledger records how much had run to waste when the bottle took its first drop, so filling first and flushing
  afterwards does not count.
- **An unfinished job keeps most of the credit.** Flushed and filled with the tap left open, or with the bottle still
  in hand or off the mark, is 0.7: the sample itself is good, the plant is not left as found.
- **The trial doesn't end when the safe state is reached.** With the usual 1 s hold, a robot that shut the tap and
  then opened it again, or filled the bottle before the flush and flushed after, could end a trial early.
  `SUCCESS_HOLD_S` is longer than the run, so the end state decides.
- **Water is a ledger.** The simulator has no liquids, so the task counts millilitres each control step from the
  lever's position and the bottle's mouth under the spout. The flow lamp at the tap's foot lights while the tap runs,
  standing in for the stream a person would see. The bottle's level is not drawn; the ledger's volume is what the
  lines would show on a real bench.
- **The bottle has a handle post.** A straight-down gripper is a solid column 7 cm wide and 20 cm tall above its
  finger tips, and the bottle's mouth is only 3.5 cm above where the fingers hold a 9 cm bottle, so a bottle gripped
  by its body can't be put under a spout: the spout would be inside the gripper. The holder's post sits 7 cm to the
  left of the bottle's mouth, which keeps the gripper's body 1.9 cm clear of the spout's tip. The post is held with the
  pads 4 cm apart on the way in, 1.5 cm clear of the bottle's rim.
- **The spout points left, the lever points at the arms, and the lever is long.** The same gripper column rules out
  pushing a short handle right under the spout or next to the pipe, so the spout runs 7 cm to the left along the top
  of the pipe and the lever is an 8 cm bar pushed near its tip, 1 cm short of the pipe. The bottle slides in under the
  spout from the left at a height with 2 cm under the spout and 2 cm over the cup's rim, which is why the cup is 4 cm
  tall and the pipe 20 cm.
- **Nothing latches.** The lever has no spring and stays where it is pushed; the scorer reads it at the end. The tap
  weighs 3 kg on a flange and stays put when pushed.
- **Unintended strategies.** Shoving the tap off the cup or knocking it over: the stream lands on the bench, and 30 mL
  there caps the score at 0.5. Tipping the bottle to lose an overfill: the first 30 mL spilled caps the score, and the
  bottle is no longer filled. Overfilling: no fill credit, at most 0.3. Dropping the bottle into the waste cup: the
  holder's bar lands on the cup's wall, so it can't stand there, and a bottle lying anywhere isn't on the mark. Holding
  the bottle on the mark: 0.7, the gripper's tool point is still at the post. Opening the tap after the bottle is
  down: scored as open.
- **Demo trims.** `TRIM` holds per-spot offsets for the open-loop demo, all zero until the GPU run says otherwise.

## Runtime and seeds

- The scripted demo takes about 47 s of simulated time (934 steps at 20 Hz): the left arm's plan ends at 38 s and the
  right arm shuts the tap after it.
- Dream-1 runs the full 2,400 steps (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
