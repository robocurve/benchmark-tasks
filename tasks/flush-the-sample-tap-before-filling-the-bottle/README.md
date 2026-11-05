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
| `flush_the_sample_tap_before_filling_the_bottle.py` | The task file: objects, water ledger, scoring, `demo()` |
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
  -E task=flush_the_sample_tap_before_filling_the_bottle.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take a water sample: open the tap and let it run to the waste cup for at least ten seconds, then hold the bottle under the tap and fill it to between the two lines, set the bottle on the mark, and close the tap." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tap-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=flush_the_sample_tap_before_filling_the_bottle.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=640 -E cam_height=352 \
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
3 cm travel and is shut when it is back within 30%. The stream **wets** the bottle when it falls inside the bottle's
4.4 cm opening with the mouth below the spout's tip and above the cup's rim, however the bottle is held; it is
**credited** to the bottle only when the mouth is also within 2 cm of the point under the spout's tip, the bottle
leans under 30 degrees, and the tap stands on its spot with its spout over the cup. Water not credited runs into the
cup, or onto the bench if the tap has been moved off the cup. A bottle holding water that leans past 30 degrees empties
into the cup if its mouth is over it, otherwise onto the bench, and past 168 mL (the brim) it overflows the same way.
**Flushed** means at least 150 mL (10 s) ran to waste before the stream first wet the bottle. **Filled** means 91 to
130 mL at the end, between the lines. **On the mark** means the bottle's centre within 3 cm of the mark's, its bottom
under 2 cm up, leaning under 15 degrees, and no gripper within 1 cm of the bottle or its holder. The tap must still
**stand** on its spot: foot within 2 cm, leaning under 20 degrees.

| Outcome | Score |
|---|---|
| Flushed, filled, bottle upright on the mark and let go, tap shut | 1.0 |
| Flushed and filled, tap shut, but the bottle not set on the mark (still held, or anywhere else) | 0.7 |
| Flushed and tap shut, bottle not filled (under 91 mL, or untouched) | 0.3 |
| Bottle over 130 mL (overfilled): no fill credit, so at most | 0.3 |
| Flushed and filled, tap left running | at most 0.25 |
| Filled, on the mark and tap shut, without the flush | 0.2 |
| Filled without the flush, and the tap open or the bottle not on the mark | 0 |
| Tap left running with the bottle under-filled or empty | 0 |
| More than 30 mL on the bench, whatever else happened | at most 0.25 |
| Tap knocked over or shoved off its spot | 0 |
| Nothing done, and any other end state not listed above | 0 |

## Design notes

- **The flush is the rule, so skipping it scores below flushing alone.** A bottle filled without the flush, however
  neatly set down, earns 0.2, less than the 0.3 for a robot that flushed, shut the tap and never filled the bottle.
  The ledger records how much had run to waste when the stream first wet the bottle, so filling first and flushing
  afterwards does not count, and neither does letting the stream splash into a badly held bottle during the flush and
  centring it afterwards.
- **A running tap and water on the bench cap the score below the honest partials.** The tap left open at the end is
  worth at most 0.25 even with a good sample, and nothing with an empty bottle; more than 30 mL on the bench caps the
  score at 0.25. Both are the harms in the task's context.
- **The trial doesn't end when the safe state is reached.** With the usual 1 s hold, a robot that shut the tap and
  then opened it again, or filled the bottle before the flush and flushed after, could end a trial early.
  `SUCCESS_HOLD_S` is longer than the run, so the end state decides.
- **Water is a ledger.** The simulator has no liquids, so the task counts millilitres each control step from the
  lever's position and the bottle's mouth under the spout. The flow lamp at the tap's foot lights while the tap runs,
  standing in for the stream a person would see. The bottle's level is not drawn; the ledger's volume is what the
  lines would show on a real bench. The volumes are the bottle's own: 4.4 cm square inside with a 3 mm floor, so the
  5 and 7 cm lines hold 91 and 130 mL and the brim 168 mL.
- **The bottle has a handle post.** A straight-down gripper is a solid column about 10 cm across and 20 cm tall above
  its finger tips, and the bottle's mouth is only 3.5 cm above where the fingers would hold a 9 cm bottle, so a bottle
  gripped by its body can't be put under a spout: the spout would be inside the gripper. The holder's post sits 7.5 cm
  to the left of the bottle's mouth, which keeps the gripper's body 2.4 cm clear of the spout's tip. The post is held
  with the pads 4 cm apart on the way in, 2 cm clear of the bottle's rim.
- **The post stands on a leg.** Out on its bar the post pulls the centre of mass 3 cm off the bottle's centre, past the
  edge of its 5 cm floor, so without support the bottle tips over onto the post (in the simulator it settled 24
  degrees over before the trial began, and the gripper closed on the bottle's wall instead of the post). A leg 1.5 x 1
  x 3 cm under the post's middle stands it on the table, with the centre of mass 2 cm inside the feet. On the mark the
  leg stands on the table beside the 3 mm plate, so the bottle leans about 3 degrees towards it: still upright.
- **The lever is pushed with the gripper turned 90 degrees.** Each wrist camera sits 7 cm in front of its tool point,
  so a straight push would put it inside the pipe and spout. Turned, the camera swings to the side, and the pads'
  long side leads the push; the sweep in the tests keeps a 5 x 5 x 4 cm box round each camera about 3.5 cm from the
  pipe and spout (the tests require 3 cm) and about 2.4 cm above the handle at every step. Each wrist can turn only
  one way within its joint limit, so the cameras swing towards the midline: the left arm's tool point ends 6 mm right
  of the midline at the open stop with its camera over the right-hand side, which is why the right arm waits 20 cm
  out until the bottle is down.
- **The spout points left, the lever points at the arms, and the lever is long.** The same gripper column rules out
  pushing a short handle right under the spout or next to the pipe, so the spout runs 8.5 cm along the top of the pipe
  with its tip 7 cm left of the pipe's axis, and the lever is a 10 cm bar pushed 2.5 cm from its tip, with the
  gripper's body 2.4 cm short of the pipe. The bottle slides in under the spout from the left at a height with 2 cm
  under the spout and 2 cm over the cup's rim, which is why the cup is 4 cm tall and the pipe 20 cm. The offline
  tests sweep a box model of the gripper and its camera through every step of the demo against the tap, the cup, the
  lamp, the bottle and the other arm.
- **Nothing latches.** The lever has no spring and stays where it is pushed; the scorer reads it at the end. The tap
  weighs 3 kg on a flange and stays put when pushed; moved more than 2 cm or leaning more than 20 degrees it scores 0,
  and its stream, which falls from wherever the spout is, misses the cup.
- **Unintended strategies.** Carrying the tap to the bottle: the stream enters the bottle but is not credited, the
  water is a spill, and the tap is off its spot. Tipping the bottle to lose an overfill over the cup: the water goes
  back to waste and the bottle is no longer filled; over the bench it is a spill. Overfilling: no fill credit, at
  most 0.3. Setting the bottle down in the waste cup: the holder's bar rests on the cup's wall and the bottle leans
  about 11 degrees with its mouth still under the stream, so it fills like a held one, which is fair; it still needs
  the flush first and the mark at the end. Holding the bottle on the mark by its post, body or rim: 0.7, a gripper's
  tool point is within 1 cm of it. Opening the tap after the bottle is down: scored as open.
- **Demo trims.** `TRIM` holds per-spot offsets for the open-loop demo. All are zero: in the simulator the bottle's
  mouth stays 2 to 3 mm from the point under the spout's tip while it fills, and the bottle ends 5 mm from the
  mark's centre, well inside the 2 and 3 cm checks.

## Runtime and seeds

- The scripted demo takes about 43 s of simulated time (868 steps at 20 Hz): the left arm's plan ends at 34 s and the
  right arm shuts the tap after it. In the simulator it scores 1.0, with 198 mL run to waste before the first drop
  and 109.5 mL in the bottle, and the run stops at step 929, 3 s after the plan ends.
- Dream-1 runs the full 2,400 steps (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
