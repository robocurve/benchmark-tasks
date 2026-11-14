# Pipette dye into three cups of water

A row of four clear cups: dark blue stock, then cups 1, 2 and 3 with water. The robot takes a squeeze-bulb transfer
pipette from its holder and moves one pipetteful at a time down the row (stock to cup 1, cup 1 to cup 2, cup 2 to
cup 3), so the cups end at a half, a quarter and an eighth of the stock's strength. Lab staff call this a serial dilution. Then it stands the pipette back
in its holder, empty, and lets go.

The skill is real liquid-handling technique on a single arm: squeeze before dipping, release under the liquid to
draw, lift, squeeze inside the next cup to dispense, and let go only once the tip is out of the liquid (releasing
while still dipped draws it back up). The order is the protocol: each cup is made from the one before it.

## Files

| File | What |
|---|---|
| `pipette_dye_into_cups.py` | The yam_arena task file: objects, `success()`, `score()`, `distance()` and the scripted `demo()` |
| `ledger.py` | The liquid ledger: volumes and contents of each cup, moved by squeezes (pure Python) |
| `benchkit.py` | Tools built from several boxes with spring-loaded sliders (the bulb's sides), and per-step hooks |
| `test_ledger.py`, `test_pipette_dye_into_cups.py` | Offline tests: no GPU needed |

`benchkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pipette_dye_into_cups.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Use the transfer pipette to dilute the dark stock by half in each cup. The dark stock cup is at the front of the row and cups 1, 2 and 3 stand behind it, front to back, each holding water. Move one pipetteful from the stock into cup 1, then one from cup 1 into cup 2, then one from cup 2 into cup 3. Then stand the empty pipette back in its holder and let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/dye-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pipette_dye_into_cups.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Use the transfer pipette to dilute the dark stock by half in each cup. The dark stock cup is at the front of the row and cups 1, 2 and 3 stand behind it, front to back, each holding water. Move one pipetteful from the stock into cup 1, then one from cup 1 into cup 2, then one from cup 2 into cup 3. Then stand the empty pipette back in its holder and let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/dye-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/dye-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial ends at 1.0 once all three cups are right and the pipette has stood
still in its holder for 1 s; otherwise it scores the last step.

Offline tests (no GPU), with `yam_arena` importable:

```bash
python -m pytest -q test_ledger.py test_pipette_dye_into_cups.py
```

## Scoring

A cup is **right** when it holds one or two pipettefuls and its strength is its target: cup 1 half the stock, cup 2 a
quarter, cup 3 an eighth. On a real bench the operator stirs each cup, reads its volume against the 5 and 10 mL
marks, and compares it in a top photo with reference cups made by hand at 1:2, 1:4, 1:8 and 1:16 at the same fill;
its closest reference must be its target. The sim makes the same call: the dye fraction is within a factor of
&radic;2 of the target, the geometric midpoint between neighbouring references.

| Outcome | Score |
|---|---|
| Cups 1, 2 and 3 right, pipette standing empty in its holder, nothing gripping it | 1.0 |
| Cups 1, 2 and 3 right, pipette held, not in its holder, or not empty | 0.9 |
| Cups 1 and 2 right (cup 3 not) | 0.6 |
| Cup 1 right (cup 2 not) | 0.3 |
| Cup 1 not right, whatever else | 0 |
| The tip went back into an earlier cup's liquid after entering a later one, or liquid was carried back to an earlier cup | 0 |
| More than one pipetteful taken from the stock | 0 |
| Any of the above with liquid squeezed out anywhere but into or just above a cup (3 cm over its opening) | at most 0.5 |
| Nothing done | 0 |

Cups count strictly in order because each is made from the one before: a right cup 2 without a right cup 1 isn't
possible by the protocol, so it earns nothing.

## Design notes

- **Why a transfer pipette.** A micropipette needs one hand to hold it and a thumb to press the plunger; on two
  parallel-jaw arms that means both grippers and both wrist cameras over the same cup, which collide. A squeeze bulb
  is worked by one gripper. Each side of the bulb gives 9 mm, about what a real bulb gives. A light grip (1.5 mm in)
  holds it by friction, about three times its weight; past 70% of travel is a squeeze, below 30% a release. So
  holding never counts as squeezing, and the gripper has a 5 mm window of width in which it holds without squeezing.
- **Technique decides what moves.** The pipette draws only when released with the tip under the liquid. Releasing
  in air draws nothing; squeezing anywhere but into or just above a cup is a spill; releasing while still dipped
  after dispensing draws a pipetteful back up (harmless here, as in real mixing by pipetting). The pipette has to be
  empty when it is put away.
- **Colour shows progress.** Each cup's liquid takes on its current dye strength as transfers happen, so the
  videos show the gradient forming, and a vision policy sees what it has done.
- **Going back is the contamination rule.** The protocol uses one pipette, so dipping it back into an earlier cup
  after a later one carries diluted liquid upstream. That scores 0 even if the final colours happen to come out
  right.
- **Demo trims.** The arm's wrist settles a few degrees off vertical, and a 12 cm pipette turns that into up to a
  centimetre at the tip, so the open-loop demo aims off by the error measured at each spot (`TRIM`). They're
  calibration for the demo only and play no part in scoring.
- **Reach.** Everything sits in front of the left arm (the cups at y 12, the holder at y 18), where a straight-down
  grip reaches at every height the demo uses; the right arm stays at its ready pose. The cups' 3 cm openings leave
  the stem 11 mm of clearance on every side.
- **Let go.** Nothing can grip the bulb without pushing its sides in, so "both sides back at rest" is how the sim
  knows the pipette was let go; on a real bench, the operator opens the grippers before the photo.

## Runtime and seeds

- The scripted demo takes about 62 s of simulated time (1236 steps at 20 Hz), about 4 minutes of wall time on an L40S.
- Dream-1 runs the full 2,400 steps unless it finishes first (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
