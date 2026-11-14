# Prepare a serial dilution on a powered magnetic stirrer

A row of four clear cups sits on an operating four-position magnetic stirrer: dark blue stock, then cups 1, 2 and 3
with water. The robot takes a squeeze-bulb transfer
pipette from its holder and moves one pipetteful at a time down the row (stock to cup 1, cup 1 to cup 2, cup 2 to
cup 3), allowing each destination to mix before sampling it, so the cups end at a half, a quarter and an eighth of
the stock's strength. Lab staff call this a serial dilution. Then it stands the pipette back
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

## Powered stirring setup

Coordinates are measured in centimetres from the midpoint between the YAM arm bases: x points forward and y points
to the robot's left. The tabletop is z = 74.75 cm.

- Tape an 18 x 7 x 1.2 cm four-position magnetic-stirrer deck at x = 23.75, y = 12, with its long axis along x and
  its green power indicator facing the arms.
- The four 3.4 x 3.4 cm stations are centred at x = 17, 21.5, 26 and 30.5 cm, all at y = 12 cm.
- Tape one clear acrylic cup to each station. Each cup has 3 mm walls, a 3.0 x 3.0 cm inner opening and a 4 cm
  height. Put one 14 x 3 x 2 mm PTFE magnetic stir bar flat in each cup.
- Set all four stations to 600 rpm and leave them powered continuously throughout the trial. Before starting, verify
  that all four bars rotate stably and that no cup splashes.
- The first station holds 10 mL stock (1 mL blue food colouring in 100 mL water). Stations 2-4 hold 5 mL clean water.
  Tape the 2.2 cm inner-width, 5 cm tall pipette holder at x = 24, y = 18 cm.

The simulated stir bars show their starting geometry. Fluid motion is represented by a conservative ledger: after a
dispense, a powered cup remains unqualified for 1.0 second. Aspiration during that interval is recorded as an unmixed
draw and permanently makes the score zero. Powered mixing is therefore part of the measured protocol.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pipette_dye_into_cups.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "With the four-position magnetic stirrer already running, use the transfer pipette to dilute the dark stock by half in each cup. Move one pipetteful from the stock into cup 1, allow it to mix, then one from cup 1 into cup 2, allow it to mix, then one from cup 2 into cup 3 and allow it to mix. Stand the empty pipette back in its holder and let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/dye-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pipette_dye_into_cups.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "With the four-position magnetic stirrer already running, use the transfer pipette to dilute the dark stock by half in each cup. Move one pipetteful from the stock into cup 1, allow it to mix, then one from cup 1 into cup 2, allow it to mix, then one from cup 2 into cup 3 and allow it to mix. Stand the empty pipette back in its holder and let go." \
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

A cup is **right** only after its powered mixing dwell, when it holds one or two pipettefuls and its strength is its
target: cup 1 half the stock, cup 2 a
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
| Liquid was drawn from a destination less than 1.0 s after its latest dispense | 0 |
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
- **Mixing is a prerequisite.** Every cup remains on an independently driven 600 rpm station. A 1.0 s dwell begins
  after each dispense. Drawing from that cup before the dwell completes is an unmixed draw and scores zero. The
  open-loop demo waits at the destination before starting the next transfer.
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

- The scripted demo includes three powered-mixing dwells and remains below the 2,400-step limit at 20 Hz. Fresh GPU
  timing and scores must replace the earlier evidence before submission.
- Dream-1 runs the full 2,400 steps unless it finishes first (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
