# Fill a bucket to the brim from a garden hose's trigger nozzle, then stop the water

Instruction given to the policy: `Pick up the hose nozzle, fill the blue bucket to the brim with it, then stop the
water. Don't let it overflow or spill.`

![Fixed starting layout](setup.png)

A pistol-grip spray nozzle lies on its side on a paver patio, laid in a drip tray, at the end of a garden hose that
runs from an open tap through a digital water meter.
One hand picks the nozzle up off the ground (dragging the hose), turns it handle-down and aims it into an empty
6.3 L pail; the other hand closes round the handle and the sprung trigger and squeezes. Water comes out only while
the trigger is squeezed, so the robot must watch the level rise and let go when the pail is full to the brim, not
before and not after it runs over. The arms and their bases wear clear plastic splash sleeves.

## Files

| File | What it holds |
| --- | --- |
| `task.py` | the task file: the per-step water update, `score()`/`success()` and the two-arm scripted `demo()` |
| `parts.py` | every part, built procedurally: drip tray, patio, pail, tap and pillar, water meter, 31-link hose, nozzle and trigger, the arms' splash sleeves |
| `water.py` | the water model (NumPy only): valve flow, the stream's trajectory, the pail's volume and level |
| `kit.py` | multi-shape rigid bodies, sprung joints between bodies that start rotated, glass materials, meshes redrawn every step and visual extras on the arms' links, hooked into `yam_arena` in memory |
| `meshes.py` | lathe, swept-tube and rounded-section meshes for the visible parts |
| `hoseskin.py` | the hose's visible skin: one smooth tube (and its twisting stripe) through the simulated links, redrawn every step |
| `run_demo.py` | runs one trial, records the cameras and writes `trace.jsonl` and `summary.json` |
| `test_task.py` | offline checks of the water model and every scoring outcome (no simulator) |
| `check_plan.py` | offline replay of the scripted demo through the arm kinematics |

No downloaded meshes are used; the robot and room come from the installed `yam_arena` package.

## Run and score one trial

From the repository root, in the environment with Isaac Lab-Arena, `yam-arena` and Inspect Robots:

```bash
python tasks/hose-fill-bucket-to-brim/run_demo.py --out "$HOME/yam-eval/hose-$(date +%Y%m%d-%H%M%S)"
```

This plays the scripted demo (`--policy yam_scripted`) with `object_jitter=0`, seed 0, 20 Hz control and at most
1,600 steps, records the three cameras to `<out>/videos/`, and writes the water's state every 0.1 s to
`<out>/trace.jsonl` and the outcome to `<out>/summary.json`. Success is `task_score == 1.0` in `summary.json`. A
trial takes about 4 minutes on an RTX 5090, and its raw frames take about 2 GB.

The same trial straight through Inspect Robots (swap `--policy` to run another policy):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/hose-fill-bucket-to-brim/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Pick up the hose nozzle, fill the blue bucket to the brim with it, then stop the water. Don't let it overflow or spill." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames --log-dir runs/hose-fill-bucket-to-brim
```

Offline checks, no GPU:

```bash
python -m pytest -q tasks/hose-fill-bucket-to-brim/test_task.py   # 28 tests
python tasks/hose-fill-bucket-to-brim/check_plan.py               # IK replay of the demo
```

## Scoring

The score accounts for the water the same way in simulation and in a real trial:

- **metered**: litres that left the hose, the water meter's total;
- **in the pail**: litres the pail holds;
- **spilled = metered - in the pail**: whatever missed the pail, ran over its brim or splashed out.

Full = at least 5.73 L in the pail (the surface within 15 mm of the brim, of 6.30 L brimful).

| Outcome | Score |
| --- | --- |
| pail tipped more than 15 deg at any moment, or more than 1.0 L spilled | 0 |
| full, water off for at least 1 s, at most 0.25 L spilled | 1.0 |
| at least 85% of the brimful volume (5.36 L) in the pail | 0.75 |
| at least 50% (3.15 L) | 0.5 |
| at least 25% (1.58 L) | 0.25 |
| less | 0 |

The trial ends with success once the full-score state has held for 1 s; otherwise the score is the one at the end of
the run.

**In simulation** these are read from the state every control step: the meter's total is the nozzle's flow summed
over the steps, the pail's water is what the stream put into it (capped at the brim), the tilt is the pail's
orientation and the water is off while the trigger is released past 6 deg. `trace.jsonl` logs `metered_l`,
`in_bucket_l` and `spilled_l` (with `missed_l` and `overflow_l`, its split by cause).

**In a real trial** the same three quantities are measured:

| Quantity | How | Equipment |
| --- | --- | --- |
| metered | the meter's reading after the trial - before it, times its correction factor | digital garden-hose water meter screwed onto the tap, the hose onto it (totalising, reads to 0.01 L, counts from 1 L/min or less) |
| flow fixed at 12 L/min | a regulator in the meter's inlet, checked once per session (a 30 s full squeeze gives 6.0 +- 0.3 L by the scale) | pressure-compensating 12 L/min flow-regulator washer |
| in the pail | (pail after - pail empty) at 1 kg per litre; full = at least 5.73 kg of water | bench scale, 10 kg capacity, 1 g resolution |
| tilt, water off | the recorded video: the pail's tilt, and the trigger released with the meter not counting for 1 s | the station's cameras |

The tap stays open and the nozzle closed at the start and at the end, so the hose holds the same water both times
and everything the meter counted left through the nozzle. Spilled water is not collected: it stays in the drip
tray, and the absorbent pavers do not affect the score.

Calibrate the meter once per session: three full-flow fills of about 6 L into the pail, each weighed; correction
factor = weighed / metered; the three fills must agree within 0.05 L. Error after calibration: about 1% of the
metered water (0.06 L at 6 L) plus the scale's 1 g, a quarter of the 0.25 L limit for full credit.

**Protecting the robot:** a clear polyethylene sleeve over each arm (links and gripper housing; the fingers and
the wrist cameras' lenses left out) and over each base, the patio in its drip tray, and the power supplies and
controllers off the table. Wipe the grippers and the nozzle dry between trials.

## The water model

The trigger is a free 25 g body on a sprung hinge under the barrel (0.26 N m preload against its rest, 0.022 N m per
degree, a hard stop at 22 deg). Its measured angle sets the flow: nothing below 6 deg, rising linearly to 12 L/min at
20 deg. The stream leaves the 10 mm flood outlet along the barrel at flow / outlet area (2.5 m/s at full flow) and
falls under gravity; it is traced until it meets the water, the pail's floor or inner wall (into the pail), or the
outer wall, the rim's top or the ground (spilled). Water above the brim runs over and is spilled. The meter counts every litre
that leaves the nozzle. The pail weighs its 0.38 kg plus 1 kg per litre of water it holds. The water body, the stream (thinning as it falls), foam where it strikes
the water and dark wet patches where water reaches the ground are drawn from this model every step; the water and
stream render as clear glass with water's refractive index (the cameras' path tracer is given 6 bounces so they show
through to the pail).

Development switches for `run_demo.py` (never set for a scored trial): `HOSE_FILL_S` (the demo's squeeze time), `HOSE_AIM_DY` (move the demo's aim sideways, m) and `HOSE_PREFILL_L` (start with water in the pail, for previews).

Not modelled: splashing (a real trial counts water splashed out of the pail as spilled), the stream breaking up or
hitting the robot, sloshing, and water in the hose. The drip tray, the tap and the meter are fixed bodies; the
sleeves are visual only (no collision or mass).

## The scripted demo

Left hand: over the nozzle, down round the barrel's flat-sided rubber grip, close, lift, turn the nozzle handle-down
and bring the dial 6 cm above the brim just inside the near rim, aimed 45 deg down. Right hand: come in from the right
beside the nozzle, close round the handle and the trigger together (the squeeze is internal to the nozzle, so it does
not push the held nozzle about), hold 28.8 s, open, back off. The fill time is open loop from the 12 L/min flow; the
water stops at 5.98 L, 8.6 mm below the brim.

## Results (seed 0, no jitter)

| Run | Metered | In the pail | Spilled | task_score |
| --- | --- | --- | --- | --- |
| Scripted demo | 5.98 L | 5.98 L (8.7 mm below the brim) | 0 | 1.0 (success at 47.4 s) |
| Shortcut: keep squeezing 7 s past full (`HOSE_FILL_S=35.8`) | 7.38 L | 6.30 L (brimful) | 1.07 L (ran over) | 0 (0.75 before it ran over) |
| Dream-1 (`--policy dream`, 320 x 176 cameras, 1,600 steps) | 0 | 0 (pushed the nozzle 7 cm across the pavers, never picked it up) | 0 | 0.0 |

Missing the pail is checked offline (`test_aiming_off_the_pail_spills` in `test_task.py`).

## Environment

Python 3.12, `inspect-robots` 1.0.0, `inspect-robots-yam` 0.38.0, Isaac Sim 6.1.0.0, Isaac Lab-Arena 0.3.0 (editable),
`yam-arena` 0.1.0 (editable), NumPy 2.3.1, PyTorch 2.11.0+cu128. `run_demo.py` records the installed versions in
`<out>/environment.json` and a copy and SHA-256 of every task file in `<out>/source/`.
