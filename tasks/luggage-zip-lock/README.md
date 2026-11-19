# Lock both zip pullers of a hard-shell suitcase into its built-in lock

A closed hard-shell carry-on lies on its back on the table, its built-in TSA zip lock facing the robot on the
straight run of zip along its long side. Both zip sliders stop a few centimetres short of the lock, one on each side,
each with a loose puller tab on a hinge. The robot draws each slider to the lock by its puller, swings the puller flat
along the zip, and drives it straight into its slot in the lock until a spring catch drops into the hole near the
puller's tip. Then it lets go. Nothing here zips cloth: the case is closed and its two rigid shells hold the zip's
tooth rows in line; the task is the last step, locking. The case is sized after the Samsonite Freeform Spinner
Carry-On (54.6 x 36.8 x 22.9 cm, 2.59 kg, polypropylene, integrated TSA lock).

The scripted demo reads object poses from the simulator (case pose, slider positions, tab angles) to plan its
motions. It moves the parts only through the grippers, by contact.

## Requirements

The official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), which provides
`inspect-robots` and the `yam_arena` embodiment. For Dream-1, a Dream API key in `~/.dreamlabs/credentials` and the
Dream-1 bridge on port 8211. Everything else is in this folder:

- `zip_lock.py`: the task file (scene, mechanism, random start, scorer, scripted demo).
- `zipdemo.py`: the scripted demo's control hooks for `yam_scripted` (staged planning, sag correction, a multi-start
  IK fallback).
- `dev/run.sh`: one trial with logs (used for every run below).

## Run and score a trial

From this folder:

```bash
INSTR="Close the suitcase's zip up to its lock and lock both zip pullers into the lock, then let go."
ARGS=(--embodiment yam_arena -E task=zip_lock.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360
      --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames)

# Dream-1
inspect-robots run "${ARGS[@]}" --policy dream -P server_url=http://127.0.0.1:8211 --seed 0 --log-dir ~/yam-eval/zip-dream

# The scripted demo, which reaches full score (seeds 0, 1, 2 shown)
inspect-robots run "${ARGS[@]}" --policy yam_scripted -P hold_s=1 --seed 0 --log-dir ~/yam-eval/zip-demo-s0

# The same demo made wrong on purpose (see Results): fixed, short, crooked, holding
ZIP_FAIL=crooked inspect-robots run "${ARGS[@]}" --policy yam_scripted -P hold_s=1 --seed 0 --log-dir ~/yam-eval/zip-crooked

# The mechanism probe (no arm motion): pushes each slider with 10 N, then pulls each puller with 5 N
ZIP_PROBE=m1 inspect-robots run "${ARGS[@]}" --policy yam_scripted --seed 0 --max-steps 140 --log-dir ~/yam-eval/zip-m1
```

The run prints `task_score` at the end. A trial is scored at its last step; it ends early once the full score has
held for 1 s. With `ZIP_DEBUG=<file>` set, every control step adds one line to that file: each side's slider gap to
its stop, tab angle, tip depth in the slot, catch lift, the case's pose, the grippers' clearance and the score;
`<file>.plan` gets the planned and measured tool poses. `dev/run.sh <name> <policy> <seed> <steps> [ENV=VAL ...]`
wraps the command with these logs. Runtime: about 3 to 9 minutes of wall time per trial (up to 1,200 steps) on a shared GPU.

## Scoring

Tools: ruler, luggage scale (spring scale), a pen mark on each puller 10 mm and 18 mm from its tip.

| Check, per side | Real trial | In the simulator (`report` in `zip_lock.py`) |
|---|---|---|
| Slider at its stop, 0.15 | gap between the slider's front and the lock housing 2 mm or less | slider position along its joint |
| Puller in its slot, +0.15 | the puller lies flat along the zip (within 10 degrees) and its 10 mm mark is inside the slot | tab hinge angle within 10 degrees of flat toward the lock, tip at least 10 mm past the slot's mouth |
| Caught, +0.20 | its 18 mm mark is inside the slot and a 5 N pull on the luggage scale, along the zip away from the lock, does not draw it out | tip at least 18 mm in, and the catch's plunger lifted less than 1 mm (down in the puller's hole; riding on the tab it is lifted 2.75 mm) |

Score = side A + side B, at most 1.0. It is 0 for the trial if the case was pushed more than 10 cm or tipped more
than 10 degrees, or if either gripper ends within 30 mm of a slider, puller or catch.

Outcomes per side: nothing done 0; slider at its stop with the puller not in its slot (swung out, or jammed
crooked) 0.15; puller part way in, short of the catch, slider not at its stop 0.15; puller in at least 10 mm but the
catch not down, slider at its stop 0.30; caught 0.50 (a caught puller has its slider at the stop).

## Setup

Exactly as in `task.json`. Those positions are the reference; every trial's seed draws the case's pose and each
side's slider gap and tab angle from the ranges in the setup text (`layout` in `zip_lock.py`). The framework turns
`--seed N` into its own episode seed; each trial's draw is written to the first line of the `ZIP_DEBUG` log.

## How the mechanism works

All by contact, joints and springs; no part is attached or released by a rule.

- Each slider is a body on a prismatic joint along the zip, with its end stop at the lock housing (a joint limit).
  The zip's resistance is a velocity drive that holds the slider still up to 3 N.
- Each puller is a body on a hinge pin on its slider (axis square to the case side, stops at flat either way,
  90.5 degrees), lightly damped so it stays where it is left.
- Each catch is a 4 x 4 mm plunger on a prismatic joint across its slot, pushed into the slot by a spring (500 N/m
  with 4 mm preload: 2.0 N seated, 3.4 N lifted 3.5 mm). Its mouth side is a 30 degree ramp, so the puller's tip lifts
  it on the way in; its other side is flat, so once it drops into the puller's 5 x 5 mm hole the puller cannot be
  drawn back out. A puller that is pushed in crooked meets the slot's mouth off line and swings away on its hinge.

## Results

All runs on the final code (`dev/run.sh`, 1,200-step limit; the run ends early once 1.0 has held for 1 s, or when
the scripted plan is used up). Per-side readings at the last step: slider gap to its stop, tab angle (90 = flat
toward the lock), tip depth in the slot, catch lift.

| Run | Seed | Steps | Score | What happened |
|---|---|---|---|---|
| Scripted demo (reads object poses from the simulator) | 0 | 841 | 1.0 | A: 0.7 mm, 91.4 deg, 19.3 mm in, catch down (lift 0.10 mm). B: 1.0 mm, 91.3 deg, 19.0 mm in, lift 0.24 mm. Case moved 4.8 mm |
| Scripted demo | 1 | 893 | 1.0 | Both caught. Case moved 4.2 mm |
| Scripted demo | 2 | 996 | 1.0 | Both caught. Case moved 5.4 mm |
| `ZIP_FAIL=fixed`: the demo planned for the reference start, reading nothing | 1 | 921 | 0.0 | Sliders left where they started, 45 and 68 mm short; the grippers only knocked the tabs (to 64 and 57 deg) |
| `ZIP_FAIL=crooked`: each puller pushed in 30 degrees off line | 0 | 868 | 0.3 | Both sliders at the stop (0.15 each). Both tabs jammed at 57 deg, 33 deg off line, against the lock's mouth; neither in its slot, catch never down |
| `ZIP_FAIL=short`: each puller pushed in only part way | 2 | 1,058 | 0.0 | Both tabs flat but only 9.3 mm in (under 10 mm), sliders 10.7 mm short of the stop, catch not reached: nothing holds them |

Dream-1, seed 0, for the full run limit (1,200 steps): 0.0. Model-run video: top camera, left wrist camera and right wrist camera side by side.

Demo video: top camera, left wrist camera and right wrist camera side by side, seed 0.

## Real-world build

Any 55 cm hard-shell carry-on with a built-in TSA zip lock on its long side, lying on its back on the table. Before
each trial, set the start from the seed's draw (first line of the `ZIP_DEBUG` log): put the case at its pose, draw each
slider back to its gap with a ruler, and set each puller's angle with a printed protractor. To score, use a ruler, a
pen mark on each puller 10 mm and 18 mm from its tip, and a luggage scale for the 5 N pull. The only force in the task
is the push on the pullers (a few newtons), so a real trial is safe.
