# Test that a seat belt retractor locks: pull the webbing out steadily, then jerk it

The robot tests a seat-belt emergency-locking retractor the way a vehicle technician does. It pulls the webbing out
150 mm at a steady pace (the retractor must pay out freely), jerks the webbing (the retractor must lock and stop it
within 15 mm), then lets go so that the lock frees and the spring winds the webbing back in. The tongue plate is the
handle; the retractor is bolted to a steel test plate on the table. Automotive service and vehicle-inspection
technicians do this check in every crash-repair, recall and pre-purchase inspection.

The ability is controlling the speed of a pull: the retractor's lock is sensitive to how fast the webbing is pulled
out, so the same pull at two speeds gives two different outcomes. The check is a safety duty: a belt that locks on a
slow pull, never locks on a jerk, or stays locked after release fails the car. Nothing is placed or fastened.

Physics, not the instruction, defeats the natural plan. The instruction asks for 15 cm of pull-out and then a jerk. A
policy that pulls the first 15 cm at the speed of a jerk (or that begins with the jerk) locks the webbing before 150 mm
have come out, and that gates the whole run to 0 for good. A policy that jerks too softly never locks the webbing and
the pull simply continues. Only a steady pull followed by a sharp one scores.

Instruction given to the policy (23 words; it states the goal only, never the speeds):

`Pull the seat belt webbing out 15 cm, then jerk it so the retractor locks, then let go so it winds back in.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the
submitted runs:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 52b2ed8d2929 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the retractor as one fixed PhysX articulation (a frame link and a webbing link joined by a
spring-damper prismatic joint whose drive is switched to a stiff hold by the lock), built from box colliders in
`task.py`. No fork of yam_arena and no third-party assets.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

19 tests feed synthetic pull-out traces (20 Hz) to the scorer: a steady pull then a jerk then letting go scores 1.0;
no pull, a steady pull alone, a creep at 0.2 m/s to full travel, a jerk alone and a jerk at about 110 mm all score 0
(the last two are the early-jerk gate); the same pull at 152 mm or more followed by a jerk scores 1.0; the speed is
taken over 0.1 s (0.30 m/s is a jerk, 0.29 is not) and a first pull at 0.26 m/s is not a steady pull; an early jerk is
not cleared by winding the webbing back in, only by a simulator reset; running on more than 15 mm after the jerk
scores 0.25; stopped but not wound in scores 0.5; the 5 mm wound-in tolerance (5.1 mm is 0.5, 5.0 mm is 0.75 or 1.0);
a pad 29 mm from the tongue or a hand moving at 6 mm/s restarts the release timer (0.75), 0.95 s of release is not
enough; and the lock trigger uses the same speed as the scorer.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pull the seat belt webbing out 15 cm, then jerk it so the retractor locks, then let go so it winds back in." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/seat-belt-retractor-lock-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scored runs

| Run | yam-isaac-lab-arena | Commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 1.0.0 | 0059d02a1a91 | 52b2ed8d2929 | 1200 | 3 min | adhoc_a52ee32a.json | 0.0 | max_steps |
| Scripted demo (`linkdemo.py`) | git checkout | 0059d02a1a91 | 52b2ed8d2929 | 273 (success held 1 s) | not timed | result.json | 1.0 | steady pull to 165 mm at 0.08 m/s, jerk locked at 212 mm, wound back in, grippers clear |

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                        # scripted demo
python linkdemo.py task.py cheat_out --variant jerk_only   # a cheat variant (or DEMO_VARIANT=jerk_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the webbing link's pose (the pull-out) and the robot's own grippers. It never reads the
lock state, the joint values or the record.

1. Right arm, top-down, pads closing across the tongue's 45 mm width (opened 75 mm on the approach), lowered until the
   pad tips are 3.5 mm into the tongue's 4 mm edges, closed to 43 mm.
2. Steady pull toward the robot at 0.08 m/s (4 mm a control step) until the seen pull-out passes 165 mm (the peak speed
   over 0.1 s is 0.081 m/s); holds still 0.6 s.
3. The jerk: 60 mm commanded in 0.1 s. The webbing locks at 212 mm and does not move again; the pads slip on the locked
   tongue.
4. Opens the gripper (the lock frees when no hand holds the tongue; the spring winds the webbing in), lifts 80 mm and
   moves 100 mm sideways, then waits.

The demo scores 1.0 (`success`, episode ended at step 273 of 1,200): peak speed 0.377 m/s at the jerk, the webbing
stopped 0.0 mm past the jerk point, wound back in to 0 mm, both grippers at least 30 mm clear and still for 1.95 s.
Worst IK error 8.7 mm and 2.5 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (52b2ed8d2929); every one scores
exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. | 0.0 |
| `slow_only` | Steady pull to 169 mm, hold, let go. | Peak speed 0.081 m/s; the webbing never locks; it winds back in. | 0.0 |
| `jerk_only` | The 60 mm jerk straight away, then let go. | Locks at 49 mm: early-jerk gate. | 0.0 |
| `short_slow` | Steady pull to 80 mm, then the jerk. | Locks at 130 mm, before 150 mm had come out steadily: gate. | 0.0 |
| `soft_pull` | Steady pull to 165 mm, then on to 236 mm at 0.20 m/s. | Peak speed 0.197 m/s, under the 0.30 m/s the lock needs: never locks. | 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures the webbing's
pull-out e (mm, 0 with the tongue against the slot), its pay-out speed v over the last 0.1 s, and each gripper's pad
clearance to the tongue, webbing and retractor and its speed. The record keeps flags, peaks and held times for the
whole episode and is cleared only by the simulator reset, so letting go and starting again never clears an early jerk.

- Gate, any time in the run, gives 0: v reached 0.30 m/s before a steady pull-out to 150 mm was done (steady: v under
  0.25 m/s throughout). A first jerk locks the webbing early, which is the failure the check exists to catch.
- 0: no steady pull-out to 150 mm, or no jerk of at least 0.30 m/s after it.
- 0.25: steady pull then jerk, but the webbing ran on more than 15 mm past the point where the jerk began.
- 0.5: and the webbing stopped within 15 mm of the jerk point.
- 0.75: and the webbing is wound back in to 5 mm or less at the end.
- 1.0: and both grippers are at least 30 mm from the tongue, webbing and retractor and moving under 5 mm/s for the last
  1 s (the release gate).

Real-trial measurement line: film the webbing from above at 60 fps with a 0.5 m steel rule laid beside it. Read e at
the slot from the printed ticks (every 5 mm, labelled every 50 mm; interpolate by eye) and take v from the tick count
over 6 frames (0.1 s); the 15 mm run-on is read from the ticks; the release gate is the pads' distance from the tongue
on the rule and the hand's speed over 6 frames at 60 fps.

## Setup and materials

- Retractor: a passenger-car emergency-locking retractor, steel frame 90 x 70 x 110 mm, dark grey painted steel,
  0.5 kg, bolted with four M8 bolts to a 200 x 160 x 8 mm grey steel test plate that is clamped to the table. Its
  webbing slot faces the robot, 60 mm above the table. Webbing-sensitive (flywheel) lock, tuned on the bench so that a
  0.08 m/s pull runs free and a pull over 0.30 m/s catches within 15 mm (verify with the video procedure above).
- Webbing and tongue: 46 mm wide, 1.2 mm thick black polyester webbing ending in a silver steel tongue plate
  85 x 45 x 4 mm (120 g); 100 mm of webbing plus the tongue lie outside the slot at rest (the tongue centre at
  (0.408, -0.12, 0.808)); 0.15 kg in all. White ticks every 5 mm are printed along the webbing from the slot.
- Return spring: 0.2 N at rest rising to about 2 N at 0.2 m of pull-out (check with a 5 N spring scale at 0 and
  0.2 m), with a light damper so that the webbing winds in at about 0.2 m/s.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

Fidelity: the webbing is a rigid 1.2 mm blade 0.40 m long on a slider with 0.30 m of pull-out (its tail sticks out
behind the frame where a real belt winds on a spool). The lock is a brake that holds the slider once the pull-out
speed over 0.1 s passes 0.30 m/s (real units trip on webbing acceleration) and frees when no hand holds the tongue.

## Reset between trials

Nothing is consumed. Let the webbing wind fully in (tongue against the slot, ticks at 0), check the lock is free by
one slow hand pull of 100 mm (it must run free) and one quick pull of 20 mm (it must catch), then let go and let it
wind in. Re-seat the tongue and check the spring with the scale at 0.2 m (about 2 N). The test plate stays clamped on
its taped outline.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
