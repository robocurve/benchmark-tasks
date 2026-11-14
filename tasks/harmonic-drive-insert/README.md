# Seat a strain-wave flexspline unit in a robot joint housing, turning it until the teeth mesh

The robot stands a flexspline unit (steel cup with a blue collar and a hubbed wave generator) into the circular spline
of a black joint housing. The unit's teeth start exactly on top of the housing's tooth tips, so a straight push stalls;
the robot has to turn the unit about its axis by a few degrees until its teeth are in line with the gaps, push it in,
and let go with the collar flat on the housing face. Technicians who build robot arm and humanoid joints do this with
every strain-wave (harmonic drive) actuator.

Category: Impact on AI timelines (robot building and assembly). The ability is turning a toothed part about its axis
to find the mesh while pushing it home; nothing is latched, clipped or placed, and no moving part is sorted.

Physics, not the instruction, defeats the natural plan. The housing's 52 teeth have 1.05 mm tips and 1.9 mm gaps; the
unit's 1.0 mm teeth fit a gap with 0.5 mm clearance a side (1.15 deg of turn). The unit stands with its teeth on the
tips, so pushing straight down stops 12 mm above the seat with the collar clear of the housing.

Instruction given to the policy (25 words; goal only):

`Seat the blue flexspline unit in the black joint housing so its collar sits flat on the housing and the teeth mesh, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop demo runner), `tests/test_scoring.py`, `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S):

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 2909d48f9886 (`sha256sum task.py | cut -c1-12`)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).
`linkkit.py` and `linkdemo.py` must sit next to `task.py`; importing `linkkit` hooks `yam_arena.scene` so the two
bodies (compound colliders built from boxes and cylinders in `task.py`) spawn without forking yam_arena. No
third-party assets.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

18 tests feed synthetic records to the scorer: the full score; nothing and a unit stalled on the tooth tips both score
0; the phase edge (1.0 deg passes, 1.1 does not), lateral (1.0 mm), tilt (3.0 deg) and stall-height (13 mm) edges of
the 0.25 rung; meshed at 6.0 mm vs 6.1 mm; seated at 0.5 mm vs 0.6 mm; the cocked gate (8.0 deg) beating a seated end
and not firing at 7.9 deg or far from the housing; a unit moving at 4 mm/s or spinning at 3.5 deg/s scores 0.75; release
at 29 mm vs 30 mm, at 6 mm/s, 19 vs 22 control steps (1 s), and one pad step 20 mm away restarting the timer; the record
is clean after a reset; phase wrapping (the opposite tooth patch also lines up, 26 pitches away); pose-rate speed.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Seat the blue flexspline unit in the black joint housing so its collar sits flat on the housing and the teeth mesh, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/harmonic-drive-insert-dream
```

`task_score` reports 1.0 when `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`;
otherwise it is `score()` at the end (`SCORE_OVER = "end"`). With `object_jitter=0` the seed changes nothing.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant straight_down  # a cheat variant (or DEMO_VARIANT=straight_down)
```

The script reads only camera-observable poses (housing and unit poses, the unit's turn angle from them) and the robot's
own grippers. Right arm: top-down on the hub's two flats (12.5 mm pad opening on a 14 mm hub), lifts, nulls the seen
offset over the bore at 40 mm, lowers to 16 mm, measures the seen turn error (-3.41 deg), turns the wrist to remove it
(-0.13 deg left), lowers to 14 mm, presses to 2 mm below the seat, waits, lets go and lifts 80 mm. The left arm stays
at the ready pose, clear. The demo scores 1.0 (`success`, 502 of 1,200 steps); the unit ends with its collar on the face
(g 0.0 mm, 0.3 mm off axis, tilt 0.0 deg), still, grippers 390 and 63 mm clear. Worst IK error 0.09 mm.

Variants, each run in Isaac Sim with the submitted task file (2909d48f9886); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | The unit stays on the table. | 0.0 |
| `straight_down` | Carried over the bore and pushed straight in without turning. | Teeth stall on the tips with the collar 12.5 mm above the face, 3.1 deg out of line; never lined up. | 0.0 |
| `half_turn` | Turns only half the needed angle, then pushes. | Still 1.9 deg out of line, stalls at 12.0 mm; never lined up. | 0.0 |
| `drop_over_bore` | Releases the unit 40 mm above the bore. | It falls on the tooth tips and cocks 23.7 deg: gate. | 0.0 |

## Scoring (`score()`, at the end; `success()` must hold 1 s)

Every control step the record (`Record` in `task.py`, a `linkkit.History`, cleared only by the simulator reset)
measures in the housing frame: g (collar-bottom height above the face; 12.0 mm when the tooth tips first touch, 0
seated), lat (axis offset), tilt, phase (index-line angle minus half a pitch, modulo the 6.92 deg pitch), the unit's
speed and spin (from its pose change between steps), and each gripper's pad clearance and speed.

- Gate, any time in the run, gives 0: tilt 8 deg or more with g under 13 mm and lat under 12 mm (cocked in the bore).
- 0.25: at some time g at most 13 mm, phase within 1.0 deg, lat at most 1.0 mm, tilt at most 3 deg (teeth in line).
- 0.5: at some time g at most 6 mm with lat at most 1.0 mm (teeth meshed).
- 0.75: seated at the end (g at most 0.5 mm, lat at most 1.0 mm, tilt at most 3 deg).
- 1.0: seated, still (under 3 mm/s and 3 deg/s) for 1 s, and released: every pad point of both grippers at least 30 mm
  from the unit and housing and moving under 5 mm/s for 1 s.

Real trial: phase from the top video against a printed 0.5 deg ring on the housing face (mark at 3.46 deg) and the
white index line on the collar; g and tilt from a side video frame with a ruler and protractor; seated when a 0.5 mm
feeler gauge cannot enter under the collar at four points 90 deg apart; stillness and release from the video.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 502.
- A recorded demo run took about 2.5 min wall clock on one L40S, including about 1 min of Isaac Sim start-up.
- Seeds: `--seed 0` for every run; the scene is never jittered and the demo is deterministic.

## Scored runs

| Run | inspect-robots-dream | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 0.1.0 | 0059d02a1a91 | 2909d48f9886 | 1200 | 3 min | adhoc_dd3a1667.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | - | 0059d02a1a91 | 2909d48f9886 | 502 (success held 1 s) | about 2.5 min | demo_out/result.json | 1.0 | turned 3.4 deg, pressed home, collar flat, both grippers let go |

- Dream-1 run: made at filing with the command above, on this exact `task.py`.

```bash
inspect-robots inspect adhoc_dd3a1667.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_dd3a1667.json
```

## Materials

- Joint housing: anodised aluminium, black, 0.35 kg, 80 mm across, 30 mm tall, bolted to the table at (0.300, -0.100),
  bore up. Circular spline in the bore: 52 teeth, tips r 24 mm, roots r 26 mm, 1.05 mm wide, 12 mm long, 6.92 deg pitch;
  plain cavity to 16 mm deep below. Friction 0.4 / 0.3. A printed 0.5 deg ring on the face (mark at 3.46 deg).
- Flexspline unit: steel, grey with a blue collar, 0.18 kg, standing at (0.300, -0.250): cup 47 mm across, 14 mm long;
  two patches of 5 teeth (1.0 mm wide, 12 mm long, tips r 25.5 mm); 63 mm x 3 mm collar; hub 14 x 22 x 40 mm with two
  wrench flats. Friction 0.5 / 0.4. Verify the 0.5 mm side clearance with a 0.5 mm feeler gauge between a tooth and its
  gap, and the 1.15 deg turn tolerance with the printed ring.
- Scale: collar, cup, hub, housing and mass are at size-20 strain-wave scale; the tooth pitch is 3.0 mm against about
  1.5 mm on a real size-20 unit so that sub-millimetre clearances are buildable and simulable (see `fidelity`).
- Positions, colours and friction for every body are in `task.json` and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Stand the unit on its lead end at (0.300, -0.250) on a taped 47 mm ring with the hub flats facing
+-x and the index line along +x (turned until it sits on the housing's tooth-0 mark). Pull the previous unit straight out
of the housing and wipe both toothed surfaces. The housing is bolted down.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text), and the scene uses no
third-party assets or brands.
