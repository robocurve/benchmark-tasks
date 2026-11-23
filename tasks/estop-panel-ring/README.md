# Mount an emergency-stop button in a panel hole with its tab in the notch, then thread its locking ring on behind

The robot rolls a mushroom emergency-stop button until its anti-rotation tab lines up with the notch in a panel hole,
pushes the barrel through, and holds the button seated from the front while the other arm reaches behind the panel and
turns the hex locking ring about half a turn onto it. At the end the button sits flush in the panel with its tab in the
notch, the ring is snug against the back of the panel, and both grippers have let go. Electricians and industrial
machinery mechanics do this for every operator panel of a new robot cell.

The ability is turning a keyed part in hand to the angle a hole demands before it will go in (here a 100 deg roll,
read from the tab and corrected closed loop), then advancing a ring by turning it, pinched on its flats, through a half
turn while the other hand keeps the part seated. It is not pressing a button, sliding a plate or closing a hinged
door. Physics, not the instruction, makes the order: pushed in at the wrong roll the 2.8 mm tab meets the plate and the
barrel stops at the face (0.0); and the ring turns only while the barrel stands in the hole.

Instruction given to the policy (26 words; it states the goal only, never the roll, the notch or the turn):

`Fit the red emergency-stop button flush in the panel hole, then screw its yellow locking ring snug against the back of the panel, and let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the Dream-1
trial:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: d8a866b279c0 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log). The demo and cheat runs below used the same file, whose first 12 hex are d8a866b279c0.

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the panel as a PhysX articulation (a fixed plate with a slide joint and a spin joint carrying
the ring, spawned through Isaac Lab's `ArticulationCfg`) and the button as a compound rigid body, and calls the task's
per-step record after every control step. No fork of yam_arena and no third-party assets: every body is built from box
and cylinder colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

17 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0 and an untouched start scores 0.0; the
insert rung needs the barrel 8 mm in (6.0 mm yes, 6.1 no), the tab within 12 deg (12.0 yes, 12.1 no), lateral error at
most 3 mm and tilt at most 8 deg; the seated rung needs the collar within 2 mm (y 3.0 mm yes, 3.1 no) and tilt 6 deg or
less; the ring rung needs 90 deg on a seated button (89.9 no) and turning with the button unseated earns nothing; snug
is 165 to 185 deg (164.9 and 185.1 fall to 0.75) with the ring face within 0.5 mm of the panel; 0.95 s of release is
not enough and 1.0 s is; one pad 29 mm away restarts the release timer and 30 mm does not; a hand at 10 mm/s, a button at
3 mm/s or a ring at 5 mm/s is not still; a button falling off the table gates the run to 0 even if it is later seated;
the record is clean after a reset; the thread engages only with y at most 3.5 mm, the tab within 12 deg, and tilt at
most 8 deg; the tab-angle arithmetic; the panel hole is round and clear of the 21.8 mm barrel at every angle and solid
0.8 mm outside it, the 5 mm notch passes the 2.8 mm tab and closes above 13.5 mm, and a tab turned 90 deg meets plate;
the thread lead (a half turn is 6 mm, 165 deg leaves 0.5 mm); and the ring's 30 mm hex with a 0.2 mm clearance bore.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the red emergency-stop button flush in the panel hole, then screw its yellow locking ring snug against the back of the panel, and let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/estop-panel-ring-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant no_roll       # a cheat variant (or DEMO_VARIANT=no_roll)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the button's and the ring's poses (the tab angle, the button's axis and the ring angle
are computed from them) and the robot's own grippers. It never reads joint values or the record.

1. Left arm, pads closing 20 deg off horizontal: reaches in from the front over the button lying on its stand, grips the
   40 mm head at its centre of mass (so it does not swing in the pads), lifts it and carries it to 70 mm in front of the
   panel at hole height.
2. Left, closed loop: reads the tab angle from the button's pose and rolls the hand about the button's axis until the
   tab is within 1 deg of the notch (a 100 deg roll); levels the axis from its pose; then moves in 4 mm at a time, each
   time centring the barrel tip on the hole from the seen tip position, until the collar is on the panel face. It keeps
   holding the head.
3. Right arm, once the button is seen seated: reaches over from behind, pads on two opposite flats of the hex ring
   (the closing axis follows the ring's angle), and turns its wrist about the hole axis by up to 95 deg while moving
   forward 12 mm per turn; it lets go, backs off, and re-grips on the flats for the second stroke until the ring has
   turned about 175 deg.
4. Right backs away. Left lets go, backs 70 mm away and rises 50 mm. Both wait.

The demo scores 1.0 (`success`, episode ended at step 1072 of 1,200): the barrel entered with the tab 1.2 deg from
the notch, the collar sits 1.7 mm off the panel face (y 2.7 mm), the ring is turned 175 deg with its face 0.16 mm from
the panel back, and both grippers end 47 and 84 mm clear. Worst IK error 21 mm and 12 deg (a transient while the left
hand lifts the button), 2 mm and 10 deg on the right.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file; every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the button stays on its stand. | 0.0 |
| `no_roll` | Carries the button to the hole as presented (tab 100 deg off) and pushes it in. | The barrel stops with the collar 14 mm off the panel face: the tab meets the plate beside the notch. | 0.0 |
| `right_only` | Only the right arm turns the ring, as in the demo. | The ring is locked until the barrel stands in the hole, so it cannot turn. | 0.0 |
| `drop_in_front` | Rolls the button correctly, carries it to the hole, then lets go about 70 mm in front of the panel. | It falls to the table and is never in the hole. | 0.0 |

A left-arm-only run (seat the button, never thread the ring) scores 0.5.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures: y (the collar's
back face above the panel's mid-plane; the panel front face is at 1 mm), the tab's angle from the notch, the barrel
centre's lateral error from the hole axis, the button's tilt from the panel normal, the ring's angle and advance, the
button's and ring's speeds, and each gripper's pad clearance to the button and ring and its speed. The record keeps
flags, peaks and held times for the whole episode and is cleared only by the simulator reset.

- Gate, any time in the run, gives 0: the button falls off the table (centre more than 30 mm below the table top).
- 0.25: the barrel went at least 8 mm into the hole (y at most 6 mm) with the tab within 12 deg of the notch, lateral
  error at most 3 mm and tilt at most 8 deg.
- 0.5: the button was seated: y at most 3 mm (collar within 2 mm of the panel face), tab within 12 deg, tilt at most
  6 deg.
- 0.75: the ring was turned 90 deg while the button was seated, or the end state is right but a release or stillness
  timer is short.
- 1.0: at the end the button is seated, the ring is turned 165 to 185 deg with its face within 0.5 mm of the panel back,
  every pad point of both grippers is at least 30 mm from the button and ring and both grippers move under 10 mm/s
  for 1 s, and the button and ring are still (under 3 and 5 mm/s) for 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung and `distance()` is 1 - score. Real trial: a top photo
with a 2 mm feeler at the collar; the tab seen through the notch from behind; a painted line across ring and panel to
read the ring's turn on video; a 0.5 mm feeler between the ring and the panel back; video frames for the release and
stillness.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 1072.
- Runtime on one L40S: about 8 to 12 min wall clock for a demo run on the shared VM, plus about 2 min of Isaac Sim
  start-up. A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (every body is fixed or placed by the
  task file). The scripted demo reads only camera-observable poses and is closed loop, so small differences between runs are corrected.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 1072. The
  video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png`.
- Dream-1 run: made at filing with the command above, on this exact `task.py` (d8a866b279c0). Result: score 0.0 after 1200
  steps and 7 minutes, termination reason max_steps, log adhoc_b45d01dc.json.

```bash
inspect-robots inspect adhoc_b45d01dc.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_b45d01dc.json
```

## Materials

- Panel: a 180 x 110 x 2 mild-steel plate, 0.3 kg, grey, plane at y = -0.040, standing on a fixed 180 x 60 x 65 mm steel
  bench-frame block (5.5 kg, bolted to the table). A 22.5 mm round hole at (0.24, -0.040, 0.8675) with a 5 mm notch above
  it, cut to 13.5 mm from the hole centre. Verify: a 21.8 mm plug gauge passes with the tab-sized (2.8 mm) key in the
  notch; a 22.5 mm go gauge enters the hole.
- Emergency-stop button: ABS, 70 g: red 40 mm head 22 mm long, black 31 mm collar 4 mm long, dark grey barrel 21.8 mm
  diameter and 13 mm long (2 mm tip chamfer, 3-start thread of 4 mm pitch) and a yellow tab 2.8 mm wide and 1.9 mm long
  standing 12.9 mm from the axis, so the tab is 0.1 mm shorter than the panel is thick. The notch passes the tab with 1.1
  mm either side, which tolerates a roll error of about 4.9 deg before the tab touches the wall. Friction 1.0 / 0.9.
- Button stand: aluminium 47 x 26 x 30 mm, fixed, two 4 mm rails 15.5 mm either side of the centre.
- Locking ring: yellow nylon, 20 g, hex 30 mm across flats, 5 mm thick, hex bore 22.4 mm across flats, internal 3-start
  thread of 4 mm pitch (12 mm lead). It waits coaxial with the hole, its front face 6 mm behind the panel back, in a
  printed starter carrier that does not touch the button. Verify the thread: a half turn takes it 6 mm (a 6 mm gauge
  block between ring and panel back fits at the start and the ring seats on it after 180 deg).
- Layout chosen for reach: the arm closing axes (20 deg off horizontal for the left hand) and the right hand's approach
  (20 deg yawed, 25 deg down) were checked in inverse kinematics along the whole demo path.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Back the ring off to its carrier (front face 6 mm behind the panel back), lay the button on its
stand with the tab pointing 100 deg from straight up toward -x (mark an arrow on the head), wipe the barrel and notch
clean and check the 21.8 mm plug passes the hole with the tab in the notch. The panel, frame and stand are fixed on
taped outlines.

## Fidelity limits

The parts are rigid. The hole is a 0.75 mm stepped circle, so it is a little larger than a true round hole near its top
and bottom edges. The ring runs on a fixed helical track about the hole axis (12 mm per turn, only while the barrel
stands in the hole with the tab in the notch) instead of on the barrel's own thread, so the ring does not clamp the
button; starting the thread and cross-threading are not modelled. The sprung gasket and contact block are omitted.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
