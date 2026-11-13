# Secure a lab gas cylinder upright in its bench clamp before using it

The robot stands a green aluminium lecture bottle (2 in x 12 in, 0.6 kg, round foot) upright between the two cheeks of a
bench clamp, holds it there while a cross bolt slides across the front of its shoulder and a cam lever is pushed over
centre, and then lets go. At the end the bottle stands upright (within 3 deg, on the clamp's centre line), bears on the
locked bolt, and both grippers are clear. Compressed gas cylinders must be secured upright before use (OSHA
29 CFR 1910.101(b), CGA P-1), and a technician does this by hand with a lecture bottle in a bench clamp.

The ability is bimanual stabilise-while-latching with a round-footed body: one hand brings the bottle from its lean onto
the clamp's centre line and keeps it there, the other closes the clamp around it. It is not a pick and place (the bottle
never leaves the bench) and not a stack, and the hard part is physical: the round foot has no stable upright pose, the
bottle has to be dragged and tipped to the centre line through friction, and the lever has to be pushed over centre.

Physics, not the instruction, defeats the natural plans. The bottle rests leaning back 7.5 deg with its domed shoulder on
the back plate, 22 mm behind the track of the bolt, so a hand that only closes the clamp (the bolt and lever close on
nothing) leaves it at 7.5 deg, outside the 3 deg limit. A hand that only holds never closes the bolt and lever, which are
0.47 m from the right arm's base; and a round foot (centre of mass 140 mm above a foot of 25 mm radius) has no stable
upright pose, so a bottle that is let go before the bolt is home falls forward or back to the plate (5 deg lean, again
outside 3 deg). The two-arm role is therefore forced: the right arm stands the bottle up and holds it, the left arm closes
the bolt and the cam lever, and the bottle is let go only once the bolt is home and locked.

Instruction given to the policy (13 words; it states the goal only, never the method or the safety rule):

`Secure the green lecture bottle upright in the bench clamp, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: a9d6130a9f6f (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the clamp as a PhysX articulation (a fixed plate with a prismatic bolt and a revolute cam
lever, USD joints with limits and drives, spawned through Isaac Lab's `ArticulationCfg`) and the bottle as a compound
rigid body, and calls the task's per-step record after every control step. No fork of yam_arena and no third-party
assets: every body is built from box, cylinder and capsule colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests        # 21 scorer tests, pure Python (yam_arena must be importable for TABLE_TOP)
```

The tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; the start (bottle leaning 7.6 deg, bolt
open) and doing nothing score 0; the operator alone closing bolt and lever on the leaning bottle scores 0; the holder
alone, releasing with the bolt open and the bottle fallen, scores 0; tilt 3.0 deg is upright and 3.1 deg is not; axis
offset 4.0 mm and 4.1 mm; bolt 80.0 mm and 79.9 mm; lever 85.0 deg and 84.9 deg; without the cam lock only the 0.25
rung is earned; the topple gate at 40.0 deg versus 39.9 deg, never cleared by a later good end state (and cleared by a
simulator reset); a hand 29 mm from the clamp versus 30 mm; a hand at 5.0 mm/s versus 4.9 mm/s; 0.95 s of release versus
1.05 s; a bottle still for 1.9 s versus 2.1 s; a bolt pulled back after the lever closed leaves 0.5; and `measure()` on
synthetic link states gives the right tilt, axis offset and bolt travel.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Secure the green lecture bottle upright in the bench clamp, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cylinder-bench-clamp-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant left_only     # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the bottle's pose, the bolt's and lever's link poses (the bolt travel and lever angle are
computed from them) and the robot's own grippers. It never reads joint values, the lock state or the record.

1. Right arm (holder): pads open 94 mm, approaching from the front with the hand pitched 20 deg down (a level approach is
   out of this arm's reach), closes on the bottle 60 mm above the table with the pads commanded 16 mm short of it, so they
   grip hard. Then up to eight corrections from the seen pose: drag the foot onto the clamp's centre line, then pull
   the hand back until the axis is vertical (the foot lags the pads). The bottle is held at 0.6 deg.
2. Left arm (operator), starting 9 s in, fingers closed: the fingertips on the bolt knob's outer face (hand heading 60 deg
   from straight ahead, pitched 50 deg), pushes the bolt 85 mm along its guide until it stops at the keeper block, backs
   off 40 mm, then puts its fingertips on the lever's front face near the tip and swings them about the lever's hinge
   line (seen from the link poses) for 75 deg; the cam passes over centre and snaps to 100 deg, locking the bolt.
3. Right arm: seen leaning 0.05 deg back, it rotates the pads about the grasp point a little forward until the bottle is
   seen leaning 0.2 deg forward (its shoulder on the bolt), opens the pads over 1.2 s, backs away 90 mm and lifts 40 mm.
   Left backs away 90 mm and lifts 40 mm. Both wait.

The demo scores 1.0 (`success`, episode ended at step 763 of 1,200): the bottle was first upright with the bolt home at
the start of the lever swing, the lever closed (100 deg) with the bolt locked, and at the end the bottle stands at
0.2 deg forward on the bolt (its foot 1.6 mm forward of the line), still, with both grippers at least 30 mm clear. Worst IK error
2.1 mm and 8.3 deg on the right arm; on the left arm 25.8 mm and 7.1 deg in one transit move (the push and lever contact
poses and the swing solve to under 0.1 mm).

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (a9d6130a9f6f); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the bottle stays leaning 7.6 deg. | 0.0 |
| `left_only` | The left arm closes bolt and lever as in the demo; the right arm stays at ready. | The bolt travels 85 mm and the lever closes (100 deg, locked), but the bottle still leans 7.6 deg on the plate, 22 mm behind the bolt: never upright, no rung. | 0.0 |
| `right_only` | The right arm stands the bottle up and holds it for 12 s, then lets go and backs away; the left arm stays at ready. | Upright (0.6 deg) while held, never with the bolt home; after the let-go it falls forward and lies at 90 deg: topple gate. | 0.0 |
| `release_early` | As the demo, but the right arm lets go as soon as the bottle is upright and the left arm's bolt comes about 10 s after the let-go (the time one hand needs to stand the bottle up, let go and cross over). | Upright (0.6 deg) while held; after the let-go it starts to fall within a second and lies at 90 deg 3 s later, before the bolt is home (the bolt then travels the full 85 mm): topple gate. | 0.0 |
| `lever_first` | The left arm pushes the lever over without closing the bolt first; the right arm holds, then lets go as in the demo. | The lever closes (100 deg) but the bolt stays at 1.5 mm, so the cam never locks it and no rung is earned; after the let-go the bottle falls (topple gate). | 0.0 |

On timing: in simulation a released bottle takes a second or two to start to fall, so the clamp can catch a bottle if
the bolt arrives soon enough. The first `release_early` runs we made (the bolt 2 s and then 4 s after the let-go) scored
1.0 for that reason, and the variant was lengthened to the single-hand timing above. A real round-footed bottle falls as
soon as it is let go, so the real trial is stricter than the simulation on this point.

## Build log (about 25 simulator runs on the shared L40S, 5 to 6 min each)

- First builds: the bottle's tilt had to be built into its colliders (a link rotation is not applied to a single rigid
  body), and the hand's pads slid over the bottle at 3 mm squeeze and at 0.9 kg. Pads commanded 16 mm short of the
  bottle, the grip moved down to 60 mm and a 0.6 kg bottle fixed that (the first 1.0).
- That first 1.0 used V rods tangent to the upright bottle. The cheats showed such rods are a stable back rest: a bottle
  stood against them and let go stays there, so one hand could stand it up, let go and then close the clamp. Setting the
  rods back made an upright bottle free, but then it fell sideways onto a rod when let go.
- Final geometry: side cheeks 1.5 mm each side (no lateral fall) and a back plate 22.8 mm behind the upright shoulder
  (an unheld bottle falls forward or back to the plate), with the holder tipping the bottle a little forward about its
  pads so it rests on the bolt rather than falling back after the let-go: 1.0 at step 763, repeated unchanged.
- The cheats were run on that geometry. `release_early` scored 1.0 when the bolt came 2 s, then 3 s after the let-go
  (the clamp caught the falling bottle) and 0.0 at 10 s, as described above. All variants ran on this task file except
  that the 17 s wait in the `release_early` branch was added after the other runs; the demo path and the other variants
  do not use it.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures: the bottle's
tilt (its axis from vertical), its axis offset (where its axis crosses the 0.20 m height, from the clamp's centre line at
x 0.170, y 0), its speed and spin (from its pose change between control steps, what a camera sees), the bolt's travel
(mm from retracted), the lever's angle, whether the cam lock is set, and each gripper's pad clearance to the bottle and
clamp and its speed. The record keeps flags, peaks and held times for the whole episode and is cleared only by the
simulator reset.

- Gate, any time in the run, gives 0: the bottle's tilt reached 40 deg (it toppled). The start lean of 7.6 deg is no
  gate.
- 0.25: the bottle upright (tilt at most 3.0 deg and axis offset at most 4 mm) with the bolt home (at least 80 mm) at some
  time.
- 0.5: upright, bolt home, lever closed (at least 85 deg) and locked at some time.
- 0.75: that state at the end.
- 1.0: that state at the end, the bottle still (under 3 mm/s and 3 deg/s) for the last 2 s, and the release gate: every pad
  point of both grippers at least 30 mm from the bottle and clamp and both grippers moving under 5 mm/s for the last 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: a plumb line or 300 mm spirit level along the
bottle (3 deg is 16 mm over 300 mm) and a ruler from the clamp's centre line for the axis offset; the bolt travel marked
on its side (home at 80 mm or more, scale); a protractor on the lever; a spring scale pulling the knob 10 N moves a
locked bolt less than 1 mm; a ruler for the 30 mm hand gap; 60 fps video for stillness (3 mm/s is 0.05 mm per frame).

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 763.
- Runtime on one L40S: a demo run took about 5 to 6 min wall clock (3.3 to 3.8 control steps/s plus Isaac Sim start-up).
  A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (the clamp is fixed and the bottle is
  never jittered). The scripted demo is deterministic (the repeats ended in the same state to 0.01 mm); Dream-1 is a
  hosted model without a seed.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 763. The video
  is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png` (frame 0
  of the top camera).
- Dream-1 run: score 0.0, max_steps, 1200 steps, 3 minutes, log adhoc_94428f68.json.

```bash
inspect-robots inspect adhoc_94428f68.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_94428f68.json
```

## Materials

- Lecture bottle: an aluminium 2 in x 12 in bottle, green paint, 0.6 kg (a real one is 0.5 to 1.4 kg), friction 0.4 / 0.3:
  a 50 mm diameter body with a domed foot, 310 mm long, with a brass valve stub 22 mm in diameter and 40 mm long on top,
  345 mm tall in all. It rests on the table on its foot, leaning back 7.5 deg with its domed shoulder on the plate.
- Back plate: mild steel, 20 x 200 x 300 mm, bolted down; its face is 47.8 mm behind the upright axis (x 0.170).
- Side cheeks: two steel rods 24 mm in diameter and 90 mm long at y +-38.5 mm, spanning 110 to 200 mm above the table,
  each tied to the plate by a 47.8 x 12 x 12 mm arm; 1.5 mm clear of the bottle on each side.
- Cross bolt: a 10 x 120 x 12 mm steel bar (60 g) with a red 16 x 16 x 20 mm knob, sliding 85 mm in a guide bridge
  (88 x 36 x 15 mm), joint friction 0.6 N; home stops it 4 mm short of the keeper block (12 x 15 x 40 mm) and 2.5 mm in
  front of an upright bottle's body (254 to 266 mm up). Retracted, its tip is 10 mm clear of the bottle.
- Cam lever: an 8 x 12 x 50 mm blue arm (20 g) with a red tip, hinged on the guide bridge on a horizontal pin; it stands up
  at 0 deg and snaps over centre to 95 deg (spring detent 0.02 N m/rad); past 80 deg with the bolt home its pad locks the
  bolt (a friction brake in the simulator; a cam pad in a real clamp).
- Layout and physics chosen for reach and feasibility: the bottle stands at x 0.170 so both arms reach it (the right arm
  needs its hand pitched 20 deg down); the clamp is a cross bolt, not a strap, because a strap needs a rope or chain
  model; the bottle is 0.6 kg so the arm can drag its foot across the table. Positions, colours and friction for every
  body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Raise the cam lever to 0 deg (the bolt is then free), slide the bolt back to its retracted stop, and
stand the bottle on its foot leaning back 7.5 deg with its domed shoulder on the back plate (foot centre 0.1584 m from the
arm bases, on a taped mark). The plate, cheeks, guide and keeper are bolted down and need no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our own
code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public Isaac Lab
and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no third-party assets or
brands.
