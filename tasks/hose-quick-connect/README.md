# Connect a garden hose to its tap with a quick-connect coupler and let it lock

The robot connects a garden hose to an outdoor tap through a ball-lock quick-connect coupler. One hand pulls the
coupler's sprung collar back and holds it there while the other lifts the hose's brass male fitting, lines it up with
the bore and pushes it in until its flange meets the coupler; the collar is then released, springs forward over the
locking pawl, and the fitting is held. At the end the collar is forward, the pawl is in the nipple's groove so that the
fitting holds a pull, and both grippers have let go. Everyone who waters a garden, washes a car or fills a paddling pool does this.

The ability is the engage direction of a sprung-collar coupling: hold a spring-loaded sleeve back with one hand for the
whole length of a push made by the other, then release it at the right moment so it locks. Nothing is extracted,
released or unplugged, and nothing is placed on a target: the collar's land holds the pawl down, so the nipple's nose
cannot pass the pawl until the collar is back 6 mm or more. The same fitting pushed against a closed collar stops at
the pawl; a fitting that stops a few millimetres short of the mouth leaves the pawl standing on the shaft's shoulder
instead of in the groove, which holds nothing, and the score gives that no credit.

Physics, not the instruction, defeats the natural plans. A hose end lies on the table with its hose anchored at the
other end, so it has to be lifted and carried in line with the bore (the hose resists being dragged straight at the
coupler from the side). The pawl and the collar's land hold the nipple out until the collar is held back; both
hands are busy at once.

Instruction given to the policy (20 words; it states the goal only, never the technique or the collar):

`Connect the garden hose to the tap's quick-connect coupler so that it locks on, then let go of both parts.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: 70e1c63cf9f3 (`sha256sum task.py | cut -c1-12`)
- for the Dream-1 trial: inspect-robots-dream 1.0.0, yam-arena 0059d02a1a91, task file 70e1c63cf9f3 (equals
  `eval.environment_revision` in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the coupler (a fixed body with a sprung sliding collar and a sliding pawl) and the hose with
its nipple (a chain of revolute links) as PhysX articulations, built through Isaac Lab's `ArticulationCfg` with USD
joints, limits and drives, and calls the task's per-step record after every control step. No fork of yam_arena and no
third-party assets: every body is built from box, cylinder and capsule colliders in `task.py`. `linkkit.py` and
`linkdemo.py` are our own code under the MIT licence.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; doing nothing, pushing against the
closed collar, pulling the collar with no nipple in, and every position-perfect trace in which the pawl is not in the
groove (the old full-score state) all score 0; engaged with the collar still held back is 0.5 and with the collar
forward but the hands on is 0.75; engagement from the pawl pose (tip 6.9 mm from the shaft axis counts, 7.1 does not;
a flange 4.5 mm short does not); flange 3.0 mm from the mouth is seated and 3.1 is not; tilt 12.0 deg vs 12.1; offset 4.0 vs 4.1 mm; collar
1.0 vs 1.1 mm; a pad point 29 mm clear restarts the release timer and 30 mm does not; a hand at 4.9 vs 6.0 mm/s; the
fitting at 2.9 vs 3.1 mm/s; 0.95 s of release is not enough and 1.05 s is; seating once and then pulling the nipple out
again keeps the rung at 0.5; the record is clean after a reset; measure() on stand-in link states gives the right
collar travel, flange offset, nose depth, tilt and sideways offset, and the plug's speed from its pose change between
control steps.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Connect the garden hose to the tap's quick-connect coupler so that it locks on, then let go of both parts." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/hose-quick-connect-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant no_collar      # a cheat variant (or DEMO_VARIANT=no_collar)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the coupler body, collar and fitting poses and the robot's own grippers. It never reads
joint values, the pawl or the record.

1. Right arm: top-down on the fitting's boot (pads on its two flats, 10 mm squeeze), lifts it 45 mm, backs it off
   along the bore line level with the staging pose, then slides it sideways to the bore axis, 40 mm in front of the
   mouth, with the nipple pointing along the bore; it nulls any slip from the seen fitting pose.
2. Left arm, once the fitting is staged: pads over the collar's finger rails, just in front of their thumb ridges;
   pulls the collar back 8.5 mm, closed loop on the seen collar position (the ridges give a positive stop for the pads).
3. Right arm, once the collar is seen back 7 mm: pushes the fitting in 3 mm at a time, correcting its sideways and height
   error from the seen pose at every step, until the flange is seen at the mouth. Its nose cams the pawl up on the way.
4. Right arm leans 2 mm on the flange for 1.5 s so the pawl can drop into the groove. Left arm lets go: the collar springs
   forward over the pawl. Right arm lets go and backs away; left arm backs away; both wait.

The demo scores 1.0 (`success`, episode ended at step 859 of 1,200): the collar peaked at 10.0 mm, and at the end the
flange is 1.9 mm from the mouth with the pawl tip in the groove (centre 10.9 mm behind the flange face, 5.8 mm from the
shaft axis, the groove core being 5.5 mm), the collar at 0.0 mm, tilt 8.7 deg, offset 2.5 mm, both grippers 171 and
105 mm clear. Worst IK error 0.1 mm and 0.01 deg. In an earlier run of this demo with the previous scorer the flange stopped 3.4 mm short with the pawl on the
shaft's shoulder and the old scorer still gave 1.0; see Revision.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (70e1c63cf9f3); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `left_only` | Left arm alone pulls the collar back 8.5 mm and lets go; the fitting is staged and never pushed in. | The collar springs forward; the nipple never enters (flange 38.6 mm out). | 0.0 |
| `right_only` | Right arm alone brings the fitting to the mouth and pushes it in, collar left forward. | The nose stops on the pawl, 17.7 mm out; no engagement. | 0.0 |
| `short_stop` | Both arms; the push stops with the flange 10 mm out and the collar is let go at once (the old scorer's seat window allowed 3 mm, and the old demo ended 3.4 mm out on the shoulder). | The pawl stands on the shaft, 8.5 mm out; the collar stays held up by it (5.7 mm back); never in the groove. | 0.0 |

## Build log (44 runs, then 8 revision runs (5 demo attempts, 8 cheat runs); short ones to find the pull and the push, 3 to 6 min each on the shared L40S)

| Stage | Change | Result |
|---|---|---|
| 1 | first scene (hose 0.28 m, plug 36 mm body, collar pulled by squeezing its sides) | 0.0: the hose, anchored to one side, was too short to bring the fitting in line with the bore |
| 2 | plug boot 62 mm, hose clamp behind the coupler, hose 0.20 m straight | staged the fitting to 0.5 mm; the hands no longer touched |
| 3 | finger rails and thumb ridges on the collar | the squeezed sleeve had jammed on the body; the ridges give a positive stop |
| 4 | collar spring 800 to 300 N/m | the arm could pull 4 N but not 11 N at this reach: collar 10 mm back |
| 5 | bore 18 to 20 mm; the pawl 5 mm above the axis, 1.5 mm under the land; the nose rounded | the flat nose could not cam the pawl up; with the round nose the nipple slides in |
| 6 | push in 3 mm steps with sideways correction; seat window 3 mm / 12 deg / 4 mm (the pawl has 1.25 mm of slack in the groove, the hose tips the fitting) | 1.0 at 1,148 steps |
| 7 | shorter waits | 1.0 at 817 steps, `success` true (the old scorer; the flange was in fact 3 mm short of the mouth) |
| 8 | revision: scorer reads the pawl pose; pawl rest tip 5 to 3.5 mm above the axis; 18 mm bore tried and dropped (the tilted fitting wedged) | demo run 1 with the engagement check scored 0: the flange stopped 3.4 mm short and the pawl stood on the shoulder |
| 9 | groove 6 to 8 mm wide; the right hand leans 2 mm on the flange for 1.5 s before the collar is let go | engaged, but the fitting slid back 1.4 mm and its pawl centre was 0.02 mm outside the check window: 0.5 |
| 10 | groove 9 mm wide (4 to 13 mm), check window 6 to 11 mm | 1.0 at 840 steps; but `right_only` (no collar pulled) also scored 1.0: with the pawl 3.5 mm up the 3 mm land no longer stopped the nose |
| 11 | land 1.6 mm over the pawl (body top wall 4 mm), 0.5 s hold for the lock rung | demo 1.0 at 859 steps, `success` true; `right_only` 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures, in the coupler
frame: the collar's travel from the seen link states, the nipple's flange offset f along the bore (mm from the mouth,
negative in front), nose depth (f + 28 mm), the fitting's tilt from the bore axis and sideways offset, its speed (from
its pose change between control steps, what a camera sees), and each gripper's pad clearance to the coupler and the
fitting and its speed. The record keeps flags, peaks and held times for the whole episode and is cleared only by the
simulator reset, so unplugging and starting again never clears it.

- Engaged (the simulation's equivalent of the real 10 N pull test): the pawl link's tip, read from its pose in the
  nipple's frame, is in the groove, that is its centre 6 to 11 mm behind the flange face (the 3.5 mm pawl inside the 4
  to 13 mm groove) and within 7.0 mm of the shaft axis (core 5.5 mm, shoulder 7.75 mm: at least 0.75 mm below the
  shoulder, so the fitting cannot be pulled out past it).
- Seated: engaged, flange within 3.0 mm of the mouth (-3.0 to +1.5 mm), tilt at most 12 deg, offset at most 4 mm.
- 0: the pawl was never held in the groove with the flange at the mouth for 0.5 s (a pass-through does not count). Positions alone, a short push, a closed collar or one hand
  earn nothing.
- 0.5: the nipple was seated (engaged) for 0.5 s at some time, but at the end it is not locked with the collar forward (pulled out
  again, or the collar held back).
- 0.75: at the end, engaged and seated with the collar forward (at most 1 mm back) and the pawl under its land, so a pull
  on the hose is carried by the pawl against the groove wall; the fitting or the hands are not yet quiet for 1 s.
- 1.0: that, with the fitting still (under 3 mm/s) and the release gate (every pad point of both grippers at least 30 mm
  from the coupler, collar and fitting, both grippers moving under 5 mm/s), together for 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: a steel rule fixed along the bore on the table
(1 mm marks from -30 to +3 mm) and a scribed 7 mm line on the body, read in the top video at 20 Hz for collar travel,
flange offset and nose depth; a try-square for tilt; the top and side video for the 1 s hold, stillness and 30 mm
clearance. Full credit also requires the real 10 N pull: with the trial otherwise finished (collar forward, hands clear for 1 s), a
spring scale hooked to the hose end just behind the boot pulls 10 N along the bore for 5 s (hand pull, read on the
scale); the fitting must move under 1 mm and stay in the coupler. A fitting that comes out scores 0.5 at most, whatever
the video shows. The simulation's engagement check (above) stands in for this pull.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 859.
- Runtime on one L40S: a demo run took about 4 min wall clock on the shared VM, plus about 2 min of Isaac Sim start-up.
  A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout. The scripted demo is deterministic
  (runs 42 to 44 ended in the same state to 0.1 mm).

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 859. The video
  is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png` (frame 0
  of the top camera).
- Dream-1 run: score 0.0, ended with max_steps after 1200 steps, about 9 minutes wall clock, log adhoc_bbe8201a.json, on this
  `task.py` (70e1c63cf9f3).

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials

- Tap-side coupler: brass, a 30 x 30 x 50 mm square body with a 20 x 20 mm bore 40 mm deep (5 mm walls), mouth at world (0.150, 0),
  axis 30 mm above the table, on a tap stand with a 22 mm spigot (0.5 kg, fixed, friction 0.3).
- Locking pawl: steel, 3.5 x 10 x 10 mm, 5 g, sliding in a 4.5 mm slot 8 mm behind the mouth; its tip stands 6.5 mm into the
  bore, 3.5 mm above the axis; a 100 N/m spring holds it down; 6 mm of travel.
- Sprung collar: black anodised aluminium, 44 x 24 x 42 mm, 40 g, friction 1.0 / 0.9, 3 mm clear of the body, 10 mm of
  travel on a 300 N/m spring (1.2 N at rest, 4.2 N at the stop); a 3.5 mm roof whose land sits 1.6 mm over the pawl's top (the top wall of the body is 4 mm thick),
  open in front of the land; two finger rails 52 mm over both and 3 mm thumb ridges at the rear.
- Hose: green PVC, 25 mm outer diameter, 5 links of 40 mm (12 g each), alternating vertical and horizontal hinges with
  0.003 N m/rad springs, anchored to the table by a 30 x 50 x 25 mm clamp post at (-0.100, 0).
- Male nipple fitting: brass, 120 g, friction 0.4 / 0.3: a 15.5 mm shaft 28 mm long with a rounded nose, a 9 mm groove
  (11 mm core) 4 to 13 mm behind the flange, a 26 x 26 x 8 mm flange and a 24 x 24 x 62 mm crimp body with
  strain-relief boot.
- Layout and physics chosen for reach and feasibility: the coupler is 0.15 m from the arm bases so that the left arm
  can pull the collar and the right arm can bring the fitting in line; the collar spring is light (4.2 N at full stroke)
  because the arm could not pull a stiffer one at this reach; the hose is anchored behind the coupler so the fitting can
  be lifted into line with the bore.
- How to build and verify it: the collar's spring rate by spring scale on the rails (1.2 N at the start, 4.2 N at
  10 mm); the pawl's intrusion by depth gauge (6.5 mm); the land clearance (1.6 mm) and sleeve clearance (3 mm) by feeler
  gauge; the groove (9 mm wide, 11 mm core) and shaft (15.5 mm) by calliper; the pawl rest height by depth gauge (tip 3.5 mm above the bore axis, 6.5 mm into the bore, so that it reaches the 11 mm core wherever the 15.5 mm shaft sits in the 20 mm bore); the lock by the 10 N pull, which is a scored step of the real trial.
- Fidelity limits: one steel pawl stands in for the balls of a ball-lock; the bore is square around a round shaft; the
  hose is 5 stiff links 0.20 m long; the collar spring is light.

## Revision

Reviewer points (TK-GD5BJP) and what changed:

1. Construct validity, "the 20 mm bore permits the 15.5 mm shaft to sit low enough that the pawl can miss its 11 mm
   groove core while the collar returns forward". The pawl's rest position was 5 mm above the bore axis and its slide
   cannot go lower, so with the shaft 2.25 mm low on the bore floor the pawl hung 1.75 mm above the core and held
   nothing. The pawl now rests 3.5 mm above the axis (tip 6.5 mm into the bore), so wherever the shaft sits in the bore
   (at most 2.25 mm off axis) the tip reaches the core: 5.75 mm from the shaft axis at the lowest, against 7.75 mm for
   the shoulder. The groove is 9 mm wide (4 to 13 mm behind the flange) so the 3.5 mm pawl has 2.75 mm of travel in it,
   and the collar's land is 1.6 mm over the pawl's top: the pawl rides up 2.0 to 3.25 mm on the shaft but at most about 1.4 mm in the groove, so the collar returns only with the pawl in the groove (a first fix with a 3 mm land let the fitting go in with the collar forward and scored 1.0 for the right hand alone). An 18 mm bore was tried and dropped: the tilted fitting wedged in it.
2. Scoring, "does not require the stated 10 N pull test or otherwise verify engagement". Full credit now needs the lock:
   in the real trial the 10 N pull (spring scale, 5 s, under 1 mm, the fitting stays in) is a scored step; in the
   simulation the scorer reads the pawl's pose in the nipple's frame and counts a lock only while the pawl tip is in the
   groove (centre 6 to 11 mm behind the flange face, within 7.0 mm of the shaft axis) with the flange within 3 mm of the
   mouth, together with the collar forward. Collar position and flange tolerance alone no longer score at all: every
   rung needs the engagement, so a fitting that sits at the mouth with the pawl on its shoulder earns 0.
3. "Update the 0.75 rung as well so it does not describe an unchecked fitting as locked". The 0.75 rung is now "engaged
   (pawl tip in the groove, checked from the pawl pose) and seated, collar forward, not yet quiet", and the old 0.25 and
   0.5 rungs, which were credit for positions alone, are gone (0.5 is now "was engaged once, not locked at the end").
   The old demo ended 3.4 mm short of the mouth with the pawl on the shoulder and still scored 1.0 on the old rule; the
   demo now leans on the flange for 1.5 s so the pawl drops in, and ends engaged.
4. Cheat for the named loophole: `short_stop` (the fitting stopped short with the pawl on the shaft and the collar let go),
   and `left_only`, `right_only` (one hand alone), each score 0 (table above). The older `nothing`, `no_collar`,
   `collar_only` and `drop_at_mouth` variants stay in `task.py`; `no_collar` and `collar_only` are the same runs as
   `right_only` and `left_only`.

## Licence

`task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py`: MIT, Copyright (c) 2026 the task authors.
