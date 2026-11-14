# Lower a tool chest lid by its side handle while its stay release is held

The robot closes the lid of a hand-carry toolbox, a chore nearly every household has: in a garage, shed or van the lid
is propped open on a ratchet stay and has to be put down without dropping it. The orange aluminium lid (1.3 kg, a
226 x 472 x 4 mm plate with 20 mm lips) rests propped open at 25 deg. The stay lets the lid be raised but not lowered
unless its red release knob (a spring-return pull tongue on the chest's left end wall) is held pulled out 6 mm or more.
With the knob out the lid swings shut by itself, so it must be lowered slowly by its flat side handle (a pull tab on the
lid's right side), with the grippers kept out of the gap under the plate and out of the hinge, where a person's
fingers would be. At the end the lid rests flat on the rim, the knob has sprung home and both grippers have let go.
Category: Societal relatability (a familiar everyday household task).

### The real ratchet stay, pawl and knob linkage (build it as follows)

Frame: hinge line x 0.36, z 0.9075 (the rim); the stay is on the chest's left inner wall (y +0.224).

| Part | Size, material | Mounting | Travel and behaviour |
|---|---|---|---|
| Toothed stay rod | 3 x 12 x 160 mm steel flat bar; sawtooth teeth, 4 mm pitch, 1.5 mm deep, on its last 60 mm | pinned to the inside of the lid's left lip by a 3 mm pin, 150 mm from the hinge line and 15 mm below the plate | slides 31 mm through the guide between shut and 25 deg open (pin to guide 76 mm shut, 107 mm at 25 deg) |
| Guide block | 12 x 14 x 30 mm steel, 3.2 mm slot | pivots on a 3 mm pin in a bracket on the left inner wall, 80 mm in front of the hinge line and 45 mm below the rim | rocks about 25 deg with the rod |
| Pawl | 2 mm steel plate, 10 mm wide, 3 mm pin in the guide | bears on the teeth under a 0.3 N leaf spring | ramps over the teeth when the lid is raised (any angle holds); blocks the lid going down |
| Bellcrank | 2 mm steel, arms 24 mm and 8 mm (3:1) | pinned to the guide bracket | 6 mm of knob travel lifts the pawl 2 mm, clear of the 1.5 mm teeth; it drops back when travel falls under 4 mm |
| Pull rod and knob | 3 mm steel rod through a 5 mm hole in the left end wall (centred x 0.20, z 0.08 above the table top (0.8275 in the station frame)); red 10 x 60 x 30 mm tongue outside | collar stops the rod at 20 mm out | 150 N/m extension spring, no preload: 0.9 N at 6 mm, 2.1 N at 14 mm, so the knob springs home when let go |

Release behaviour: with the knob under 4 mm the lid can go up but not down; 6 mm or more out frees it (the lid then
falls shut by itself unless held); letting the knob go part way re-engages the pawl at the next tooth below (a drop of
up to 4 mm of rod travel, about 3 deg of lid angle). Verify the build: a 5 N spring scale on the knob reads about 0.9 N
at 6 mm; the lid does not move down with the knob at 5 mm and does at 6 mm; a protractor on the hinge shows the stay
holding the lid at any raised angle. The simulation models this as a hinge limit (see Materials).

The ability is the controlled lowering of a self-closing hinged lid against a closing speed limit while the other hand
holds a spring release open and the grippers stay out of the pinch gaps. Nothing latches or snaps and nothing is
threaded. A slam is zeroed for good because a dropped 1.3 kg lid dents its rim and can hurt a
hand, not merely for speed.

Physics, not the instruction, forces both arm roles. The stay holds the lid at 25 deg whatever the tab arm does, so the
tab arm alone never lowers the lid (it scores 0). The knob arm alone frees the lid, which then falls shut at several
times the speed limit (a slam, gated to 0). The arm bases stand 0.61 m apart and reach 0.35 m: the knob is 0.5 m or
more from the right arm and the tab 0.5 m or more from the left arm, and the release has to be held for the whole
descent, since letting go mid-way catches the lid where it is. Letting go of the lid early is a slam as well.

Instruction given to the policy (20 words; it states the goal only, never the release, the speed limit or the pinch
zones):

`Close the orange lid of the grey tool chest by its side handle until it rests flat on its rim, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the
demo and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 270924715a1c (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the chest as a PhysX articulation (a fixed body with a hinged lid and a sprung sliding knob, USD
revolute and prismatic joints with limits, spring drive and damping, spawned through Isaac Lab's `ArticulationCfg`) and calls the task's per-step record
after every control step. No fork of yam_arena and no third-party assets: every body is built from box colliders in
`task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

20 tests feed synthetic records at 20 Hz to the scorer: a demo-shaped trace scores 1.0; nothing scores 0; the tab arm
alone (the knob never moves, the lid stays on its stay) scores 0; the knob arm alone (the lid falls 25 deg in 0.4 s) is
a gated slam; the speed gate is 9.4 deg in 0.25 s (0.148 m/s) no gate and 9.7 deg (0.153 m/s) gate, a slam and a fast
lift are gated for good even when the lid ends shut and released, and a pad in a pinch zone beats a perfect finish; the
lid is shut at 0.6 deg and not at 0.7 deg; the 0.25 rung needs the lid below 20 deg (20.5 deg is not enough) and the 0.5
rung below 15 deg (14.0 yes, 16.0 no); grippers 29 mm from the chest score 0.75 and 30 mm score 1.0; the knob 3 mm out
is not home and 1.9 mm is; the release timer needs a full second and restarts when a gripper moves at 20 mm/s; a lid
creeping at 6 mm/s is not still; the record is clean after a reset; and the pinch zone is right: 20 mm under the plate
in its footprint is the rim zone, 20 mm above it, 50 mm below it, at the side tab and at the knob are not, 24 mm from
the hinge line is the hinge zone and 26 mm is not, and the zone follows the lid when it is open.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Close the orange lid of the grey tool chest by its side handle until it rests flat on its rim, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/lid-close-no-pinch-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                         # scripted demo
python linkdemo.py task.py cheat_out --variant slam         # a cheat variant (or DEMO_VARIANT=slam)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure (link poses of the chest, the lid angle and the knob travel computed from them) and the
robot's own grippers. It never reads joint values, the stay state or the record.

1. Left arm: moves 40 mm above the knob tongue, goes straight down onto it (pads closing along x), closes the pads 7 mm
   below the 10 mm tongue thickness and pulls it 14 mm out along y (the pawl lifts at 6 mm), then holds it.
2. Right arm, in parallel: moves out from the lid tab along a tilted approach (pads closing about along the plate's
   normal), slides in and closes the pads 4 mm below the 8 mm tab thickness. It waits until the knob is seen out 8 mm,
   then lowers the lid 25 deg over 11 s (peak free-edge speed 0.025 m/s) and nudges from the seen hinge angle until it
   is within 0.3 deg of shut.
3. The left arm waits until the lid is seen shut, then both open their grippers and back out 80 mm along their
   approach.

The demo scores 1.0 (`success`, episode ended at step 507 of 1,200): the knob peaks at 11.0 mm, the lid is shut and
still, peak free-edge speed 0.025 m/s, no pad in a pinch zone, both grippers clear and still with the knob home. Worst
IK error 0.1 mm and 0.0 deg while grasping and lowering (5.7 mm and 0.6 deg on the final back-out).

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (270924715a1c); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | The lid stays on its stay at 25 deg. | 0.0 |
| `left_only` | The left arm pulls the knob and holds it; the right arm stays at home. | The lid is freed and falls shut: free edge at 0.39 m/s, gate. | 0.0 |
| `right_only` | The right arm pinches the tab and lowers; the left arm stays at home. | The stay holds the lid at 25 deg; it never leaves its stay. | 0.0 |
| `slam` | Both arms work; the tab arm lowers to 20 deg and opens its gripper. | The lid falls from 20 deg: free edge at 0.35 m/s, gate. | 0.0 |
| `fast` | Both arms work; the tab arm lowers the lid in 0.5 s. | Free edge at 0.26 m/s, gate. | 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures the lid angle q
(0 = plate on the rim), the free-edge speed (the free edge's travel over the last 0.25 s, 0.226 m from the hinge, what a
60 fps video reads), the knob travel, and for each gripper its pad-point clearance to the chest and its speed. The
record keeps flags, peaks and held times for the whole episode and is cleared only by the simulator reset, so letting
the lid fall and lifting it again never clears the violation.

- Gate, any time in the run, gives 0: a free edge faster than 0.15 m/s (2.5 mm per frame at 60 fps); or any pad point
  of either gripper inside the lid's pinch zone: the gap under its plate (within 40 mm below the plate, over its
  footprint) or within 25 mm of its hinge line. The demo's pads stay at the side tab and at the knob, outside the
  plate's footprint.
- 0.25: the lid was lowered below 20 deg (it cannot leave its 25 deg stay unless the knob is out 6 mm).
- 0.5: the lid was lowered below 15 deg.
- 0.75: the lid is shut at the end (q at most 0.6 deg, the free edge within 2.4 mm of the rim).
- 1.0: also the lid still (under 5 mm/s) for the last 1 s, and the release gate: every pad point of both grippers at
  least 30 mm from the chest, both grippers moving under 10 mm/s and the knob home (under 2 mm) for 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score.

Real trial, with a stated tool for each check: a protractor on the hinge (the stay holds at 25 deg; the rungs are 20 and
15 deg); a ruler on the knob (6 mm out frees the lid, under 2 mm is home; a 5 N spring scale reads about 0.9 N at 6 mm
and 2.1 N at 14 mm); 60 fps video next to a mm scale for the free-edge speed (at most 37 mm of travel in any 15
frames); a 2 mm feeler gauge that must not pass under the shut lid's free edge anywhere along its width; blue tape on
the fingertips and tape lines on the chest (40 mm below the plate edge and a 25 mm band along the hinge) for the pinch
gate, read from the video; a ruler for the 30 mm release clearance; the video for stillness.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 507.
- Runtime on one L40S: a demo run took about 3 to 5 min wall clock on the shared VM, including about 2 min of Isaac Sim
  start-up. A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout (the chest is fixed and nothing is
  jittered). The scripted demo is deterministic (two full runs ended in the same state).

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 507. The
  video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png`
  (frame 0 of the top camera).
- Dream-1 run: score 0.0, max_steps, 1200 steps, 8 minutes, log adhoc_eb8b0cdc.json.

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials

- Chest body: grey powder-coated steel, 4.0 kg, fixed to the table, an open case 220 x 460 mm in plan (x 0.14 to 0.36,
  y -0.23 to 0.23) and 160 mm high (rim at z 0.9075) with 6 mm walls and floor. The footprint is that of an 18 inch
  hand-carry chest; the height is that of a low-profile one, so that the arm can reach the tab with the lid propped open.
- Lid: orange aluminium, 1.3 kg, a 226 x 472 x 4 mm plate with a 3 x 20 mm lip down its front edge and both side edges
  and a flat 40 x 50 x 8 mm pull tab sticking out of its right side (centred at y -0.261), on a hinge along the line
  x 0.36, z 0.9075 (damping 0.02 N m s/rad, friction 0.8).
- Ratchet lid stay: the lid starts open at 25 deg. It can be raised, and the stay then holds it at the new angle; it
  cannot go down while the release knob is under 6 mm. Built as a joint limit (the simulated stand-in for the rod, pawl and bellcrank above) that follows the lid and lifts when the
  knob is out 6 mm and catches again below 4 mm; in a real chest it is a ratchet rod with a pawl freed by a lever or
  cable from the knob.
- Release knob: a red 10 x 60 x 30 mm pull tongue (0.05 kg, friction 1.5) on the left end wall, centred at x 0.20,
  y 0.26, z 0.8275, sliding out along y from 0 to 20 mm against a 150 N/m spring with 2 N s/m damping.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Raise the lid to 25 deg (protractor on the hinge; the pawl catches it) and let the knob spring home
(ruler, under 2 mm). The chest is fixed on taped outlines and needs no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.

## Revision

Reviewer points on this task, and what changed (task.py, linkkit.py and linkdemo.py are unchanged, so the demo, the
cheat variants and the task file hash are as filed):

1. Importance: the safety domain needs harm to a person, and the scene only has robot grippers; closing a toolbox
   fits societal relatability. Changed: the category is now Societal relatability; the context and the opening of this
   README explain the familiar household toolbox-closing chore; the "physical safety constraint" wording is gone.
2. Clarity and reproducibility: the ratchet rod, pawl and knob linkage lacked dimensions, mounting and travel. Changed:
   the section "The real ratchet stay, pawl and knob linkage" gives the rod, guide, pawl, bellcrank, pull rod and spring
   with sizes, mounting points, travel (31 mm rod stroke, 6 mm and 4 mm knob thresholds, 2 mm pawl lift against 1.5 mm
   teeth, 20 mm knob stop), release and re-engage behaviour, and how to verify it; task.json setup and fidelity say the
   same.
3. No loophole was named by the reviewer. The existing cheats (nothing, left_only, right_only, slam, fast) still score 0.

