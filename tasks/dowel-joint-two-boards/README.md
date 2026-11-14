# Join two boards on three glued dowels, pressing them together evenly

The robot brings two loose boards together end to end so that three dowels glued into one board's end enter three
sockets in the other board's end at the same time, presses the joint closed and lets go. The pale birch board carries
the dowels, the dark walnut board carries the sockets, and each board lies beside its own arm. Carpenters and
cabinetmakers (about 900,000 US carpenters, BLS) join face frames, rails and furniture parts this way every day.

The ability is seating several pins into several sockets at once while the two members stay square. The sockets give
0.5 mm of clearance a side over 20 mm of engagement, so a joint pressed with one end first twists, the dowels bind and
the seam stays open. Nothing latches and nothing is placed on a mark: the dowels are the only guide.

Physics, not the instruction, forces both arms. Each arm covers only its own half of the bench with a board in hand:
the left arm cannot take its gripper past y -0.075 at x 0.36 with the gripper pointing down (inverse kinematics error
reaches 10 mm just beyond that and 20 mm at y -0.13), and carrying the dark board to the pale board needs y -0.135, so the dowel tips never reach
the sockets. The right arm is the mirror image. A board that is only pushed along the waxed bench is shoved away
rather than mated, and neither board is clamped, jigged or stopped. The `left_only` and `right_only` runs below show it.

Instruction given to the policy (23 words; it states the goal only, never the technique or the tolerance):

`Join the pale board to the dark board on the three dowels so their ends meet flat on the bench, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 6da3395c3502 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in
  the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn each board as a compound rigid body (box and cylinder colliders) and calls the task's
per-step record after every control step. No fork of yam_arena and no third-party assets.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

19 tests feed synthetic seam measurements to the scorer: a demo-shaped trace scores 1.0; nothing moved scores 0; a
board carried across and stopped 30 mm short scores 0; the rung limits (tips in at 19.0 mm and not 19.1, half in at
10.0 and not 10.1, seated at 0.5 mm and not 0.6, edges flush at 1.0 mm and not 1.1, yaw 2.0 deg and not 2.1); a
wedge that is backed out and pressed on square still scores 1.0 (only the end state counts), while a joint left
wedged scores under 0.75; a 3 mm sideways skew never reaches the first rung; the release gate at 30 mm and not 29.9, a 0.9 s release is
not enough while 1.05 s is, a hand above 10 mm/s or a board above 5 mm/s blocks it; a board tilted 3.5 deg is not
seated; the record is clean after a reset; and measure() on real board poses gives the 510 mm start gap, 0 at the
seat, the corner-gap difference that 3 deg of yaw makes (80 mm x sin 3 deg) and the sideways offset.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Join the pale board to the dark board on the three dowels so their ends meet flat on the bench, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/dowel-joint-two-boards-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). For repeat trials add `--epochs K`; with `object_jitter=0` the seed changes nothing in the
scene.

```bash
inspect-robots inspect adhoc_0098d7f4.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_0098d7f4.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant left_only      # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the two boards' poses and the robot's own grippers.

1. Both arms pick: top-down on the board's two long edges, 45 mm from the board's centre toward its outer end (the
   wrists need room at the seam), pads closing along x, 1.5 mm squeeze. The grasp transform is measured once.
2. Both slide their boards along the bench to 100 mm from the seam line (centres 200 mm apart, seam faces 50 mm
   apart), the left arm placing the dark board against the pale board's seen pose. Three or four servo passes null
   the sideways offset and yaw seen from the board poses (0.2 mm and 0.1 deg are the targets; the arms sag a few mm).
3. Both advance 25 mm toward the seam over 10 s, the dark board tracking the pale board's seen pose every control
   step. The dowel tips enter the sockets at about 21 s, and both arms press 1.5 mm past the seat.
4. Right waits until left has seen the seam closed, both open their grippers and rise 70 mm, and the demo waits.

The demo scores 1.0 (`success`, episode ended at step 630 of 1,200): the corner gaps were 0.0 mm and 0.0 mm, the
edges 0.2 mm out of flush, yaw 0.00 deg, both boards flat, both grippers 53 mm clear, and the corner gaps never
differed by more than 0.1 mm. Worst IK error 0.1 mm and 0.01 deg. After a second reset both boards are back within
0.07 mm and at rest, and the history was cleared.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (6da3395c3502); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the seam ends stay 510 mm apart. | 0.0 |
| `left_only` | The right arm idles at home; the left arm carries the dark board across the bench toward the pale board and presses. | IK error 49 mm: the board stops with a 34.5 mm seam gap and 12.6 mm sideways, the dowel tips 14 mm short of the sockets. | 0.0 |
| `right_only` | The mirror image: the left arm idles; the right arm carries the pale board across. | IK error 49.5 mm: 34 mm seam gap, 12.4 mm sideways, the tips outside the sockets. | 0.0 |
| `one_end_first` | Both arms, but the dark board is turned 4 deg against the pale board as they close. | The two corner gaps stop at 19.5 and 24.5 mm: the near dowel is 0.5 mm into its socket and binds, the seam never closes, so the end state is not seated. | 0.0 |
| `skew_3mm` | Both arms, with the dark board 3 mm to one side. | The dowel tips stop on the sockets' end face (20 mm gap) with the boards 3.1 mm out of line. | 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset the record (`Record` in `task.py`, a `linkkit.History`) measures, from the two board
poses: the seam gap at each corner of the seam end (along the pale board's long axis, 0 when the ends touch), the
sideways offset of the dark board's centre line, the yaw between the boards, and each board's tilt from flat; and from
the simulator each gripper's pad clearance to both boards and its speed, and the boards' speed. It keeps flags, peaks
and held times for the whole episode and is cleared only by the simulator reset, so pulling the boards apart and
starting again is allowed: there is no permanent penalty for a wrong first try that was undone, and a wedged joint
simply never reaches the seated rungs, since the corner gaps cannot close while a dowel binds (2.9 deg of yaw binds
the 0.5 mm socket clearance).

- 0.25: at some time both corner gaps were at most 19 mm (the 20 mm dowel tips are in), the offset at most 1 mm and
  the yaw at most 2 deg.
- 0.5: after that, at the end both corner gaps are at most 10 mm (more than half in) but the joint is not seated.
- 0.75: seated at the end: both corner gaps at most 0.5 mm, edges flush within 1 mm, both boards within 3 deg of flat.
- 1.0: seated, and every pad point of both grippers at least 30 mm from both boards and both grippers moving under 10
  mm/s for the last 1 s, with the boards still (under 5 mm/s).

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 minus the score. Real trial: a feeler gauge at the two seam corners
(0.5 mm and the 19 mm tip depth on a depth rod), a try square or straightedge along the long edge for the 1 mm
offset, a protractor for the 3 deg tilt, a tape for the 30 mm clearance, and a 60 fps overhead video over a 1 mm grid
for the corner-gap difference in every frame once a tip is in, and for stillness.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 630.
- Runtime on one L40S: a demo run took about 7 min wall clock with other simulations on the box, including about
  2 min of Isaac Sim start-up. A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout. The scripted demo is
  deterministic; Dream-1 is a hosted model without a seed.

## Scored runs

| Run | inspect-robots-dream | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 0.1.0 | 0059d02a1a91 | 6da3395c3502 | 1200 | 3 min | adhoc_0098d7f4.json | 0.0 | max_steps |
| Scripted demo, recorded (`linkdemo.py`) | - | 0059d02a1a91 | 6da3395c3502 | 630 (success held 1 s) | about 7 min | demo_out/result.json | 1.0 | both boards slid in, dowels entered together, seated, edges flush, both grippers 53 mm clear |

- Demo: `python linkdemo.py task.py demo_out` writes `demo_out/result.json` with `task_score = 1.0`. The video is
  `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot is `demo_out/setup.png` (frame 0
  of the top camera).
- Dream-1 run: 0.0 (max_steps) in 1200 steps and 3 min, log adhoc_0098d7f4.json, on this exact `task.py`.

## Build log (the last demo run recorded)

| Run | Change | Result |
|---|---|---|
| 1 | first build: boards at x 0.25, y +-0.245, gripped at their centres | 0.0: the left wrist hit its roll limit (20 deg IK error) and the dark board ended 9.6 deg off |
| 2 | the left arm grips with the pads closing the other way round | 0.0: the two wrist housings met at 175 mm tool spacing and the boards were shoved apart at a 25 mm gap |
| 3 | grip 45 mm from the board centre toward its outer end | 1.0 at step 630, but a single arm could still carry its board nearly to the sockets |
| 4 | boards moved to x 0.34, y +-0.31 | 1.0 at step 630; a one-armed carry still reached the tips |
| 5 | boards moved to x 0.36, y +-0.33 (each arm's reach ends 135 mm short of the seat); the scorer no longer asks both boards to travel (goal only) | 1.0 at step 630; `left_only` and `right_only` stop 34 mm short; all five variants 0.0 |
| 6 | the permanent wedge gate removed (only the end state is scored); real-trial build added | 1.0 at step 630; all five variants and `nothing` 0.0 |

## Building the real trial

- Boards: 18 x 80 x 150 mm birch and walnut, planed square (squareness within 0.2 mm, checked with a try square).
- Dowels: three 8.0 mm beech dowels, 20 mm long with a 0.6 mm chamfered tip, pushed 20 mm into three 8 mm round
  holes drilled in the pale board's end (a dry press fit, so no glue is used in the trial); pitch 25 mm and height
  9 mm above the bench, checked with a drilling jig and calipers.
- Sockets: three square mortises 9.0 x 9.0 mm, 22 mm deep, in the dark board's end at the same pitch and height,
  cut with a 9 mm square-hole mortise chisel or routed with a 9 mm bit and squared. Check each with a 9.0 mm square
  gauge block (it must enter 22 mm) and a 9.2 mm one (it must not), so the clearance is 0.5 mm a side around the 8 mm
  dowel.
- Bench: a waxed laminate top. Check the friction by pulling a board with a spring scale on the bench (the 123 g board
  moves at about 0.3 N, friction 0.25).
- Nothing clamps or stops either board; they sit on taped outlines 0.51 m apart, the real-world equivalent of a
  workshop with parts staged at the two ends of a long bench and a worker holding one in each hand.

## Materials

- Pale board: birch, 18 x 80 x 150 mm, 123 g, tan (0.87, 0.74, 0.52); bench friction 0.25. Three beech dowels, 8 mm
  diameter, 20 mm long with a 0.6 mm stepped tip, glued 20 mm out of its +y end at 25 mm pitch, 9 mm above the bench.
- Dark board: walnut, 18 x 80 x 150 mm, 140 g, (0.36, 0.24, 0.16); three 9 mm square sockets 22 mm deep in its -y
  end at the same pitch and height, built from 4 solid columns and 6 floor and roof boxes (0.5 mm clearance a side).
- Bench: waxed laminate, the station table at z 0.7475; nothing clamps, jigs or stops a board.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Pull the boards apart and wipe the sockets and dowels clean (dry fit, no glue). Set the pale board
with its dowel end toward the middle and the dark board with its socket end toward the middle, centres at (0.36, -0.33)
and (0.36, +0.33), edges square to the x axis on taped outlines, 0.51 m between the seam ends.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
