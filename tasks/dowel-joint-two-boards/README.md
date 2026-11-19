# Join two boards on three glued dowels, pressing them together evenly

The robot brings two loose boards together end to end so that three dowels glued into one board's end enter three
sockets in the other board's end at the same time, presses the joint closed and lets go. The pale birch board carries
the dowels, the dark walnut board carries the sockets, and each board lies beside its own arm. Carpenters and
cabinetmakers (about 900,000 US carpenters, BLS) join face frames, rails and furniture parts this way every day.

The ability is seating several pins into several sockets at once while the two members stay square. The sockets are
round, one 8.4 mm bore per 8.0 mm dowel in the simulation (0.2 mm of clearance a side over 20 mm of engagement; see
"Building the real trial" for the real fit), so a joint pressed with one end first twists, the dowels bind and the
seam stays open. Nothing latches and nothing is placed on a mark: the dowels are the only guide.

Physics, not the instruction, forces both arms. Each arm covers only its own half of the bench with a board in hand:
the left arm cannot take its gripper past y -0.075 at x 0.36 with the gripper pointing down (inverse kinematics error
reaches 10 mm just beyond that and 20 mm at y -0.13), and carrying the dark board to the pale board needs y -0.135, so the dowel tips never reach
the sockets. The right arm is the mirror image. A board that is only pushed along the waxed bench is shoved away
rather than mated, and neither board is clamped, jigged or stopped. The `left_only` and `right_only` runs below show it.

Instruction given to the policy (23 words; it states the goal only, never the technique or the tolerance):

`Join the pale board to the dark board on the three dowels so their ends meet flat on the bench, then let go.`

Files in this folder: `task.json`, `task.py` (scene, final-state record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Revision 3 (reviewer feedback on the second review, point by point)

| Reviewer point | What changed |
|---|---|
| The real trial used loose 9 mm square mortises, not snug round dowel sockets | Real trial: 8.0 mm beech dowels in 8.0 mm brad-point drilled round holes 22 mm deep, a dry hand-pressed fit (hole 8.0 to 8.1 mm, so 0 to 0.1 mm clearance across), checked with 8.1 mm go and 8.3 mm no-go pin gauges. Simulation: each square socket is replaced by a round 16-sided ring of thin boxes around an 8.4 mm bore (the old square was 9 mm); the dowel tip lead-in and the 22 mm depth are unchanged, so the insertion and the alignment skill stay. The simulated bore is still 0.15 to 0.2 mm a side looser than the real one, stated in the fidelity note. |
| The tip and half-in rungs had no lower seam-gap or relative-height bound | Every rung (tip, half-in, seated) now needs both seam gaps at least -2 mm (overlapping boards fail), the two boards within 2 mm of the same height (a stacked board fails), both flat within 3 deg, edges flush within 1 mm and yaw within 2 deg. `measure()` now returns the height difference `dz`. New tests cover stacked and overlapping states on every rung. |
| The text said final state only but the code kept earlier credit | The history flags `tips_in` and `half` are gone. The score is the rung of the last measurement plus the current 1 s release run, so a state reached earlier and then undone earns nothing. No damage condition is scored (nothing in the scene can break), and the written scoring text says so. A test seats the joint, then pulls it apart, and checks the score follows the final state. |

Demo: the demo script is rewritten for the round sockets (an extra alignment pass 4 mm before the dowel tips reach
the sockets, a slower final approach, tighter servo tolerances). The demo and cheat results quoted below were measured
on the previous revision (9 mm square sockets, run-history scoring). They have not been rerun on this scene; this
revision's own runs are recorded in the logs the filing step attaches.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 3fa6450fe99c (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in
  the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn each board as a compound rigid body (box and cylinder colliders) and calls the task's
per-step record after every control step. No fork of yam_arena and no third-party assets.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

29 tests feed synthetic seam measurements to the scorer: a demo-shaped trace scores 1.0; nothing moved scores 0; a
board carried across and stopped 30 mm short scores 0; the rung limits (tips in at 19.0 mm and not 19.1, half in at
10.0 and not 10.1, seated at 0.5 mm and not 0.6, edges flush at 1.0 mm and not 1.1, yaw 2.0 deg and not 2.1); the
lower seam-gap bound at -2.0 mm and not -2.1 on every rung, with overlapping boards (-3 to -150 mm, or one corner
overlapped) scoring 0; stacked boards (18 mm of relative height, at gaps from -50 to 19 mm) scoring 0 on every rung, and
the 2.0 mm height bound on the tip, half-in and seated rungs; flat, square and flush bounds on the partial rungs; a
state reached earlier and then undone (tips in then pulled out, seated then pulled apart) earning nothing; a
wedge that is backed out and pressed on square still scores 1.0 (only the end state counts), while a joint left
wedged scores under 0.75; a 3 mm sideways skew never reaches the first rung; the release gate at 30 mm and not 29.9, a 0.9 s release is
not enough while 1.05 s is, a hand above 10 mm/s or a board above 5 mm/s blocks it; a board tilted 3.5 deg is not
seated; the record is clean after a reset; and measure() on real board poses gives the 510 mm start gap, 0 at the
seat, the corner-gap difference that 3 deg of yaw makes (80 mm x sin 3 deg), the sideways offset, the height
difference of stacked boards and the negative gaps of overlapping boards. These tests were run offline against a stub of
the simulator import; they do not exercise the round-socket physics.

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
inspect-robots inspect dream_log.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' dream_log.json
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
3. Both advance toward the seam over 7 s to the point 4 mm short of the dowel tips (centres 162 mm apart, seam gap
   24 mm), stop for another servo pass that nulls the sideways offset and yaw seen from the board poses, then advance
   the last 12 mm over 9 s, the dark board tracking the pale board's seen pose every control step, and both arms press
   1.5 mm past the seat. With the old square sockets the tips entered at about 21 s; the round 8.4 mm sockets leave
   0.2 mm of clearance a side, so the extra pass and the slower approach were added and are untested.
4. Right waits until left has seen the seam closed, both open their grippers and rise 70 mm, and the demo waits.

Result of the previous revision (9 mm square sockets, before the changes above; not rerun on this scene): the demo
scored 1.0 (`success`, episode ended at step 630 of 1,200) with the corner gaps 0.0 mm and 0.0 mm, the edges 0.2 mm
out of flush, yaw 0.00 deg, both boards flat, both grippers 53 mm clear. Worst IK error 0.1 mm and 0.01 deg. The round
sockets add about 4 s of demo time (about step 710 expected, inside the 1,200-step limit).

Variants (`DEMO_VARIANT`). The results in the table were measured on the previous revision's scene (square sockets);
the scorer now also rejects any state outside the rung bounds, so each variant is expected to still score 0 on this
revision, but that has not been run:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the seam ends stay 510 mm apart. | 0.0 |
| `left_only` | The right arm idles at home; the left arm carries the dark board across the bench toward the pale board and presses. | IK error 49 mm: the board stops with a 34.5 mm seam gap and 12.6 mm sideways, the dowel tips 14 mm short of the sockets. | 0.0 |
| `right_only` | The mirror image: the left arm idles; the right arm carries the pale board across. | IK error 49.5 mm: 34 mm seam gap, 12.4 mm sideways, the tips outside the sockets. | 0.0 |
| `one_end_first` | Both arms, but the dark board is turned 4 deg against the pale board as they close. | The two corner gaps stop at 19.5 and 24.5 mm: the near dowel is 0.5 mm into its socket and binds, the seam never closes, so the end state is not seated. | 0.0 |
| `skew_3mm` | Both arms, with the dark board 3 mm to one side. | The dowel tips stop on the sockets' end face (20 mm gap) with the boards 3.1 mm out of line. | 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Only the final state counts. Every control step the record (`Record` in `task.py`, a `linkkit.History`) measures, from
the two board poses: the seam gap at each corner of the seam end (along the pale board's long axis, 0 when the ends
touch, negative when they overlap), the height difference of the two boards, the sideways offset of the dark board's
centre line, the yaw between the boards, and each board's tilt from flat; and from the simulator each gripper's pad
clearance to both boards and its speed, and the boards' speed. The rung is read from the last measurement and from how
long the release condition has held without a break up to the end. Nothing earlier earns credit: a state reached and
then undone (tips in then pulled out, seated then pulled apart) scores what the end state scores, and a wrong first
try that was backed out costs nothing. No damage condition is scored, because nothing in the scene can break. A wedged
joint never reaches the seated rung, since the corner gaps cannot close while a dowel binds (a geometric estimate for
the simulated 0.4 mm of diametral clearance over 20 mm of engagement is about 1.1 deg of yaw).

Bounds shared by every rung below the top one: both seam gaps at least -2 mm (boards that overlap cannot score), the
two boards within 2 mm of the same height (a board stacked on the other cannot score), both boards within 3 deg of flat.

- 0.25: at the end both corner gaps are between -2 mm and 19 mm (the 20 mm dowel tips are in), the offset at most 1 mm
  and the yaw at most 2 deg.
- 0.5: at the end both corner gaps are between -2 mm and 10 mm (more than half in), offset and yaw as above, but the
  joint is not seated.
- 0.75: seated at the end: both corner gaps between -2 mm and 0.5 mm, edges flush within 1 mm, the boards within 2 mm
  of the same height and within 3 deg of flat.
- 1.0: seated, and every pad point of both grippers at least 30 mm from both boards and both grippers moving under 10
  mm/s for the last 1 s, with the boards still (under 5 mm/s).

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 minus the score. Real trial: a feeler gauge at the two seam corners
(0.5 mm and the 19 mm tip depth on a depth rod), a try square or straightedge along the long edge for the 1 mm
offset, a straightedge laid across both boards' top faces with a 2 mm feeler gauge for the height bound (a stacked or
overlapped board shows as 18 mm), a protractor for the 3 deg tilt, a tape for the 30 mm clearance, and a 60 fps overhead video over a 1 mm grid
for the corner-gap difference in every frame once a tip is in, and for stillness.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The previous revision's demo succeeded at step 630.
- Runtime on one L40S: a demo run took about 7 min wall clock with other simulations on the box, including about
  2 min of Isaac Sim start-up. A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout. The scripted demo is
  deterministic; Dream-1 is a hosted model without a seed.

## Scored runs

| Run | inspect-robots-dream | yam-arena commit | task.py sha256 (12) | Steps | Wall time | Log | task_score | Reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 | 0.1.0 | 0059d02a1a91 | 3fa6450fe99c | not run | not run min | dream_log.json | not run | not run |
| Scripted demo, previous revision (`linkdemo.py`, square sockets, not this task.py) | - | - | - | 630 (success held 1 s) | about 7 min | not attached | 1.0 | both boards slid in, dowels entered together, seated, edges flush, both grippers 53 mm clear |

- Demo: `python linkdemo.py task.py demo_out` writes `demo_out/result.json` with the demo's `task_score`. The video is
  `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot is `demo_out/setup.png` (frame 0
  of the top camera).
Dream-1 run: not run on this revision. The run box ran out of compute credits before the revised task.py could be trialled, so no Dream-1 score is claimed for it.

## Build log (the last demo run recorded)

| Run | Change | Result |
|---|---|---|
| 1 | first build: boards at x 0.25, y +-0.245, gripped at their centres | 0.0: the left wrist hit its roll limit (20 deg IK error) and the dark board ended 9.6 deg off |
| 2 | the left arm grips with the pads closing the other way round | 0.0: the two wrist housings met at 175 mm tool spacing and the boards were shoved apart at a 25 mm gap |
| 3 | grip 45 mm from the board centre toward its outer end | 1.0 at step 630, but a single arm could still carry its board nearly to the sockets |
| 4 | boards moved to x 0.34, y +-0.31 | 1.0 at step 630; a one-armed carry still reached the tips |
| 5 | boards moved to x 0.36, y +-0.33 (each arm's reach ends 135 mm short of the seat); the scorer no longer asks both boards to travel (goal only) | 1.0 at step 630; `left_only` and `right_only` stop 34 mm short; all five variants 0.0 |
| 6 | the permanent wedge gate removed (only the end state is scored); real-trial build added | 1.0 at step 630; all five variants and `nothing` 0.0 |
| 7 | revision 3: round 8.4 mm sockets, bounds on every rung, history credit removed, demo given a tip-side alignment pass | not run on this scene when this text was written; the results of runs 1 to 6 are from earlier revisions |

## Building the real trial

- Boards: 18 x 80 x 150 mm birch and walnut, planed square (squareness within 0.2 mm, checked with a try square).
- Dowels: three 8.0 mm beech dowels (fluted, the standard 8 mm size), 40 mm long with the free end chamfered 0.6 mm,
  pushed 20 mm into three 8.0 mm round holes drilled in the pale board's end (a dry hand-pressed fit, so no glue is
  used in the trial), leaving 20 mm out; pitch 25 mm and height 9 mm above the bench, checked with a drilling jig and
  calipers.
- Sockets: three round dowel sockets, 22 mm deep, in the dark board's end at the same pitch and height, drilled with
  an 8.0 mm brad-point bit on a drill-press depth stop (the usual dowel joint: the hole measures 8.0 to 8.1 mm, so the
  dowel is a snug slip fit with 0 to 0.1 mm of clearance across, entered by light hand pressure). Check each hole with
  an 8.1 mm pin gauge (must enter the full 22 mm) and an 8.3 mm one (must not).
- Simulation versus real: the simulated socket is a round bore 8.4 mm across, so 0.2 mm a side against 0 to 0.05 mm
  a side in the real wood. The simulator keeps 1 mm contact offsets and a 0.1 mm closed-loop alignment, so a bore
  tighter than this would jam in every run; the real press-fit friction of a few N is not modelled either (bench and
  bore friction are 0.25). The alignment skill, the 20 mm engagement and the binding under yaw are preserved.
- Bench: a waxed laminate top. Check the friction by pulling a board with a spring scale on the bench (the 123 g board
  moves at about 0.3 N, friction 0.25).
- Nothing clamps or stops either board; they sit on taped outlines 0.51 m apart, the real-world equivalent of a
  workshop with parts staged at the two ends of a long bench and a worker holding one in each hand.

## Materials

- Pale board: birch, 18 x 80 x 150 mm, 123 g, tan (0.87, 0.74, 0.52); bench friction 0.25. Three beech dowels, 8 mm
  diameter, 20 mm long with a 0.6 mm stepped tip, glued 20 mm out of its +y end at 25 mm pitch, 9 mm above the bench.
- Dark board: walnut, 18 x 80 x 150 mm, 140 g, (0.36, 0.24, 0.16); three round dowel sockets 22 mm deep in its -y
  end at the same pitch and height. Each is a ring of 16 thin boxes (3 mm radial, 22 mm deep) around an 8.4 mm bore
  inside a 15 mm square cut-out, with 4 solid columns and 6 floor and roof boxes around the cut-outs (0.2 mm
  clearance a side around the 8 mm dowel).
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

## Run logs

This revision (task.py SHA-256 prefix 3fa6450fe99c) was not run in the simulator: the run box ran out of compute credits before the revised scene could be run. Any demo or cheat result described above comes from the previous revision unless its line says otherwise. The scripted demo is deterministic, and `python linkdemo.py task.py demo_out` runs it on the revised scene. The videos linked from task.json show the previous revision.
