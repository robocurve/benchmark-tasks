# Seat a robot controller's add-on board on its two rows of header pins without bending any pin

The robot presses an add-on board (a shield with two rows of 20 pins, 6 mm long) home onto a controller's header. The
board starts level on the pin tips, set down by hand on the header: its underside is 6.0 mm above the header top,
exactly the pin length, so the tips are the only support and nothing props it. The controller is a green board on an
aluminium carrier plate with a black 2 x 20 female header 7.62 mm wide (the two pin rows 5.08 mm apart plus 1.27 mm of
plastic each side); the add-on board is blue with a 25 mm aluminium heat sink on top. At the end the board is fully
seated (underside within 0.3 mm of the header top), no pin has been bent, and both grippers have let go. Electronics
assemblers and robotics technicians do this whenever they build or repair a robot controller stack.

The ability is pressing a thin part home evenly. The mating force is small (3.3 N, 0.08 N per pin) but the pins enter
cleanly only when the board stays within 1.5 deg lengthwise and 4 deg across as the tips pass 1.5 mm into the sockets.
Pressed off-centre, the pins on the loaded side enter first, the board rocks, and the leading pins bend. The board is a
thin sheet with nothing to push on except the heat sink, so the robot must press through the heat sink's centre. If the
robot lifts the board clear and lets go, it falls back onto the pin tips: nothing holds it in the air.

Instruction given to the policy (21 words; it states the goal only, never the technique):

`Press the blue add-on board fully onto the controller's header pins without bending any pin, then let go with both grippers.`

Honest limits. Only one arm has a role; either arm can do the press (the `left_only` and `right_only` variants both
score 1.0, they are the same press by the two arms). The pins and sockets are not collision-modelled: the board rides a
vertical slide and two tilt hinges, with no brake in the air (above the header it is free and falls onto the tips; on
the tips its weight restores it flat; on or in the sockets the pins' grip is a spring latch that holds it where it was
pushed, and a stiff spring resists rocking). The bend is a rule on tilt at 1.5 mm into the sockets. Lateral play on the
tips is not modelled. See `task.json`, fidelity.

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`. `linkkit.py` and `linkdemo.py` are the same kit files as the
program's other linkage tasks, unchanged.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo and
cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: f22242315a3a (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision`
  in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

17 tests feed synthetic records to the scorer (they import `task.py`, so run them in the simulator's Python
environment): a demo-shaped trace scores 1.0; nothing scores 0; lifting the board off the header and letting go scores
0; lengthwise tilt 1.5 deg is no gate and 1.6 deg (either sign) is; across 4.0 deg is no gate and 4.1 is; tilt while the
pins are only on the tips is no gate; a bend beats a clean seated end; 0.3 mm high is seated and 0.31 is not; pressed
and pulled back out is 0.5; a seated board with a hand on it scores 0.75; the release gate is 30 mm and 1.0 s, and a
hand over 5 mm/s, a board over 3 mm/s or 3 deg/s, or a late movement keeps 0.75; the record is clean after a reset.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Press the blue add-on board fully onto the controller's header pins without bending any pin, then let go with both grippers." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/header-pins-align-press-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Ladder: 0 = a pin bent at any time, or never pressed in; 0.5 = pressed within 2 mm of the seat,
unbent; 0.75 = seated at the end; 1.0 = seated, still 1 s, and both grippers 30 mm clear and slower than 5 mm/s for 1 s.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant hover_release  # a cheat variant (or DEMO_VARIANT=hover_release)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes the camera videos, `setup.png` and
`result.json`. The script reads only what a camera could measure (the board's pose and the robot's own grippers).

1. The right arm reads the board's pose, puts closed fingertips on the heat sink's centre and presses straight down
   (commanded 45 mm past the top; the arm sags, so the force is limited by its tracking).
2. The board goes in evenly (seat height 0.07 mm seen, tilt under 0.2 deg); the arm opens and lifts away.

The demo scores 1.0 (`success`, episode ended at step 249 of 1,200): no pin bent, seated, both grippers clear. Worst IK
error 18 mm and 5 deg (the arm sags under the press).

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (f22242315a3a):

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | The board stays on the pin tips. | 0.0 |
| `hover_release` | Pinches the heat sink, lifts the board 25 mm clear of the header, opens (the old loophole: a released board held above the pins). | The board falls onto the pin tips and rests there, 6 mm up; never pressed in. | 0.0 |
| `end_press` | Closed fingertips on the board's end, 27 mm off centre. | The board stalls at 5.6 mm and is never seated. | 0.0 |
| `left_only` / `right_only` | The same centre press by one arm only. | Either arm seats the board. This is not a cheat: the task has one working arm. | 1.0 |

## Build log (key runs, shared L40S)

| Run | Change | Result |
|---|---|---|
| 1 | tilt hinges free, no brakes | the board tipped over at reset (the hinge-link masses sat off the axis) |
| 2 | masses centred, support links moved beside the board | tilt stable; pressed only to 4.5 mm (arm force) |
| 3 | commanded press depth 45 mm | 1.0 |
| 4 | a latch where the pins grip, advancing on any push | demo 1.0, but gravity ratcheted the board in (`hover_release` scored 1.0) |
| 5 | latch needs 1 mm of push to advance, 3.3 N | demo 1.0, `hover_release` 0.0, `end_press` 0.0 |

## Revision

Review of PR #1891 (TK-V6HHU5), three points not met. What changed:

1. Construct validity (board released 9.4 mm above a header whose pins are 6 mm): the board now starts with its underside
   6.0 mm above the header top, exactly the pin length, resting on the pin tips. No spacer, no prop.
2. Clarity and reproducibility (reset not reproducible; header too narrow): the header is now 7.62 mm wide, so both pin
   rows (5.08 mm apart) fit with 1.27 mm of plastic each side (it was 5.08 mm, no wider than the row spacing). The start
   state (level, tips on the header top) and its reset (set down by hand, 6 mm gauge block laid beside it then removed,
   protractor check) are written out in `task.json`.
3. Feasibility (simulation brakes held a released board): there is no brake in the air any more. Above the header the
   slide and both tilt hinges are free and gravity drops the board onto the tips; on the tips its weight restores it
   flat; the pins' grip is a latch that acts only once the tips are on the header (3.3 N, 0.08 N per pin). A board that
   is lifted clear and released falls back; it is not suspended and its tilt is not locked. The cocked start and the
   "levelled and held" 0.25 rung were removed with the tilt prop, since a board cannot rest cocked on its pin tips.
4. Loophole cheats added: `hover_release` (lift the board clear and let go, the state the old brakes kept alive) scores
   0; `end_press` (off-centre press) scores 0.
5. The tests, `task.json` (setup, reset, scoring, fidelity) and this README were updated to match.
6. Not changed: only one arm has a role, so `left_only` and `right_only` both score 1.0.

## Dream-1 trial

The Dream-1 run is not done yet. The table below is filled in from the run.

| Run | inspect-robots | yam-arena | task SHA | steps | minutes | log | task_score | reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 (`--policy dream`, command above) | 1.0.0 | 0059d02a1a91 | f22242315a3a | not run | not run | dream_log.json | not run | not run |

```bash
inspect-robots inspect dream_log.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' dream_log.json
```

## Run logs

The demo and cheat results above were run in Isaac Sim on the program's run box with this task.py (SHA-256 prefix f22242315a3a). Their logs stayed on that box, which ran out of compute credits before they could be copied, so no log is committed with this revision. The scripted demo is deterministic: `python linkdemo.py task.py demo_out` reproduces it. The videos linked from task.json show the previous revision.
