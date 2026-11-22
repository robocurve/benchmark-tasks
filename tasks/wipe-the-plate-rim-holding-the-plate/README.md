# Wipe the plate rim, holding the plate

A plated main course stands on the wet stainless-steel pass in the middle of the table: meat, mash and greens in the
middle of the plate, and three smudges of sauce on its rim (near side, right side and far-right). A folded, damp towel
pad lies on a small steel stand to the right. The instruction is only: "Clean up the plate's rim with the towel pad
before the plate goes out to the guest." Plating guides say to wipe the rim before a plate goes out (Culinary Arts
Switzerland: with a damp cloth, before serving; Chefitt: before it leaves the kitchen; Personal Touch Dining, a
caterer: every rim, before sending out, where one cook polishes, wiping off drips). Wet porcelain on a
wet steel pass slides when a towel is dragged across it, so one arm has to hold the plate still by its rim while the
other wipes, and the arranged food must stay untouched. The instruction states neither, so the task checks whether the
robot works both out unprompted:

- **Strictly bimanual: hold and wipe.** With the same wiping and nobody holding the plate (`no_hold`), the plate slid
  19.94 cm and turned 25.85 degrees under the towel, and the trial scored 0.0. Once the towel has touched
  the plate, the plate must stay within 1 cm and 5 degrees of where it was.
- **Pressure and coverage, not a pass over the rim.** A smudge is clean only after the towel has been rubbed across it
  with at least 0.8 N for 10 steps (0.5 s). A towel that only skims the rim (`light`) scores 0.0; leaving one smudge
  (`two`) scores 0.67.
- **A keep-out zone over the food.** A finger pad within 1 cm of the food, or the towel on it, scores 0 for the rest of
  the trial, even if the rim ends up clean.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes for yam_arena task files (visual-only parts for the sauce smudges and the food), and per-step readings: finger-pad points, part poses and the towel-plate contact force |
| `demo.py`, `democore.py` | The scripted full-score demo and its failure and cheating variants (`democore.py` is shared with our other kitchen tasks) |
| `demo_results.json` | Each variant's score and end-state measurements, from the runs reported below |
| `setup_shot.py`; `setup.png` (on the task board) | The starting layout, and the script that renders it from `task.py` |
| `demo.mp4` (on the task board) | The full demo (score 1.0): the scene camera above, the two wrist cameras below |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold or position no longer matches the text) |

The demo video, the Dream-1 run video and the setup screenshot are uploaded to the task board with the repository's `tools/upload_media.py`; `task.json` names them by media ID.

`kitchenkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. For this task it also reads the contact force
between the towel pad and the plate from the simulator each step (`kk.watch_contact("towel", "plate")` in `task.py`).

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clean up the plate's rim with the towel pad before the plate goes out to the guest." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/plate-rim-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The trial ends
when the task reports success.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Control rate:** the scorer measures the towel's speed over a 0.05 s step, so run at `control_hz=20`.

**Runtime:** the scripted demo succeeds at step 592 (29.6 s of simulated time at 20 Hz). A model trial runs up to
2,400 steps, plus the policy's own response time. Measured on Nov 15: the Dream-1 run (`dreamlabs/dream-1-beta`, 2,400 steps, `rows_per_request=12`) took 6.3 min of wall-clock time on one NVIDIA RTX PRO 6000 Blackwell GPU, simulator start-up included.

## Scripted demo

```bash
python demo.py task.py demo_out                          # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_no_hold --variant no_hold     # any variant below; --no-record skips the video
python demo.py task.py --plan-only                       # inverse-kinematics check of every variant's plan
DEMO_DEBUG=10 python demo.py task.py demo_out            # also print the wipe state every 10 steps
```

The left arm opens its gripper to about 2.8 cm, pinches the plate's lip on its left side and holds the plate. The
right arm picks up the towel pad, lowers it onto each smudge in turn (near, right, far-right), presses it and rubs it
back and forth along the rim: six strokes of 0.4 s, 2.5 cm to each side of the smudge (commanded; the soft wrist lags
it). While pressing and rubbing, it lowers or raises the towel until the towel-plate contact is about 1.2 N, and its
targets on the plate follow the plate's measured shift and turn. At the right smudge the grip is turned a quarter turn
about the vertical, so the fingers lie along the rim (closing across it, the inner finger came within 1 cm of the
food). The plan then puts the towel back on its stand and the left arm lets go, but the recorded run ends on success at
step 592, while the far-right smudge is being rubbed.

| Variant | What it does | Steps | Score |
|---|---|---|---|
| `full` | the left arm holds the plate by its lip; the right arm presses the towel on each smudge (near, right, far-right) and rubs it along the rim | 592 | **1.0** (success at step 592; rubbing steps near 29, right 33, far-right 21; the plate moved up to 1.64 cm from its start, and after the towel's first touch slid up to 0.3 cm and turned up to 0.96 degrees; food not touched) |
| `no_hold` | the right arm wipes the same way; nobody holds the plate | 848 | 0.0 (after the towel's first touch the plate slid up to 19.94 cm and turned up to 25.85 degrees, moving up to 19.93 cm from its start, over the 3 cm limit; the towel followed the plate, so all three smudges still came clean: rubbing steps near 30, right 20, far-right 19) |
| `light` | as `full`, but the towel only skims 2 mm above the rim band (no pressure) | 894 | 0.0 (no rubbing step on any smudge; the plate moved up to 1.69 cm from its start, and after the towel's first touch slid up to 0.37 cm and turned up to 0.37 degrees) |
| `two` | as `full`, but the far-right smudge is left | 726 | 0.67 (near and right clean, far-right never rubbed: rubbing steps near 29, right 33, far-right 0; the plate moved up to 1.72 cm from its start, and after the towel's first touch slid up to 0.37 cm and turned up to 0.88 degrees) |
| `food` | as `full`, but after the first (near) smudge the towel is dragged across the food | 970 | 0.0 (a right-arm finger pad came within 1 cm of the food box at step 330; the plate was also pushed up to 10.76 cm from its start, over the 3 cm limit, and turned up to 12.08 degrees) |

The numbers are from `demo_results.json`. The plate's movement is its largest distance from its start; its slide and
turn are the largest since the towel first touched it. Variants that do not succeed run their whole plan. The trial ends once the task's success has held for 1 s (yam_arena's default `success_hold_s`), so a cheat placed after the last smudge would never be scored; the `food` variant
therefore drags the towel across the food after the first smudge. In that run the plate was also pushed more than
3 cm, so the move limit alone would also have scored it 0.

## Scoring

Every step, from poses and the towel-plate contact force; the score at the end counts.

- **Rubbing:** a step rubs a smudge when the towel pad's centre is within 2 cm (horizontally, in the plate's frame) of
  the smudge's centre, the towel presses on the plate with at least 0.8 N, and it moves at least 1 cm/s relative to
  the plate. A smudge is clean after 10 rubbing steps (0.5 s) in total; they need not be consecutive.
- **1.0:** all three smudges clean, with the plate kept within 1 cm and 5 degrees of where it was when the towel first
  touched it (gripping the lip may nudge it before then), and none of the zero conditions below.
- **Partial:** one third per clean smudge (0.33, 0.67), capped at 0.5 if the plate slid more than 1 cm or turned more
  than 5 degrees once the towel touched it.
- **0:** from the moment the plate is more than 3 cm from its start, is lifted more than 2 cm, or anything comes near
  the food: a finger pad of either arm within 1 cm of the food box (7 x 7 cm on the plate's centre, 4 cm tall from the
  top of its base), or the towel pad overlapping it.

`report()` records the towel-plate contact force, the smudge under the towel and its speed, the rubbing steps per
smudge, the clean smudges, the plate's largest move, slide and turn, whether the food was touched (with the first step
and by what) and whether the plate was lifted.

## Real-world trials

Use a square porcelain plate about 24 cm across with a raised lip, on a stainless-steel shelf or tray wetted with
water, and a small folded damp towel. Put three small smudges of a thick, dark sauce on the rim at the near, right and
far-right positions, and real food or a stand-in in the middle. Before the trials, check that the plate slides when its
rim is wiped with nobody holding it, as it does in simulation. The real sauce replaces the rubbing count: a smudge is
clean when no sauce is left on the rim, judged from an overhead camera. Track the plate with a marker on its rim from
the same camera (1 cm and 5 degrees from where it was when the towel first touched it; 3 cm from its start), and score
0 if a gripper or the towel touches the food. No force sensor is needed.

## Context and simulation fidelity in full

`task.json` keeps condensed versions of these two fields (the task board limits their length). The full text:

**Simulation fidelity.** Rigid bodies: boxes, so the plate is square and the towel pad is a rigid block; the towel-plate contact force is read from the simulator each step. Liquids are not simulated. The wet pass and the damp towel are friction settings (0.02 on the steel, 0.10 on the plate's base, 0.8 on the towel pad), so a towel pressed and dragged across the rim pushes an unheld plate along the pass. The sauce does not wipe off: the smudges are visual and stay in place, so the scorer stands in for cleaning by counting the steps in which the towel rubs over each smudge with at least 0.8 N while moving relative to the plate. The food has no collision either: the scorer stands in with a keep-out box over the middle of the plate that no finger pad may come within 1 cm of and the towel may not overlap.
