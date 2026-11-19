# Transplant seedling plugs into the empty cells of a part-planted cell strip

> Transplant the seedling plugs into the empty cells of the strip, one plug per cell.

Transplanting, lifting seedling plugs out of a propagation flat and setting each into its own cell of a cell pack or
strip, is a greenhouse grower's core operation. Here a six-cell strip runs across the table between the arms and is
already part-planted: cells 2 and 5 hold plugs. Four new plugs stand in two seedling flats, one in front of each arm.
Each new plug goes into its own empty cell. A plug set into an occupied cell wastes both seedlings and does not
count; knocking out a plant that is already in scores zero.

**Reliability stressor (one fixed level): partial state.** Which cells are free must be read from the scene; filling
cells in a fixed pattern puts plugs on top of the original plants. Each arm reaches only its own half of the strip,
so both arms work.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/transplant-plugs-into-cell-strip/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Transplant the seedling plugs into the empty cells of the strip, one plug per cell." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/plug-dream
```

The scripted full-score demonstration (both arms, open loop): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/plug-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (gripper clearance, both arms' inverse kinematics, and the scoring on a
kinematic replay plus negative controls): `python tasks/transplant-plugs-into-cell-strip/check_task.py`.

Diagnostics: set `PLUG_TRACE=/path/trace.jsonl` before a run to log, every control step, all plugs' positions and
both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 400 control steps (20 s at 20 Hz); a Dream-1 trial runs until the
transplanting has held for 1 s or the run limit of 2,400 steps (2 minutes). No seeds: `-E object_jitter=0` keeps every
object exactly where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Cell strip** (`strip_floor`, `strip_wall_*`, `strip_divider_*`): black plastic, one row of six 52 mm square cells,
  3 cm deep, 6 mm walls (35.4 x 6.4 cm). It runs left to right, centred 27 cm forward on the centre line. Cell
  centres are 5.8 cm apart: 14.5, 8.7 and 2.9 cm left, then 2.9, 8.7 and 14.5 cm right. Fixed to the table.
- **Original plants** (`planted_2`, `planted_5`): plugs standing on the floors of cells 2 and 5 counting from the left.
- **Seedling plugs** (all six): upright cylinders of growing medium, 36 mm diameter, 55 mm tall, 40 g, dark brown.
  Standing in a cell, a plug's top is 2.5 cm above the cell walls.
- **Seedling flats** (`flat_left_*`, `flat_right_*`): green plastic, inside 24 x 9 cm, a 1.5 cm lip around a 1 cm
  floor, centred 20 cm forward and 30 cm left or right (in front of each arm). Fixed to the table.
- **New plugs** (`plug_1` to `plug_4`): `plug_1` and `plug_2` in the left flat at 15 and 25 cm forward; `plug_3` and
  `plug_4` in the right flat at 15 and 25 cm forward.

Nothing else is on the table.

## Scoring

A plug is **in** a cell when its centre lies inside that cell's 5.8 cm square seen from above and is less than 3 cm
above where a standing plug's centre would be (a plug carried over a cell is not in it). A new plug is **planted**
when it is in one of the four empty cells (1, 3, 4 and 6 from the left), seated on the cell floor (its centre less
than 2 cm above a standing plug's, so a plug leaning on a cell wall counts), still (its centre moved less than 5 mm over the last 0.3 s), and no other plug is in that cell.

- **0.2 per planted new plug** (0 to 0.8), at the end of the trial.
- **1.0:** all four new plugs planted, one per empty cell, none still held (no gripper tool point within 4.75 cm of a
  planted plug's centre), and both original plants in their cells, held for 1 s. The trial then ends. Completed less
  than 1 s before the run limit scores 0.8.
- **0, defects:** an original plant knocked out at any time (its centre leaves its cell or rises more than 3 cm), or
  any plug falling off the table.

## Scoring a real trial

Film from above. Count the formerly empty cells holding exactly one new plug, sitting on the cell floor, 0.2 each.
Full score when cells 1, 3, 4 and 6 each hold one new plug, the original plants in cells 2 and 5 are undisturbed and
the grippers are clear, for 1 s. An original plant lifted or pushed out of its cell, or any plug on the floor,
scores 0.

## The demonstration

Scripted, open loop, both arms at once. Each arm plants the two plugs from its own flat into the two empty cells on
its side of the strip: a straight-down grip on the plug's upper part (1.5 cm below its top) with the pads closing
along the forward axis, so the gripper housing never reaches across the row of cells; lift so the plug's bottom
clears the tops of the planted plugs by 3.5 cm; carry it over its cell; lower until the finger tips are 6 mm above
the cell walls and let go: the plug drops about 1 cm onto the cell floor. The left arm fills cell 3 then cell 1 while
the right arm fills cell 6 then cell 4, so the two grippers are never both near the centre line.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
