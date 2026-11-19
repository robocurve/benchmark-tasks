# Kit gearbox parts into a fitted kit tray, with look-alike shafts

> Kit the gearbox parts into the kit tray, each part in its own fitted pocket.

Kitting, picking a product's parts and placing each into its own fitted pocket of a kit tray so the assembler
receives a complete and correct kit, is an assembly-line kitter's core operation. Four gearbox parts lie on the
table: two shafts in front of the left arm, a gear and a bearing in front of the right arm. Each goes into its own
pocket of a foam kit tray between the arms.

**Reliability stressor (one fixed level): look-alike parts.** The two shafts have the same section and colour and
differ only in length (120 vs 90 mm). The short shaft also fits the long shaft's pocket, so putting it there leaves
the long shaft with nowhere to go. Likewise the bearing fits the gear's pocket, but the gear does not fit the
bearing's.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/kit-gearbox-parts-into-fitted-tray/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Kit the gearbox parts into the kit tray, each part in its own fitted pocket." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/kit-dream
```

The scripted full-score demonstration (both arms, open loop): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/kit-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (pocket fit, gripper clearance, both arms' inverse kinematics, and the scoring
on a kinematic replay plus negative controls): `python tasks/kit-gearbox-parts-into-fitted-tray/check_task.py`.

Diagnostics: set `KIT_TRACE=/path/trace.jsonl` before a run to log, every control step, all parts' positions and both
grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 480 control steps (24 s at 20 Hz); a Dream-1 trial runs until the kit has held
for 1 s or the run limit of 2,400 steps (2 minutes). No seeds: `-E object_jitter=0` keeps every object exactly where
`task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Kit tray** (`tray_floor`, `pocket_*`): blue foam, a 1 cm floor with four fitted pockets (1 cm walls, 2 cm tall),
  all centred 27 cm forward, fixed to the table. Each pocket leaves 7.5 mm of play around its own part:
  - long-shaft pocket, 13.5 x 4 cm (long side forward), centred 14 cm left;
  - short-shaft pocket, 10.5 x 4 cm, centred 6 cm left;
  - bearing pocket, 5 x 5 cm, centred 2 cm right;
  - gear pocket, 7.5 x 7.5 cm, centred 11 cm right.
- **Long shaft** (`long_shaft`): steel-grey square bar, 120 x 25 x 25 mm, 200 g, lying along the forward direction at
  20 cm forward, 30 cm left.
- **Short shaft** (`short_shaft`): the same bar, 90 mm long, 150 g, at 20 cm forward, 21 cm left.
- **Gear** (`gear`): dark-grey disc, 60 mm across, 20 mm thick, 70 g, at 20 cm forward, 30 cm right.
- **Bearing** (`bearing`): silver disc, 35 mm across, 12 mm thick, 60 g, at 20 cm forward, 20 cm right.

Nothing else is on the table.

## Scoring

A part is **in** a pocket when its centre lies inside the pocket's inside rectangle seen from above and is less than
1.5 cm above where it would rest on the tray floor. A part is **kitted** when it is in its own pocket, still (its
centre moved less than 5 mm over the last 0.3 s), and no other part is in that pocket.

- **0.2 per kitted part** (0 to 0.8), at the end of the trial. A part in the wrong pocket earns nothing, and the part
  it displaces cannot be kitted while it stays there.
- **1.0:** all four parts kitted and none still held (no gripper tool point within 4 cm of a kitted part's centre),
  held for 1 s. The trial then ends. A kit completed less than 1 s before the run limit scores 0.8.
- **0, defect:** any part falling off the table.

## Scoring a real trial

Film from above. Count the pockets that each hold their own part and nothing else, lying in the pocket, 0.2 each.
Full score when all four pockets hold their own parts and the grippers are clear, for 1 s. Any part on the floor
scores 0.

## The demonstration

Scripted, open loop, both arms. Straight-down grips with the pads closing across each part (across the shafts' width,
across the gear's and bearing's diameter), centred on the part; lift so the part clears the tray by about 6 cm; carry
it over its pocket; lower until the finger tips are 6 mm above the pocket walls and let go: the part drops about
2.3 cm onto the pocket floor. The left arm kits the long shaft, then the short one; the right arm kits the gear, then
waits until the left arm has left the centre and kits the bearing.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
