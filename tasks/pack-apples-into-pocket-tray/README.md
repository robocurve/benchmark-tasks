# Pack the sound apples into a pocket tray, one per pocket, leaving the bruised one out

> Pack the good apples into the pocket tray, one per pocket.

Grading fruit by eye and packing the sound fruit into pocket trays is a packhouse packer's core operation; culls
such as bruised apples must stay out of the pack. Five apples of the same size lie in a field crate: four sound red
ones and one bruised apple that differs only in colour. Each sound apple goes into its own pocket of a 2 x 2 pulp
tray. Packing the bruised apple is a quality defect and scores zero.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/pack-apples-into-pocket-tray/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pack the good apples into the pocket tray, one per pocket." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/pack-dream
```

The scripted full-score demonstration (left arm, open loop): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/pack-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (gripper clearance, the demo's inverse kinematics, and the scoring on a
kinematic replay plus negative controls): `python tasks/pack-apples-into-pocket-tray/check_task.py`.

Diagnostics: set `PACK_TRACE=/path/trace.jsonl` before a run to log, every control step, all apples' positions and
both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 700 control steps (35 s at 20 Hz); a Dream-1 trial runs until the pack has
held for 1 s or the run limit of 2,400 steps (2 minutes). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Field crate** (`crate_floor`, `crate_lip_*`): wood, inside 32 cm (forward) x 22 cm (across), a 2 cm lip around
  a 1 cm floor. Centre 25 cm forward, 23 cm left. Fixed to the table.
- **Sound apples** (`apple_1` to `apple_4`): red, 70 mm diameter, 180 g each, on the crate floor at 15 cm forward /
  18 cm left, 15 / 28, 25 / 28 and 35 / 28 (forward / left, cm).
- **Bruised apple** (`bruised_apple`, the cull): dull brown-red, same size and mass, at 25 cm forward / 18 cm left,
  between the sound ones.
- **Pocket tray** (`tray_floor`, `tray_wall_*`, `tray_divider_*`): pulp, 21 x 21 cm, a 1 cm floor with 1 cm-thick
  walls and dividers 3 cm tall forming four 9 cm square pockets. Centre 25 cm forward, on the centre line between the
  arms, 5 mm right of the crate. Fixed to the table.

Nothing else is on the table.

## Scoring

A sound apple is **packed** when its centre lies inside a pocket's 9 cm square seen from above, it rests on the tray
floor (its centre less than 5 cm above it), it is still (under 1 cm/s and 0.5 rad/s), and no other apple shares that
pocket.

- **0.2 per packed sound apple** (0 to 0.8), at the end of the trial.
- **1.0:** all four sound apples packed, one per pocket, none still held (no gripper tool point within 2 cm of any of
  them), and the bruised apple not over the tray, held for 1 s. The trial then ends. A pack completed less than 1 s
  before the run limit scores 0.8.
- **0, defects:** the bruised apple anywhere over the tray at any time (packed as a cull), or any apple falling off the
  table.

## Scoring a real trial

Film from above. Count the pockets holding exactly one sound apple, resting on the pocket floor, 0.2 each. Full score
when all four pockets each hold one sound apple, the grippers are clear, and the bruised apple is not in the tray, for
1 s. The bruised apple ever over the tray, or any apple on the floor, scores 0.

## The demonstration

Scripted, open loop, left arm only. For each sound apple in turn: a straight-down grip with the pads closing on its
equator (opened to about 85 mm around the 70 mm apple), lift so its bottom clears the tops of the other apples by 3.5 cm, carry
it over its pocket, lower until the finger tips are 6 mm above the dividers, and let go: it drops about 1.5 cm onto the
pocket floor. Pockets are filled far-right first (the arm reaches that pocket only up to about 12 cm, so that first trip is
carried lower, before anything is packed), then far-left, near-left and near-right. The right arm holds its ready pose.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
