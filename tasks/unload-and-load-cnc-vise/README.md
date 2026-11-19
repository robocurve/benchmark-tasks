# Unload the finished part from a CNC vise, then load the blank against both stops

> Unload the finished part to the output tray, then load the blank into the vise against both stops.

Between cycles, a CNC machine operator unloads the finished part from the vise and loads the next blank seated
against the vise's fixed jaw and side stop, the datum the program machines from. This changeover is the operator's
core, repeated operation. A blank loaded off its stops is machined out of position (scrap); loading with the
finished part still in the vise crashes the cycle.

**Reliability stressor (one fixed level): order plus precision.** Unloading must come before loading, and the blank
counts as seated only within 2 mm of both the fixed jaw and the side stop and square to them. The vise sits between
the arms; the output tray is on the right and the blank on the left, so both arms work.

This differs from the accepted `bench-vise` (clamping a part at a marked height) and
`load-blanks-into-gravity-magazine` (stacking blanks into a feeder): here the objective is the between-cycle
changeover itself, unloading the finished part and then seating the next blank against the vise's datum stops.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/unload-and-load-cnc-vise/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unload the finished part to the output tray, then load the blank into the vise against both stops." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cnc-dream
```

The scripted full-score demonstration (both arms, open loop): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/cnc-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (geometry, both arms' inverse kinematics, and the scoring on a kinematic replay
plus negative controls): `python tasks/unload-and-load-cnc-vise/check_task.py`.

Diagnostics: set `CNC_TRACE=/path/trace.jsonl` before a run to log, every control step, both parts' positions and
bounding boxes and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 500 control steps (25 s at 20 Hz); a Dream-1 trial runs until the changeover
has held for 1 s or the run limit of 2,400 steps (2 minutes). No seeds: `-E object_jitter=0` keeps every object
exactly where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Machine vise** (`vise_body`, `vise_fixed_jaw`, `vise_side_stop`): dark grey iron. A 14 x 14 cm body, 4 cm tall,
  whose top is the vise floor, centred 30 cm forward and 3 cm right. A fixed jaw 2 cm thick runs along its far edge
  and a side stop 2 cm thick along its left edge, both 15 mm tall. The jaw's face is 35 cm forward; the side stop's
  face is 2 cm left of the centre line. Fixed to the table.
- **Finished part** (`finished_part`): bright machined aluminium block, 50 mm (forward) x 40 mm x 30 mm tall, 150 g,
  seated in the vise against the fixed jaw and the side stop (centre 32.5 cm forward, on the centre line).
- **Blank** (`blank`): the same block, saw-cut dull grey, lying on the table at 22 cm forward, 27 cm left, its 50 mm
  side forward.
- **Output tray** (`tray_floor`, `tray_lip_*`): green, inside 12 x 12 cm, a 1.5 cm lip around a 1 cm floor, centred
  24 cm forward and 27 cm right. Fixed to the table.

Nothing else is on the table.

## Scoring

A part is **in the vise** when its centre lies over the vise floor inside the jaw and side stop (seen from above) less
than 1.5 cm above where it would rest on it; **in the tray** when its centre lies inside the tray's inside square less
than 1.5 cm above where it would rest on the tray floor. **Still** means its centre moved less than 5 mm over the last
0.3 s. The blank is **seated** when it is in the vise, its far face is within 2 mm of the fixed jaw and its left face
within 2 mm of the side stop, and it is square (its footprint no more than 3 mm larger than its own size either way,
about 3 degrees).

- **Milestones, 0.25 each** (0 to 0.75), at the end of the trial: the finished part out of the vise; the finished part
  in the tray and still; the blank in the vise and still, with the finished part out.
- **1.0:** the finished part in the tray, the blank seated, both still and no gripper tool point within 4 cm of
  either, held for 1 s. The trial then ends. Completed less than 1 s before the run limit scores 0.75.
- **0, defects:** the blank and the finished part in the vise at the same time (loading over an unloaded part), or
  either part falling off the table.

## Scoring a real trial

Film from above. 0.25 once the finished part is out of the vise, 0.25 more once it rests in the output tray, 0.25
more once the blank rests in the vise. Full score when the blank's far and left faces are within 2 mm of the fixed
jaw and the side stop (a 2 mm feeler gauge does not fit either gap) and both grippers are clear, for 1 s. The blank
put into the vise while the finished part is still there, or a part on the floor, scores 0.

## The demonstration

Scripted, open loop. Grips are straight down with the pads closing across the 40 mm width, on the part's upper half
only (finger tips 3 mm above the stops' tops), so a held part can be slid into the vise's corner without the fingers
touching the stops. The right arm unloads the finished part to the output tray and moves clear. The left arm then
brings the blank over the vise, lowers it 12 mm short of both stops and 2 mm above the floor, slides it sideways onto
the side stop (still 4 mm off the fixed jaw, so the jaw adds no friction), then forward onto the fixed jaw, commanding
12 and 6 mm past contact so it presses against both, and lets go.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
