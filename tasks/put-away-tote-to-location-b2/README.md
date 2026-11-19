# Put a tote away in rack location B2, read from the rack's row tape and bay dots

> Put the tote away in location B2 of the bin rack.

Put-away, carrying received stock to the storage location its label calls for and setting it into that location, is
a warehouse put-away associate's core operation. Stock put away in the wrong location is lost to the inventory system
until someone finds it. Here a two-row, three-bay bin rack stands between the arms; two locations already hold totes.
The orange tote on the receiving spot goes into location B2.

**Reliability stressor (one fixed level): a hidden rule.** The location code has to be read from the rack's own
markings, as associates do: rows are marked by coloured tape along the bins' front edges (row A red, row B blue), and
bays by yellow tape dots on the floor in front of them (one, two, three). The neighbouring locations A2, B1 and B3
look the same as B2; putting the tote down in any of them scores 0.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/put-away-tote-to-location-b2/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the tote away in location B2 of the bin rack." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/putaway-dream
```

The scripted full-score demonstration (left arm, open loop): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/putaway-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (geometry, the demo's inverse kinematics, and the scoring on a kinematic replay
plus negative controls): `python tasks/put-away-tote-to-location-b2/check_task.py`.

Diagnostics: set `PUTAWAY_TRACE=/path/trace.jsonl` before a run to log, every control step, all totes' positions, the
location the tote is in, and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 260 control steps (13 s at 20 Hz); a Dream-1 trial runs until the tote has
rested in B2 for 1 s or the run limit of 2,400 steps (2 minutes). No seeds: `-E object_jitter=0` keeps every object
exactly where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Bin rack** (`rack_step`, `row_*`): blue plastic bins in two rows of three bays. Each bin is 9 cm (forward) x
  8.5 cm inside, with 6 mm walls 3 cm tall above a 1 cm floor. Row A stands on the table, centred 22 cm forward; row B
  stands on a 4 cm grey steel step directly behind it, centred 31.6 cm forward. Bays are centred 9.1 cm left (bay 1),
  on the centre line (bay 2) and 9.1 cm right (bay 3). Fixed to the table.
- **Row tape** (`row_A_tape` red, `row_B_tape` blue): along the top of each row's front wall.
- **Bay dots** (`bay_N_dot_*`): yellow 1 cm squares on the table 1.5 cm in front of row A: one dot in front of bay 1,
  two in front of bay 2, three in front of bay 3.
- **Stored totes** (`stored_A1`, `stored_B3`): grey, already in locations A1 and B3.
- **Tote to put away** (`tote`): orange, 60 (forward) x 55 x 50 mm tall, 250 g, standing on the table at 14 cm forward,
  27 cm left.

Nothing else is on the table.

## Scoring

The tote is **in** a location when its centre lies inside that bin's inside rectangle seen from above less than 1.5 cm
above where it would rest on the bin floor. **Still** means its centre moved less than 5 mm over the last 0.3 s.

- **Milestones, 0.25 each** (0 to 0.75), at the end of the trial: the tote lifted 3 cm off the table (at any time);
  the tote over B2 (centre inside B2's rectangle seen from above, at any height, at any time); the tote in B2 and
  still.
- **1.0:** the tote in B2, still, no gripper tool point within 4 cm of its centre, and both stored totes still in A1
  and B3, held for 1 s. The trial then ends. Completed less than 1 s before the run limit scores 0.75.
- **0, defects:** the tote ever resting in a wrong location (in any other location and still), a stored tote knocked
  out of its location, or any tote falling off the table.

## Scoring a real trial

Film from above. 0.25 once the tote is lifted, 0.25 more once it has been over B2, 0.25 more while it sits in B2.
Full score when it sits in B2 with the gripper clear and the grey totes undisturbed, for 1 s. The tote set down in any
other location, a grey tote pushed out, or a tote on the floor scores 0.

## The demonstration

Scripted, open loop, left arm only. A straight-down grip on the tote's upper part (pads closing across its 55 mm
width, opened to about 80 mm), lift so its bottom clears row B's bin walls by 3 cm, carry it along the front of the
rack to bay 2 and then straight back over the empty A2 into B2, lower until the finger tips are 6 mm above the bin
walls and let go: the tote drops about 1 cm onto the B2 floor. The right arm holds its ready pose.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
