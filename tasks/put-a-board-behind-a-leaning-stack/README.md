# Put a long serving board away at the back of a board rack, behind two boards leaning on its rail

Kitchen boards are kept standing on edge in a rack, leaning back on its rail. The longest board goes at the back,
against the rail, so the boards in front lean on it. But the boards already in the rack cover that space: they lean on
the rail. Putting the long board away means tipping the leaning boards toward you with one hand and keeping them
there, sliding the long board in behind them with the other, then letting them lean back onto it. Let go early and
the gap closes on the board; tip them past upright and they fall out of the rack.

Instruction: *Put the long board away at the back of the rack, without taking the other boards out.*

Files: `task.py` (the task file: objects, `score`, `demo`), `partkit.py` (compound rigid parts and the reset and step
hooks, credited below), `task.json`.

## What is on the table

Positions in cm from the midpoint between the arm bases (x straight out, y to the left), heights above the table, for
layout 0 (see *Layouts*).

- **Board rack** (loose, beech, 2.0 kg): floor 13 x 51 cm (x by y), 1.2 cm thick, from x 28 to 41 and y -35 to +16.
  A 1 x 1 cm lip runs along its near edge (x 28 to 29). A 2 x 2 cm back rail (x 39 to 41, top 10 cm above the table)
  stands on two 2 x 2 cm end posts at y -35 to -33 and +14 to +16. There are 10 cm of floor between the lip and the
  rail, and 47 cm between the posts.
- **Two short serving boards** (loose, bamboo, 24 x 16 x 1.0 cm, 0.27 kg each): standing on their long edges in the
  rack between y -12 and +12, the front one's foot against the lip, the rear one leaning on it and on the rail, both
  leaning back about 41 degrees.
- **Long serving board** (loose, bamboo, 32 x 12 x 1.2 cm, 0.30 kg): standing on its long edge in the drying stand.
- **Drying stand** (fixed, beech): base 34 x 6 x 1 cm centred at (20, -4), long side along y, with two 34 x 2 x 3 cm
  jaws leaving a 1.5 cm slot.

Board sizes are those of common bamboo serving and cheese boards; masses follow from bamboo's density
(about 0.6 to 0.7 g/cm3). The rack is sized for two to three such boards.

## Layouts

The rack and the boards in it move together each trial; the drying stand does not. Trial k of a run (k = 0, 1, 2, ...
in the order `--epochs` runs them) uses row k mod 7:

| row | rack shift x (cm) | rack shift y (cm) |
|---|---|---|
| 0 | 0.0 | 0.0 |
| 1 | +1.0 | -5.0 |
| 2 | -1.0 | +5.0 |
| 3 | +0.5 | +3.0 |
| 4 | -0.5 | -3.0 |
| 5 | +1.0 | +4.0 |
| 6 | -1.0 | -4.0 |

`BOARD_RACK_LAYOUT=<row>` in the environment pins one row for every trial (as for the screenshot, row 0). The
scripted demo plans from this table only; it never reads the simulator's object poses.

## Run and score a trial

Copy this folder to `~/put-a-board-behind-a-leaning-stack` on the GPU machine. Always run with `-E object_jitter=0`.

```bash
export INSTR="Put the long board away at the back of the rack, without taking the other boards out."

# a policy (here Dream-1): 1,200 steps = 60 seconds at 20 Hz
inspect-robots run --embodiment yam_arena --policy dream -E task=$HOME/put-a-board-behind-a-leaning-stack/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/board-rack-dream

# the scripted demo, two trials (layouts 0 and 1)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=$HOME/put-a-board-behind-a-leaning-stack/task.py -E object_jitter=0 -E control_hz=20 --instruction "$INSTR" --scorer task_score --max-steps 700 --epochs 2 --store-frames --log-dir ~/yam-eval/board-rack-demo
```

The run prints `task_score` (the score at the end of the run). `TASK_DEBUG=1` prints each board's lean, lowest point
and extent in the rack's frame every second.

Runtime per trial: the demo ends at 600 steps (30 s of trial time); on a shared NVIDIA L40S about 4 minutes of wall
clock per trial, plus about 50 s of Isaac start-up per run. Built against `yam-isaac-lab-arena` at `0059d02`.

## Scoring

Measured in the rack's own frame at the end of the run.

- **1.0**: nothing has moved for 1 s (every board and the rack slower than 1 cm/s and 0.1 rad/s); both grippers are at
  least 3 cm from every board; all three boards stand on the rack floor (lowest point within 3 mm of it) and lie
  between the lip, the rail and the end posts; the long board's foot is nearest the rail; and the short boards lean
  back 5 to 40 degrees (onto the long board). The long board may lean -5 to 40 degrees.
- **0.5**: the long board is in the rack but the stack is not back on it: everything at rest, grippers clear, all
  three boards on the rack floor and inside it, the short boards standing upright or leaning back (-5 to 40 degrees),
  but either the short boards stand upright instead of leaning back (under 5 degrees), or the long board's foot is
  in front of or between theirs.
- **0** otherwise: a board still held or touched, a board on the table or lying flat, or either short board's lowest
  point more than 1 cm above or below the rack floor at any step of the trial (taken out of the rack). That last one
  is latched for the trial and cleared only when the next trial resets; putting the board back does not undo it.

Leaning back is measured from vertical, positive with the top toward the rail. Set in against the rail, the long
board rests at 4 to 7 degrees depending on where its foot lands, or stands upright if nothing leans on it; that
should not decide the score, so its range starts at -5 degrees. The 0.5 rewards the hard part (the long board in at
the back, nothing taken out) when the stack was not let back onto it.

## Scripted demo and shortcut checks

`demo()` (played by `--policy yam_scripted`):

1. The closed left gripper comes down behind the short boards' tops, 3 cm in from their far end (beyond the long
   board's place), and pulls them toward the robot until they stand about 6 degrees short of upright. It never
   grips. Released there they fall back onto the rail; pulled past upright they fall out of the rack.
2. The right gripper takes the long board by the middle of its top edge, across its 1.2 cm thickness, lifts it out of
   the stand, carries it to the rack's open end (beside the short boards, not over them), lowers it to 0.8 cm above
   the floor between the held boards and the rail, slides it 18 cm along the rack to its place, sets it down and lets
   go with the fingers opened only to 2 cm.
3. The left gripper backs away slowly: the short boards lean back onto the long board, which leans on the rail.
   Both arms leave.

Results (`TASK_DEBUG=1`, end state in the rack's frame):

| run | trials | score | end state |
|---|---|---|---|
| demo, layouts 0 to 6 | 7 | 1.0 each | short boards lean 26 to 29 deg, long board 4.5 to 7.4 deg with its foot nearest the rail, all on the floor, at rest, grippers clear |
| do nothing | 1 | 0 | long board still in the stand |
| demo, but keep holding the stack forward at the end | 1 | 0 | boards in place, left gripper not 3 cm clear |
| push the stack past upright and let go, then insert | 1 | 0 | both short boards fall out over the lip (latched at step 180); they end flat on the table |
| one arm only: slide the long board in without holding the stack | 1 | 0 | the long board jams against the short boards' end and stays sticking out past the rack's end post; short boards untouched |
| stand the long board in front of the short boards | 1 | 0 | there is no floor in front of them: it ends on the table |
| fixed script (always layout 0) on layouts 0, 1, 2 | 3 | 1.0, 0, 0 | layout 1: the long board hits the stack and falls; layout 2: it ends 1.2 cm past the end post |
| lift the front short board 2.5 cm and set it back, then insert | 1 | 0 | latched at the lift (step 480); the board set back also blocked the long board, which fell |
| insert, then try to lift the front short board while the stack is held | 1 | 0.5 | the grip failed and pushed both short boards exactly upright, where they stand balanced; long board in at the back; nothing taken out, grippers clear |

**Dream-1** (one full run, layout 0, 1,200 steps): 0.0. It ran the whole trial without refusing; both arms hovered
near their start and the drying stand, reached toward the rack's front edge around 42 to 50 s, and touched neither the
leaning boards nor the long board, which ended where they started.

The "in front of or between" half of the 0.5 cannot happen with this rack: the short boards' feet stand against the
lip, so there is no floor in front of them. The upright-stack half did happen (row above).

## What makes this task distinctive

One hand holds, the other inserts, and neither can do the other's job. The gap behind the boards exists only while
they are held: a closed gripper can only push them, from behind, so it has to keep them just short of upright,
leaning on its fingers, for the whole insertion: pull too far and they topple out of the rack, ease off and the gap
closes. The long board can't be dropped in from above (the held boards' tops are in the way) and can't be wedged in
(it jams), so it has to go in from the rack's open end at floor height and slide along between the held boards and
the rail, then be let go before the stack comes back onto it. A photo of the end state shows where the board goes,
not how to keep two boards from falling while it gets there.

## Licences

The task (all files in this folder) is under the MIT licence. `partkit.py` is adapted from `tasks/din-rail-mount`
(MIT). No third-party models or textures are used.
