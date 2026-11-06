# Complete a Latin-square layout

A slate-grey board on the table is taped into a 3 x 3 grid. Nine painted wooden blocks, one of every combination of
three colours (red, yellow, blue) and three lengths (4, 8, 12 cm), make up the layout. Three are fixed to the board
already; the other six lie in two columns beside it. The robot must place the six loose blocks so that every row and
every column holds one block of each colour **and** one block of each length:

> Three blocks are already fixed on the 3 x 3 board. Put the six loose blocks in the six empty squares, one block
> per square, so that every row and every column has one red, one yellow and one blue block, and one short, one
> medium and one long block.

This is a Latin square on two attributes at once (a Graeco-Latin square of order 3): the counterbalanced layout that
field trials, assay plates and robot-evaluation grids use so that position does not confound the condition. With the
three fixed blocks given, exactly one layout meets every rule (checked by enumeration in the tests), so the robot
has to reason about both attributes together and then carry out six pick-and-places, each with a quarter turn.

| File | What |
|---|---|
| `task.py` | the task file: objects, `success`, `score`, `distance`, scripted `demo` |
| `task.json` | the submission fields (`media` is added by the upload script) |
| `test_task.py` | offline checks (NumPy and pytest, no Isaac Sim) |

## Run a trial

From a checkout of the arena (yam-isaac-lab-arena, tested at `bd62771`) on the GPU VM, with this folder at
`tasks/latin-square-layout/`. Use the fixed layout (`-E object_jitter=0`): the scorer itself does not depend on the
start (it reads only where blocks end up), but the scripted demo is open loop and needs the staging positions in
`task.py`, and a fixed start keeps Dream-1 runs comparable. No seed changes the scene.

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/latin-square-layout/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Three blocks are already fixed on the 3 x 3 board. Put the six loose blocks in the six empty squares, one block per square, so that every row and every column has one red, one yellow and one blue block, and one short, one medium and one long block." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/latin-square-layout-dream
```

Scripted demo (39.9 s = 798 steps of waypoints per arm; the plan plus about 100 steps):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/latin-square-layout/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Three blocks are already fixed on the 3 x 3 board. Put the six loose blocks in the six empty squares, one block per square, so that every row and every column has one red, one yellow and one blue block, and one short, one medium and one long block." \
  --scorer task_score --max-steps 960 --store-frames --log-dir ~/yam-eval/latin-square-layout-demo
```

Videos, one MP4 per camera, in each log's `frames/` folder:

```bash
for log in ~/yam-eval/latin-square-layout-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score`: 1.0 when the trial ended in success, otherwise `score()` at the last step.

Runtime per trial: the run limit is 2,400 steps (2 minutes of simulated time at 20 Hz) for Dream-1 and 960 steps
(48 s) for the demo. Wall clock on an RTX 5090, including Isaac start-up: 83 s for the demo and 233 s for
Dream-1.

Offline tests (42 tests, about 2-3 minutes):

```bash
YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest tasks/latin-square-layout/test_task.py -q
```

## Scoring in one paragraph

Six lines (three rows, three columns), each worth 1/6. A line counts when each of its three squares holds exactly
one block resting on the board (centre inside the square's tape centre lines, bottom within 6 mm of the board top)
and the three blocks are three different colours **and** three different lengths. Success = all six lines with
every loose block still, held 1 s. Exact rules are in `task.json` `scoring` and in `task.py` (`cell_of`, `seated`,
`layout`, `line_ok`).

- **Why per line, not per attribute check.** Scored as 12 independent checks (6 lines x 2 attributes), a random
  fill of the six squares would average 0.37-0.43 by chance. Requiring both attributes per
  line drops the random-fill average to 0.10 and the "colours right, lengths ignored" average to 0.28 (both
  enumerated in `test_a_random_fill_or_a_colour_only_pattern_earns_little`), while a correct layout scores 1.0 and
  each further correct line still earns partial credit.
- **Heading does not matter.** A block turned in its square, or stood on end, still counts; only which square its
  centre is over and whether it rests on the board.
- **Crowding voids a square.** Two blocks in one square (including a loose block pushed in beside a fixed one)
  make that square empty, so dumping spare blocks anywhere cannot help.

## Shortcuts that score 0 (all tested)

| Strategy | Score |
|---|---|
| Do nothing | 0: no line is complete at the start (at most two givens per line) |
| Pile every loose block into one square | 0 |
| Every colour rule met, lengths ignored (or the reverse), in an unlucky arrangement | 0 (12 such fills each; average 0.28) |
| Knock the blocks off the board or the table | 0 |
| Stack a loose block on a fixed one | that block is not seated; its own square stays empty |

## Design numbers

- **Gripper.** At `FINGER_OPEN` (-0.04695 m per finger) the pads are 94 mm apart (arena `constants.py`); the
  usable grip is taken as 84 mm (1 cm margin). Every block is gripped across its 25 mm width, 59 mm under that. The
  demo opens only to 45 mm on the way down (10 mm per side) to keep finger travel, and the off-centre grip, small
  (under 1 mm measured on 16-25 mm bars).
- **Sizes for the camera.** Lengths step by 4 cm (40/80/120 mm). Projected with the station's calibrated top camera
  at 320 x 176, each step is 6-8 px in every square and staging spot (`test_the_three_lengths_differ_by_several_pixels_in_the_top_camera`);
  size differences under 2 cm do not show at this resolution. Colours are saturated red, yellow and blue on a slate-grey board.
- **Squares.** 80 mm front to back by 160 mm left to right. A 25 mm-wide block has 24.5 mm of slack front and back
  (where any off-centre grip lands, because the fingers close along x at the release), and a 120 mm block 17 mm at
  each end. The tape is 1 mm thick, so blocks set down on it still count.
- **Grip and carry heights.** The TCP grips 19 mm above a block's bottom (pad tips 6 mm above the support). A carried
  block's bottom travels 45 mm above the board, 20 mm over any block already standing there. Blocks are set down
  from 3 mm; after letting go the pads rise 3 cm before going up and moving on.
- **Turns.** Loose blocks are staged long side front to back so the two staging columns (y = +-32 cm) stay out of
  the region the arms' ready pose hides from the top camera. Each is picked with yaw 0 and turned a quarter turn
  during the 2 s carry (left arm to -90 deg, right arm to +90 deg), so it lands long side left to right. A few
  degrees of turn loss do not matter: heading is not scored.
- **Reach and timing.** Left arm: yellow medium to the near-middle square, yellow long and blue medium to the left
  column. Right arm: red medium and yellow short to the right column, red long to the far-middle square. The right
  arm starts 5.2 s (half a move) later, so one arm is always at its staging column while the other sets a block on
  the board; with both arms in phase, the two wrists came within 9 mm over neighbouring squares. IK over the whole
  plan at 20 Hz is under 0.5 mm / 0.1 deg with every joint inside its limits; the parked pose turns the wrist back to
  yaw 0 (yaw -90 deg out there is past joint6's limit).
- **Clearances (offline sweep of the arm collision hulls).** Every link stays more than 15 mm from every block it is
  not handling, the pad tips stay above the table, and the two arms' hulls stay more than 5 cm apart throughout.
- **Camera.** Rays from the top camera to every square and every block top pass at least 5 cm from both arms' link
  chains at the ready pose (first frame) and at the parked pose (last frame).

## What makes this task distinctive

The layout is defined only by a rule on rows and columns, on two attributes at once (colour and length). With the
three fixed blocks given, exactly one layout satisfies it, so every placement is constrained by its row and its
column on both attributes. No complete example is on the table to copy, and a layout that gets colour right but
ignores length (or the reverse) can score 0. Each of the six blocks also needs a quarter turn on the way, because
the loose blocks are staged lengthwise front to back.

## Assets and licences

None. Every object is a primitive box defined in `task.py`; the room and robot come from the arena.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success at step 703; 83 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 2,400-step run limit; 233 s wall clock.
- Offline tests: 42 passed.
- Top camera: in `setup.png` all nine blocks and the whole grid are visible, and the three lengths are clearly different. The
  trial ends on success before the right arm parks, so in the demo's last frame the right arm covers part of the
  right-hand column; the rest of the board stays visible.
