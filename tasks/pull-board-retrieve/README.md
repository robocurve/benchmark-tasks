# Pull a board to retrieve an out-of-reach jar

A spice jar stands on the far end of a long narrow board, beyond the reach of either arm in any gripper
orientation. The board's near end is within reach. The robot has to slide the board toward itself, carrying the
jar on it, and then pick the jar into a basket. Indirect retrieval through the object's support (a tray, a
cutting board, a placemat) is something people do all the time; simulated task sets almost always start the
objects within reach.

| File | What |
|---|---|
| `task.py` | the task file: objects, `success`, `distance`, `score` (`SCORE_OVER = "end"`), `demo` |
| `task.json` | the submission metadata (`media` is added by the upload script) |
| `test_task.py` | offline checks, NumPy and pytest only |

## Run a trial

On the VM with `yam-isaac-lab-arena` set up (its Quickstart), copy this folder to `~/pull-board-retrieve`. Always
run with `-E object_jitter=0` (see Seeds).

```bash
export INSTR="The spice jar is out of reach on the board. Pull the board toward you to bring the jar closer, then put the jar in the basket."
```

Scripted demo (full score expected):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$HOME/pull-board-retrieve/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1000 --store-frames --log-dir ~/yam-eval/pull-board-demo
```

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$HOME/pull-board-retrieve/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/pull-board-dream
```

Videos (one MP4 per camera, in each log's `frames/` folder):

```bash
for log in ~/yam-eval/pull-board-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score: ...` at the end: 1.0 when the episode ends in success, otherwise `score()` of the
final state. Put that number in `task.json` `scores`, e.g. `"Dream-1 run: 0.4; demo: 1.0"`.

Offline checks (here or on the VM):

```bash
YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q test_task.py
```

`YAM_ARENA` is only needed when the `yam_arena` package is not installed. All 28 checks pass against
yam-isaac-lab-arena at commit bd62771 (about 30 s).

## Runtime per trial

- Demo: 41.7 s of plan (834 steps at 20 Hz); the episode ends with success at about step 833 (otherwise
  `yam_scripted` holds 3 s more, 894 steps); `--max-steps 1000` covers it.
- Dream-1: up to 2,400 steps (2 minutes of simulated time) unless it succeeds earlier.
- Wall clock on an RTX 5090, Isaac start-up included: 90 s for the demo, 231 s for the Dream-1 run.

## Seeds

`-E object_jitter=0` is required. A seeded reset shifts each loose object independently by up to 3 cm in x
and y, so the jar could move up to 6 cm sideways relative to the 6.4 cm wide board and start off it, on the table
out of reach, where no strategy can score. No seed is needed with jitter 0; the layout is fixed.

## Assets and licences

None. Every object is a primitive defined in `task.py` (boxes and one cylinder); no meshes, textures or library
assets are used.

## Design notes

- **Reach (measured offline with the package's IK, `test_task.py`).** On the midline, a straight-down grip
  reaches the table out to 0.40 m (0.32 m at 12 cm above it) and as close as 0.04 m. In any gripper orientation
  the TCP reaches about 0.70 m on the midline (about 0.78 m along an arm's own line). The jar starts at 0.78 m
  on the midline, 0.84 m from each base: straight down every arm and yaw tried falls more than 5 cm short, and in
  any orientation the TCP falls more than 3 cm short of the jar's centre and cannot get its pad tips to the jar's
  near face either.
- **Changes from the original sketch (35 x 6 x 1.5 cm board, jar outside straight-down reach) and why.** A jar
  that is only outside *straight-down* reach (0.52 to 0.70 m) can still be grasped from the side with the arm
  stretched out, which would score full marks without touching the board. So the jar sits beyond reach in any
  orientation, on the midline where both arms are farthest. To bring it from 0.78 m into straight-down reach
  takes 0.48 m of travel, more than one stroke allows (the grip point moves from 0.37 to 0.07 m), so the board is
  48 cm long and the demo pulls hand over hand: 30 cm, re-grip, 18 cm. The board's section is a stock pine 1x3
  (19 x 64 mm), which is easy to buy and fits the gripper.
- **Gripper.** Fully open, the pads are 94 mm apart (`FINGER_OPEN = -0.04695` m per finger in `constants.py`,
  checked in the package's `tests/test_constants.py`). With at least 1 cm of margin, nothing grasped may exceed
  84 mm: the board is gripped across its 64 mm width and the jar across its 45 mm diameter. The pad tips sit 13 mm
  past the TCP, so the demo grips the board with the TCP 19 mm above the table: tips 6 mm above the table, 13 mm
  of pad on the board.
- **Demo (left arm only; the right arm holds its ready pose).** Stroke 1: grip the board at x 0.37, slide it to
  0.07 in 10 s. Stroke 2: re-grip at 0.37, slide to 0.19 in 8 s. Both strokes are smoothstep moves with peak speed
  under 5 cm/s and peak acceleration under 0.02 m/s^2. (The first sim run used 6 s and 5 s, peak 7.5 cm/s: the jar
  started wobbling on its rim when the board broke loose, reached 12 deg of tilt and fell when the grip let go.)
  Then a top-down pick of the jar at x 0.31 (where the strokes leave it in sim; 0.30 on paper), carried at 13 cm
  above the table and set down 8 mm above the basket floor.
- **Jar height 6 cm (was 8 cm).** With the slow strokes the 8 cm jar (height/diameter 1.8) still rocked to 1.8 deg
  in stroke 2, because the dragged board chatters and the simulator has no rolling resistance to damp a rim wobble.
  A squat jar (height/diameter 1.3) gives a policy that pulls faster than the demo more margin, so a failure says
  "pulled carelessly" rather than "hit a simulator artifact". The recorded demo uses the 6 cm jar.
- **Scoring.** `SCORE_OVER = "end"`: the final state decides. With `"best"`, a jar that passed through the
  0.4 zone and then went off the table (flung toward the arms or dropped on the floor) would keep 0.4, contradicting
  "0 if the jar falls off the table". The 0.4 zone (x at most 0.40 m, |y| at most 0.40 m) is where both arms reach
  straight down; one stroke alone leaves the jar at 0.48 m, outside it, so partial credit means the retrieval is
  actually done. The basket lies wholly inside the zone, so a jar in the basket that has not settled still gets 0.4.
  "In the basket" asks for the jar's bottom within 1 cm of the floor, so a jar held up inside the basket does not count.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 90 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 2,400-step run limit; 231 s wall clock.
- Offline tests: 28 passed.
