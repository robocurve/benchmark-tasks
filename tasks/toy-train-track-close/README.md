# Close a wooden toy train track loop with the one loose piece that joins both connectors

A loop of wooden toy train track lies on the table with one large curve missing, nearest the right arm. Three loose
pieces lie beside it: a large curve of the same kind, a straight and a short curve. The robot must close the loop with
the piece that fits and let go of it.

Instruction given to the policy:

`Close the gap in the wooden train track with the piece that fits.`

Wooden train track (BRIO and the many sets made to fit it) is one of the most common toddler toys, and closing or
mending a loop of it is something most people who have played with a small child have done. The pieces join by a peg,
a round knob on a narrow neck, dropped from above into the next piece's keyhole socket. In a loop with one piece
missing, a peg sticks into the gap on one side and a socket waits on the other. The new piece has to come down with its
socket over that peg and its own peg over the far socket, both at once. Of the three loose pieces only the large curve
spans the gap, and it lies a quarter turn from the way it goes in. Set down turned the other way round, or off its
line, it rests on a peg or a socket's rim instead of joining.

The task tests three things:

- choosing the piece that fits from what is on the table;
- turning it to the gap's heading;
- seating two peg-and-socket joints together.

This differs from tasks that push a train along a track already laid, or that set a single piece into a recess: here
two joints on a loose loop have to be made at once.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the track pieces, `score()`, `success()` and `distance()` |
| `demo.py` | Scripted demo through `inspect-robots`; writes `result.json`, videos and `setup.png` |
| `partkit.py` | Rigid parts built from several boxes and cylinders |
| `clearance.py` | Arm-to-arm clearance from the YAM collision meshes (logging only) |
| `task.json` | The submission details |

Offline tests: `tests/test_toy_train_track_close.py` in the team repository (23 tests, no simulator).

## Run a trial

In the Isaac Lab-Arena environment, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Close the gap in the wooden train track with the piece that fits." \
  --scorer task_score --store-frames --max-steps 1200 --log-dir runs/dream
```

Scripted demo and its wrong attempts:

```bash
python demo.py task.py demo_out                                       # full score
DEMO_VARIANT=straight python demo.py task.py straight --no-record     # the straight set in the gap
DEMO_VARIANT=short python demo.py task.py short --no-record           # the short curve set in the gap
DEMO_VARIANT=unturned python demo.py task.py unturned --no-record     # the right curve without the quarter turn
DEMO_VARIANT=offset python demo.py task.py offset --no-record         # the right curve 6 mm off its line
DEMO_VARIANT=hold python demo.py task.py hold --no-record             # the right curve set in place, never let go
```

The demo is open loop. It uses the fixed layout, the clock and the arm's own joint readings. It turns the set-down by a
fixed 2.4 degrees, the turn the curve takes in the pinch as measured in two Isaac batches; this is a constant, not
read from the simulator.

## Scoring

Everything is read at the end, from the video and a photo from above. Nothing scores while a gripper fingertip is
within 30 mm of any piece.

- **Joint made:** a peg down in its neighbour's socket, its centre within 3.5 mm of the socket's centre (the socket is
  16 mm across, the peg 11.5 mm), with both pieces flat on the table (within 2.5 mm and 3 degrees).
- **1.0:** the loop is closed with the loose large curve and all eight joints are made. No piece moves more than 1 mm
  and no fingertip comes near over the last 1 s (21 samples at 20 Hz).
- **0.5:** that curve lies flat in the gap with at least one of its two joints made, or the loop is closed but not yet
  still.
- **0.25:** that curve lies in the gap but rests on a peg or a socket's rim: its middle is within 20 mm of the gap's
  middle and its heading within 10 degrees of the gap's.
- **0:** anything else. This includes a straight or short curve in the gap, the right curve turned the other way
  round, the curve still held, and the loose pieces left where they were.
- **Fall:** if any piece falls off the table, the trial scores 0 for good; the record is cleared only when the scene is
  reset.

`task_score` is `score()` at the end of the run.

## Scores

- Scripted demo: 1.0 at step 323, in two batches.
- Wrong attempts:
  - `straight` 0.0;
  - `short` 0.0;
  - `unturned` 0.0;
  - `offset` 0.25 (the curve rests on the peg 19 degrees up);
  - `hold` 0.0.
- Dream-1 run (1,200 steps): 0.0. It moved the right arm about over the loose pieces and the gap for the whole minute,
  and the loop stayed open.

## Runtime and seeds

1,200 control steps (1 minute at 20 Hz) per trial; the scripted demo finishes in about 320. One fixed layout: run
with `-E object_jitter=0`. There is no other randomness.

## Fidelity

Every piece is rigid beech at the listed size, with its mass from its volume (about 0.72 g/cm3).

- **Curves:** box segments along the arc, one of them square across the piece's middle.
- **Sockets:** two cheeks either side of a 16 mm square keyhole behind a 9 mm throat.
- **Pegs:** a 6 mm neck and an 11.5 mm knob with a rounded top (a 9 mm step on its top 2.5 mm).
- **Estimated:** the neck's and throat's widths and the peg's height.
- **Not modelled:** the running grooves.

Wood on wood and on the table: friction 0.45. The loop's pieces are loose, as on a real table, so a piece pushed sideways
drags its neighbours.

## Real-world build

All of it costs about $35.

- **Track:** BRIO-compatible beech wooden track. Two BRIO 33342 Large Curved Tracks packs (4 pieces each) give the eight
  large curves. Add one straight track (144 mm, BRIO A) and one short curved track (BRIO E1, inner edge 90 mm).
  Unbranded compatible track works if it meets the same dimensions:
  - 40 mm wide and 12 mm thick;
  - a peg 11.5 mm across on a 7 mm neck;
  - a socket 15 to 17 mm across.
- **Layout:** print the loop's outline at 1:1 and tape it to the table, its centre 300 mm in front of the midpoint
  between the arm bases. Leave the place nearest the right arm empty.
- **Checks:**
  - a photo from above at the end, to see each peg in its socket and each piece flat (a piece riding on a peg stands
    proud by about 12 mm, which shows against a 10 mm card gauge);
  - pushing a wooden train once round the loop by hand after the trial: it crosses every joint without derailing when
    the loop is closed.
- **Hazards:** none. The pieces are smooth wood with rounded edges and nothing sheds on the grippers.

## Licence

The task is released under the MIT licence. It uses no library or third-party assets: every piece is built from boxes
and cylinders in `task.py`. BRIO is named only to identify the track standard; the geometry was written for this task
from published dimensions.
