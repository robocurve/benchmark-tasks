# Park the hot iron on its rest

A bimanual YAM station task in Isaac Lab Arena. A hot steam iron stands on an ironing board, with a folded
shirt between it and a heat-resistant rest beside the board. The robot has to lift the iron over the shirt
without the iron ever touching it, set it on the rest soleplate down, and let go. The whole task is one task
file, [`park_hot_iron.py`](park_hot_iron.py), for the `yam_arena` embodiment from
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (Isaac Sim 6, Isaac Lab 3, Isaac Lab
Arena 0.3.1). The scene is built from boxes only, so nothing is downloaded.

## Run a trial

From this folder, in the Arena environment:

```bash
INSTRUCTION="The iron is hot. Move it from the ironing board onto the grey heat-resistant rest, soleplate down, without letting it touch the shirt lying on the board, and let go of it."

# Dream-1 (needs DREAM_API_KEY in ~/.dreamlabs/credentials)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=park_hot_iron.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=park_hot_iron.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/demo

# One MP4 per camera, in runs/<policy>/frames/<run>/
inspect-robots video runs/dream/*.json
```

The run prints `task_score` at the end and keeps it in the log.

- **Run limit:** 1,200 control steps (60 s at 20 Hz).
- **Seeds:** none. `-E object_jitter=0` starts every object exactly where the task file puts it, the layout in
  the screenshot. With a seed and `object_jitter > 0`, the iron and the shirt are each shifted by up to that
  much, and scoring measures the shirt against wherever it started.
- **Runtime per trial:** the demo trial is 330 steps (16.5 s simulated) and took about 1 minute of wall time on
  an L40S, plus a few minutes for Isaac Sim's first start on a fresh machine. A Dream-1 trial runs to at most
  1,200 steps.

## Scene

Positions are in cm, from the midpoint between the two arm bases: x forward, y left, heights above the table
top. The arm bases are at y = +30.5 (left) and y = -30.5 (right).

| Object | Type and material | Size (cm) | Position (centre) | |
|---|---|---|---|---|
| Ironing board | box, wooden board with a beige cotton cover, legs omitted | 28 (x) x 64 (y) x 2.5 | x 30, y -6, on the table (spans y -38 to +26) | fixed |
| Iron | one box, dark-blue plastic body, soleplate on its underside | 20 (x) x 8 (y) x 10 (tall) | x 30, y 2, standing soleplate down on the board | loose, 1.2 kg |
| Shirt | thin rigid sheet, light-blue cotton shirt folded flat | 24 (x) x 16 (y) x 0.5 | x 30, y 17, on the board (spans y 9 to 25) | loose, 0.2 kg |
| Iron rest: pad | box, grey silicone | 24 x 12 x 0.8 | x 30, y 35, on the table | fixed |
| Iron rest: lip | four boxes, grey silicone, 1 cm wide, 2 cm tall, around the pad | outer 26 x 14 | around the pad (spans y 28 to 42) | fixed |

From right to left: the iron stands between the arms, 3 cm to the right of the shirt's edge; the shirt lies
on the board's left end; the rest is 2 cm past the board's end, in front of the left arm. The shirt is between
the iron and the rest, so the iron has to be carried over it.

## Scoring

Scored by `success()` and `score()` in the task file, every control step:

- **0** if, at any time, the shirt moves more than 1 cm (any corner of it, from where it started), the iron
  comes within 2 mm of the shirt (touching it), or the iron falls off the table (its lowest point more than
  5 cm below the table top). These are checked first and override everything below.
- **0.25** once the iron's lowest point has been 3 cm or more above the board's top.
- **0.5** once no part of the iron's footprint has been over the board.
- **1.0** when, for 1 s in a row: the iron sits soleplate down within 10 degrees of flat, its footprint fully
  inside the lip's inner edges, on the pad (lowest point within 1 cm of the pad's top) and still (under
  1 cm/s and 0.5 rad/s); and it has been let go, with every fingertip of both grippers at least 3 cm from its
  surface. The trial ends there.

The partial credits are the furthest stage reached during the trial. The touch test is an exact
oriented-box test between the iron and the shirt. The fingertips are the tips of the four finger pads, placed
by forward kinematics on the YAM URDF from the arms' joint positions. Every condition can be checked with a
tape measure, a level and a photo.

**Dependency:** the task-file hooks receive only object states, so the scorer reads the episode clock and the
arms' joint positions from the `yam_arena` embodiment that calls it. It was tested with the yam-isaac-lab-arena
version the launchable pins (commit `0059d02`). If that state can't be found, scoring raises an error rather
than guessing.

## Results

With `-E object_jitter=0`, 20 Hz control and 320 x 176 camera frames:

| Policy | Runs | task_score |
|---|---|---|
| Dream-1 (`dream`) | 1 | 0.0: reached for the iron with the right arm and nudged it several times but never lifted it, 1,200 steps |
| Scripted demo (`yam_scripted`) | 3 | 1.0 each: success after 330 steps (16.5 s) |

The demo grips the iron with the left arm, lifts it, carries it left over the shirt (the soleplate stays at
least 3.6 cm above the fabric), sets it in the rest, lets go and rises clear. Its plan aims higher and 1.2 cm
further out than the geometry alone would, because the arm sags about 2.5 cm under the 1.2 kg load.

## Fidelity

- Rigid bodies only. The shirt is a thin rigid sheet, so it can slide but not crumple or fold.
- The iron is a single rigid box with no handle shape, cord or water tank. The task-file format makes each box
  its own rigid body, so the soleplate and handle can't be separate boxes. Its 1.2 kg is a real steam iron's
  mass, but its size is a travel iron's (20 x 8 x 10 cm against a steam iron's 25 x 11 x 13 cm): a YAM gripper
  opens 9.4 cm, so it grips the 8 cm body directly, where a person would hold the handle. The soleplate is
  the box's underside and isn't drawn in a different colour.
- Heat is not simulated. The "hot" iron is the reason for the rest and the no-touch rule, not a physical
  effect.
- The ironing board is a flat tabletop board lying on the table, its legs omitted.
- Every loose object has rubber-like friction (static 1.2, dynamic 1.0), as all `yam_arena` task objects do.

## Files

- `park_hot_iron.py`: the task (objects, `success()`, `score()`, `distance()` and the scripted `demo()`).
- `task.json`: the task's details for the task board.
