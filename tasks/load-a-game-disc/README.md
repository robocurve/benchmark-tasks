# Take a game disc out of its case and load it into a slot-loading console

A games console stands upright on the table, with a slot-loading disc drive whose slot is in its front face. Beside
it, a game case lies open on a storage box with the game's disc on its hub. The robot has to take the disc out of
the case and put it into the console.

**Instruction:** "Take the game disc out of its case and put it into the console."

Two real mechanisms make this a two-handed, millimetre-precise job:

- **The case's hub.** A game case holds its disc on a hub with spring teeth that grip the edge of the disc's centre
  hole. The disc comes off only while the hub's centre button is pressed, which draws the teeth in. Lifting the disc
  without pressing the button just lifts the whole case with it (and with a real disc, risks cracking it). So one
  hand presses the button while the other lifts the disc.
- **The console's slot.** The slot is 5 mm wide behind a short flared mouth, for a disc 1.2 mm thick and 120 mm
  across. The disc has to go in edge first, standing upright, square to the slot and centred in it, until the drive
  takes it (7 cm in) and draws it the rest of the way.

The disc is wider (120 mm) than the gripper opens (94 mm) and only 1.2 mm thick, so it can only be held by its edge,
which the case's thumb cut-out leaves free, and it has to be turned from lying flat to standing on edge on the way.

## Where this comes up

- **Home entertainment.** Loading a game, film or music disc is an everyday task in homes with games consoles,
  Blu-ray players and car stereos with slot-loading drives.
- **Care and assistance.** People with limited hand function find the press-the-hub-and-lift step and lining a disc
  up with a slot difficult; it is a typical small request for a home-assistance robot.
- **Media handling in offices and libraries.** Libraries lend discs in the same cases, and archives load discs into
  slot and tray drives for reading.

## Potential impact

- **Two hands with different jobs on one small object.** One hand presses a 12 mm button on the hub while the other
  lifts a 1.2 mm disc by its edge a few centimetres away.
- **Handling a thin, wide, fragile object by its edge,** and turning it from flat to upright in the fingers.
- **Millimetre alignment for insertion.** The slot leaves under 2 mm either side of the disc; a disc a few degrees
  off square catches the console's face.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: scene, the hub's teeth, the console's drive, scoring, the instruction |
| `disckit.py` | Builds the disc, the case (an articulation: case, hub button on a sprung slide, three teeth on radial slides) and the fixed parts |
| `demo.py` | The scripted demo, and a control run (lifting without pressing the hub) |
| `test_scoring.py` | Offline tests of the scoring rule and the disc geometry (no simulator) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the game disc out of its case and put it into the console." \
  --scorer task_score --max-steps 2400
```

Run limit: 2,400 steps (2 minutes at 20 Hz). Second layout: set `DISC_LOT=B` in the environment (default `A`).

## Scripted demo

```bash
python demo.py task.py out_dir                  # layout A; DISC_LOT=B for layout B
python demo.py task.py out_dir --mode nopress   # control: lift the disc without pressing the hub
```

1. The left hand comes straight down on the hub, fingers closed, and presses its centre button 2.5 mm: the teeth draw
   in.
2. The right hand, level and pointing forward with its fingers one above the other, reaches through the case's thumb
   cut-out and pinches the disc's edge (18 mm of it), then lifts the disc 1.5 cm straight up off the hub. The left
   hand lets go of the button.
3. The right hand carries the disc, still flat, in front of the console and turns its wrist a quarter turn: the disc
   stands upright, square to the slot.
4. It slides the disc straight into the slot. Once 7 cm is in, the drive takes it; the hand lets go and backs away,
   and the drive draws the disc in.

The script is open loop: every pose comes from the layout constants in `task.py`, and object states are read only for
the result file.

## Scoring

Tools: the trial video and a ruler.

- **0 for the whole trial** if at any time the disc touches the table (dropped), outside the console.
- Otherwise:
  - **1.0** once the drive has taken the disc: it went in upright (within 10 degrees), centred in the slot (within
    2.5 mm across it and 12 mm up or down) and 7 cm deep, and was drawn in;
  - **0.5** if the disc went at least 2 cm into the slot;
  - **0.25** if the disc came off the hub (its centre 1 cm or more from its place on the hub);
  - **0** otherwise.

In code: `_watch()` (the teeth, the drive and what happened during the trial) and `score()` in `task.py`. The teeth
draw in while the button is pressed 1.5 mm or more.

## Verification

| Run | Score | Notes |
|---|---|---|
| Scripted demo, layout A | 1.0 | Disc drawn in by the drive at step 475 (24 s) |
| Scripted demo, layout B | 1.0 | Same script; step 476 |
| Control: lift without pressing the hub | 0.0 | The teeth held the disc on the hub; it never came off |
| Dream-1, 2,400 steps | 0.0 | Both arms stayed near their bases, grippers half open; neither reached the case |
| Arms still | 0.0 | The disc stays on its hub |
| `pytest test_scoring.py` | 6 passed | Score tiers, the drop gate, the disc's geometry (flat and upright in the slot), and constants |

## Runtime and seeds

About 3 minutes per demo run and 8 minutes per Dream-1 run on one L40S, including about a minute of start-up.
`object_jitter=0`; the layout is fixed per lot (A or B), and there is no randomness in the task.

## Fidelity

- The disc (120 mm, 1.2 mm, 15 mm centre hole, 16 g) and the case (135 x 171 mm tray half and cover half, 70 g) are
  rigid, with real dimensions. The hub's teeth are three small blocks on radial slides, moved in by the task file
  while the button is pressed 1.5 mm or more; the button is a sprung slide (600 N/m, 2.5 mm travel).
- The console's drive is not modelled mechanically: once the disc is 7 cm into the slot, upright and centred, the
  task file draws it in at 8 cm/s and holds it. The console is fixed to the table.
- Scratches and fingerprints on the disc's data side are not judged.

## Real-world build

A standard Blu-ray-style game case (135 x 171 x 14 mm closed) with a press-to-release hub, opened flat on a storage
box 8.5 cm tall, its near edge overhanging the box by 3 cm, with a thumb cut-out (44 x 32 mm) in the tray's near
edge; a 120 mm game disc on the hub. A slot-loading games console standing upright, its slot 10 cm above the table,
at the positions in `task.json`. Check the disc for scratches after each trial.

## Third-party assets

None: every object is built from primitive shapes in `disckit.py` and `task.py`. No brand, logo or product name is
modelled.
