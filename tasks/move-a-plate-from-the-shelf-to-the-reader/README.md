# Move a plate from the shelf to the reader

A shelf, a flat grey plate reader with a sliding door, and a lidded sample plate. This is everyday instrument tending:
take the plate from the shelf, take its lid off, set it in the reader the right way round and shut the door. The lid and
the corner are what a technician checks before a read. The plate starts the wrong way round, so a straight move loads
it backwards: it has to turn half a turn on the way.

The left arm can do it all. The door starts parked under a hood at the reader's back and is pulled towards the arms.

## Files

| File | What |
|---|---|
| `move_a_plate_from_the_shelf_to_the_reader.py` | The yam_arena task file: objects, `success()`, `score()`, `distance()` and the scripted `demo()` |
| `benchkit.py` | Tools built from several boxes with sliders (the door), and per-step hooks. This copy adds `parts` to `Slider`, for the door's handle and window frame |
| `test_move_a_plate_from_the_shelf_to_the_reader.py` | Offline tests: no GPU needed |

`benchkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=move_a_plate_from_the_shelf_to_the_reader.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the plate from the shelf. Take off its lid and put the lid on the table. Set the plate in the reader with its orange corner at the reader's orange corner, then shut the door." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/plate-reader-dream

# The scripted demo, which is built to reach full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=move_a_plate_from_the_shelf_to_the_reader.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Take the plate from the shelf. Take off its lid and put the lid on the table. Set the plate in the reader with its orange corner at the reader's orange corner, then shut the door." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/plate-reader-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/plate-reader-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial ends at 1.0 once the plate is in, the right way round, the lid is off,
the door is shut, and plate and lid have stood still for 1 s; otherwise it scores the last step.

Offline tests (no GPU), with `yam_arena` importable:

```bash
python -m pytest -q test_move_a_plate_from_the_shelf_to_the_reader.py
```

## Scoring

Scored from the end state, with one latch. The plate is in when all four corners are inside the low walls, it leans
under 10 degrees and it is on the reader's floor, not up on a wall. Its corner is right when its orange corner is
within 2.5 cm of the reader's orange corner, seen from above. The lid is off when it is down on the table or the shelf,
none of it over the plate or the reader. The door is shut when its near edge is within 1 cm of the reader's near edge.

| Outcome | Score |
|---|---|
| Plate in, corner right, lid off, door shut | 1.0 |
| Plate in, corner right, lid off, door not shut | 0.7 |
| Plate in, lid off, corner wrong, door not shut | 0.3 |
| Plate in, lid off, corner wrong, door shut | 0.2 |
| Plate in, corner right, lid not off (still on it, in the reader, or knocked to the floor) | 0.2 |
| Plate in, corner wrong, lid not off | 0.1 |
| Lid off, plate still flat on the shelf, door open or shut | 0.1 |
| Plate anywhere else: on the table, on the reader's top, across the low walls | 0 |
| The plate leaned over 30 degrees at any moment | 0, whatever happens after |
| The reader moved over 3 cm, or was turned | 0 |
| Door shut on an empty reader, lid still on | 0 |
| Nothing done | 0 |

In a real trial a person scores from one top photo and a ruler. The door has a 3 cm window where a plate loaded the
right way shows orange, a plate loaded backwards shows its pale wells and an empty reader shows the dark floor. With the
door shut, slide it back after the photo to check the plate is inside the low walls.

## Design notes

- **The corner has to be earned.** The plate starts turned half a turn from where the reader wants it. A plain pick and
  place seats it perfectly and backwards, which is the real mistake. The demo turns it in the air.
- **Backwards and shut is worse than backwards and open.** Shut, it's ready to read, with every result against the wrong
  well. Left open, a person sees it first. So 0.2 sits below 0.3, and both sit well below a right plate left open.
- **The source behind it.** No regulation is cited, because this is an ordinary job step. A [Varioskan LUX reader SOP](https://dnas.dukekunshan.edu.cn/wp-content/uploads/2024/01/MANUAL_ThermoFisher-Varioskan-LUX.pdf)
  says to place the plate "with correct orientation and MUST remove the cover lid". A
  [V-max reader guide](https://msuweb.montclair.edu/~olsenk/vmax.html) puts the A1 well in the holder's left, rear
  corner, which is where the reader's orange corner is here. Not every reader needs the lid off: this one does.
- **A standard plate, gripped above its skirt.** The footprint is the
  [ANSI SLAS 1-2004](https://en.wikipedia.org/wiki/Microplate) one, 127.76 x 85.48 mm, here 12.76 x 8.55 cm, but that
  leaves only 4 mm either side of the 9.4 cm grip, too tight for open-loop placement. So the full footprint is a 5 mm
  skirt, and the body above it and the lid are 7 cm across, with 12 mm either side. The pad tips stop 5 mm above the
  skirt. This narrows the body more than a real plate's, and the fidelity note says so. A tab on the plate wouldn't
  work, because the lid sits over it, and a grip across the other side won't fit, at 12.76 cm.
- **The half turn only goes one way.** The fingers take the long sides at a quarter turn (+90 degrees) on the shelf,
  turn through 0 and end at -90 at the reader. Wrist joint 6 runs -1.55 to 1.09 rad of its +-2.09. Starting from -90
  can't work, because +90 at the reader is about 10 degrees past the wrist's stop. A straight move at -90 is in reach,
  so loading it backwards is a real mistake.
- **The door comes towards the arms.** The door runs front to back and is hooked and pulled, so it doesn't read as the
  sideways lid pushes of other bench tasks. It parks under a hood, which stops short so the fingers fit behind the handle.
- **Unintended strategies.**
  - Tip the plate to shed the lid, or turn it over to swap its corners: 0, latched at 30 degrees.
  - Push the lid half off: not off, since part of it is still over the plate.
  - Put the lid in the reader, or prop it against the reader: not off.
  - Turn the reader round instead of the plate, or shove it: 0, each end must stay within 3 cm.
  - Shut the door first: nothing for the door until the plate is in, and it blocks the opening.
  - Shut the door with the lid on: the tab stops the door well short.
  - Leave the plate on the wall or at an angle: all four corners must be inside and the plate flat on the floor.
- **What isn't modelled.** There are no wells or liquid and nothing is read. Spilling is judged only by the 30 degree
  lean. The window is an open gap in the door, not glass. The lid is opaque in the simulator, so the orange corner shows
  once the lid is off.

## Runtime and seeds

- The scripted demo takes about 58 s of simulated time (1,154 steps at 20 Hz).
- Dream-1 runs the full 2,400 steps unless it finishes first (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
