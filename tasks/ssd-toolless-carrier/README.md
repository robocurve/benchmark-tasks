# Load a 2.5-inch SSD into a tool-less drive carrier

Before a spare drive can be hot-swapped into a server, it has to be loaded into the server's drive carrier (the sled
that slides into the drive bay). Most current servers use tool-less carriers: there are no screws; pins on the
carrier's side rails go into the drive's side mounting holes, and one rail flexes outward so the drive can get past
its pins. The robot gets a bare 2.5-inch SSD and an empty carrier and must fit the drive into it, right way round,
with all four pins in its holes and the sprung rail closed, so it is ready to go into the server.

The physical difficulty is the sprung rail. The pins stand 2 mm into the space between the rails, so the drive can't
be pushed straight down into the carrier, and the rail springs shut as soon as it is let go. One hand has to hold the
rail open by its tab while the other sets the 7 mm-thin drive down beside the fixed pins and slides it onto them, its
holes lined up within 0.75 mm along it and 0.75 mm in height; only then can the rail be let back so its pins spring
into the other two holes. Pushing on the carrier drags it (it is loose on the table), and the drive turned end for end
or upside down does not fit.

## Objects

Positions in cm from the midpoint between the two arm bases, on the table (x forward, y left).

- **SSD:** a standard 2.5-inch SATA SSD, aluminium case, 10.0 x 6.985 x 0.7 cm, 55 g, white label on top, black SATA
  connector at one end. Two side mounting holes on each side, 3 mm above the bottom face, 14.0 mm from the front end
  and 9.4 mm from the connector end (76.6 mm apart), as on a real 2.5-inch drive; modelled 4 mm square and 3 mm deep.
  It lies flat on the table, label up, its centre at (30.0, -20.0), long side along y, connector toward the right (-y).
- **Carrier:** a tool-less 2.5-inch drive carrier, steel with a dark plastic handle bezel, 14.0 x 7.6 x 1.5 cm,
  0.2 kg, loose on the table, centred at (30.0, 0.0) with its long side along y: handle bezel (0.8 cm deep, 1.5 cm
  tall, with a blue release handle 0.4 cm in front of it) at the left (+y) end; the drive's place is the rear 10 cm
  (the right end), where it rests on two 4 mm-tall cross ribs with its connector end flush with the carrier's rear edge.
  Two 3 mm side rails stand 0.5 mm clear of the drive's sides, 1.2 cm tall, each only around its pins (a 5.2 cm gap in
  the middle, at the drive's centre); a low 3 mm bar joins each rail along its length.
  - The **fixed rail** (nearer the arms) carries two fixed pins.
  - The **sprung rail** (far side) is a separate 30 g part that slides outward up to 7 mm against a 150 N/m spring (a
    real carrier's rail flexes outward the same way). It carries the other two pins, and a blue pull tab (1.2 cm wide,
    0.25 cm thick, standing from 0.8 to 3.4 cm above the table) on a bracket 8 mm outboard of the rail's front end.
  - **Pins:** 2.5 mm square, 7 mm above the table (in line with the drive's holes when it rests on the ribs), 14.0 mm
    and 90.6 mm from the drive's front end, each standing 2 mm into the space between the rails, past the drive's side.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `ssdkit.py` | Rigid parts built from boxes, with spring-loaded sliders, for yam_arena task files |
| `demo.py` | The scripted full-score demo |
| `test_scoring.py` | Offline checks: every scoring outcome, and the geometry of the drive, its holes and the pins (no GPU) |
| `fabrication/` | Real-world build: OpenSCAD parts, `BUILD.md` (materials, printing, assembly, acceptance checks), and a check of every dimension against `task.py` |

`ssdkit.py` hooks `yam_arena.scene` and the embodiment's reset without modifying yam_arena, so the task runs with the
standard `yam_arena` embodiment and `task_score` scorer. It follows the part kit used by other tasks in this
repository.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/ssd-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1800     # writes demo_*_cam.mp4, setup.png, result.json
python -m pytest test_scoring.py fabrication/test_fabrication.py   # offline checks
```

1. The left arm pinches the sprung rail's pull tab (its hand leaning 30 degrees to the left, so the two wrists stay
   clear of each other), draws the rail about 6 mm open and holds it. At the same time the right arm pinches the SSD
   across its width and lifts it.
2. The right arm carries the drive over its place and lowers it to just above the ribs, 3 mm toward the open rail, so
   its holes clear the fixed pins; slides it across onto the fixed pins; sets it down on the ribs and lets go.
3. The left arm lets the rail back slowly: its pins spring into the drive's other two holes. It lets go.

The demo is open loop: its targets come from the published layout (`task.py` constants), and it reads no object
poses while it runs; the arms' lag and how the drive settles in the fingers were measured once and are set as
calibration constants in `demo.py`. It reaches 1.0 (success) at step 417 (21 s).

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). Positions are
measured relative to the carrier, from where a seated drive sits (on the ribs, centred between the rails, connector
end flush with the rear edge).

- Only the drive's state at the time is judged: nothing earlier in the episode (turning it the wrong way on the way in,
  or letting it drop) counts against it.
- **1.0 (success):** all four pins in the drive's holes, the sprung rail closed (within 1 mm of its rest position),
  and the drive lying flat in the carrier (right way up with its connector toward the rear, its centre within 5 mm of
  its seated position in top view, every bottom corner within 1 mm of the seated height; the same below), the drive
  having moved less than 1 mm over the last 1 s. The episode ends once this has held for 1 s.
- **0.75:** all four pins in and the drive lying flat in the carrier, but not yet still for 1 s or the rail not closed.
- **0.5:** both pins of one side in their holes, the drive lying flat in the carrier.
- **0.25:** the drive lying flat in the carrier, no side's two pins both in.
- Otherwise 0.

A pin counts as in its hole when its tip is inside the hole (within 2 mm of the hole's centre along the drive and in
height) and at least 1 mm past the drive's side. With a ruler: the drive sits flat on the ribs, all four pins can be
seen in the holes, and the rail stands against the drive's side.

## Runtime and seeds

About 1.5 minutes per trial for the full 1,800 steps on an L40S, plus about 1 minute for Isaac Sim to start. The
layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

Rigid bodies built from boxes. The SSD has a real 2.5-inch drive's size, mass and hole positions; its holes are
modelled 4 mm square and 3 mm deep and the pins 2.5 mm square (a real drive's holes are M3, 3 mm round, and a real
carrier's pins about 2.8 mm round), which leaves 0.75 mm of clearance round each pin, enough for the simulator's
contact handling. The sprung rail is one part on a prismatic joint with a spring drive (150 N/m, 7 mm travel): a real
rail is part of the carrier and flexes outward, with a few newtons of force; here it is softer so that pulling it does
not drag the 0.2 kg carrier, which is loose on the table (friction 0.8). The rails stand only around their pins, with
a gap at the drive's centre for fingers, as on many tool-less carriers. The SATA connector is not mated (the carrier
is loaded on the bench, not in a server). Light angular damping is added to the parts. No contact force cap is
enforced in simulation.

## Real-world build

`fabrication/BUILD.md` has the complete build: OpenSCAD sources for the printed carrier frame (with its guide-rod
mounts), the sprung rail with its pull tab, and a 2.5-inch SSD dummy with the task's 4 mm side holes. It also has a
bill of materials, print settings, assembly steps and acceptance checks. The sprung rail runs on two 3 x 26 mm ground
steel rods, each with a 0.3 mm-wire, 5 mm OD compression spring (0.075 N/mm each, 0.15 N/mm the pair). It stops at
7 mm of travel, and steel 2.5 x 6 mm dowel pins stand 2 mm past the drive's side. Every dimension is in
`fabrication/params.scad`, and `fabrication/check_params.py` checks it against `task.py`. The carrier stays loose on
the table; mark the outlines of the drive and the carrier for resets.

## Assets

All geometry is defined in `task.py` from boxes; no external assets.
