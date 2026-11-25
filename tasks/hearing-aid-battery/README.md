# Change a hearing aid's zinc-air battery with a battery magnet

Two YAM arms change the size 13 zinc-air battery of a behind-the-ear (BTE) hearing aid lying on a silicone mat, the way a
carer does it for an older person. One hand holds the aid. The other works a standard hearing-aid battery magnet: it swings the
battery door open by its pull tab, lifts the dead cell out of the door's cradle and slides it off into a used-battery
cup. The first hand peels the new cell's long pull tab (the cell stays in its dial pack) and drops the tab in the cup.
The magnet then lifts the new cell by its bare + face, lowers it into the cradle + side up and slides off it sideways,
and pushes the door shut past its detent until it clicks. Then both hands let go. No finger may touch a cell.

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena),
Isaac Lab-Arena 0.3.1).

## Run a trial

Any policy, here Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')" \
  --scorer task_score --max-steps 4800 --store-frames --log-dir runs/dream
```

Scripted demo (scores 1.0), the same command through Inspect Robots' CLI with the demo registered as a policy:

```bash
python run_demo.py runs/demo                    # 640 x 360; --width 320 --height 176 for Dream-1's size
TASK_DEMO=tab python run_demo.py runs/tab       # other modes: see below
```

The run prints `task_score` at the end. `TASK_CLOSEUP=<folder>` also saves a demo-only close-up of the door (`cam.py`;
not a policy camera).

**The demo is a scripted reference that reads the poses of visible objects.** Each control step it reads, from the
simulator, the poses of the aid, its door, the two cells, the tab and the magnet (all in view of the cameras), and its
own arms' joints. It never reads the scorer, the detent's state, the magnet's force or the tab's bond. A policy gets
only the cameras and the joints. `demo.py` holds the demo; `demokit.py` its arm primitives.

## How it is scored

`rules.py` is the scorer, a tracker fed once per control step (`task.py` feeds it) with what the overhead video and a
look at the door would show. Tiers exactly as in `task.json`'s `scoring`:

- **1.0**, with the hands clear for 1 s: the old cell lies in the cup; the tab is off the new cell and in the cup; the new cell lies seated
  in the door's cradle with its + face toward the door's + mark (in the simulation: its axis within 20 degrees of the
  door's, its centre within 1.5 mm of the cradle's axis and 1 mm of the seated height); the door is clicked shut (under
  2 degrees and latched by the detent); the aid is let go: every part of both grippers and the whole magnet at least 3 cm
  from the aid and its door.
- **0.5**: the same, but the door shut to within 20 degrees and not clicked.
- **0.25**: the old cell and the tab in the cup, the new cell seated + side up with its tab off, the door open more than
  20 degrees.
- **0.1**: the door has been opened more than 20 degrees and the old cell is in the cup.
- Anything else: 0.
- **0**, always: a finger touched a cell (a finger box of either gripper within 0.5 mm of a cell's surface, at any step:
  cells are handled only with the magnet); a cell was dropped (any cell landing on the mat or the table, or below the
  table, anywhere but in the cup); at the end, the new cell in the door upside down, or with its tab on; the aid pushed
  off the mat;
  the aid held or touched at the end (the hands-clear check is part of every tier above 0).

Failures latch for the trial: a touch or a drop is never forgiven by a later state.

### Scoring a real trial

- **Video.** A phone 50 cm over the mat on its 3x camera films the trial at 1080p (the mat about 30 cm across the frame,
  about 0.3 mm per pixel; the 7.9 mm cell is about 28 px). It shows any finger on a cell, any cell falling, the tab and
  cells landing in the cup, and the hands let go at the end.
- **The door.** After the 1 s hold: the door is flush with the housing; then hold the aid door-down over the mat and shake
  it 3 times through 5 cm: a clicked door stays shut, an unclicked one swings open. Open it and photograph the cell beside
  the door's + mark: the + face is the flat face with the air hole; the - side is the narrower raised cap.
- **Optional function check:** close the door, wait 2 minutes (zinc-air cells need air), cup the aid in a hand: it
  whistles.

## Scripted failures (scorer checks)

| `TASK_DEMO` | what the demo does | score |
|---|---|---|
| `full` | the whole change | 1.0 |
| `tab` | skips the peel: the new cell goes in with its tab on (its tail stops the door at about 37 degrees) | 0 |
| `door` | pushes the door only to about 10 degrees, not past the click | 0.5 |
| `drop` | lets the new cell down onto the mat beside the door, then seats it as usual | 0 |
| `flip` | the scene starts with the new cell + side down (its tab already in the cup); the demo seats it as it lies | 0 |

Offline checks of the rules on hand-built trials, and of the magnet and the detent, with no simulator:
`python -m unittest test_rules test_task`.

## Fidelity

- Rigid bodies at the harness's 240 Hz; nothing in the harness is changed.
- **Door detent.** The door is a body on a PhysX revolute joint (`bodykit.Hinge`, 0 to 95 degrees) whose spring drive the
  detent rule in `task.py` switches each control step. Latched: a spring of 0.0026 N m per degree toward shut, so about
  0.75 N at the pull tab pushes it the 3 degrees that unlatch it. Between 6 and 2 degrees while closing: a ramp spring
  (0.002 N m per degree toward 6 degrees, about 0.75 N to push through); under 2 degrees it latches again: the click.
  Beyond 6 degrees the pin's friction holds the door where it is left (a spring toward a resting angle that trails the
  door by at most 2 degrees, so about 0.3 N at the pull tab moves it). A real detent is a snap of the door's plastic; the
  forces are nominal, to be measured on the real aid.
- **Magnet.** A hold, not a field. A cell face that comes within 1.5 mm under the tip's 5 mm face, with most of the tip's
  face over it, is drawn up and kept face to face at that place on the tip by a force and torque set every physics step
  (240 Hz) from the bodies' states, at most 0.25 N in all (a small NdFeB magnet's pull on a steel can, about 30 times a
  real cell's weight) and at most 0.15 N across the face (its friction). When more is needed (a wall stops the cell while
  the tip slides on, or it is pulled off), the cell is let go, and it is taken again only after the tip has left it. The
  same force acts back on the magnet. It acts on either face of a cell (the + face, or the smaller - cap), and on nothing
  else; a real magnet also draws a cell from a few millimetres further away.
- **Tab.** The tab is a body joined to the new cell by a PhysX fixed joint (`bodykit.Bond`) that breaks for good at 0.4 N
  or 3 mN m: the tab's peel force as a single threshold (no gradual peel front). Pulled steeply, the joint carries the cell
  out of its pocket with the tab; pulled low across the cell, the pocket's wall holds the cell and the tab comes off. Its
  film is drawn 0.15 mm thick; its collider is 0.8 mm (tail) and 0.3 mm (disc); its mass is rounded up to 0.5 g.
- **Cells:** rigid, 3 g in the simulation (a real size 13 is 0.8 g; rounded up so the solver handles a cell between the
  tip and a wall), their rolling damped as on a silicone mat. The can and the - cap are separate colliders, so a cell
  sits on the cradle's lip with its cap through the hole only + side up. The cells and the tab separate from an overlap at no more than 0.05 m/s
  (PhysX's maximum depenetration velocity), so a stiff arm pressing one into a wall does not fling it.
- **Clearances (assumed, to measure):** the cradle leaves the 7.9 mm cell 0.85 mm of play all round and its wall stands
  3 mm; the dial pack's pocket is 9.4 mm across and 2 mm deep.
- The aid (6 g) and its door (2 g, rounded up from about 1 g) are rigid; the mat is rigid with high friction (1.0). The
  cup, the dial pack and the holder are fixed (taped down).

## Runtime and seeds

- Demo: about 2,220 control steps (111 s simulated); the full state holds from about 110 s, and the run ends a second later.
  About 3.5 control steps per second of wall time on one RTX 4090, so a recorded demo takes about 11 minutes.
- The run limit is 4,800 steps (4 minutes at 20 Hz), about twice the demo, which leaves a policy room to retry a pick.
- `object_jitter=0` keeps the layout fixed, so no seed is needed; the scripted runs repeat exactly.

## Files

- `task.py`: the scene, the detent, the magnet, the per-step tracker, `score()` and `success()`, and the demo's entry point.
- `rules.py`, `test_rules.py`: the scoring rules and their offline tests. `test_task.py`: offline tests of the magnet
  and the detent.
- `parts.py`: every part as plain shapes and simple colliders, with the sizes below.
- `bodykit.py`: multi-shape rigid bodies, joints and the hooks into the harness (no harness file is edited).
- `demo.py`, `demokit.py`, `run_demo.py`: the scripted demo. `gripgeo.py`: the gripper's collision boxes, for the
  finger-touch and hands-clear checks. `steptime.py`: step timing. `cam.py`: the demo video's close-up (only with
  `TASK_CLOSEUP` set; not a policy camera).

## Parts and sources

Every mesh is generated by `parts.py` from plain shapes; there are no third-party assets. Sizes:

- **Hearing aid:** Sharper Image Digital Behind-the-Ear Hearing Aid (OTC, size A13 zinc-air cell, swing-out battery door),
  or any BTE with a size 13 swing-out door. Modelled: body 36 x 13 x 11 mm, 6 g, with its ear hook (drawn only); door
  hinged on a pin at one corner of its battery end, a cradle 9.6 mm across (0.6 mm wall) over a 0.6 mm lip with a 6.4 mm
  hole, a 5 x 1.5 x 3.5 mm pull tab at its free corner, a + mark. The housing's shell over the cavity leaves 0.4 mm over
  a correctly seated cell. Sizes and forces are
  assumptions to measure and record on the real aid.
- **Cells:** size 13 (IEC PR48) zinc-air, 7.9 mm across and 5.4 mm thick (can 4.7 mm, - cap 5.6 mm across and 0.7 mm),
  0.8 g; Duracell Activair EasyTab size 13, whose long orange pull tab (here 6 x 14 mm, folded up) serves as the handle.
- **Dial pack:** the cells' own pack, about 48 mm across and 5 mm thick, turned to its open pocket (9.4 mm, 2 mm deep).
- **Battery magnet:** a standard hearing-aid battery magnet with brush: an 8 mm flat-sided shaft, the magnet (5 mm face) at
  the end of a 12 mm neck, 81 mm from the face to the brush. Holder: a 40 x 40 x 25 mm block with a 9 mm hole 20 mm deep (any pen block).
- **Cup:** a 30 mL (1 oz) plastic medicine cup, modelled 40 mm across and 30 mm tall.
- **Mat:** grey silicone, 240 x 300 x 3 mm.

## Licence

MIT
