# Shelve a case into a full open-backed shelf

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1).

Shelving whole cases into rows is regular work for store stockers (Stockers and Order Fillers, SOC 53-7065): on
backroom stock shelving, as top-stock above the sales shelves, and on the sales floor of warehouse-club and
limited-assortment stores. Here a low chrome wire shelf with an open back stands on the station's own table, on two
timber battens clamped to it. On its shelf, 33 cm above the
table, stand three 4.4 kg cases of canned soup, C1 to C3. The biggest gap in their row is 16.0 cm, narrower than a case
(23.2 cm). A lighter case, N (1.5 kg of rice, with a hand hole in each long side), stands on the table in front of the
shelf, its far part under it.

The instruction is only: "Put the case from the table on the shelf, in the row with the other cases."

Pushing N straight into the gap drives C1 and C2 back, out of the open back of the shelf, and they fall onto the table
behind it: damaged stock. The way a stocker does it is physical work: slide C1 and C2 sideways first (over the arms'
payload, they can only be slid, at full reach, without driving them back; about 13 N each), then lift N by its hand holes with both
hands, set it on the shelf with its centre just over the edge, and push it into the slot until it is flush.

## Files

| File | What it is |
|---|---|
| `task.py` | the task file: objects, the tracker and the scorer (`score()`, `success()`) |
| `layout.py` | every size and position, and the scoring constants |
| `build_assets.py` | writes `assets/*.usda`, `assets/textures/*.png` and `hand_hole_template.svg` (needs NumPy and Pillow, not the simulator) |
| `scenekit.py` | USD bodies for task files |
| `usdkit.py` | small mesh, material and collider writers used by `build_assets.py` |
| `demo.py`, `demokit.py` | the scripted demo (closed loop) and its runner |
| `gatekit.py` | shared scoring helpers |
| `check_score.py` | offline checks of the scorer on hand-built states |
| `assets/` | `room.usda` (the shelf and its battens), `soup_case.usda`, `new_case.usda`, textures |
| `hand_hole_template.svg` | a 1:1 cutting template for N's hand holes |

## The rig: the standard station

The station as it stands (`SCENE = "station"`): its table, the two arms on their own mounts on the table (0.61 m
apart), the top camera on its stand, the gains and 20 Hz control. The shelf, standing on two 2x4 battens, is the task's
own fixed body on the table (`assets/room.usda`); the four cases are free bodies. yam_arena itself is not changed, and the run command is
the standard one.

## Run a trial

```bash
source <ARENA_DIR>/yam.env
python build_assets.py           # once; the assets are also shipped
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    --instruction "Put the case from the table on the shelf, in the row with the other cases." \
    --scorer task_score --max-steps 3600 --policy ...
```

Nothing is random (`object_jitter=0`). The trial ends once the full-score state has held for 1 s; otherwise it runs to
its 3,600-step limit (3 minutes at 20 Hz, 2.7 times the demo's 1,347 steps). The demo's run took 5.0 minutes of wall
time on one 8 GB GPU with the three cameras at 640 x 360.

## Scripted demo

```bash
python demokit.py task.py demo_out --mode full          # the full-score demo: demo_out/full/demo.mp4, setup.png
python demokit.py task.py demo_out --mode shove,drop    # the failure variants
```

The demo reads object poses from the simulator (privileged) and the arms' own joints. In order:

1. **C1 to the left.** The left hand, closed, level, pointing forward and turned 14° toward C1 (the wrist gives about as
   much under the load), enters the middle of the 16 cm gap at the cases' mid-height (39 cm above the table) and
   pushes C1's side face 5 cm to the left with the side of the hand, closed loop on the face. The hand drifts 1.5 cm
   toward the robot as it pushes, so its friction draws C1 forward, not back. It backs out.
2. **C2 to the right**, the same with the right hand. The slot is now about 26 cm.
3. **Take N.** Each hand points straight inward, jaws part open up and down, turned low and well in front of the shelf
   first. It slides its lower finger 2.5 cm into one of N's hand holes, 3.9 cm toward N's near end from its centre, and
   its upper finger over the box's top, then closes on the wall between them at about 40 N per finger.
4. **Draw it out and lift it.** Up 8 cm, back until N's far end is 3 cm in front of the shelf, up until N's bottom is
   4 cm above the shelf. Throughout, both hands turn against any pitch of N and the hand on its low side rises against
   any roll, read from N's pose, so it stays level.
5. **Carry it in.** Lined up with the slot, N is carried in over the shelf and lowered until the hands are 4 mm short of
   C1's and C2's fronts. N's centre is then just behind the shelf's front edge. It is set down level.
6. **Let go.** The jaws open; the fingers slide out of the holes and off the top; the hands back away.
7. **Push it home.** The right hand, closed, pushes N's front face to the front-edge line (x 42.0), closed loop on the
   face, then backs off.
8. **Clear.** Both arms go back to their ready pose (in joint space) and hold still.

Failure variants (`--mode`): `shove` (no room made; N is carried straight into the 16 cm gap and pushed home with both
hands), `drop` (room made; N let go with its centre in front of the shelf's edge), `nothing` (the arms stay still).

## Scoring

`score()` and `success()` read every control step, from the cases' poses and the hands' collision hulls (the URDF's).

- **Latches, over the whole trial, never cleared:** *dropped* (a case's lowest corner falls 3 cm or more below its last
  supported height while no hand touches it: off the shelf's back onto the table behind it, off its front or ends, or
  out of the hands) scores 0; *out of the back* (any corner of any case at x ≥ 75.6, the shelf's back edge) caps the
  score at 0.25.
- **1.0:** no latch; N on the work shelf in the row (long side within 15° of front to back, front face no more than
  2 cm behind the front-edge line x 42.0), upright (top within 10° of level) and wholly on the shelf; C1 to C3 on the
  shelf and wholly on it; every finger point 3 cm or more from every case; every case under 1 cm/s; held 1 s.
- **0.5:** no latch, N in the row with its centre over the shelf, not on its side, C1 to C3 on the work shelf, but
  short of 1.0: sticking out of the front, tilted 10° to 45°, a hand not clear, a neighbour not wholly on the shelf, or
  a case still moving.
- **0.25:** no drop; either N in the row with its centre over the shelf, not on its side, C1 to C3 on the work shelf,
  but a case went out of the back; or N anywhere other than resting on the work shelf (on the table, or still held,
  even just above the shelf), C1 to C3 on it, none ever out of the back, and a clear gap of 23.8 cm (N's width and
  6 mm) along the front-edge line.
- **0** otherwise, including N on its side or upside down (its "THIS SIDE UP" face more than 45° from up: on its side
  it is 14.3 cm wide and would fit the original 16 cm gap), N on top of a case on the work shelf, or a neighbour taken
  off the work shelf.

Room can be made more than one way. N needs 7.2 cm more than the 16 cm gap. The demo slides both neighbours about 5 cm,
to 26 cm, 1.4 cm on each side of N. One neighbour alone can do it only tightly: C1 slid 8.0 cm left against the posts,
or about 16 cm right against C2, leaves 24.0 cm, 4 mm on each side of N; C2 has only 6.0 cm beyond it (7.3 cm with C3
pushed too). These routes are legitimate, harder, and score the same.

### Results

Scripted runs (GPU physics, the harness's own embodiment):

| Run | Score | What happens |
|---|---|---|
| `full` | 1.0 | C1 and C2 slid 4.9 and 5.0 cm apart (turned 6.4° and 5.3°, none out of the back); N pinched at its hand holes, drawn out from under the shelf, lifted, carried and set down with its centre 1.4 cm over the shelf's edge (tilt 0.4°), let go, pushed home to x 41.94; held 1 s. 1,347 steps (67 s). |
| `shove` | 0 | No room made: N driven into the 16 cm gap pushes C1 and C2 back; C1 out of the back at step 491; pushed home, both go off the back and end on the table behind the shelf (dropped at step 843). |
| `drop` | 0 | Room made, N let go with its centre 5 cm in front of the shelf's edge: it tips off onto the table (dropped). |
| `nothing` | 0 | The arms stay still. |

Offline checks of the scorer on hand-built states (`python check_score.py`), each the expected score:

| State | Score |
|---|---|
| nothing done | 0 |
| room made only, N still on the table | 0.25 |
| C1 alone slid against C2 (24.0 cm at the left posts), or 8.0 cm left against the posts; N still on the table | 0.25 |
| either of those, N then in the 24.0 cm gap, flush | 1.0 |
| room made, N held 3 cm above the work shelf | 0.25 |
| room made, N still on the table, a case went out of the back | 0 |
| N in the slot, flush, upright, hands clear, all still | 1.0 |
| the same, a hand not clear / not still | 0.5 |
| N left 5 cm proud of the front | 0.5 |
| N leaning 30° in the slot | 0.5 |
| N in, C3 3 cm past the shelf's end | 0.5 |
| N stood on its side in the 16 cm gap, no room made | 0 |
| room made, N on its side / upside down in the slot | 0 |
| N in, a case went out of the back and was pulled back | 0.25 |
| N in, C2 left 3 cm out of the back | 0.25 |
| N in, a neighbour pushed off the back onto the table (dropped) | 0 |
| N in (flush or proud), C1 lifted off the shelf onto the table, no drop | 0 |
| N dropped from the hands | 0 |
| N set on the shelf 5 cm behind the front-edge line | 0 |
| N set on top of C1 | 0 |
| N in the slot but turned 20° across the row | 0 |

## Setup (positions in cm from the midpoint between the arm mounts: x forward, y to the robot's left; heights above the table)

| Part | Where and what |
|---|---|
| Rig | the station as it stands: its table (110 x 130, top 0.25 below the arm mounts), the two YAM arms 61 apart on their own mounts, the top camera on its stand, the wrist cameras. Each arm's ready pose is the station's. Nothing is random. |
| Shelf (fixed) | Regency 14" x 42" x 14" NSF Chrome Dunnage Shelf (460EC1442D11): one 14 x 42 in wire shelf on four 14 in posts with feet. Shelf top surface at 33.0, set with the split sleeves (and shims). It spans x 40.0 (front edge) to 75.6 (back edge), y −64.6 to +42.1; posts at x 41.6 and 74.0, y +40.5 and −63.0. The table runs on to x 91, 15.4 behind the shelf's back edge. |
| Battens (fixed) | two 2x4s laid flat (8.9 x 3.8), from x 40.0 to the table's back edge (x 91): the left one centred on y +40.5 under the left feet, the right one from y −65.0 (flush with the table's right edge) to −56.1 under the right feet. Each post is strapped to its batten just above its foot; three 6 in C-clamps hold the battens to the table (the left one at its back end, the right one twice along the table's right edge). |
| C1, C2, C3 | soup cases (Uline S-18385 boxes, outside 29.5 x 23.2 x 14.3, twelve 10.75 oz cans each) on the shelf, long side front to back, fronts at x 42.0 (2.0 behind the shelf's front edge), backs 4.1 inside its back edge; in y C1 +31.2 to +8.0, C2 −8.0 to −31.2, C3 −37.2 to −60.4. Gaps: 8.0 to the left posts, 16.0 between C1 and C2, 6.0 between C2 and C3, 1.3 to the right posts. |
| N | the same box with 1.5 kg of rice in sealed bags and "THIS SIDE UP" arrows, upright on the table, x 30.0 to 59.5, centred on y 0: its far 19.5 under the shelf. A hand hole in each long side panel: 4 x 1-1/4 in, round ends, centred along the panel, top edge 2 in below the box's top. |
| Estimates | each soup case 4.4 kg; friction 0.3 between a case and the wire shelf (so sliding one sideways takes about 13 N), and 0.5 between cases. |

## Real trial

The station stays as it is, and the arms stay on their own mounts. Only two battens are cut.

### Placement and fixing

The left and back feet stand 17 to 25 cm in from the table's edges, beyond a C-clamp's reach, so the unit stands on two
battens that run to the edges, and the clamps hold the battens.

- Cut two 2x4 battens to reach from 40.0 cm in front of the line through the arm mounts' centres to the table's back
  edge (51 cm on the station's table). Lay them flat on the table along x: the right one with its outer face flush with
  the table's right edge, the left one centred 40.5 cm left of the mounts' midpoint.
- Stand the unit on them, square to the arms: its front edge 40.0 cm in front of the mounts' line, its left end 42.1 cm
  left of the mounts' midpoint (the right end is then 64.6 cm right of it, 0.4 cm inside the table's edge).
- Fit the shelf on the posts' split sleeves at the highest groove that leaves its top surface at or below 33.0 cm above
  the table, then bring it to 33.0 cm (within 3 mm, tape measure) with plywood shims under the feet.
- Strap each post to its batten just above its foot with a 1 in two-hole strap and two wood screws.
- Clamp the battens to the table with three 6 in C-clamps: one at the left batten's back end, over the table's back
  edge, and two along the right batten, over the table's right edge. The shelf, posts and battens are then one rigid
  frame, as in the simulation.
- Mark the front-edge line (x 42.0) on the shelf's front bar with tape at each case's front corners, and the shelf's back
  edge with a strip of tape on the posts, for the side view.
- Mark N's outline on the table with tape: x 30.0 to 59.5, centred on y 0.

### The cases

- **C1 to C3:** real 12-can cases of 10.75 oz condensed soup, made up in Uline S-18385 boxes (outside 29.5 x 23.2 x
  14.3 cm).
- **Stand-ins, for trials where cases may fall** (the shove, an untried policy): the same box holding 4.4 kg of sand
  in sealed bags, the bags packed so that they cannot shift. They slide and fall as the soup cases do, and a fall
  costs nothing. In these trials the floor behind the table is kept clear, where a case pushed off the back may land.
- **N:** the same box, loaded with 1.5 kg of rice in sealed bags, "THIS SIDE UP" arrows on its sides. Two hand holes,
  one in each long side panel, cut with a box cutter to `hand_hole_template.svg` (print at 100 %): 4 x 1-1/4 in with
  round ends, centred along the panel, the top edge 2 in below the box's top edge, as ASTM D6804 (Standard Guide for
  Hand Hole Design in Corrugated Boxes) advises for hand-hole placement. Box makers also offer die-cut hand holes as an
  option. The hole size and position are this task's choice under that guide.

### Parts and cost

| Part | Price (USD) |
|---|---|
| Regency 14" x 42" x 14" NSF Chrome Dunnage Shelf, 460EC1442D11 (WebstaurantStore, November 2026) | 34.99 |
| three 6 in C-clamps | 30 |
| one 8 ft 2x4, for the battens | 5 |
| four 1 in two-hole straps and wood screws | 5 |
| Uline S-18385 boxes, a bundle | 20 |
| 36 cans of 10.75 oz condensed soup (three cases) | 55 |
| rice and play sand, in sealed bags | 20 |
| a phone tripod, for the side view | 20 |
| plywood offcuts, for the shims | 5 |

Total (estimate): about $195.

### Scoring views and tools

The station's top camera; a phone on a tripod at the table's side, level with the shelf, looking along its back edge
against the tape there; a tape measure; a phone level app (or a protractor) on N's top.

### Reset

Nothing is consumed. Put C1 to C3 back on the tape marks on the shelf's front bar, and N on its outline on the table,
its hand holes to the sides. A soup case that fell is checked for split seams and dented cans, and replaced if damaged.
N's hand holes are cut once; a box whose hole edges have torn is replaced. About three minutes per reset (estimate).

## Fidelity

- Rigid bodies on GPU PhysX at 240 Hz, 20 Hz control.
- The wire shelf collides as a solid 2.5 cm plate of its outline; its wire deck and trusses are drawn but do not
  collide. Posts are boxes of their diameter; the battens are boxes; the shelf, posts and battens are one fixed body.
- Each soup case is one rigid block of uniform density: its cans cannot shift and its cardboard does not crease.
- N's walls are 4 mm boxes with the two hand holes cut out of the long side walls; its rice is one block below the holes.
  Its mass is spread by the colliders' volumes, so its centre of mass is low, as rice in the bottom of a box is. The
  hole size and position are this task's choice under ASTM D6804.
- The friction values are estimates (setup). The robot's fingers and the table keep the harness's default materials.
- The scorer reads every case's pose each control step: drops from the lowest corner, the back edge from the corners'
  x. Hand contact and "clear" are measured from points on the robot's own collision hulls (the URDF's) to each case's
  box.

## Sources and licences

- U.S. Bureau of Labor Statistics, Occupational Employment and Wages, May 2023, 53-7065 Stockers and Order Fillers
  (employment 2,872,680; general merchandise retailers 889,180; food and beverage retailers 631,160):
  https://www.bls.gov/oes/2023/May/oes537065.htm
- O*NET OnLine, 53-7065.00 Stockers and Order Fillers (task: "Stock shelves, racks, cases, bins, and tables with new or
  transferred merchandise."): https://www.onetonline.org/link/summary/53-7065.00. This page includes information from
  O*NET OnLine by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the
  CC BY 4.0 license. O*NET® is a trademark of USDOL/ETA.
- Back ledges for open wire shelves, sold to keep stock from falling off: Uline, Wire Shelving Side and Back Ledges
  ("Prevent items from falling off the shelf."): https://www.uline.com/BL_4213/Wire-Shelving-Side-and-Back-Ledges;
  Regency 460EC42LDG, the ledge for a 42 in shelf ("helping to contain items on a shelf and prevent them from falling
  off"): https://www.webstaurantstore.com/regency-39-5-8-x-5-15-16-chrome-wire-shelf-ledge-for-42-wire-shelving/460EC42LDG.html
- ASTM D6804, Standard Guide for Hand Hole Design in Corrugated Boxes: https://store.astm.org/d6804-24.html
- Regency 14" x 42" x 14" NSF Chrome Dunnage Shelf, 460EC1442D11 (for identification only):
  https://www.webstaurantstore.com/regency-14-x-42-x-14-nsf-chrome-dunnage-shelf/460EC1442D11.html
- Uline S-18385 corrugated box, outside 11 5/8 x 9 1/8 x 5 5/8 in (for identification only):
  https://www.uline.com/Product/Detail/S-18385/Corrugated-Boxes-200-Test/11-1-4-x-8-3-4-x-5-Corrugated-Boxes
- Case labels: generated in `build_assets.py`; their lettering is rendered with DejaVu Sans (Bitstream Vera / DejaVu
  fonts licence, free to use and redistribute).
- Everything else (shelf, battens, cases) is generated from primitives in `build_assets.py`; no third-party meshes or textures.

## Licence

MIT
