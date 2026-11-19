# ceiling-tile-lay-in

**Instruction:** "Put the ceiling tile back into its opening in the grid."

A 2 ft x 2 ft lay-in tile of a suspended T-bar ceiling has been pushed up out of its bay and left lying askew
(turned 12 deg) across the tops of the four tees round its own opening, as it is left after plenum access. The robot
re-lays it from below, hands up in the opening, the way it is done on the job: turn the tile back square on the narrow
tee tops with the fingertips under it, centre it over the opening, let it down 38 mm between the tee webs and set it
flat on the 24 mm flanges, without knocking the loose neighbouring tiles.

General maintenance and repair workers (about 1.5 million in the US), electricians (about 0.8 million) and HVAC,
fire-alarm and low-voltage technicians open and close suspended ceilings on most service calls in offices, schools,
shops and hospitals. What makes it hard:

- the tile only has about 2 mm to spare a side between the tees' bulbs, and must come down within about 0.3 deg of
  square and 2 mm of centre, or a corner lands on a tee top and it lies cocked;
- it overlaps each flange by under 9 mm, so a tile set down off centre drops through;
- it is 0.6 m across and rests on four narrow steel edges: the fingertips can only turn it by friction from below
  while carrying part of its weight, and one hand alone just spins or tips it, so both hands work together, one
  each side;
- the hands work overhead, under the tile, with the ceiling hiding most of the work from the fixed camera.

## Demo

The scripted demo is closed loop on the tile's measured pose (`DemoController` in `task.py`; a scripted demo may read
simulator state). Each hand is held level with the jaws closed, tipped up 15 deg, so the top of the gripper is a
"fingertip pad" under the tile.

1. Both hands reach in under the tile (0 to 3.5 s), each under a point 200 mm either side of its centre (30 mm toward
   the far side), 50 mm below its underside.
2. They rise to it (1.5 s) and keep it lifted about 3 mm off the tee tops: their height is raised or lowered each step
   from the tile's measured height, so they carry most of its weight.
3. Turning and centring (closed loop): each step the two hand positions are carried round the tile's measured
   centre by up to 0.3 deg and toward the bay centre by up to 1.5 mm, i.e. always a little ahead of where the tile is;
   friction under the tile turns and moves it. This ends when the tile is within 0.3 deg of square and 1.5 mm of
   centre for 6 consecutive steps.
4. Square and centred, the tile is no longer held by the tee tops: its far side drops between the tee webs onto its
   flange while the hands hold the near side; the hands then let it down at 20 mm/s until the near side lands.
5. The hands go straight down and back, out of the opening.

The demo reaches 1.0 in 294 control steps (14.7 s).

## Scoring

| Score | Condition (milestones latch) |
|---|---|
| 0.10 | turned to within 5 deg of square while over the opening (centre within 60 mm of the bay centre) |
| 0.20 | square and centred: within 1 deg of square, centre within 5 mm of the bay centre |
| 0.60 | seated: all four bottom corners within 3 mm of the flange face, bearing at least 4 mm on every flange, at rest (< 1 cm/s) for 1 s |
| 0.35 | instead of seated: at rest in the opening on a flange but cocked (a corner more than 3 mm high or a bearing under 4 mm) |
| +0.04 each | neighbouring tile undisturbed (5 of them, given once seated): never lifted more than 3 mm at a corner, never shifted more than 5 mm |
| +0.20 | clean finish (given once seated): both hands withdrawn (every TCP 50 mm or more below the flange face or 50 mm clear of the tile) for 2 s with the tile still seated |
| 0 | dropped (latches): the tile lands on the bench, or is more than 150 mm below the flange face with no TCP within 100 mm of it for 0.5 s. A tile carried down through the opening and brought back up (e.g. to put it in diagonally) is not dropped. |

Doing nothing scores 0 (the tile starts 12 deg askew on the tee tops). The scorer reads only `root_poses` of the six
tiles and the measured TCPs.

**Real trial:** the 0.1 / 0.2 milestones (tile still on top of the grid) are read from the video from below, against a tape line on the near tee and a pencil mark at the tile's centre: tile edge parallel to the tee within 5 deg / 1 deg, centre mark within 5 mm of the bay-centre mark on a cross-string; a straightedge or
steel rule held across the opening from below shows the tile face flush with the grid face (within 3 mm) on all four
sides; pencil or tape marks on the neighbouring tiles' edges plus the video show any lift or shift; the video shows a
drop and the hands' withdrawal.

## Real parts and build

| Part | Specification |
|---|---|
| Tile (x6) | 2 ft x 2 ft square-edge lay-in mineral-fibre tile, 603 x 603 x 16 mm (23-3/4 x 23-3/4 x 5/8 in), about 1.1 kg, e.g. Armstrong 1810 Cortega. One is the work piece, five fill the neighbouring bays. Mineral fibre sheds dust and its edges wear: replace the work tile every 20 trials, or use a vinyl-faced washable lay-in tile of the same size (e.g. Armstrong Clean Room VL), which does not shed. |
| Grid | 15/16 in exposed-tee suspended-ceiling grid on a 24 in (609.6 mm) module, e.g. Armstrong Prelude XL (main beam 7301, 4 ft cross tee XL7342, 2 ft cross tee XL7328), cut to a section of 2 x 3 bays (1.22 x 1.83 m): 24 mm white flanges, tees 38 mm tall (web plus bulb). |
| Test frame | four 40 x 40 mm aluminium T-slot posts at the section's corners, 0.69 m tall, bolted or clamped to the bench; 40 x 40 mm perimeter rails at their tops; the grid hangs from them on 12 ga hanger wire with its flange face 462 mm above the bench, level within 1 mm. |
| Plenum obstruction | three 1-5/8 in (41 mm) steel strut channels (e.g. Unistrut P1000) across the frame top, their undersides 220 mm above the flange face. |

Positions (x forward from the arm bases, y to the left, z up; bench top 0.7475 m):

- Tee centre lines at x = 0.195, 0.805, 1.415 m and y = -0.914, -0.305, 0.305, 0.914 m. The target bay is the middle
  bay of the near row, centre (0.50, 0.00); clear opening 585.6 mm square between flange edges.
- Frame posts at (0.165, +-0.944) and (1.445, +-0.944); strut channels at x = 0.30, 0.70, 1.11.
- The work tile starts centred at (0.506, -0.004), turned 12 deg counter-clockwise (seen from above), lying across
  the four tee tops (its underside 38 mm above the flange face, 500 mm above the bench).

## Simulation

- PhysX 240 Hz, 20 Hz control. All parts rigid.
- The grid is one fixed body: per tee a 24 x 2 mm flange, a 2 mm web and a 3 x 8 mm bulb, each an analytic box
  collider (`boundingCube` approximation). Cooked convex hulls of the thin static boxes (under about 8 mm) generated no
  contacts at all in this build, so the analytic boxes are used.
- Tiles are 603 x 603 x 16 mm, 1.1 kg rigid boxes; the five neighbours are loose bodies resting on their flanges.
- Friction: tile 1.0 / 0.9 (paper-faced mineral fibre), grid 0.35 / 0.30 with a `min` combine mode so the tile slides
  on the painted-steel tee tops at about 0.35, while against the robot's fingers (PhysX default material 0.5) it
  averages 0.75. (The robot has no rubber pads in the model.)
- Real tiles flex a little and their edges crumble slightly, which makes a real tile more forgiving to drop in than
  the rigid model; the model is the harder case.
- Cameras: the rig's top camera sits just below the ceiling plane, so it sees the bench, the arms and the ceiling
  edge-on (the tile visibly askew, then flush); the wrist cameras see under the tile. The recording-only overview
  camera is re-aimed under the ceiling at the bay (`aim_overview`).

## Assets and licenses

All geometry is generated by this task's code (MIT). No third-party assets.

## Reproduction

```
python3 ceiling-tile-lay-in/run.py --output ../ceiling-tile-lay-in-outputs
```

Runtime per trial: about 30 s of simulated time at most (run limit 800 steps). Fixed layout, no randomisation.
