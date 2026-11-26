# Core a trimmed cabbage half: cut along both sides of the core to its tip, pull the core out, leave the leaves stacked

**Instruction:** Cut the pale core out of the cabbage half with the knife, set the core on the board, put the knife
down, and leave the green leaves stacked in place.

A trimmed half of a green cabbage lies cut face up on a cutting board. Its pale core is a triangle at the root end,
with green leaf layers on both sides of it and in front of it. The robot picks up a boning knife by its handle,
cuts down along both sides of the core's outline so the two cuts meet at the core's tip, puts the knife down, and
draws the freed core out through the root end, leaving the leaves stacked where they were.

## Why this task

- O*NET 35-2021.00 Food Preparation Workers, core task: "Wash, peel, and cut various foods, such as fruits and
  vegetables, to prepare for cooking or serving." Cutting the core out of cabbage before slicing it is routine in
  kitchens, delis and salad and coleslaw prep.
- The hard part is aiming a free knife at an outline that shows only as a colour change on the cut face: cutting
  wide wastes leaves, cutting narrow leaves hard core behind, and the two cuts have to meet at the tip. Then the
  freed core has to be drawn out without dragging the loose leaf layers with it.

## Scene (true size, one fixed starting layout per lot)

| Item | What it is | Size and source |
|---|---|---|
| Cutting board | OXO Good Grips Carving & Cutting Board 11272900, polypropylene, non-slip feet | 535.9 x 368.3 mm (maker); 20 mm thick (estimate); fixed, centre (0.52, 0) m, top at z = 0.7675 m |
| Knife | Mercer Culinary Millennia M23850, 6 in (152 mm) flexible boning knife, high-carbon steel blade, Santoprene/polypropylene handle | blade 152 mm long (maker), 21 mm wide, 1.8 mm thick; handle 128 x 30 x 30 mm; 140 g in all (handle size and mass estimated from maker photos) |
| Cabbage half | Green (white) head cabbage, Brassica oleracea var. capitata, halved through the stem and trimmed to a centre strip (see "Real trial") | 140 x 76 x 60 mm, 300 g: three stacked leaf layers of 85 g around a 45 g pale core; narrower toward both ends |
| Core | Pale, dense stem tissue at the root end | outline a triangle 50 mm wide at the root face and 40 mm long to its tip; modelled through the full height |
| Ruler | Steel rule with 10 mm ticks, for photos | 200 x 12 x 2 mm, lying flat near the board's left edge |

Coordinates are metres in the station frame (x away from the robot, y to its left). The board top is the only
support. The root end points toward the robot. The leaves start 10 mm in from the root face, so the core's base is
exposed there, as on a real stem end.

- **Lot A:** cabbage centre (0.330, 0.070), square to the board; knife handle centre (0.330, -0.135, 0.7835), blade
  pointing +x, lying flat.
- **Lot B:** cabbage centre (0.320, 0.075), turned 15 degrees counter-clockwise; knife handle centre
  (0.330, -0.100, 0.7835), blade pointing +x.
- Ruler at (0.35, 0.175) in both lots. No jitter, no seeds. The screenshot shows lot A.

Drawings: `fixture/GEOMETRY.md` (cabbage, knife, ruler) and `fixture/root-grasp.svg` (root-end grasp).

## Score

**Score 0 for the trial, whatever happens later, if at any moment:**
- the knife blade touches either gripper;
- the blade comes within 30 mm of the gripper that is not holding the knife by its handle;
- the core is moved more than 15 mm relative to the leaves before both sides of its outline are cut;
- the knife cuts more than 25 mm into the leaves beyond the core's outline.

**Otherwise it is judged at the end, when both grippers are at least 5 mm clear of the cabbage and the knife; the
knife, the core and the bottom leaf layer rest on the board; and nothing has moved more than 1 mm for 1 s. If these
end conditions are not met, the score is 0.**

| Score | Condition | How it is read in a real trial |
|---|---|---|
| 1.0 | The core is out, at least 10 mm from every leaf; every leaf layer is within 10 mm of where it started; the cuts ran at most 5 mm outside the outline into the leaves; no piece of core longer than 10 mm is left on the leaves | top photos beside the ruler before and after; ruler on the removed piece and on the leaves (steps below) |
| 0.67 | As 1.0, but the cuts ran up to 10 mm into the leaves, or a piece of core longer than 10 mm is left on the leaves | same photos and ruler |
| 0.33 | At least one knife cut along the outline (from 2 mm inside it to 10 mm outside it) at least 20 mm long and 5 mm deep, but no higher score. This is also the most a trial earns if a leaf layer ends more than 10 mm from where it started | video, stepped frame by frame; ruler on the cut |
| 0 | Anything else | |

Only the highest level reached counts. Lots A and B are averaged.

A cut counts only when the knife is held by its handle and its edge leads: drawn along its length or pushed edge
first, within 45 degrees of the blade's own plane (a draw may lean up to about 12 degrees toward the spine). Pushing
the flat of the blade or its spine through the cabbage does not cut.

**How the simulation scores it (`task.py`, `fixture/cut_geometry.py`).** `observe()` runs every control step. Each
side of the outline is a grid of points 2 mm apart (24 along the side by 29 up its height). A point is cut when the
edge of the held, moving knife sweeps through it anywhere from 2 mm inside the outline to 10 mm outside it, and the
edge's motion over the last 0.5 s passes the edge-first test above (`edge_leads()`). When 98% of both grids are cut,
the fixed joint holding the core to the leaves is released. How far the cuts ran into the leaves is the farthest any
point of the swept edge reached inside a leaf layer, measured from the outline, sampled every 1 mm along every step
of the trial, including slits beyond the tip and cuts made by moving the cabbage past the blade. The simulation adds
small margins for its point sampling: 2.2 mm on the hand distances (so 32.2 mm and 7.2 mm are used for 30 and 5 mm),
3 mm for the holding gripper touching the blade, 1.8 mm on the 10 mm core gap, 0.9 mm (half the blade thickness) on the depth into the leaves, and 2.8 mm (one grid
cell) on the length of uncut core left. The hold test uses the grippers' collision-hull faces sampled every 3 mm.
`score()` returns the level; `success()` is score 1.0.

## Real trial

**Cabbage.** One fresh green head cabbage per trial (smooth, round "green" or "white" cabbage sold whole by weight
in supermarkets): 1.5 to 2.5 kg, 160 to 190 mm across, firm, not split or wilted. Not red, Savoy or pointed cabbage.
Keep at 2 to 5 C and use within 30 minutes of taking it out.

**Trimming the half (all cuts with a chef's knife on a second board; about 3 minutes).**
1. Peel off loose outer leaves.
2. Halve the head lengthwise through the middle of the stem.
3. Lay one half cut face up. Cut a 76 mm-wide strip from its middle with two lengthwise cuts 38 mm either side of
   the stem's centre line, parallel to it.
4. Turn the strip on its side and slice off its rounded back so it is 60 mm thick, measured from the cut face.
   Trim the far (non-stem) end so it is 140 mm long. Leave the stem end as it is.
5. Weigh it (kitchen scale, 1 g) and keep strips of 250 to 350 g whose pale core triangle on the cut face is 35 to
   55 mm wide at the stem end and 30 to 60 mm long; otherwise use the other half or a new head. Record its mass,
   size and the core triangle's width and length.

**Before the run.** Lay the strip cut face up on the board in the lot's position, stem end toward the robot, with
the knife and ruler as listed. Photograph the top straight down with the ruler in frame and mark the pale
triangle's outline on the photo. Fit clean, food-safe, cut-resistant covers (EN 388 cut level C or higher) over
the gripper pads; their thickness counts in the grasp width.

**After the run (about 5 minutes).** Do not touch anything until the video has ended. Then:
1. Photograph the top straight down with the ruler. Every leaf edge visible in the photo must be within 10 mm of
   where it was in the start photo.
2. Measure the gap between the removed core and the nearest leaf (10 mm).
3. In a top view of the removed piece, measure the widest green strip between the pale core's edge and each cut
   edge (5 or 10 mm). Also measure how far any knife slit left in the leaves runs out from the outline (the same
   limits).
4. Look along both cut faces left on the leaves: pale core tissue more than 2 mm thick still attached there is
   core left behind; measure its longest piece (10 mm).
5. Weigh the removed piece and the leaves separately and record both next to the start mass.

**Reset.** A new strip, a clean board, the knife washed by hand and dried as the maker's care note says
("carefully wash cutlery by hand with warm water and mild- to medium-strength soap"), new pad covers. Every piece
of cabbage is collected and thrown away.

**Safety.** The knife is a real sharp knife. Run real trials only with the rig's own stop and guard procedures and
with nobody within the arms' reach. The 30 mm blade rule in the score protects the robot's other hand.

## What is simplified (fidelity)

- The half is four rigid bodies: three stacked leaf layers (85 g each, 19 mm thick, 1 mm apart) and a 45 g core,
  held together by fixed joints until cut. Real leaves are curved, nested shells and flex; these do not.
- PhysX cannot cut. The knife passes through the cabbage without contact forces (it still collides with the board
  and the grippers). Cutting is the declared rule above; there is no cutting force, blade bending or juice. So
  nothing in the simulation needs steadying: one hand could do the whole task. In a kitchen the other hand holds
  the half still while the blade goes through it.
- The demo holds the blade 30 degrees off vertical and lowers it straight down, and the cut is the vertical surface
  its edge sweeps. Edge motion up to 45 degrees off the blade's plane counts (the demo's is 30). A real blade pushed
  down at that tilt drifts toward its own plane, so in a real trial the cuts are judged by where they actually end
  (the green strip and slit measurements above).
- The core is modelled as a full-height triangular wedge. In a real half the dense tissue is deepest at the stem end
  and tapers, and the two cuts lift it out together with the leaf bases inside the same wedge.
- While the steadying gripper still pinches the leaves after the core is freed, the three layers stay joined to
  each other; they become loose layers as soon as it opens and must stay stacked on their own for the final 1 s.
- Masses, friction and the knife's handle size are estimates; no specimen or knife was measured.

## Demo

The scripted demo (`demo()`) plays fixed waypoints worked out from each lot's visible starting layout; it reads no
object state during a trial. The left gripper pinches the leafy end across its 54 mm width. The right gripper picks
up the knife by its handle, carries it behind the core, rolls the blade 60 degrees from flat (30 degrees off
vertical) so the hand stays clear of the cabbage, lines the edge up 1 mm outside one side of the outline with the
tip 1 mm short of the apex, and lowers it 93 mm, through to the board. It draws the blade 20 mm back toward the
root, lifts it, and does the same on the other side, then puts the knife back where it was and lets go. The left
gripper opens, grips the core's exposed base at the root end, draws it 50 mm straight out, lifts it, sets it on the
board 100 mm to the side and moves away.

`fixture/calibration.json` holds fixed corrections for each lot: the cabbage's settled pose under the steadying
gripper, and the arm's tracking offset and tilt while it holds the knife. They were measured once from the task's
own log of scripted runs (`fixture/calibration-input.jsonl` for lot A, `fixture/calibration-input-B.jsonl` for lot
B) and then frozen. The demo reads them when it builds its waypoints and never reads a live pose.

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Cut the pale core out of the cabbage half with the knife, set the core on the board, put the knife down, and leave the green leaves stacked in place." \
  --scorer task_score --max-steps 2400 --epochs 2 --store-frames
```

`--epochs 2` runs lot A, then lot B; the score is their mean. `TASK_LOT=A` or `B` pins one lot. The demo takes about
85 s of simulated time per trial (the limit, 2400 steps, is 120 s). For Dream-1 use `--policy dream`.

## Results

| Run | Lot A | Lot B | Mean |
|---|---|---|---|
| Scripted demo | 1.0 | 1.0 | 1.0 |
| Dream-1 | 0.0 | 0.0 | 0.0 |

Demo, lot A: both outline sides fully cut by 31.5 s, the cuts reached at most 3.1 mm into the leaves, the core was
set on the board 100 mm away, the upper layers ended within 2 mm of the bottom one, and the 1 s hold was met at
84 s. Lot B: the same, at most 4.0 mm into the leaves, done at 83 s. Each trial ended in success (score 1.0).

Dream-1, same instruction and lots: 0.0 in both. In lot A it pulled both arms back toward their bases and held
them there until the 2400-step limit; it never touched the knife or the cabbage. No refusal or error was reported.

## Shortcuts and what they score

| Shortcut | Result |
|---|---|
| Pull or push the core out without cutting | It is joined to the leaves and only moves with them, so the whole half moves; forcing it 15 mm out of place relative to them before both sides are cut is 0 for the trial; sliding the whole half more than 10 mm caps the trial at 0.33 |
| Stab or touch the outline with the knife tip | A tip covers almost none of the outline and makes no 20 x 5 mm cut: 0 |
| Sweep the flat of the blade or its spine through the outline | Not edge first: no cut, 0 |
| One straight chop across the half | Crosses the outline along one line and cuts far more than 25 mm into the leaves: 0 for the trial |
| Cut more than 10 mm outside the outline | Not a cut on the outline, so the core never comes free: 0 |
| Cut one side, then pull the core | The other side still holds: at most 0.33, and 0 if it is pulled 15 mm |
| Cut both sides and leave the core in place | The core is not 10 mm clear of the leaves: 0.33 |
| Hold on to the core or the knife at the end | Grippers not 5 mm clear, no 1 s still: 0 |
| Spread the leaves, or slide the whole half away, to get at the core | A layer more than 10 mm from where it started: at most 0.33 |
| Push the knife with an open gripper, or hold it by the blade | Only a knife held by its handle in a closed gripper cuts; touching the blade is 0 |

`near_misses.py` checks these and the exact 2, 5, 10, 20 and 25 mm and 45 degree boundaries offline (62 checks,
plus six whole-scene cases in `CASES`).

## How this differs from close tasks

- TK-N9C8DA (slice a loaf with a claw grip): one straight cut across a loaf; here two cuts must follow an outline on
  the food and meet at a point, and the piece between them is drawn out.
- TK-VT2J8N (peel a cucumber): strips the outside surface; here the cuts go through the food along an inner
  boundary.
- TK-JRVAY3 (grate cheese), TK-W7HHVG (cut a cable tie), TK-53WVPU (cut a held cord): cutting through or across an
  object, not removing an inner part while keeping the rest together.
- TK-PMQUWA (apple wedger) and TK-8XJF7J (apple tube corer): a fixed tool is pressed straight through the fruit;
  here a free knife is aimed at a visible outline.

## Files

`task.py` (scene, scoring, demo), `fixture/cut_geometry.py` (cut rule), `fixture/calibration.json` and its two input
logs (frozen demo corrections), `fixture/GEOMETRY.md` and `fixture/root-grasp.svg` (drawings), `parts.py`,
`lots.py`, `tilt.py` (shared task kit), `near_misses.py` (offline checks). Code MIT; text CC BY 4.0. No maker CAD,
photos or logos are used.
