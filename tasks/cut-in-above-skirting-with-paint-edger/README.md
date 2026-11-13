# Cut in a wall above a skirting board with a paint edger

Repainting a room starts with cutting in: a clean band of new paint along the skirting boards, door casings and ceiling, painted before the open wall is rolled, because a roller cannot reach into the corner. A paint edger makes this easier for people who are not painters: a flat paint pad on a small head, with two guide wheels along one long edge and a palm grip on the back. Run along the top of the skirting board, the wheels hold the pad's lower edge just clear of the board while the pad, kept flat and pressed lightly on the wall, paints the wall right down to the board. If the wheels ride up, a strip of bare wall is left above the board; if the pad tips or drops, it smears paint on the board, which then has to be wiped and repainted.

Here a section of drywall with a white skirting board along its foot stands on the table, its face toward the robot. A loaded edger stands in its rest to the left of the wall. The robot takes the edger, sets its pad on the wall just above the board, brings its wheels down onto the board's top edge, runs it along the board across the 300 mm stretch marked by two green tape tabs, lifts it off and stands it back in its rest. The score reads where the new paint went: the 40 mm band of wall above the board must be painted, especially its lowest 5 mm (the cut line), and the board must stay clean.

Category: Societal relatability. Most people who have repainted a room have cut in along a skirting board, many of them with an edger like this one. The demonstration uses the left arm only; the scorer does not care which arm does what.

## Run and score

With the official simulator installed, run from this folder with zero object jitter and 20 Hz:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=cut_in_above_skirting_with_paint_edger.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 \
  --instruction "The paint edger's pad is loaded. Paint the wall along the skirting board between the two green tabs: run the edger with its pad flat on the wall and its wheels on the board's top edge, then stand it back in its rest." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ./demo-run
```

The model run uses the same command with `--policy dream`. The harness reports task_score at the end. Encode recorded cameras with `inspect-robots video <run.json>`. Scores are in task.json.

**Seeds.** The layout is the same for every seed. With nonzero object jitter the edger's start is shifted (x, y) by the seed; the demonstration reads where the edger stands before it reaches for it.

**CPU checks.** `python -m pytest -q -p no:cacheprovider` in this folder, with yam_arena installed, runs the scorer on the start, the goal and named partial and cheat layouts, and `test_red_team.py` steps the paint record and the scorer through whole trials.

Runtime per trial: at most 2,400 control steps (120 s simulated). The scripted demonstration's plan lasts 38.1 s (762 steps); the hand is clear of the edger by 33.4 s and the score reaches 1.0 about 1 s later, about 30% of the limit. Wall-clock time depends on the GPU.

## Scoring

**Grid.** The wall's face, the board's top and the board's face are read on a 5 mm grid (real trial: a square-on photo through a 5 mm grid transparency; the board's top is photographed from above). A cell is painted when at least half of it carries the new paint. In simulation the paint record keeps, for every cell, whether the pad has painted it (see "Paint" below); paint never comes off.

**Band.** The wall from the board's top (89 mm above the table) up to 40 mm above it, between the tabs' inner edges (y = 0 to 300 mm): 8 rows of 60 cells, 480 cells. **Cut line:** the band's lowest row (89 to 94 mm above the table), 60 cells. **Trim paint:** new paint on any cell of the board's top or face (real trial: any blue mark 2 mm or more across).

**In its rest.** The edger stands on both wheels on the rest's base (each wheel's bottom from 4 mm below to 3 mm above the base's top), the centre of the pad's face from 10 mm behind to 8 mm in front of, and within 30 mm either side of, where it started, the pad facing the wall within 15 degrees and the grip upright within 8 degrees. Released: every gripper's TCP at least 30 mm from the edger's bounding box. Still: under 10 mm/s and 0.05 rad/s, and the swivel turning under 0.05 rad/s.

| Score | Condition |
| --- | --- |
| 1.0 | at least 95% of band cells and 95% of cut-line cells painted, no trim paint, and the edger in its rest, released and still for the last 1 s |
| 0.75 | the same paint and no trim paint, but the edger not in its rest (still held, moving, or put down anywhere else) |
| 0.5 | at least 80% of band cells and 80% of cut-line cells painted, no trim paint |
| 0.25 | at least 40% of band cells painted, with or without trim paint: any trim paint limits the score to 0.25 |
| 0 | anything else, including an edger that never paints the wall |

**Sticky 0** for the rest of the trial: the pad touches the table (any corner of the pad within 1 mm of the table top), or the edger falls off the table. Paint on the wall above the band or beyond the tabs costs nothing. Success is 1.0, held for 1 s.

What the tiers mean: a full stroke with the wheels on the board and the pad flat paints every band and cut-line cell. If the wheels ride 1.6 mm or more above the board's top, the cut line stays bare (0.25 at most). If the pad's lower edge leans more than about 1.2 degrees away from the wall, the cut line is missed. A pad that drops 1 mm below its proper line, or is set down on the board, paints the board. Setting the pad on the wall at a few places along the stretch, instead of a stroke, paints the band just as well and scores the same.

## Paint

The pad is a rigid plate (the pile's thickness, 4 mm, behind its face). Each control step the record marks every cell whose centre lies within the pad's outline plus 0.5 mm, and between 4 mm behind and 2 mm in front of the pad's face, measured along the pad's normal from the pad's reported pose. The 2 mm in front stands for the pile's reach: pressed lightly, the real pile still touches the wall where the rigid plate is up to about 2 mm away, so a pad tipped by up to about 1 degree still paints the whole band. The 0.5 mm past the outline stands for paint at the pile's edge. The same rule paints the board's top and face, which is how a pad that sinks below its wheels' line, or is set down on the board, leaves trim paint.

Graphics: each cell has a thin blue square, hidden until the record marks the cell (the board's cells too). The squares have no collision and the scorer never reads them.

## Physical setup

Millimetres; x forward from the line through the two arm bases, y left (+) or right (-) of the midline, z up from the table top. Arm bases at y = +305 and -305.

- **Wall section (fixed):** 1/2 in (12.7) gypsum drywall, 380 long (y = -60 to +320), 200 tall, painted a pale warm grey (the old colour). Its face is the plane x = 250, toward the robot. It is screwed to a pine 2x4 frame (38 x 89: bottom and top plates and two end studs) behind it, and the frame is fixed to the table.
- **Skirting board (fixed):** primed MDF square-edge baseboard, 11/16 x 3-1/2 in (17.5 x 89), white semi-gloss, nailed along the wall's foot for its full length. Its face is at x = 232.5 and its flat top is 17.5 deep at z = 89. Two green painter's-tape tabs, 15 wide and 25 tall, are stuck on its face at the top, their inner edges at y = 0 and y = +300.
- **Paint edger (loose):** Shur-Line Edger Pro (item 2006561), shown standing in its rest in the setup image.
  - Head: red polypropylene, 117 x 93 x 9, carrying a 4-3/4 x 3-3/4 in (121 x 95) Shur-Line refill pad, white pile on a stiff clear backing, 4 thick, loaded with mid-blue latex wall paint.
  - Guide wheels: two black wheels, 14 across and 5 wide, axles normal to the pad, at the corners of the head's lower long edge, 103 apart (centres). They hold the pad's lower edge 1.5 above the surface they roll on; their rims run 6.5 to 11.5 behind the pad's face.
  - Grip: a black palm grip on the head's back, centred across the head. Its upper part is 58 across its flats, 38 deep from the head's back and 40 tall; its lower part is 44 across, 28 deep and 38 tall. A pole socket opens at its upper end. It turns on a swivel about the axis normal to the pad, 8 above the pad's centre: free, with stops at 90 degrees each way.
  - Mass: about 100 g with the paint (grip 30 g; head, pad, wheels and paint 70 g).
- **Edger rest (fixed, made for the trial):** a grey-painted 12 plywood base, 85 x 160 (x = 177 to 262, y = +335 to +495), 3 lower over its front 30 (under the pad), with a pine ledge 32 x 80 and 19 tall (x = 182 to 214, y = +375 to +455) under the grip's lower end.
- **Start:** the edger stands in the rest on both wheels with the grip's lower end on the ledge, pad facing the wall (+x) and touching nothing; the centre of the pad's face is at x = 237, y = +415, 61 above the table. Both grippers are open at the ready pose. Nothing else is on the table.
- **Friction (estimates):** wet pad on painted drywall 0.40 static / 0.35 dynamic; guide wheels on the board 0.05 (they turn freely; combined as the lower of the two); grip 0.6 / 0.5; skirting board 0.30 / 0.25; rest 0.5 / 0.4.

## Colliders

Every collider is a box or a cylinder in its part's frame, millimetres.

| Part | Collider | Centre | Size or radius x height | Notes |
| --- | --- | --- | --- | --- |
| wall (origin x 250, y 0, table top) | drywall | (6.35, 130, 100) | 12.7 x 380 x 200 | face at x = 0 |
| | bottom plate | (57.2, 130, 19) | 89 x 380 x 38 | pine |
| | top plate | (57.2, 130, 181) | 89 x 380 x 38 | pine |
| | end studs | (57.2, -41 / 301, 100) | 89 x 38 x 124 | pine |
| skirting (same origin) | board | (-8.75, 130, 44.5) | 17.5 x 380 x 89 | |
| | tape tabs | (-17.6, -7.5 / 307.5, 76.5) | 0.2 x 15 x 25 | green |
| rest (origin x 237, y 415, table top) | base | (-32.5, 0, 6) | 55 x 160 x 12 | |
| | base under the pad | (10, 0, 4.5) | 30 x 160 x 9 | |
| | ledge | (-39, 0, 21.5) | 32 x 80 x 19 | pine |
| edger grip (body: base) | upper grip | (-32, 0, 27.75) | 38 x 58 x 39.5 | held; flats 58 apart |
| | lower grip | (-27, 0, -11) | 28 x 44 x 38 | |
| | pole socket | (-32, 0, 47.8) | r 11.5 x 0.6 | |
| edger head (body: head, hinge x at (-13, 0, 8)) | pad | (-2, 0, 0) | 4 x 121 x 95 | face at x = 0 |
| | head | (-8.5, 0, 0) | 9 x 117 x 93 | |
| | guide wheels | (-9, +-51.5, -42) | r 7 x 5, axis x | wheel material |
| | pad-release tabs | (-14.5, +-55.5, 0) | 3 x 6 x 30 | |
| | guide-lift slide | (-15, 0, -42) | 4 x 40 x 8 | |

The edger's frame: origin at the centre of the pad's face, x out of the face, y along the long edge, z toward the end away from the wheels.

## Demonstration and runtime

The scripted demonstration (`demo()`) uses the left arm; the right arm waits at the ready pose.

1. The ready pose holds the left hand above the wall's left end, so the hand first comes back toward the robot, then down beside the grip; it closes on the grip's flats and lifts the edger 91 mm straight up out of the rest.
2. It carries the edger to the wall's right end, pad 14 mm in front of the wall and wheels 14 mm above the board.
3. It sets the pad on the wall (commanded 2.0 mm past the wall's face), slides it down until the wheels sit on the board's top, and presses down (commanded 4.8 mm past the board's top). Pressing down harder than into the wall keeps the pad's lower edge on the wall: pushing on the pad alone tips the arm's soft wrist (about 1.5 degrees per newton), so the pad's lower edge would lean away from the wall.
4. It runs the edger 200 mm along the board in 12 s, the pad's centre from y = 250 to y = 50, so the pad covers y = -10 to 310.
5. It lifts the wheels off the board (the pad slides 17 mm up the wall), takes the pad off the wall, carries the edger back above the rest at the same height (clear of the board's end) and lowers it onto its wheels and ledge at the rest's own place, lets go, lifts the open hand 60 mm and moves away. The YAM's two fingers are driven independently, so on the 58 mm grip the edger slides against the fully open finger as it leaves the rest (the reads below take that out); as the fingers open, that finger pushes it back to the middle of the hand, and the edger comes to stand about 17 mm left of where it started (the rest allows 30).

**What the demonstration knows.** It plans from the task's layout and reads the simulator three times (allowed for scripted demonstrations): the edger's position before it reaches for it (any start shift, clamped to 15 mm); the pad's position and yaw relative to the hand just before the pad goes to the wall, so it aims the pad rather than the hand (offset clamped to 20 mm, yaw to 3 degrees: the 58 mm grip slides about 17 mm across the closed fingers as the edger leaves its rest, then holds); and the same again above the rest. Each read prints one line (`EDGER_START`, `EDGER_IN_HAND`). The runtime also prints `EDGER t=...` every 20 steps: the pad's gaps from the wall at its four corners, the pad's lower edge and the wheels above the board, the swivel angle, and band, cut-line and trim counts.

## Fidelity and approximations

- **The pad is rigid.** The real pad's pile (about 3 to 4 mm of synthetic fibre on a stiff backing, judged from the maker's photos) gives by roughly 1 to 2 mm under the 1 to 2 N a person presses with (an estimate, to be measured); here a rigid plate stands in for it and paints up to 2 mm in front of its face (see "Paint"). The pile's give is a tolerance in this task, not the skill: the skill is keeping the pad flat on the wall and the wheels on the board's top while moving along it. A rigid plate is less forgiving than a real pad, not more.
- **Paint is scripted.** No liquid: the record marks cells by geometry. Paint load, drips, thin spots and paint squeezed past the pad's edge by pressing too hard are not modelled; the pad holds enough paint for this stretch.
- **Wheels.** The guide wheels are rigid rims with friction 0.05 against the board, standing in for free-turning wheels; they slide rather than roll.
- **Swivel.** The grip turns on the head freely about the pad's normal; the real swivel's range and friction must be measured (the maker lists a swivel handle; a retailer lists 180 degrees of turn).
- **Estimates.** The grip's shape, the wheels' size and offset, the pad's thickness, the masses and every friction are estimates; the pad's face size is the maker's.
- Wall, board and rest are fixed to the table.

## Dimension sources

- Shur-Line Edger Pro, item 2006561: "4.75 in. x 3.75 in." (maker's listings at large retailers), swivel handle, retractable guides, ejectable pad; the guide wheels and grip are scaled from the maker's product photos. Refill pads 4-3/4 x 3-3/4 in.
- Baseboard: 11/16 x 3-1/2 in primed MDF is a common stock size; drywall 1/2 in; 2x4 lumber 1-1/2 x 3-1/2 in actual.
- To measure on the real tool before a real trial: the wheels' diameter and the pad's lower edge above their rolling line; the pad's pile thickness and how far it gives under 1 to 2 N; the grip's shape; the swivel's axis, range and friction; the mass with a loaded pad.

## Files and licence

- `cut_in_above_skirting_with_paint_edger.py`: the task (objects, scorer, demonstration).
- `paint_runtime.py`: the paint film's graphics, the guide wheels' material and the reset hooks (simulator side).
- `yam_parts.py`, `yam_joints.py`: the parts and joints kit, unchanged.
- `test_red_team.py`: step-through checks of the paint record and the scorer.
- `test_cut_in_above_skirting_with_paint_edger.py`: the scorer on named layouts, the demo's reach and length.
- `task.json`, `README.md`.

All geometry is built from primitives in these files; no third-party assets are used.
