# Lay a click-lock vinyl floor tile

**Instruction:** "Click the loose vinyl tile into the laid row: angle its tongue into the groove, end against the spacer, lower it flat, then let go."

Rigid-core vinyl (SPC) and laminate floors float: they are not glued or nailed, and each tile is held only by the
click joints along its edges. To join a tile to the row already laid, the installer tips it up about 20 degrees,
puts its long tongue edge into the row's groove, pushes it home along the whole edge and lowers it flat. Lowering
drops the tile's locking tooth behind the locking bump on the row's lower lip, and the joint is then locked against
being pulled apart or lifted. Carpenters (O\*NET 47-2031.00) and floor layers (47-2042.00) do this for every tile of
a floor. The maker's instruction for the long edge is "Angle the long tongue edge of the plank into the long groove
edge of the first plank" (MSI hybrid rigid-core installation guide, step 5), and each row starts against 3/8 in
(9 mm) spacers at the wall.

Here the table top is the subfloor. One row is laid and fixed. The loose tile has been set against it with its
tongue resting on the row's locking bump, and its near edge on a batten, so a gripper can get under it. Both
arms pinch the near edge, tip the tile up on its tongue, slide it along the row to the wall spacer, push the
tongue home under the upper lip, and lower it.

## Running a trial

```
inspect-robots run --embodiment yam_arena -E task=lay_click_lock_vinyl_tile.py \
    --instruction "Click the loose vinyl tile into the laid row: angle its tongue into the groove, end against the spacer, lower it flat, then let go." \
    --policy yam_scripted --scorer task_score --max-steps 1600
```

- Scene: `station`. Control at 20 Hz. Run limit: 1,600 steps (80 s). The scripted demo's waypoints span 25 s; it reaches success at
  step 447 (22 s; the score first reads 1.0 at about step 428 and success needs it held 1 s), so the run limit is 3.6x the
  demo. A trial takes about 9 to 12 minutes on one shared GPU.
- No object jitter. Everything starts in one fixed layout, and the seed changes nothing.
- `--scorer task_score` gives 1.0 on success and otherwise the score at the end (`SCORE_OVER = "end"`).
- Offline checks: `pytest test_lay_click_lock_vinyl_tile.py` (generated scorer tests: start, goal, partial and
  cheat layouts).

## Scoring

The scorer reads only the loose tile's pose. The release check reads the robot's joint state to find each
gripper's centre point. Seam, lippage and near-edge height are measured 10 mm in from each end of the tile.

| Quantity | Definition | Real trial |
|---|---|---|
| Seam | gap between the tile's far top edge and the laid row's top edge (x 0.460); + is open | feeler gauge |
| Lippage | tile top beside the seam minus the laid tiles' top | straightedge and feeler gauge |
| Near edge | height of the tile's near top edge above lying flat | ruler |
| End gap | wall face to the tile's end at the new row's spacer | ruler |

| Rule | Value |
|---|---|
| End locked | seam 0.4 mm or less (overlap at most 2 mm), lippage within 0.5 mm, near edge within 1 mm of flat. In a real trial, confirm with a 10 N pull on the near edge, flat along the floor toward the robot: the seam must not open past 0.4 mm. In simulation the geometry does this check: a flat tile with a seam under 0.4 mm has its tooth behind the bump. |
| End against the row | seam 10 mm or less (overlap at most 2 mm), near edge within 2 mm of flat |
| Released and at rest (needed for any credit) | each gripper's centre point at least 30 mm from the tile's bounding box; the tile's two near corners and the middle of its far edge each move less than 0.5 mm between 50 ms steps; all held for 1 s |
| 1.0 | both ends locked, end gap 12 mm or less (the spacer gives 9.5 mm) |
| 0.75 | both ends locked, end gap over 12 mm and up to 40 mm (not slid to the spacer) |
| 0.5 | both ends locked with the end gap over 40 mm, or one end locked and both against the row (racked: the other end rides on the bump) |
| 0 | anything else: a tile lowered before its tongue was in (riding on the bump along its length, seam open, about 1.4 mm high), the untouched start (still tipped on the batten), held, moving, tipped up, on top of the laid row, short of the row, off the table |
| Sticky 0 | the tile ever drops below the table top by 50 mm (off the table) |
| Success | 1.0, held for 1 s (`SUCCESS_HOLD_S`) |

## Layout (metres, world frame; arm bases at (0, +/-0.305, 0.750); table top 0.7475)

| Object | Type | Material | Size | Mass | Pose |
|---|---|---|---|---|---|
| Laid row | fixed, 34 convex pieces | SPC vinyl tile | a 0.300 cut piece (y -0.300..0) and a full tile (y 0..0.600); 0.300 deep, 0.005 thick | (fixed) | top joint line at x 0.460, tiles to x 0.760; lower lip to x 0.453 |
| Loose tile | free rigid body, 38 convex pieces | SPC vinyl tile | 0.600 x 0.300 x 0.005 (lower lip to 0.307) | 1.55 kg | tipped 9.26 deg about its far edge, tongue on the laid row's locking bump; far top edge at x 0.452 (8 mm short of the seam), 6.3 mm above the table; near top edge at x 0.156, 54.6 mm up, its lower lip on the batten's far top edge; ends at y -0.290 and +0.310 |
| Wall | fixed box | 18 mm plywood | 0.600 x 0.018 x 0.100 | (fixed) | face at y -0.3095, x 0.180..0.780 |
| Spacers | fixed boxes | blue plastic | 0.040 (x) x 0.0095 (y) x 0.030 | (fixed) | against the wall face, x 0.390..0.430 and 0.590..0.630 |
| Batten | fixed box | sawn pine | 0.047 (x) x 0.200 (y) x 0.050 | (fixed) | x 0.1045..0.1515, y -0.100..0.100 |

Friction (static/dynamic): tiles 0.4/0.3, wall, spacers and batten 0.5/0.4.

**Tile.** MSI Trecento rigid-core luxury vinyl tile, White Ocean (VTRWHIOCE12X24-5MM-12MIL): 12 x 24 in
(0.300 x 0.600 m), 5 mm thick with a 1 mm IXPE pad attached, 12 mil wear layer, sold 10 to a 35 lb case
(19.37 sq ft). 1.55 kg is the case weight per tile less the carton, and matches a 2.0-2.1 g/cm^3 core.

**Long-edge click profile** (mm; z from the pad's underside, pad 0..1, core 1..5; x from the top joint line, +
into the tile that carries the feature). It is an angle-in profile of the Valinge 2G type, drawn from US
9,314,936 (Valinge, "Mechanical locking system for floor panels", Figs. 1a-2c: upper lip, tongue and strip body
each about 1 mm in a 3 mm LVT panel), scaled to a 4 mm core.

| Piece | Outline (x, z) |
|---|---|
| Groove edge: upper lip | (0, 3.6) (3.0, 3.6) (3.0, 5.0) (0, 5.0) |
| Groove edge: groove | empty, x 0..3.0, z 2.0/2.2..3.6 |
| Groove edge: strip | (-4.26, 0) (3.0, 0) (3.0, 2.0) (0.4, 2.2) (-5.4, 2.2): 2.2 thick, its top sloping to 2.0 under the groove |
| Groove edge: locking bump (with the strip under it) | (-7.0, 0) (-4.26, 0) (-6.02, 3.4) (-6.35, 3.7) (-6.7, 3.7) (-7.0, 3.4): 1.5 mm above the strip, its locking face leaning back 62.5 deg (from (-5.4, 2.2) to (-6.02, 3.4)), top corners chamfered 0.3 |
| Tongue edge: head | x -7.3..0, z 3.85..5.0 (x measured from that tile's own top joint line, + outward) |
| Tongue edge: locking tooth | from 0.2 mm off the bump's locking face to the joint line, z 2.3..3.85; its underside rises from 2.3 at x -2.0 to 2.63 at the joint line |
| Tongue edge: tongue | (-2.0, 2.3) (2.6, 3.05) (2.6, 3.15) (2.35, 3.4), top at 3.4: 1.0 thick at the joint, bevelled underneath |
| Tongue edge: lower body | ends at x -7.3, 0.3 mm short of the strip's end |

Clearances: 0.2 mm over the tongue, 0.4 mm at its tip, 0.2 mm locking play, 0.1 mm or more between tooth and strip.
The locking faces are tangent to the arc about the joint's top edge, so the tooth swings in as the tile is lowered
and comes out only if the tile is tipped up again. A 2D check of the section (rotating about the top edge, and
sliding in at a fixed tilt on the lowest contact) shows the tongue gets under the lip only at tilts of 12 deg or
more; below that its tip meets the lip's face 2 mm short, and a tile lowered there rides on the bump 1.4 mm high.
The chamfers on the bump and under the tooth are lead-ins: without them the tongue, which sinks up to 0.5 mm into
the bump under load in the simulator's contact, snagged on the bump's sharp corner.

Collision pieces: each profile piece is a convex prism cut into lengths of 50 to 300 mm, so every hull's farthest
vertex is less than 70 times its internal radius (PhysX builds GPU convex hulls only below 100; longer, thin hulls
fell back to CPU collision and the tile fell through the table). 38 pieces on the loose tile, 34 on the laid row.

## Scripted demo (`demo()`, open loop, start layout and time only)

Both arms do the same thing at the two grip points, 180 mm either side of the tile's middle (clear of the batten).
The wrists are pitched to point along the tile toward its far edge, with the pads closing across its 5 mm
thickness.

1. Come in behind the near edge with the pads 50 mm apart (fully open, the lower finger would reach the table),
   pitched 9.3 deg like the tile, and slide the pads over and under the near edge (the TCP 3 mm in from the edge).
2. Pinch the near edge.
3. Tip the tile up on its tongue to 14 deg and then 20 deg, leaning in 5 mm as it rises so the tongue stays on the
   bump (tipped without leaning in, it was drawn back off the bump's outer side).
4. Slide it 10 mm along the row against the new row's spacer (commanded 6 mm past it).
5. Push the tongue home along +x (commanded 10 mm past home: the soft arm pushes through its tracking error).
6. Lower it about the joint to 8 deg, keeping the wrists pitched at least 12 deg so the gripper body clears the
   table, then open the pads to 50 mm so the near edge drops; the tooth drops behind the bump.
7. Draw the pads back out along the tile and rise. 3.5 s of stillness follow. The demo takes 25 s.

Offline checks: IK error is at most 0.1 mm, planar reach at most 0.323 m, and the gripper body (102 x 65 mm,
starting 70 mm behind the centre point) stays at least 8.8 mm above the table.

## Real-world parts list

- Tiles: MSI Trecento rigid-core luxury vinyl tile, 12 x 24 in, 5 mm with attached pad (any colour; the profile is
  the same). One full tile is loose; the laid row is a full tile and a half tile cut with a utility knife.
- Laid row fixture: fix the laid row to a plywood board (the "subfloor") with double-sided carpet tape so it cannot
  move, as if locked to the rows beyond.
- Wall: an 18 mm plywood strip, 100 x 600 mm, screwed to the board on edge.
- Spacers: 3/8 in (9.5 mm) plastic flooring spacers (for example from a laminate installation kit).
- Batten: a 47 x 50 mm sawn pine batten, 200 mm long, taped to the board.
- A 10 N spring scale for the pull check, a feeler gauge set and a straightedge.
- Safety: nothing sharp or hot; the tile weighs 1.55 kg and moves less than 60 mm above the board. Use the robot's
  usual guarding and e-stop.

## Sources

- MSI Trecento White Ocean product page (msisurfaces.com, VTRWHIOCE12X24-5MM-12MIL: 5 mm, 12 mil, 1 mm IXPE pad,
  patented click locking system); case data from resellers (10 pieces, 19.37 sq ft, 35 lb per case; 660 pieces and
  2,310 lb per pallet).
- MSI hybrid rigid-core installation guide (cdn.msisurfaces.com/files/flyers/hybrid-rigid-core-installation-guide.pdf):
  angle the long tongue edge into the groove; 3/8 in (9 mm) expansion gap on spacers.
- US 9,314,936 B2 (Pervan, Valinge Flooring Technology): angling locking systems for 3-5 mm LVT panels.
- O\*NET 47-2031.00 Carpenters (889,700 employed, 2025) and 47-2042.00 Floor Layers, Except Carpet, Wood, and Hard
  Tiles (29,800).

## Files and licences

- `lay_click_lock_vinyl_tile.py`: the task (scene, scorer, demo).
- `yam_parts.py`, `yam_joints.py`, `yam_mesh.py`: our own primitive, joint and mesh kits, unmodified copies.
- `task.json`, `README.md`, and `test_lay_click_lock_vinyl_tile.py` (generated scorer tests).
- Every shape is a simulator primitive (box) or a convex mesh generated in code from the profile above. There are
  no third-party assets, textures or fonts.
