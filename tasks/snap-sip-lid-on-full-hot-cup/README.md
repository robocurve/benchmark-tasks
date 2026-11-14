# Snap a sip lid onto a full 16 oz paper hot cup all round the rim, without tipping or squeezing the cup

Snap the lid onto the full coffee cup all the way round its rim. Don't tip the cup or squeeze its wall.

Counter workers and baristas lid almost every hot drink sold to go (O*NET 35-3023.00 and 35-3023.01). The lid goes
on the cup and is pressed down round the rim until its skirt has snapped over the cup's rolled lip all the way
round; a lid seated on part of the rim pops off or leaks when the cup is carried. The cup is light, tall and full,
and its wall is thin paperboard: a hard press on one side of the rim tips it, and a squeeze dents it. The task tests
pressing a snap lid home section by section round a whole rim with enough force to seat each section and not so
much, or so far out, that the cup tips, without squeezing the cup's wall.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=snap_sip_lid_on_full_hot_cup.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Snap the lid onto the full coffee cup all the way round its rim. Don't tip the cup or squeeze its wall." \
  --scorer task_score --max-steps 2800 --epochs 2 --store-frames --log-dir ./demo-run
```

For a model trial, replace `--policy yam_scripted` with `--policy dream`. `--scorer task_score` reports the score at
the end of the run. Each trial allows 2,800 control steps (140 s at 20 Hz). There are no seeds: the layout is fixed
and nothing is jittered. The scripted demo lasts 26.8 s (536 steps); `success()` needs a score of 1.0 held for 1 s.
`pytest` in this folder runs the offline checks (no GPU): the scorer on the start, the goal and named layouts, the
step-by-step rules (`test_red_team.py`), and the demo's reach and length.

## The demo is open loop

The scripted demo uses only the known start layout and time; it never reads the lid, the cup or their joints. The
left arm fetches the lid and presses it home two opposite sectors at a time, one open fingertip on each side of the
dome; the right arm moves to its park point (110, -220, 170) at the start and stays there.

| Time (s) | Left arm |
| --- | --- |
| 0.0-3.8 | over the lid, down, pinch the dome's wall (pads 73.2 mm apart on the 75-77 mm dome, tips 6 mm above the flat ring, 2 mm over the rim channel) |
| 3.8-8.9 | lift, carry over the cup with the skirt 10 mm above the rim, settle 0.6 s, lower 13 mm in 1.4 s until the flat ring is 0.5 mm over the rim, open to 90 mm |
| 8.9-10.2 | rise to 175 mm over the cup, set the fingers 84.6 mm apart |
| 10.2-21.8 | four pair presses over the cup's axis: sectors 2 and 6, then 1 and 5, 0 and 4, 3 and 7, turning 45 degrees between pairs. Each: down to 8 mm over the channel (0.8 s), on to touch it (0.5 s), press for 0.8 s to 15 mm below the seated channel top, hold 0.3 s, lift (0.5 s) |
| 21.8-23.8 | rise clear, park at (110, 220, 170) |
| 23.8-26.8 | still |

Each fingertip is 3.45 mm thick along the closing axis, so with the fingers 84.6 mm apart each tip spans 42.3 to
45.75 mm from the cup's axis: on the 42-46 mm channel, 3.8 mm clear of the dome, and at least 9.5 degrees inside its
sector's seams even with the hand 2.5 mm off. Pressing two opposite sectors at once keeps the push centred on the cup
and holds the light lid level between the tips. Grip yaws (degrees): sectors 2 and 6: 0; 1 and 5: -45; 0 and 4: -90;
3 and 7: -135, which keep wrist joint 6 within 69 degrees of centre. The push is set by the arm's own stiffness (its
PD gains, 80 N m/rad on joints 1-3 and 10 N m/rad on joints 4-6, give 505 N/m straight down over the cup, and joint
friction a band of about 1 N): about 7.9 N on two unseated sectors (x1.31 over their two 3 N holds) and 8.6 N once
seated, 4.3 N a sector. That push acts within a few mm of the cup's axis, well inside its 29.2 mm bottom radius, so it
cannot tip the cup (a press 44 mm out on one side would at 9.0 N).

The arm is compliant and its joints have friction, and the demo trims for both in open loop, along the horizontal
line from the left arm's base out through the cup's axis (measured on the GPU): hovering, the tool point sits 0.8 mm
farther out than commanded; the 9 mm descent to the channel leaves it 1.5 mm nearer the base; and an 8 N downward push
draws it 3.2 mm toward the base. So the aim is 0.8 mm in at the hover and 0.7 mm out at the touch, and moves to 3.9 mm
out as the push builds, which keeps the tips over the cup's axis instead of dragging the lid and cup sideways. The arms' tool
points stay at least 220 mm apart; the farthest is 328 mm from its arm's base.

## Rebuild the setup

Units mm. x forward from the line through the two arm bases, y left (+) or right (-) of the midline between them,
z up from the table top (the counter); arm bases at (0, +305) and (0, -305). Nothing else is on the counter.

- **Cup:** Dart Solo 316W-2050, 16 oz single-wall poly-coated paper hot cup, white. 88.9 across the rolled lip,
  58.4 across the bottom, 134.6 tall, 14 g. 451 g of room-temperature water tinted with instant coffee, the surface
  15 below the rim (465 g in all). Bottom centred at (120, 0).
- **Lid:** Dart Solo TLB316-0004 Traveler dome sip lid, black polystyrene, for 10-24 oz Solo hot cups. 93.0 across,
  18.5 tall, 4.2 g. Lying top-up on its skirt, centred at (150, 165), at any turn (its sip hole is not used).

### The cup (frame: bottom centre; the wall's outer radius is 29.2 + 14.0 z / 131.9 from z 0 to the lip)

| Body | Collider | Size |
| --- | --- | --- |
| base | bottom disc, convex revolve | r 29.2 at z 0 to 29.84 at z 6 |
| base | rim band, convex hull of a revolved ring (20 sides) | r 43.06 at z 130.6, the lip r 44.45 from z 132.6 to the rim top at 134.6; solid to the rim (the drawn mouth is open, down to the coffee) |
| base | coffee, convex revolve | r 29.2 at z 6 to 41.3 at z 119.6 (the surface), brown |
| panel_0 .. panel_7 | wall panel, convex hull of a 44.4-degree arc of the tapered wall, 0.45 thick | z 6 to 130.6, centred on k x 45 degrees |

The base weighs 455 g (the water and 4 g of paper) and the eight panels 1.25 g each; PhysX spreads the base's mass
over its colliders, which puts the centre of mass 70 up. Pressed straight down 44 from the axis, the full cup tips
over its bottom edge at 4.56 N x 29.2 / 14.8 = 9.0 N. Each panel is on a horizontal slide (x for panels 0, 1, 3, 4, 5,
7 and y for 2, 6), sprung outward against its stop: 630 N/m and 0.5 mm preload along the radius (the diagonal
panels' x springs are 315 N/m, since they move 0.71 mm out per mm of slide), 8.5 mm of inward travel, damping
2 N s/m, armature 0.01 kg. 630 N/m per side is 3.0 N for 9.5 mm across the cup, the force the GB/T 27590 paper-cup
stiffness test measures (the cup pressed across at two thirds of its height; minimums 2.1 to 3.8 N by size). The
scorer counts the wall squeezed when any panel is pushed more than 5 mm in along the radius. Cup friction 0.4
static, 0.35 dynamic; contact offset 1 mm, rest offset 0.

### The lid (frame: centre of the flat ring's underside)

| Body | Collider | Size |
| --- | --- | --- |
| base | flat ring (deck), cylinder | r 44.6, 1.2 thick: it bears on the rim's whole top, out past the lip's 44.45 edge, and runs on under the channel tops (the lid's base and its sectors do not collide with each other) |
| base | dome, convex revolve (20 sides) | r 38.5 at z 1.0, 37.6 at z 8.5, 32.0 at z 12.0, top at z 12.5 |
| sector_0 .. sector_7 | rim channel top, convex hull of a 44.5-degree annular slab | r 42.0 to 46.0, z 3.0 to 4.5 at q = 0 |
| sector_0 .. sector_7 | skirt, three convex pieces of 14.8 degrees | section (r, z): (45.65, 3.0), (46.15, 3.0), (46.5, -3.5), (46.05, -3.5), (45.65, -2.0) at q = 0 |

Sector k is centred on k x 45 degrees. Each is on a vertical slide from 0 down to -2.5 mm with a static friction hold
of 3.0 N and 2.2 N while it slides, damping 0.5 N s/m and armature 0.05 kg; it weighs 0.25 g and the base 2.2 g. The
armature is a numerical slide inertia carried by the base: under a press the light base jitters in its contact with
the rim, and with 5 g (or 0.5 g) of armature the GPU showed seated sectors walking (or being flung) back up while
another pair was pressed; with 50 g, seven stay on their stops and one walked back 0.5 mm over the three later presses
(still well inside the count). It changes neither the 3 N hold nor the push that seats a sector. On the cup, the flat
ring rests on the rim's top (z 0) and each unseated sector's channel stands 3.0 mm above the rim. Pressed home (q =
-2.5 mm, the slide's own stop), the channel's underside is 0.5 mm above the rim: the flat ring alone carries the lid
and every later press (on the GPU it sinks up to 0.27 mm into the rim's contact under an 8 N press), and a seated
channel moves with the lid and is never pushed back up by the rim. The skirt's inner face clears the lip by 0.82 to
1.37 mm all round, so the lid centres itself on the cup, and its flared mouth catches a lid lowered up to 1.2 mm off
centre. On the counter the lid stands on its skirt with the flat ring 3.5 mm up. Lid friction 0.3 static, 0.25
dynamic; contact offset 1 mm. Every convex collider has at most 61 hull vertices (PhysX keeps 64).

## Scoring

Scored at the end of the run from the lid's and cup's poses and the sector and panel slides; success is 1.0 held for
1 s. In a real trial: mark the rim at the eight sector centres; at the end, measure with calipers the height of the
lid's rim channel above the counter at each mark against a fully snapped reference lid, and take a top photo for the
lid's position on the cup; the video shows the cup's tilt, lift and wall.

- **Lid on the cup:** its centre within 3 mm of the cup's axis, its flat ring within 1.5 mm of the rim's top, tilted
  under 5 degrees to the cup.
- **Seated sector:** its rim channel within 0.8 mm of fully home (q <= -1.7 mm; unseated, it stands 2.5 mm proud of
  that).
- **Zero for the rest of the run**, if at any moment: the cup tilts more than 15 degrees; its bottom rises more than
  10 mm off the counter; its wall is pushed in more than 5 mm on any side; or a lid sector is pushed more than 1 mm down
  while the lid is away from the cup (centre more than 10 mm off the cup's axis, flat ring more than 10 mm above or
  below the rim, or tilted more than 20 degrees: a real lid cannot be snapped shut off the cup).
- **Zero at the end** if the cup's bottom is more than 30 mm from its start mark, or the lid is not on the cup (off it,
  or riding crooked or high on the lip).
- **Otherwise:** 1.0 all eight sectors seated; 0.75 six or seven; 0.5 three to five; 0.25 two or fewer.
- **Done**, needed for full credit: each gripper's centre point (between the pads) 30 mm or more from the lid's and
  the cup's bounding boxes; the lid and cup slower than 1 cm/s; and over the last 1 s each moved under 1 mm and
  2 degrees and no sector slide moved more than 0.2 mm. Without done the score is at most 0.5.
- How far the lid is turned on the cup does not matter. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

## Reset

Lift the lid off by its rim; replace it if the skirt is cracked or stretched; lay it top-up on its mark. Top the cup
up to 15 mm below the rim, or replace it if dented or creased, and stand it on its mark. Use zero object jitter. The
scorer's remembered events (tipped, lifted, squeezed, forced) and its 1 s windows start fresh with each trial.

## Fidelity and approximations

- **Snap skirt:** a real lid's skirt is one flexible polystyrene ring that snaps over the lip wherever it is pressed.
  Here it is eight rigid 45-degree sectors on vertical slides with a 3 N friction hold: pressed with 3 N or more a
  sector slides down onto the rim and stays; pressed less, it does not move. A sector pressed part way stays part way
  (a real skirt springs back), and a press on the seam between two sectors shares its force between them. The seat
  force (3 N per 45 degrees) and pull-off force (the same) are estimates: no maker publishes them. They must be
  measured on real TLB316 lids with a force gauge.
- **No hook under the lip:** a seated sector rests on the rim but does not hook under it, so the model would let a
  seated lid be lifted straight off. Nothing in the task pulls the lid upward; lifting it would score 0 (lid not on the
  cup).
- **Dome:** rigid. A press on the dome bears on the rim through the flat ring and seats no sector (on a real lid a
  palm press on the dome can seat part of the rim).
- **Paper wall:** eight rigid panels on sprung slides, stiffness from the GB/T 27590 test; a squeeze over 5 mm is
  scored as a dent. The band under the lip, the lip and the bottom are rigid.
- **Water:** a fixed mass in the cup's base, with no slosh or spill.
- **Facets:** the cup's rim band and the lid's dome are 20-sided (PhysX's 64-vertex limit on convex colliders), so
  the lip is 88.9 mm across its corners and 87.8 mm across its flats.
- **Friction:** paper on the counter 0.4, polystyrene on paper 0.3: estimates.
- **Not modelled:** the lid's sip and vent holes and the cup's printed fill line (neither is used).

## Sources

- Dart Solo 316W-2050 (WebstaurantStore listing 760316W): top diameter 3 1/2 in, bottom 2 3/10 in, height 5 3/10 in,
  16 oz, single wall poly coated paper, 1,000 per case at 34.5 lb.
- Dart Solo TLB316-0004 (Dart foodservice catalogue, "10-24 oz PS Plastic Dome Hot Cup Lid - Black"; Imperial Dade
  T316B): diameter 3.66 in, height 0.727 in, polystyrene, 1,000 per case; the white TLP316-0007 case weighs 9.79 lb.
- GB/T 27590-2022, Paper cups: cup body stiffness, the maximum force over 9.5 mm of wall deflection across the cup at
  two thirds of its height, 2.1 to 3.8 N minimums by capacity.
- O*NET 35-3023.00 Fast Food and Counter Workers: "Brew coffee and tea, and fill containers with requested
  beverages."; 35-3023.01 Baristas: "Prepare or serve hot or cold beverages, such as coffee, espresso drinks, blended
  coffees, or teas."

## Licence

All geometry is authored in this folder from the published dimensions above. No third-party meshes, scans or scenes
are used. `yam_parts.py`, `yam_joints.py` and `yam_mesh.py` are the parts, joints and mesh kits, copied unchanged.
