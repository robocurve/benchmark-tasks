# Cover a bowl of rising dough with a cloth: take a washcloth off a towel stand with both hands and lay it over the rim

Cover the bowl of dough with the cloth so it can rise.

## Scoring

Judged at the end of the run from the top-camera video, a top photo, a side photo through the clear bowl with a ruler stood behind it, and a flexible tape. A part of the rim is covered when, in the top photo, the cloth hides it. Overhang: along the cloth from the rim's outer edge to the nearest cloth edge, where it is shortest. Score 0 if either gripper still holds the cloth or is within 2 cm of it, the cloth is still moving (any point more than 1 cm from where it was 1 s earlier), no part of the rim is covered (doing nothing scores 0), the bowl tipped more than 15 degrees at any time, the bowl's centre moved more than 2 cm, or the towel stand was knocked over or moved more than 2 cm (in simulation the stand is fixed). Otherwise the lowest of: (1) rim covered: 1.0 if all of it; 0.5 if at least 75 % but not all; 0.25 if under 75 %. (2) shortest overhang: 1.0 if at least 2 cm; 0.75 if at least 1 cm and under 2 cm; 0.5 if under 1 cm. (3) bowl moved: 1.0 if at most 1 cm; 0.75 if more than 1 cm. (4) dough: 1.0 if the cloth does not touch the dough and its lowest point inside the rim is at most 1.5 cm below the rim's top; 0.75 if it does not touch but hangs lower; 0.5 if it touches. (5) stand: 1.0 if no part of the cloth is left on the stand's bar; 0.25 if some is. Simulation checks the rim every 5 degrees; touching = a cloth node within 2.75 mm of the dome.

How `score()` reads each condition (bowl frame: origin at the centre of the bowl's foot, z up its axis; cloth: its
1369 node positions, `objects["cloth"].nodes`):

- **Rim covered:** 72 points on the rim's top centre line (67.8 mm from the axis, 55 mm up), one every
  5 degrees. A point is covered when a cloth triangle lies over it (seen from above) no lower than 3 mm under the rim's
  top. The fraction covered is band (1).
- **Overhang:** at each covered point, the cloth's own (flat, unstretched) coordinates of the covering point give the
  distance along the cloth to its nearest edge; less half the wall (2.2 mm) it is the overhang from the
  rim's outer edge. The smallest over the covered points is band (2).
- **Bowl moved / tipped:** the bowl's foot centre, horizontal distance from where it stood at the first step (kept as a
  running maximum); tipped = its axis more than 15 degrees from vertical at any step.
- **Dough:** the dome is a spherical cap 80 mm across and 26 mm high on the inside of the bottom; touching = a
  cloth node within half the cloth's thickness + 1.5 mm (2.8 mm) of it. Sag = the rim's top minus the
  lowest cloth node inside the opening (more than 3 mm inside the inner wall); full credit at most 15 mm.
- **On the stand:** any cloth node within 15 mm of the bar's surface (along the bar's length). The stand is fixed in
  the simulation; a real trial also scores 0 if it is knocked over or moved more than 2 cm.
- **Let go:** no contact-gated hold is on and both grippers' tool points are at least 20 mm from every cloth
  node. **Still:** no cloth node has moved more than 10 mm over the last 1 s (the scorer keeps
  the nodes of the last second; one step's node velocities carry solver jitter).
- `success()` is a score of 1.0; the episode ends once it has held for 1 s.

## Run it

```
inspect-robots run --embodiment yam_arena -E task=cover_rising_dough_with_a_cloth_from_a_towel_stand.py --policy yam_scripted   # scripted demonstration
inspect-robots run --embodiment yam_arena -E task=cover_rising_dough_with_a_cloth_from_a_towel_stand.py --policy dream           # model run
```

Run limit 1,200 control steps (20 Hz, 60 s). The scripted demonstration lasts 21.3 s
(426 steps). About 7 control steps per second on an L40S: a full run takes about three minutes after
start-up. No seed is needed: the start is fixed (`-E object_jitter` moves the bowl up to 10 mm).

## What is different

Unlike folding or storing towels, the cloth is carried spread by two hands and released so it settles centred over an
object without touching what is inside. It starts where a kitchen cloth waits, hanging over a towel bar, so the hands
first take a free-hanging layer by its corners (from the sides, jaws across the layer) and slide it off the bar.

## Scene and real objects

Millimetres from the midpoint between the arm bases: x forward, y to the robot's left, heights above the counter. 1) Towel stand: Gatco 1444SN countertop hand-towel holder, satin-nickel steel, T-shaped, 267 wide (y) x 283 high, on a weighted padded rectangular base 89 deep (x); 0.73 kg. Base 140 wide and 15 thick, post 12 and bar 13 across (estimates). Bar along y, its axis at x = 285, y centred on 0. 2) Bowl: Duralex Lys 14 cm, clear tempered glass, 140 across the rim, 55 high, 500 ml; foot 70 across, wall 4.5, bottom 6 (estimates that give the 500 ml); about 270 g (estimate). Foot centred at (150, 0), so its far rim is 20 in front of the stand's base. 3) Dough stand-in: one 4 oz (112 g) can of white Play-Doh pressed into the bottom as a centred dome 80 across and 26 high, its top 23 below the rim (shape it by ruler). 4) Cloth: one Burt's Bees Baby organic cotton woven terry washcloth, Blue Solid, 254 x 254 (10 x 10 in), about 29 g and 2.5 thick (estimates). It hangs over the bar, hems parallel to it, centred on y = 0: 165 of cloth from the bar's top down the side facing the robot (that hem 125 above the counter), 89 down the back (hem 200 up). Start: both arms at their ready pose, grippers open. Frictions are estimates (cloth 1.5 in the simulator, stand 0.30, glass 0.45).

| Object | Real product | Source |
|---|---|---|
| Towel stand | Gatco 1444SN countertop towel holder, T-shape, 10.5 x 11.13 in, 1.6 lb | https://www.amazon.com/Gatco-1444SN-Modern-Rectangle-T-Shape/dp/B071S7T274 |
| Bowl | Duralex Lys bowl 14 cm / 500 ml, tempered glass, 14 x 5.5 cm, T542025AF | https://www.kitchenshop.eu/en/glass-bowl-14-cm-500-ml-lys-duralex |
| Cloth | Burt's Bees Baby Organic Cotton Woven Terry Solid Washcloths (3 pack), 10 x 10 in, Blue Solid | https://burtsbeesbaby.com/products/organic-cotton-woven-terry-solid-washcloths-3-pack-cloud |
| Dough stand-in | Play-Doh single can, white, 4 oz (112 g) | https://www.kroger.com/p/play-doh-single-can-white/0063050946340 |

Cost about USD 45. Play-Doh contains wheat; the hands touch only the cloth.

**Estimates** (not published; how to check one in a minute): the stand's base 140 x 15 mm, post 12 mm and bar 13 mm
across (ruler); bowl foot 70 mm, wall 4.5 mm, bottom 6 mm, mass 273 g (ruler,
kitchen scale; the model's inside holds 497 ml against the published 500); cloth mass 29 g and thickness
2.5 mm (kitchen scale, ruler on a folded stack); the cloth's bend: lay it with 30 mm over a table edge and
read the tip's drop with a ruler (estimate 8 mm); all frictions.

## How it is simulated

The cloth is true PhysX cloth (1,369 nodes), spawned hanging over the bar; its bend stiffness makes a 30 mm lip droop about 8 mm, as estimated for the real cloth. While both pads press the cloth it is pinned to the gripper (PhysX cloth has no static friction); opening releases it, and nothing else does. The towel stand is fixed; the dough is a rigid dome in the bowl. A let-go hem may tremble a few mm, so stillness is judged over 1 s. Masses, thicknesses, droop and frictions are estimates.

- **Cloth** (`cloth_kit.py`, a reusable module): a PhysX surface deformable sheet 254 x 254 mm, 1369
  nodes about 7 mm apart, thickness 2.5 mm, 29 g (density 180.3 kg/m3), Young's modulus
  1e+06 Pa, Poisson 0.25, bend stiffness 4.5e+03, damping 1 / bend damping 5, friction 1.5,
  rest / contact offset 1.2 / 5.2 mm, 24 solver iterations, self-collision on. It is
  spawned wrapped round the bar (bent without stretching), 165 mm down the front, and settles before the robot moves.
- **Hold:** `cloth_kit.GripRule` (every control step, any policy) attaches a hand's cloth nodes only while both
  fingers are stopped on the cloth, pressing it with at least 0.5 N; opening or easing the squeeze releases it.
- **Bowl:** one rigid body of 98 convex pieces (foot, 24 x 3 wall pieces, the dough dome), mass
  385 g (glass 273 g estimate + Play-Doh 112 g). Wall 4.5 mm, so the cloth (contact offset
  5.2 mm) cannot pass through the rim.
- **Towel stand:** fixed; a box base, a round post and a round bar 13 mm across, its top 283 mm up.

## Demonstration

Each hand comes from its own side, fingers pointing inward and tipped down 40 degrees, jaws open across the
hanging layer, and slides its pads round a bottom corner of the side facing the robot (15 mm in from the side
hem, about 20 mm above the bottom hem, read from the hanging cloth; the back hem is higher, so only one layer is between
the pads). The fingers close slowly and the contact gate takes the corners. The hands draw the cloth toward the robot
14-17 cm above the counter, so it slides over the bar and comes off when they are beside the near rim and falls over the
bowl, then go straight down beside the bowl's near flank to just above the counter, turning the fingers further down,
wait, open and rise.

Phases (s): approach 0.0-2.2, beside 2.2-3.0, in 3.0-4.5, close 4.5-6.1, settle 6.1-6.7, draw 6.7-8.7, over 8.7-10.7, slide 10.7-12.2, lower 12.2-13.7, place 13.7-15.2, wait 15.2-16.7, open 16.7-17.3, away 17.3-18.8, rest 18.8-21.3.

**Scripted run (simulation, no jitter): 1.0**, first at 20.5 s. Both holds took the corners at 6.0 s (10 nodes each, 1.9 N) and let go at 16.7 s. Three runs with jitter and logging: 1.0 each, shortest overhang 31-40 mm, no sag, bowl moved 0 mm.

**Wrong way (one hand takes its corner, the other stays parked): 0.25.** The cloth comes off the bar twisted and covers 56% of the rim.
