# Draw a pair of cafe curtains shut along their rod until the two leading edges meet within 2 cm

Close the curtains: draw both panels along the rod until their inner edges meet in the middle.

## Scoring

Judged at the end of the run from the top-camera video, a top photo and a ruler. Gap: the widest distance between the two panels' inner side edges, measured across at the clips, halfway down and at the hems (negative where they overlap). A panel is drawn when its leading clip ring has moved at least 5 cm toward the middle. Score 0 if either gripper still holds a curtain or its tool point (between the fingertips) is within 2 cm of one, the curtains are still moving (any point more than 1 cm from where it was 1 s earlier), a ring came off the rod at any time, or either panel is not drawn (doing nothing scores 0; the start gap is 21 cm). Otherwise the lowest of: (1) closure: 1.0 if the gap is at most 2 cm; 0.75 if over 2 and at most 5 cm; 0.5 if over 5 and at most 10 cm; 0.25 if over 10 cm. (2) rod: 1.0 if no cloth lies over the rod; 0.5 if some does.

How `score()` reads each condition (cloth: each panel's 1,922 node positions, `objects["curtain_l"].nodes` and
`objects["curtain_r"].nodes`; rings: each ring body's origin):

- **Gap:** the nodes of each panel's inner side edge (one per row, from the top hem to the bottom hem); the gap is
  the largest left-minus-right difference in y over the rows, so the widest place counts.
- **Drawn:** how far each panel's leading ring has moved toward the middle from where it started; both must reach
  50 mm.
- **Off the rod:** a ring whose centre is more than 20 mm from where a ring hangs on the rod (or past a board); checked
  at every step, so a ring knocked off and put back still fails.
- **Over the rod:** any cloth node higher than the rod's top.
- **Let go:** no contact-gated hold is on and both grippers' tool points are at least 20 mm from every cloth node.
  **Still:** no cloth node has moved more than 10 mm over the last 1 s (the scorer keeps the nodes of the last
  second).
- `success()` is a score of 1.0; the episode ends once it has held for 1 s.

## Run it

```
inspect-robots run --embodiment yam_arena -E task=draw_cafe_curtains_shut_along_their_rod.py --policy yam_scripted   # scripted demonstration
inspect-robots run --embodiment yam_arena -E task=draw_cafe_curtains_shut_along_their_rod.py --policy dream           # model run
pytest test_draw_cafe_curtains_shut_along_their_rod.py                                                                  # offline checks (CPU)
```

Run limit 1,200 control steps (20 Hz, 60 s). The scripted demonstration lasts 27.3 s (546 steps) and succeeds at
about 24 s. About 5 control steps per second on an L40S: the demonstration takes about two minutes after start-up. No seed is needed: the start
is fixed (`-E object_jitter` moves the rings by up to its value; 2 mm was tested).

## What is different

Drawing curtains is the commonest cloth job in a home and needs no special skill from a person, yet it is not simple
for a robot: the panel is limp and gathered in folds, it hangs from loose rings on a thin rod, and the only firm thing
to act on is a ring whose clip hangs below the rod. Other curtain tasks hang a curtain onto its rod or hook a liner's
grommets; here the curtain is already hung and must be slid closed. The robot has to (a) get each hand under the rod
and just above the folded cloth from the side, (b) push the leading clip ring along the rod without lifting it off or
jamming it, so the ring drags its panel and the folds open, (c) stop with the two leading clips side by side, and (d)
take turns in the middle, because each panel is beyond the other arm's reach (the left base is 45 cm from the right
panel's leading ring; reach is 36 cm). Drawing one panel, pushing a ring off the rod, throwing cloth over the rod or
leaving a hand on the cloth all lose credit.

## Scene and real objects

Millimetres from the midpoint between the arm bases (bases 610 apart): x forward, y to the robot's left, heights
above the table. The real objects cost a few tens of US dollars in all (rod about $4 at the listed retailers; the rest estimated).

- **Rod:** Kenney KN630/1 Strafford spring tension rod, white steel, 7/16 in (11.1 mm), adjustable 18-28 in, rubber
  ends ([McCoy's](https://www.mccoys.com/shop/p/1007632-052303-19/kenney-kn6301-spring-tension-rod-716-in-dia-18-to-28-in-l-metal-white),
  [Menards](https://www.menards.com/main/window-treatments/curtains-drapery-hardware/kenney-reg-18-28-spring-tension-rod/kn630-1v1/p-1554272864676-c-9260.htm)).
  Sprung between the boards' inner faces, level, its axis at x = 170 and 362 above the table.
- **Window frame:** two pine 1 x 3 boards (19 x 64 mm actual), 380 long, upright, 64 mm sides along x, centred on
  x = 170, inner faces at y = +230 and -230; each screwed to a 200 mm piece of pine 1 x 4 (19 x 89 mm) lying flat
  outside the window and held to the table with a small C-clamp, so the rod's spring cannot spread them. A real
  window this narrow is a small kitchen or bathroom window; the width is set by the arms' start pose (their wrists
  are 32 cm up and 25-38 cm to each side, so a rod at this height fits only between them).
- **Curtains:** Lino Rosa cafe tier pair, "24 x 12 Inch Long 2 Panels Set", linen-cotton-polyester blend
  ([listing](https://www.amazon.com/Curtains-Striped-Farmhouse-Kitchen-Bathroom/dp/B0F1TGFJBL); size and material from
  the listing). Each panel 610 wide and 305 long, about 32 g (estimate; kitchen scale). The 12 in length is the
  shortest tier sold; it is why the rod sits 36 cm up.
- **Clip rings:** Kenney window curtain clip rings, black carbon steel, 1 in (25.4 mm) inside diameter, set of 14
  ([listing](https://www.amazon.com/Kenney-Window-Curtain-Rings-Black/dp/B009U1ZQBS)); 12 used, six per panel. Ring
  mass about 9 g, wire 4 mm, clip about 12 wide, 6 thick and 16 tall (estimates; ruler and scale). Each panel's top
  hem is clipped every 122 mm: the first clip with its side flush with the panel's inner edge, the last at the outer
  corner. The cloth's top then hangs about 48 below the rod's axis and the hems clear the table by about 10.
- **Start (open):** the left panel's leading clip at y = +111 (its edge at +105), the next ring at +165, then +179,
  +193, +208 and +222; the right panel the mirror image. The cloth between each two clips lies in one fold, the first
  on the far side of the rod, then alternately near and far. Both arms at their ready pose, grippers open.
- **Friction:** ring on rod about 0.2 sliding, 0.25 to start (estimate: tilt the rod until a ring starts to slide;
  friction = tan of that angle).
- **Reset (about 1 minute):** open both grippers, arms to the ready pose; check every ring is on the rod and the rod
  level at 362; slide each panel back (leading clip's middle 111 from the middle, a pencil mark on the rod; next ring
  at 165; the rest about 14 apart to the board); settle one fold between each two clips, the first on the far side;
  let the panels stop swinging. Re-clip a ring that slipped; replace the pair when creased hard, torn or stained.

## How it is simulated

- **Cloth:** each panel is a PhysX surface deformable (1,922 nodes, 10 mm edges, 2.5 mm thick, self-collision on),
  built by `cloth_kit.py`. It starts in its open shape, laid out without stretching: the top edge follows the folds
  between the clips (corners rounded over 8 mm) and the cloth hangs straight down from it.
- **Rings:** each ring is a rigid body (`curtain_kit.py`): a torus of 12 convex pieces around the rod with an eyelet
  and a clip below it, 9 g. It hangs on the rod and slides, swings or tilts on it by contact and friction only. Each
  clip holds the cloth within 10.5 mm of its middle and 10.5 mm of the top edge by a fixed vertex attachment (the
  clip does not slip). Rod and boards are fixed.
- **Grip:** pads hold cloth only by `cloth_kit`'s contact-gated rule (both pads stopped on the cloth with a measured
  squeeze; opening releases it). The demonstration never grips.
- **Estimates:** panel mass and stiffness, cloth thickness, ring mass, clip size and ring-on-rod friction (0.25 to
  start, 0.20 sliding).

## Demonstration

Each hand rises over its own shoulder, swings round high over its panel's gathered folds, turns its closed fingers to
point forward and 20 degrees toward the middle (stacked one above the other), comes down in front of the rod just
outside its panel's leading clip, moves in under the rod (fingertips about 5 mm under it, the finger base about 5 mm
above the folds' top edge), and pushes the leading clip ring along the rod with the side of its fingers until the
ring's middle is 7 mm from the middle of the rod. It then backs out, rises and waits over its panel. The right hand
goes first; the left starts once the right is backing out, so the two hands never meet. The leading ring drags its
panel; the second ring follows when the cloth between them goes taut; the folds open. In the simulator the leading
rings end 7-9 mm from the middle and the gap is 8-10 mm.

Wrong way (one hand): only the left hand draws its panel; the right panel stays open, so the right panel is not drawn
and the score is 0.
