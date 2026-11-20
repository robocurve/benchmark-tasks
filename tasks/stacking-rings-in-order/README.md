# Stack a baby's rocking ring-stacker toy: all five rings onto its tapered post in size order, largest at the bottom

A five-ring rocking stacker matching the sizes below (for example the classic five-ring Fisher-Price Rock-a-Stack)
stands in the middle of the table. Its five rings, red (largest, 124 mm across) to blue (smallest, 89 mm), lie flat on
the table around it, mixed up. The
instruction states only the goal: *Stack all five rings on the toy's post in order of size, the largest at the bottom
and the smallest at the top, and leave the toy standing upright.*

A ring-stacking toy is in most homes with a baby, and stacking its rings in size order is a standard developmental
milestone: the North Carolina Early Learning and Development Progressions (UNC Frank Porter Graham Child Development
Institute) list "stacks five circular rings in correct size order" at about two years. What makes it a task:

- **The order matters on the toy itself.** The Rock-a-Stack's rings "have to be placed in order of size onto a tapered
  pole" ([Wikipedia](https://en.wikipedia.org/wiki/Rock-a-Stack)). In the model each ring's hole is 3 mm wider than the
  post at its own place in the stack, so a ring put on before the larger ones below it stops about 15 mm short of its
  place, part way down the post, and every larger ring is too big to pass it: the rings have to be told apart by size
  and put on largest first, and a wrong ring has to be taken off again. (How far short a ring stops on the real toy is
  not published; the order is in any case what the instruction asks and what the score checks.)
- **Each ring is threaded over the post's ball top** (32 mm across, 196 mm above the table), its hole as little as
  41 mm across. Four of the five rings are wider than the robot's hand opens (94 mm), so they have to be taken by
  the wall, one finger in the hole.
- **The toy is light and rocks.** It weighs 235 g in all on a weighted rocking base; a ring that lands off-centre or
  a gripper that knocks the post sets it rocking.

**Not a near-duplicate of hanging an ornament on a stand.** In the accepted task "Hang two glass baubles on a
dowel-branch ornament stand by threading each bauble's loop", the skill is guiding a hook over a branch. Here nothing
is hooked; a ring drops down a post by itself once it is over the top. The skill measured is choosing which ring goes
next (comparing five sizes) and the consequence of getting it wrong is physical: the taper blocks the stack.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `stackkit.py` | A small helper for rigid bodies made of several shapes (boxes, cylinders, spheres, meshes), a fixed joint between two bodies and collision filtering |
| `demo.py` | The scripted demo |
| `test_scoring.py` | Offline checks (no GPU): every scoring outcome and attempts to score without doing the task, the runtime's ending, the taper's geometry, the layout, the demo's reach, and that `task.json` states the numbers in the code |

`stackkit.py` hooks `yam_arena.scene` and the embodiment's reset and step without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and the `task_score` scorer. Nothing in it applies forces or switches joints
during a trial: the rings, the toy and the arms interact only through PhysX contact, friction and gravity.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir runs/stack-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Set
`STACK_DEBUG=1` to print, every 10 steps, each ring's height above its place, its distance from the post's axis, its
tilt and the nearest gripper, and the score's conditions.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 2400 --cam 640x352   # demo_*_cam.mp4, setup.png, result.json
DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record       # prints the CAL numbers (see below)
python -m pytest -p no:cacheprovider test_scoring.py              # offline checks
```

The rings go on largest first; each arm takes the rings on its own side of the table (left: red, yellow, blue; right:
orange, green), with the grip pitched 30 degrees, fingers leaning forward (straight down, the arms cannot reach the
post's height over the toy).

1. Each ring is taken by its wall, one finger in the hole and one outside (four of the five are wider than the hand
   opens), carried over the post and let go with its bottom 15 mm above the post's ball top: it drops down the post
   onto the ring below.
2. While one arm places a ring, the other goes back to its ready pose and then fetches the next one. At the end both
   arms are back in their ready poses and the toy stands on its own.

The demo runs open loop: it reads no simulator state. Where each ring sits in the hand once lifted is measured once
with `DEMO_CALIBRATE=1` and fixed in `CAL` in `demo.py`. The layout is fixed and the simulation deterministic.
`result.json` reports `score()` on the last step as the trial's score.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

A ring counts as **stacked** when the post goes through its hole (the post's axis is inside the hole), its axis is
within 10 degrees of the post's, it rests at its own place (the red's bottom within 5 mm of the base's top, each next
ring's bottom within 5 mm of the top of the next larger one), neither gripper is within 5 mm of it, and the toy is
upright (the post within 10 degrees of vertical). Rings are counted from the bottom in size order (red, orange, yellow,
green, blue), stopping at the first that is not stacked.

- **1.0** once all five are stacked, no part of either gripper is within 2 cm of the toy or any ring (in simulation:
  every vertex of both grippers' finger and body collision hulls), and nothing moved more than 2 mm over the
  preceding 1 s, all held together for 1 s (20 control steps in a row). `success()` returns true only on steps where
  `score()` is 1.0; the runtime ends the episode once `success()` has stayed true for its own 1 s, so an episode can
  only end with success at 1.0.
- **0.75**: all five stacked, but not yet left still and alone for 1 s.
- Otherwise **0.15 for each ring counted** (0, 0.15, 0.3, 0.45 or 0.6). A ring above a missing or out-of-order ring
  earns nothing, so a stack with a smaller ring stuck part way down earns only the rings below it.

In a real trial: the gaps between rings with a ruler (a seated ring leaves no gap under it), the order and the post's angle on a photo, the 5 mm and 2 cm clearances by eye.

## How it is modelled

All parts are rigid bodies at the toy's size; nothing is fixed to the table.

- **Rings**: 20 straight box segments each (a 20-sided ring of square section, 30 mm tall), with the outside and hole
  diameters above. Polypropylene on polypropylene, friction 0.45 static / 0.35 dynamic (estimate).
- **Post**: 31 stacked cylinders 5 mm tall, each the cone's diameter at its own bottom (60 mm at the base down to 32 mm
  155 mm up), and a 32 mm ball on top. A ring on the bare post comes to rest on the highest section wider than its hole.
- **Base**: 125 mm across and 25 mm tall: a 13 mm straight wall on a rocking dome, a spherical cap 12 mm deep (169 mm
  radius of curvature). The base and post weigh 115 g with their centre of mass 10 mm above the bottom (the real base
  is weighted), far below the dome's centre of curvature even with every ring on, so the toy rocks back upright.
- **The dome's contact with the table** is a true sphere of the dome's 169 mm radius, in a 5 g body welded to the base
  by a fixed joint, that collides only with the table (PhysX filtered pairs keep it from the rings and both arms). A
  convex mesh of the dome rests on its flat facets instead of rocking back (tried: a 249-point dome stood stably 8
  degrees over; a signed-distance-field mesh never stopped wobbling). Only the sphere's bottom 12 mm, which is exactly
  the drawn dome, can reach the table; past 21.7 degrees of tilt the base's rim, on the same sphere, touches it. The
  drawn dome itself does not collide, so a ring pushed under the base's rim, within 12 mm of the table, would pass
  into it.
- **Rocking losses**: an angular damping of 1.5/s on the base stands in for the rolling resistance that stops a real
  toy rocking after a few swings.

**Simplifications.** Rings are rigid and square in section (the real ones are rounded and hollow); the taper is
stepped by 0.9 mm every 5 mm; the masses, the base's centre of mass and the rocking damping are estimates.

## Numbers and where they come from

The task uses a five-ring rocking stacker matching the sizes below; the classic Fisher-Price Rock-a-Stack (5 rings) is
the example the sourced numbers come from.

| Quantity | Value | Source |
|---|---|---|
| Overall size and weight | 196 mm tall (25 mm base + 171 mm post), 125 mm across; 235 g in all | Sourced: Amazon's listing of the classic Rock-a-Stack (FGW58) gives 5.12 x 4.72 x 7.76 in (130 x 120 x 197 mm) and 8.3 oz (235 g); Kohl's gives 7.75 x 4.72 x 5.11 in |
| Base | 125 mm across (modelled round), 25 mm tall | Sourced: This Old Toy's base list gives the #1050 Rock-a-Stack base as 5 in wide x 4-3/4 in deep x 1 in |
| Ring outside diameters | 124, 114, 105, 95, 89 mm | Sourced: This Old Toy lists the five rings as 4-7/8, 4-1/2, 4-1/8, 3-3/4 and 3-1/2 in (red, orange, yellow, green, blue; #627 1980-89 and #1050) |
| Rings in size order on a tapered pole | | Sourced: Wikipedia, "Rock-a-Stack" |
| Ring height | 30 mm each | **Estimate**: five rings fill the 150 mm of post under its ball within the sourced 197 mm height |
| Post taper | 60 mm to 32 mm over 155 mm, 32 mm ball | **Estimate**; **measure** |
| Ring holes | 63, 57.5, 52, 46.5, 41 mm (3 mm wider than the post at each ring's place) | From the estimated taper; **measure** |
| Ring masses | 30, 27, 24, 21, 18 g | **Estimate** (120 g of the sourced 235 g); **weigh** |
| Base and post | 115 g, centre of mass 10 mm above the bottom; dome 12 mm deep (169 mm radius of curvature) | Mass: the sourced 235 g less the rings; the rest **estimate**; balance the base on a knife edge on its side for the centre of mass |
| Friction | 0.45 / 0.35 | **Estimate** for moulded polypropylene |

Sources: [This Old Toy, #1050 Rock-A-Stack](https://thisoldtoy.com/l_fp_set/toy-pages/1000s/1050-rockastack.html);
[This Old Toy, #627 Rock-A-Stack](https://thisoldtoy.com/l_fp_set/toy-pages/600-699/627-rockastack.html);
[This Old Toy, Rock-A-Stack base list](https://thisoldtoy.com/fisher-price/dept-5-nursery/b-beads-blocks-rings/rings-base.html);
[Amazon, Fisher-Price Rock-a-Stack FGW58](https://www.amazon.com/Fisher-Price-FGW58-Rock-a-Stack/dp/B01LYNRB3Y);
[Kohl's, Fisher-Price Rock-a-Stack](https://www.kohls.com/product/prd-4141250/fisher-price-rock-a-stack.jsp);
[Wikipedia, Rock-a-Stack](https://en.wikipedia.org/wiki/Rock-a-Stack).

A lab using another edition measures it with calipers and a kitchen scale and puts the numbers in `task.py`
(`RING_OD`, `RING_H`, `POST_D0`, `POST_D1`, `POST_L`, `RING_MASS`, `BASE_MASS`); the holes follow from the taper.

## Real-world set-up

All parts are bought: one five-ring rocking ring-stacker toy matching the sizes above (for example the classic
Fisher-Price Rock-a-Stack, sold in most toy shops). Mark the base's place and each ring's centre on the table with a small piece of tape (positions in
`task.json`). Nothing is consumed; the toy is soft plastic and safe to drop.

## Runtime and seeds

The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. A trial runs at most 2,400 steps (2 minutes).

| Run | Score | Steps |
|---|---|---|
| Scripted demo (`demo.mp4`, 640 x 352) | 1.0 | script ends at step 1,120; full score from step 1,149; the episode ends on success at step 1,168 (58 s) |
| Dream-1 (`dream.mp4`, 320 x 176, the command above) | 0.0 | reached for and worked at the red (largest) ring but put no ring on the post before the 2,400-step limit |