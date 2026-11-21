# Spin a lazy Susan on a pantry shelf to bring the spice jar at the back to the front, then take it out

A pantry shelf section stands on the counter against the kitchen wall. On its bottom shelf, at the front edge, a lazy
Susan carries six spice jars in a ring; the one wanted, with a red cap, stands at the back by the wall. The shelf above
is only 4 cm over the jar tops and the jars stand close together, so the red-capped jar cannot be lifted out over the
others or reached between them. One hand turns the tray by its lip, hand over hand, until the red-capped jar comes round
to the front, and keeps hold of the lip; the other hand lifts the jar over the lip and stands it on the counter mark.
No jar may tip or be flung off.

## Instruction

Take the jar with the red cap from the back of the lazy Susan and stand it on the counter mark in front of the shelf,
without tipping any jar.

The instruction gives the goal only; that the tray has to be turned first is left for the robot to work out.

## What is new here

The store is turned, not the items: the hidden jar is brought to the open front by indexing a free-spinning carrier
loaded with tall, light jars, which tip or slide if the tray is turned too hard.

- **#426** (MERGED, pull a board to retrieve an out-of-reach jar): there one jar sits on a board, and pulling the board
  straight in brings it within reach; nothing else rides on it. Here the carrier turns about a fixed bearing, six loose
  jars ride on it and tip or slide if it is turned too hard, the jar wanted is hidden behind the others under a low shelf,
  and it is turned in several pinch strokes, hand over hand, then held still while the other hand takes the jar.
- **#482** (FINAL, reach a spice jar at the back by moving the bottle in front): there the jar in front is moved out of
  the way. Here nothing in front is touched; moving another jar off the tray scores 0.
- **#393** (FINAL, share one turntable, framed as a safety task): there a single block rides the turntable between the
  arms. Here the turntable carries six loose jars that must all stay upright, and it is turned 180 degrees in several
  pinch strokes under a low shelf to bring one hidden jar to the front.
- **#674** (FINAL, a cake turntable): there the turntable spins under a smoothing tool and the load is a rigid dummy.
  Here the load is the point: six free jars ride on the tray by friction, and the turn is a positioning task (the red jar
  to within 30 degrees of the front), not a continuous spin.

## Why the task matters (category Societal relatability)

- Cooks fetch from storage and rotate stock every shift. O*NET 35-2014.00 (Cooks, Restaurant) lists as a core task
  (importance 91) "Ensure freshness of food and ingredients by checking for quality, keeping track of old and new items,
  and rotating stock", and as a supplemental task "Estimate expected food consumption, requisition or purchase supplies,
  or procure food from storage." O*NET gives 1,417,500 restaurant cooks employed in 2025 and 218,600 projected openings
  for 2025-2035.
- The same reach into a cabinet or pantry shelf for a jar at the back is done daily in home kitchens; turntables are sold
  for exactly this (the OXO 1071352 used here is a stock item at restaurant suppliers such as WebstaurantStore).

Limits: no source gives how often a jar is fetched from a turntable, or how many kitchens have one.

Sources: https://www.onetonline.org/link/details/35-2014.00 ;
https://www.webstaurantstore.com/oxo-good-grips-11-lazy-susan-turntable/2971071352.html

## Scene and positions

Positions in cm, x forward from the arm bases' midpoint, y toward the left arm, z up from the table top (the counter).

- **Shelf section** (IKEA IVAR 42 x 30 cm pine shelves, 1.8 cm thick, on IVAR side units sawn off just above the upper
  shelf, so nothing stands where the arms start): the bottom shelf on the counter,
  front edge at x 32, x 32 to 62, centred on y 2; the upper shelf's underside 20.0 cm above the counter; the kitchen wall
  at x 62.5.
- **Lazy Susan** (OXO Good Grips 11 in, model 1071352: 11 x 11 x 1.5 in, usable 9.25 in, steel ball bearings, 0.39 kg):
  tray 27.9 cm across with a 1 cm lip around a 23.5 cm floor, turning about a vertical axis at (46.0, 2.0); its front
  edge flush with the shelf's front edge; the tray floor 4.8 cm above the counter. The bearing is a hinge with Coulomb
  friction 0.02 N m.
- **Jars** (McCormick ground cinnamon 2.37 oz, UPC 052100004389: 1.77 x 1.77 x 4.38-4.41 in, 88-91 g): six, 4.5 x 4.5 x
  11.2 cm, 89 g, on a ring of radius 8.5 cm, 60 degrees apart; the red-capped one at the back (azimuth 0, by the wall).
  Jar-on-tray friction: polypropylene, static 0.76 / dynamic 0.44 (INEOS PP engineering properties).
- **Counter mark:** a 2 cm radius circle at (24, 2), 8 cm in front of the shelf.
- Both grippers start open at the ready pose.

## Scoring

All from poses. The red jar's azimuth is measured about the tray's axis, 0 at the back, 180 at the front.

| Criterion | Simulation | Real trial |
|---|---|---|
| No other jar tipped | Every other jar within 15 degrees of upright, all through the run (latched) | No jar lying or leaning on another at any time in the video |
| No jar off the tray | Every other jar's centre within the 23.5 cm usable floor, at floor height (latched) | No jar off the tray at any time |
| Tray on its bearing | The tray never more than 1.5 cm above its seat (latched) | The tray never lifted off its base |
| Red jar fronted | The red jar, upright on the tray, within 30 degrees of the front | Red jar between two tape marks on the shelf's front edge, 30 degrees each side of the front |
| Red jar on the mark | Its centre within 2 cm of the mark, base on the counter, within 5 degrees of upright | Jar base inside the 2 cm circle, standing |
| Finished | No jar held; every jar moved under 2 mm and turned under 2 degrees over the last second | Both grippers open and away from the jars; no jar moving in the last second of the video |

Score: **0** if any latch above trips (another jar tipped or off the tray, the red jar tipped on the tray, the tray lifted),
or if the red jar never came round to the front on the tray. **0.5** once the red jar has come round to the front with
nothing tipped. **1.0** with, as well, the red jar standing on the counter mark, no jar held and every jar still for one second. Doing
nothing scores 0; taking the red jar out without turning the tray scores 0.

## Real trial hardware

Prices seen 2026-11-21 in live US listings.

| Item | Listing | Price |
|---|---|---|
| Lazy Susan | OXO Good Grips 11 in Lazy Susan Turntable 1071352, WebstaurantStore: https://www.webstaurantstore.com/oxo-good-grips-11-lazy-susan-turntable/2971071352.html | $16.99 |
| Spice jars | McCormick Ground Cinnamon 2.37 oz, Shoppers (sale; regular $4.59): https://shop.shoppersfood.com/store/shoppersfood/products/87608-mccormick-ground-cinnamon-2-37-oz | 6 x $2.99 |
| Shelves | IKEA IVAR shelf, pine, 17 x 12 in, 103.181.59: https://www.ikea.com/us/en/p/ivar-shelf-pine-10318159/ | 2 x $10.00 |
| Side units | IKEA IVAR side unit 11 3/4 x 49 in, 904.846.49: https://www.ikea.com/us/en/p/ivar-side-unit-90484649/ | 2 x $15.00 |
| **Priced total** | | **$84.93** |

Not priced (gaps): red tape for the target's cap, masking tape for the counter mark and the 30 degree marks, a saw to
cut the side units down.

## Run a trial

From the package directory, in the official Arena environment:

```sh
inspect-robots run --embodiment yam_arena -E task=spin_lazy_susan_bring_back_jar_to_front.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy yam_scripted --scorer task_score --max-steps 2000 --instruction "Take the jar with the red cap from the back of the lazy Susan and stand it on the counter mark in front of the shelf, without tipping any jar."
```

Dream-1:

```sh
inspect-robots run --embodiment yam_arena -E task=spin_lazy_susan_bring_back_jar_to_front.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream --scorer task_score --max-steps 2000 --instruction "Take the jar with the red cap from the back of the lazy Susan and stand it on the counter mark in front of the shelf, without tipping any jar."
```

The run limit is 2,000 steps (100 s at 20 Hz); the scripted demo ends in about 1,330 steps.

Runtime per trial: about 13 min on one RTX 5090 shared with another job (the local scripted-demo run took 766 s
including start-up).

Seeds: none; the start is fixed (`object_jitter=0`).

**The scripted demo** is closed loop. The left hand pinches the tray's lip on its front-left and carries it round up to 45
degrees per stroke, each stroke's size set as it starts from the tray's angle, lets go, moves back and pinches again;
the stroke that brings the red jar to the front keeps hold of the lip and holds the tray still. The right hand then comes
in from the front-right to the red jar where it now stands, closes on it, lifts it 16 mm over the lip, draws it off the
shelf and stands it on the counter mark. The left hand lets go and both hands move clear.

Demo results: the scripted demo scored 1.0 in the GPU simulator (local Isaac, 1,323 steps, the red jar 9 mm from the
mark centre in a traced run). On the cloud simulator the demo scored 1.0 (1,323 steps) and Dream-1 scored 0.0 over the full 2,000-step run. In
offline MuJoCo the demo scored 1.0 (1,325 steps) at bearing friction 0.005, 0.02 and
0.05 N m, with every other jar within 0.1 degrees of upright.

**Shortcut variants** (development only, not shipped), scored in offline MuJoCo at 0.02 N m:

| Variant | What it does | Score |
|---|---|---|
| `nothing` | no motion | 0.0 |
| `no_spin` | the tray is not turned; the right hand goes for the red jar where it stands at the back | 0.0 (two jars knocked over) |
| `fling` | the same strokes carried ten times faster | 0.0 (the jars topple) |

The offline checker's own shortcuts (nothing, sweep, lift-and-drop, stopping half way, undoing the take) all score 0.
Its start check lists the three jars at the back as out of a straight-down grip's reach: they are under the upper
shelf, which is the point of the task. In its robustness runs the demo scored 1.0 in four of five start jitters
(+-5 mm, +-3 degrees) and 0.5 in the fifth, and 1.0 with friction x0.7 and 0.5 with friction x1.3; the task's start is
fixed (`object_jitter=0`).

## Offline tests

With Arena on the import path: `python -B -m pytest -q -p no:cacheprovider`. The tests drive the score bookkeeping step
by step with posed states (no simulator): the score tiers, the 30 degree front sector, the latches (a tipped or flung
neighbour, the red jar tipped on the tray, the tray lifted), the one-second stillness rule, taking the jar out without turning scoring 0, and the demo's
reach (every stroke pose within 0.5 mm of the official IK) and length, and that at the end of the GPU-simulator run
neither arm blocks the top cameras' sight lines to the red jar on the mark or to the tray's front lip (with a
positive control from the same run, the right hand over the lip as it reaches in).

## Fidelity

Rigid bodies with real contact: the tray turns on a hinge with Coulomb friction, the jars stand loose on it and ride
round by friction, and the gripper turns it by pinching its lip. The bearing's friction is not published and is checked
over a range ([DEVIATIONS.md](DEVIATIONS.md)).

## Licence

| Path | Origin | Licence |
|---|---|---|
| *.py, task.json, README.md, DEVIATIONS.md | Task, tests, documentation and bundled Arena helpers | MIT |

Every part is built from boxes in the task's own code; no third-party assets. [LICENSE](LICENSE)
