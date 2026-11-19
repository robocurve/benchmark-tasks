# Split a joined pair of disposable chopsticks cleanly and lay the two sticks side by side across a lunch jar

Split the chopsticks and lay them ready across the lunch jar.

## Scoring

Judged at the end from a photo, a ruler and the trial video.

| Score | Outcome |
| --- | --- |
| 1.0 | Clean split, and both sticks laid ready across the jar's lid |
| 0.75 | Clean split, sticks not laid ready (still held, apart, crossed, on the mat or the table) |
| 0.5 | Uneven split, sticks laid ready |
| 0.25 | Uneven split, sticks not laid ready |
| 0 | The pair is still one piece (wherever it is), or a stick has left the table |

- **Clean split (real trial):** each stick whole along its length, the widths of the two heads within 2 mm of each
  other (ruler), no splinter longer than 10 mm, nothing broken off.
- **Laid ready:** both sticks lie flat on the lid, each overhanging it at both ends, eating tips at the same end with
  a gap of 10 mm or less between them, the two sticks within 10 degrees of parallel, both grippers at least 30 mm
  from both sticks, nothing moving.

How `score()` reads each condition:

- **Split:** in a run, the joint between the two sticks has broken (it never mends). In the offline layouts, the
  two sticks are more than 2.5 mm or 3 degrees off their joined pose.
- **Uneven:** the rigid sticks cannot splinter, so this is a rule (`parting`). At the first reading where the tips
  are 1 mm or more off their joined pose, the split is uneven if the tips have moved further past each other (along
  the sticks, or across their thickness as when the pair is scissored or twisted) than apart across the slit.
  Remembered for the rest of the run.
- **Flat on the lid:** the stick's middle within 4 mm of lying on the lid, tilted no more than 8 degrees, its centre
  line over the lid (within 44 mm of the jar's axis) and the jar's axis within 76 mm of the stick's middle along
  its length; the jar itself upright within 5 degrees.
- **Tips together:** the centres of the two tip ends no more than 14.8 mm apart (10 mm gap plus one stick's width)
  and the sticks' long axes within 10 degrees.
- **Let go:** both grippers' tool points at least 30 mm from both sticks' centre lines. **Still:** each stick slower
  than 10 mm per second. **Left the table:** any part of a stick more than 50 mm below the table top.

`success()` is a score of 1.0. Doing nothing scores 0; so does pushing, lifting or turning the unsplit pair.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=split_disposable_chopsticks_and_lay_them_across_the_lunch_jar.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Split the chopsticks and lay them ready across the lunch jar." \
  --scorer task_score --max-steps 1600 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model trial use `--policy dream`. Each trial allows 1,600 control steps (80 s at 20 Hz); the scripted
demonstration takes 25.9 s. `pytest` here checks the scorer on 16 named layouts and the demonstration's reach.

## The scene

Products, sizes and positions are in task.json's `setup` (mm from the midpoint between the arm bases; x forward, y to
the robot's left; heights above the table top). Both arms start at their ready pose.

- **Place mat (fixed slab):** 349 x 451 x 2 mm, grey. It only gives the pale sticks a dark background.
- **Food jar (free body, 363 g):** a 94 mm cylinder 108 mm high with an 88 mm steel lid 34 mm high; 142 mm in all.
  It stands on the mat and is not touched in the demonstration.
- **Chopsticks (two free bodies, 3.9 g each):** each stick is a 240 x 4.8 x 4.8 mm bar plus its half of the 35 mm
  joined web at the head; the two bars are 2 mm apart. Friction 0.8/0.7, 32 position solver iterations, 1 mm
  contact offset, depenetration capped at 1 m/s (`twin_kit.py`).
- **The joint:** one PhysX fixed joint between the two sticks at the root of the slit, 85 mm from the pair's middle
  toward the head, with break torque 0.8 N m and break force 100 N. PhysX alone decides when it lets go; it stays
  broken for the rest of the trial. At the next reset a fresh joint is made, and the pair is shifted as one body.

## Scripted demonstration

Both hands do the same thing, mirrored. The demonstration reads (never moves) its own stick's pose to line up, and
freezes that reading when it closes.

| Time (s) | Each hand |
| --- | --- |
| 0.0 to 5.5 | out to the side of the tips, then level with its stick, 30 mm off, lining up |
| 5.5 to 9.0 | pad tips over its own stick only, 15 mm from the tip end, pads above and below |
| 9.0 to 10.8 | close on the stick (about 7.5 N) |
| 10.8 to 14.8 | pull straight back, 30 mm; the joint lets go in the first second |
| 14.8 to 17.8 | hold, then lift 20 mm |
| 17.8 to 22.9 | bring the stick back beside the other (6 mm between them), lower it onto the lid |
| 22.9 to 25.9 | let go, draw the hand back 80 mm |

## The two grippers work 5 mm apart

A pinch on a 5 mm stick resists turning about the pads' own axis only by friction. Two such pinches pull cleanly
only along the line that joins them, so both hands must hold their stick at the same place along the pair. In the demonstration the
two tool points are 30.6 mm apart and the fingertips of the two grippers are 5.1 mm apart while they hold (7.9 mm when
the sticks are laid back). An exact distance check of the two grippers' collision shapes over the whole
demonstration shows no contact between the arms (smallest gap 5.1 mm).

A real trial therefore needs the two arms to track within about 2 mm of each other at this pose. A lab should check
that first, with no pair on the jar: command both grippers to the grip pose, jaws open 12 mm, and look at the gap
between the fingertips; repeat ten times. If they touch, or the gap changes by more than 2 mm, recalibrate or do not
run the task on that robot.

Wrong-way run: the same demonstration with the left hand 30 mm further along the pair than the right. The unsplit
pair turned 38 degrees between the two pinches; the joint gave 1.8 s into the pull (0.8 s in the demonstration)
with the tips 6.5 mm past each other along the sticks and 0.1 mm apart across the slit: an uneven split by the rule.
The hands then laid the two sticks back on the lid, so the run scored 0.5, the most an uneven split can earn.

## Estimates and one-minute checks

Published: the pair's 240 mm length (pack listing); the jar's 5.6 in height, 3.7 in width and 0.8 lb (maker's
listing; confirm with a ruler and a kitchen scale, 142 mm, 94 mm, 363 g); the mat's 17 3/4 x 13 3/4 in. Everything
else below is an estimate.

| Estimate | Value | Check on the real item |
| --- | --- | --- |
| Stick section | 4.8 x 4.8 mm | ruler or calipers across one stick near the tips |
| Slit width | 2 mm | slide a 2 mm card or coin edge into the slit |
| Joined length at the head | 35 mm | ruler from the head end to where the slit starts |
| Mass | 3.9 g a stick, 7.8 g a pair | ten pairs on a kitchen scale, divided by ten |
| Split moment | 0.8 N m (about 4 N at the tips) | scale check in the reset notes: 250 to 600 g at 15 mm from the tip |
| Lid top | flat, 88 mm across | ruler across the lid |
| Mat thickness | 2 mm | ruler at the mat's edge |
| Friction of the bamboo | 0.8 static, 0.7 sliding | lay a stick on the lid and tilt the jar until it slides: 25 to 40 degrees |

Pair-to-pair spread is real: bamboo pairs differ in how hard they split, which is why each pack is checked once.

## Simplifications

- The sticks are rigid. Real sticks bow by a centimetre or two before they split; here the joint lets go after a few
  millimetres of pull.
- The joint breaks at one moment whatever its direction, and always leaves two whole sticks. Splinters, a lopsided
  head or a snapped stick cannot happen; the uneven tier is the rule above.
- The jar and the mat are plain solids; the jar is empty and stays closed.
- A new pair is needed for every real trial; the simulator makes a fresh joint at each reset.

## Sources

- Pair (example product, 24 cm bamboo twin, pack of 100): https://www.packstyle.jp/category/103001001/00409375.html
- Food jar: https://thermos.com/collections/food-jars/products/stainless-king-food-jar-16oz
- Place mat: https://www.ikea.com/us/en/p/svartsenap-place-mat-gray-20527968/

## Files

- `task.json`: instruction, setup, scoring, run limit, reset and fidelity notes.
- `split_disposable_chopsticks_and_lay_them_across_the_lunch_jar.py`: scene, scorer and scripted demonstration.
- `twin_kit.py`: two bodies joined by a breakable bond, its renewal between trials, readings of the hands.
- `yam_parts.py`: parts kit module, copied unchanged.
- `test_split_disposable_chopsticks_and_lay_them_across_the_lunch_jar.py`: CPU checks of the scorer and the reach.
