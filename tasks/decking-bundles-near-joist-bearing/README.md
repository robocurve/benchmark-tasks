# Land decking bundles across three joists, within a foot of the bearing end

Two decking bundles go onto a bay of steel joists, a 1:7.6 scale model (1 ft = 40 mm). On steel erection jobs,
decking bundles are often landed on joists before all bridging is installed. OSHA 1926.757 allows that only if, among
other conditions, the joists are attached at both ends and at least one row of bridging is installed and anchored
((e)(4)(iii)-(iv)), the bundle lies across at least three joists ((e)(4)(ii)), and its edge is within 1 ft of the
bearing surface ((e)(5)), because a mid-span load can buckle joists not yet fully bridged. The natural landing, centred in the bay, is the one the rule
forbids. The bundles lie on the left, out of the right arm's reach, so the left arm lands one across the left three
joists and passes the other on a mat between the arms; the right arm lands it across the right three.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
INSTRUCTION="Land the two yellow decking bundles as OSHA 1926.757(e)(4) and (e)(5) require: one flat across the left three grey joists and one across the right three, each square to the joists with its long edge within 40 mm (1 ft at this scale) of a red girder, never at mid-span and never stacked. The blue mat between the arms is the handover spot."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=decking_bundles_near_joist_bearing.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/decking-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=decking_bundles_near_joist_bearing.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/decking-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/decking-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once both halves of the bay are landed in full and
both bundles have stayed still for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_decking_bundles_near_joist_bearing.py` in an
environment with `yam_arena` installed.

## Scoring

Each half of the bay is worth 0.5: the left half is joists 1 to 3 (the three left of the centre line between the arm
bases), the right half joists 4 to 6. Each bundle counts for at most one half. The bundles are identical, so either
may land on either half; the pairing of bundles to halves that scores more is used.

| Credit | For |
|---|---|
| 0 for a bundle | unless it is still (moving under 1 cm/s and turning under 0.5 rad/s), its lowest point from 3 mm below to 0.5 mm above the joist tops (50 mm above the table: not on a girder, the table, the mat or the other bundle, and not held above them), its outline between the two girder faces (within 2 mm), tilted at most 3 degrees (either big face up), and its long side within 15 degrees of square to the joists |
| 0.5 x S x D | a bundle that passes those, for its half |
| S = 1 / 0.5 / 0 | all 3 / exactly 2 / 1 or none of its half's joists that have at least 15 mm of their 20 mm top width under the bundle's centre line (the 220 mm line along its length through its centre), measured across the joist, and that the bundle sits down on: where each of its two long bottom edges crosses the joist's centre line, its bottom is from 3 mm below to 0.5 mm above that joist's top. A bundle still held with one end or one long edge raised gets no count for the joists it hangs over; the other half's joists don't count |
| D = 1 | gap g of 40 mm (1 ft) or less from the nearer girder face to the bundle's long edge on that side, measured at the end of that edge farther from the girder; either girder counts |
| D = (70 - g) / 30 | g between 40 and 70 mm, falling linearly |
| D = 0 | g of 70 mm or more (a square bundle's centre is then 100 mm from the girder face, the edge of the mid-span band) |
| 0 overall | a bundle on the joists (lowest point from 3 mm below to 0.5 mm above their tops, over a joist) with its centre 100 mm or more from both girder faces, however it is turned: the middle 100 mm of the 300 mm span, 33 to 43 cm in front of the arm bases |
| 0 overall | a bundle on the joists (as above) with any part of its outline seen from above reaching the middle of the span, 38 cm in front of the arm bases |
| 0 overall | one bundle resting on the other: its bottom from 5 mm below to 0.5 mm above the other's top, their outlines seen from above overlapping by more than 2 mm |

A bundle left on the handover mat, on the table or on the floor earns nothing. A 220 mm bundle covers all three
joists of a half only when its centre is within 25 mm of that half's middle joist (12 cm left or right of the centre
line); centred between two joists, it covers two. A bundle over joists 2 to 4 covers only two of the left half's.

On a real table, a tape measure gives the gap from the girder face to each bundle's edge and where the bundle's centre
line crosses the joists; a phone level app gives the tilt, and a photo from above the angle to the joists.

## The scripted demo

Both arms grip straight down across a bundle's 60 mm width, the fingers closing front to back, 32 mm above its
bottom. The left arm lands the nearer bundle centred on the second joist from the left (12 cm left of the centre
line), its near long edge 10 mm from the front girder face, letting go 6 mm above the joists. It then sets the other
bundle on the mat and parks beside the bay. The right arm waits until then, takes the bundle off the mat and lands it
the same way centred on the second joist from the right, then parks beside the bay so the camera sees both bundles.

## Runtime and seeds

- The demo's plan takes about 33 s of simulated time; replayed offline, the last bundle is let go at 30.3 s and
  success holds by about 33.4 s (about 670 steps at 20 Hz, including the 2.5 s hold), within the 1,200-step limit.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
