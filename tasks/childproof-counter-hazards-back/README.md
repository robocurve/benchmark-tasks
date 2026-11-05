# Childproof a counter: hazards away from the toddler, the sippy cup to them

A toddler stands on a step-stool at the left end of a kitchen counter and can reach about half a metre along it.
The robot must move a knife, a hot mug and a pill bottle at least 550 mm from that end, past a blue divider, and bring
the toddler's own sippy cup from the far right to within 150 mm of it. The instruction names the four objects but not
where each one goes, so the robot has to tell the hazards from the child's cup itself: the hazards go away from the
child and the cup comes toward them, so clearing everything to one side does not earn full score. The cup
starts beyond the left arm's reach and the safe side is beyond it too, while the toddler's end is beyond the right
arm's reach, so every object is handed from one arm to the other at a spot on the counter's midline.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=childproof_counter_hazards_back.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "A toddler is standing on the yellow stool at the left end of the counter, which holds a grey knife, a red mug of hot coffee, an orange bottle of pills and the toddler's own green sippy cup. Childproof the counter: move whatever could hurt the toddler to the right of the blue divider, out of their reach, and whatever belongs to the toddler to their end of the counter, keeping the knife lying down and the rest upright on the counter." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/childproof-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=childproof_counter_hazards_back.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "A toddler is standing on the yellow stool at the left end of the counter, which holds a grey knife, a red mug of hot coffee, an orange bottle of pills and the toddler's own green sippy cup. Childproof the counter: move whatever could hurt the toddler to the right of the blue divider, out of their reach, and whatever belongs to the toddler to their end of the counter, keeping the knife lying down and the rest upright on the counter." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/childproof-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/childproof-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all three hazards are out of reach and the cup is
at the toddler's end, all still, for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_childproof_counter_hazards_back.py` in an
environment with `yam_arena` installed.

## Scoring

Distances are measured along the counter, side to side, from its left end (the toddler's end). "At rest" means moving
under 1 cm/s and turning under 0.5 rad/s.

| Credit | For |
|---|---|
| 0.2 | each hazard (knife, mug, pill bottle) at rest on the counter top itself (bottom within 5 mm of the top, not on the divider or another object), the knife lying on any of its four long faces with its long side tilted at most 15 degrees from level, the mug and bottle upright, tilted at most 15 degrees (any turn about the vertical), its nearest point at least 550 mm from the toddler's end, i.e. wholly right of the divider |
| 0.4 | the sippy cup at rest upright on the counter top, tilted at most 15 degrees, its centre at most 150 mm from the toddler's end, while every hazard's nearest point is at least 550 mm from that end |
| 0 overall | any of the four loose objects off the counter: its centre outside the board's outline (on the table, the stool or the floor) or its bottom 10 mm or more below the counter top |

The untouched start scores 0: every hazard is within 550 mm of the toddler's end and the cup is 790 mm from it. A
robot that clears the hazards but leaves the cup scores 0.6; one that brings the cup while any hazard is still within
550 mm of the toddler (the reach the divider marks) earns nothing for the cup. The knife's credit does not depend on
which face is up: flipped over or on its long edge it still lies along the counter; standing on its end it earns
nothing. An object still sliding or rolling at the last step earns nothing. The score is read from the final layout
only: a mug tipped over on the way and stood back up scores the same as one carried upright.

On a real counter, a tape measure from the left end gives each object's distance (the divider's right face is the
550 mm line), a ruler under the bottom shows whether it rests on the board, and a phone level app gives the tilt
(laid along the knife's long side, or against the side of a cylinder).

## The scripted demo

One handover spot on the counter's midline, 29 cm in front of the arm bases, is reached by both arms; only one arm is
near it at a time while the other waits 20 cm to its own side. The right arm picks up the sippy cup from above,
gripping its sides, and sets it down at the handover spot; the left arm takes it to the toddler's end, its centre
70 mm from the end. The left arm then brings the knife (gripped across its 30 mm width), the mug and the pill bottle
to the handover spot one at a time, and each time the right arm carries it over the divider and sets it down, the
knife 390 mm, the mug 280 mm and the bottle 170 mm right of the midline, so no carry passes over a placed hazard.
Every object is let go 8 mm above the counter. Once the bottle has left the handover spot, the left arm parks 13 cm
above the empty middle of the counter, 12 cm left of the midline, and the right arm parks 14 cm in front of the arm
bases (just short of the counter's front edge), 25 cm right of the midline. In the final top-camera view
(`demo_100.png`) the parked left arm hides the cup and the parked right arm hides the knife; `demo_066.png` shows the
cup already in place at the toddler's end.

The yellow stool marker stands just in front of the counter's front edge, 80 mm in from its left end, rather than
against the end itself: at the start, the left arm's raised pose hides the table beyond the counter's left end from
the top camera, and this spot stays in view. Distances are still measured from the counter's left end.

## Runtime and seeds

- The demo's plan takes 67.5 s of simulated time. Replayed offline, it reaches success, the 2.5 s hold included,
  after 67.3 s (about 1,350 steps at 20 Hz), well within the 3,000-step limit. A full Dream-1 trial runs at most
  3,000 steps (150 s of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
