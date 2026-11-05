# Store the cartons so no stack is taller than three times its base

Five cartons go onto two storage pads, one stack per pad. A common warehouse rule of thumb for hand stacking keeps a
free-standing stack no higher than three times the least side of its base, so the carton the robot puts at the
bottom of each stack, and how it rests, sets how tall that stack may grow. Standing the blue carton upright on the
two green cubes makes 19 cm against the 15 cm a cube allows, and all five in one tower make 27 cm against the 21 cm a
red carton allows, so the cartons must be split between the pads and the robot must choose which carton goes at the
bottom and how each one rests. The demo has each arm stack on the pad in front of it and hands one green cube across
the midline.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=stack_height_three_times_base.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put all five cartons onto the two grey pads, one stack per pad. Each stack's height above its pad must be no more than three times the shorter side of the carton at the bottom of that stack." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/stack-ratio-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=stack_height_three_times_base.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put all five cartons onto the two grey pads, one stack per pad. Each stack's height above its pad must be no more than three times the shorter side of the carton at the bottom of that stack." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/stack-ratio-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/stack-ratio-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all five cartons are counted in the two stacks,
both within their limits, and every carton has been still for 2.5 s; otherwise it scores the partial credit below
at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_stack_height_three_times_base.py` in an
environment with `yam_arena` installed.

## Scoring

All heights are measured from the pad top. The shorter footprint side (b) of a carton is taken as it rests: the blue
carton standing on its end has b = 4 cm, lying on its side also 4 cm; a green cube 5 cm; a red carton lying flat
7 cm, standing on edge 4 cm.

A carton *rests on a pad* when its bottom is within 5 mm of the pad top and its footprint overlaps the pad's
10 x 10 cm top at all, wherever its centre is. It *rests on another carton* when its bottom is within 5 mm of that
carton's top and their footprints overlap. These seat tests decide credit only. The height limit counts every carton
above the pad (its bottom higher than 5 mm below the pad top) whose footprint overlaps the pad or a carton counted in
its stack, whether or not it rests there or earns credit: a carton lying tilted on a tilted carton still counts.

| Credit | For |
|---|---|
| 0.2 | the bottom carton of a pad's stack: the only carton resting on that pad, its centre inside the pad's 10 x 10 cm outline, tilted at most 5 degrees from lying on one of its faces |
| 0.2 | each carton above it: resting on the counted carton below, its centre within 15 mm (seen from above) of that carton's centre, tilted at most 5 degrees from lying on a face; counting stops at the first carton that fails |
| 0 for the pad | no carton, or two or more cartons, resting on the pad |
| 0 overall | on a pad with a carton resting on it, the highest point of any carton counted in that pad's stack more than 3 x b above the pad top, b being that of the carton resting on the pad (the narrowest, if several do): 12 cm with the blue carton standing or lying, 15 cm with a green cube, 21 cm with a red carton lying flat, 12 cm with a red carton on its edge. A stack exactly at its limit passes |
| 0 overall | cartons stacked on the table beside the pads past the same limit: for any carton whose bottom is within 5 mm of the table top, the cartons counted from it the same way (every carton above the table whose footprint overlaps it or a carton already counted) reaching more than 3 x its b above the table top |

All five counted, both pads within their limits: 1.0. A carton on the table, on the floor, tipped, or held above the
stack (not resting on the pad or a carton) counts nothing; success additionally needs every carton still for 2.5 s. The demo's stacks are 14 cm on a green cube (limit 15 cm) and 13 cm on a red carton (limit
21 cm); another full-score split is the blue carton alone on the left pad (9 cm, limit 12 cm) and red, red, green,
green on the right (18 cm, limit 21 cm).

On a real table, a ruler held upright on the pad top gives each stack's height, a side photo shows which carton is at
the bottom and how it rests, a top photo checks each carton's centring on the one below, and a phone level app its
tilt.

## The scripted demo

Every grip is straight down, the fingers closing across the table. The left arm stands one green cube on the left
pad, then sets the other green cube down on the table on the midline, 27 cm in front of the arm bases, where the
right arm can reach it. It then stands the blue carton on its end on the first cube, letting go 8 mm above it,
lifts clear and parks behind its stack. Meanwhile the right arm lays both red cartons flat on the right pad, one on
the other, waits 3 s above its pad while the left arm carries the blue carton away from the midline, then picks up
the green cube from the midline, sets it on the red cartons, lifts clear and parks behind its stack.

## Runtime and seeds

- The demo's longest plan (the right arm's) takes about 35 s of simulated time; with the 2.5 s hold it needs about
  38 s, well within the 1,800-step (90 s) limit. On an L40S, once Isaac Sim has started, a scripted trial takes
  about 1 minute of wall time and a full Dream-1 trial 3 to 4 minutes (the first start on a new machine takes about
  4 minutes more).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
